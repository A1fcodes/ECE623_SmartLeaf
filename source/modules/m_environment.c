/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "m_environment.h"
#include <string.h>
#include "app_util_platform.h"
#include "m_environment_flash.h"
#include "drv_humidity.h"
#include "drv_pressure.h"
#include "drv_gas_sensor.h"
#include "drv_color.h"
#include "app_timer.h"
#include "pca20020.h"
#include "nrf_delay.h"
#include "fstorage.h"
#include "m_ui.h"
#define  NRF_LOG_MODULE_NAME "m_env         "
#include "nrf_log.h"
#include "macros_common.h"
#include "drv_speaker.h"

/**@brief Different GAS sensor states.
 */
typedef enum
{
    GAS_STATE_IDLE,
    GAS_STATE_WARMUP,
    GAS_STATE_ACTIVE
}gas_state_t;

#define M_GAS_CALIB_INTERVAL_MS (1000 * 60 * 60) ///< Humidity and temperature calibration interval for the gas sensor [ms].
#define M_GAS_BASELINE_WRITE_MS (1000 * 60 * 30) ///< Stored baseline calibration delay for the gas sensor [ms].


#define M_PLANT_LOW_LIGHT_CLEAR_THRESHOLD 3
#define M_PLANT_DRY_AIR_HUMIDITY_PERCENT 40
#define M_PLANT_OVERWATER_HUMIDITY_PERCENT 65
#define M_PLANT_HIGH_TEMP_THRESHOLD_CELSIUS 28.0f
#define M_PLANT_LOW_TEMP_THRESHOLD_CELSIUS 15.0f
#define SMARTLEAF_LOW_LIGHT_BEEP_FREQ_HZ  1800
#define SMARTLEAF_DRY_AIR_BEEP_FREQ_HZ     900
#define SMARTLEAF_OVERWATER_BEEP_FREQ_HZ   500
#define SMARTLEAF_BEEP_DURATION_MS         300
#define SMARTLEAF_BEEP_VOLUME               80
#define M_PLANT_HIGH_TVOC_THRESHOLD_PPB     100
#define SMARTLEAF_VOC_ALERT_BEEP_FREQ_HZ    2300

#define M_PLANT_LOW_LIGHT_COUNT_LIMIT 3

static uint8_t m_dry_air_beep_counter = 0;
static uint8_t m_low_light_count = 0;

static void temperature_timeout_handler(void * p_context); ///< Temperature handler, forward declaration.
static void humidity_timeout_handler(void * p_context);    ///< Humidity handler, forward declaration.

static ble_tes_t              m_tes;            ///< Structure to identify the Thingy Environment Service.
static ble_tes_config_t     * m_p_config;       ///< Configuraion pointer.
static const ble_tes_config_t m_default_config = ENVIRONMENT_CONFIG_DEFAULT; ///< Default configuraion.
static m_gas_baseline_t     * m_p_baseline;     ///< Baseline pointer.
static const m_gas_baseline_t m_default_baseline = ENVIRONMENT_BASELINE_DEFAULT; ///< Default baseline.


static bool        m_get_humidity                   = false;
static bool        m_get_temperature                = false;
static bool        m_calib_gas_sensor               = false;
static gas_state_t m_gas_state                      = GAS_STATE_IDLE;
static bool        m_temp_humid_for_gas_calibration = false;    ///< Set when the gas sensor requires temperature and humidity for calibration.
static bool        m_temp_humid_for_ble_transfer    = false;    ///< Set when humidity or temperature is requested over BLE.
static bool m_tempature_active = false;
static bool m_pressure_active = false;
static bool m_humidity_active = false;
static bool m_color_active = false;
static bool m_low_light_sound_active = false;
static bool m_dry_air_sound_active   = false;
static bool m_overwater_sound_active = false;
static bool m_voc_alert_sound_active = false;
static ble_tes_plant_alert_t m_plant_alert_state = 0;

static uint32_t calibrate_gas_sensor(uint16_t humid, float temp);
static uint32_t gas_load_baseline_flash(uint16_t * p_gas_baseline);
static void plant_alert_condition_set(ble_tes_plant_alert_t mask, bool active);


APP_TIMER_DEF(temperature_timer_id);
APP_TIMER_DEF(pressure_timer_id);
APP_TIMER_DEF(humidity_timer_id);
APP_TIMER_DEF(color_timer_id);
APP_TIMER_DEF(gas_calib_timer_id);

/**@brief Function for converting the temperature sample.
 */
static void temperature_conv_data(float in_temp, ble_tes_temperature_t * p_out_temp)
{
    float f_decimal;

    p_out_temp->integer = (int8_t)in_temp;
    f_decimal = in_temp - p_out_temp->integer;
    p_out_temp->decimal = (uint8_t)(f_decimal * 100.0f);
    NRF_LOG_DEBUG("temperature_conv_data: Temperature: ,%d.%d,C\r\n", p_out_temp->integer, p_out_temp->decimal);
}

static void smartleaf_play_alert_sound(uint16_t freq_hz)
{
    uint32_t err_code;

    err_code = drv_speaker_tone_start(
        freq_hz,
        SMARTLEAF_BEEP_DURATION_MS,
        SMARTLEAF_BEEP_VOLUME
    );

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("SmartLeaf speaker alert failed: %d\r\n", err_code);
    }
}

static void smartleaf_play_beep_pattern(uint16_t freq_hz, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        smartleaf_play_alert_sound(freq_hz);
        nrf_delay_ms(SMARTLEAF_BEEP_DURATION_MS + 150);
    }
}


/**@brief Function for converting the humidity sample.
 */
static void humidity_conv_data(uint8_t humid, ble_tes_humidity_t * p_out_humid)
{
   *p_out_humid = (uint8_t)humid;
   NRF_LOG_DEBUG("humidity_conv_data: Relative Humidty: ,%d,%%\r\n", humid);
}


/**@brief Function for converting the pressure sample.
 */
static void pressure_conv_data(float in_press, ble_tes_pressure_t * p_out_press)
{
    float f_decimal;

    p_out_press->integer = (int32_t)in_press;
    f_decimal = in_press - p_out_press->integer;
    p_out_press->decimal = (uint8_t)(f_decimal * 100.0f);
    NRF_LOG_DEBUG("pressure_conv_data: Pressure/Altitude: %d.%d Pa/m\r\n", p_out_press->integer, p_out_press->decimal);
}

static void plant_alert_condition_set(ble_tes_plant_alert_t mask, bool active)
{

    ble_tes_plant_alert_t next_state = m_plant_alert_state;

    if (active)
    {
        next_state |= mask;
    }
    else{
        next_state &= (ble_tes_plant_alert_t)~mask;
    }

    if (next_state != m_plant_alert_state)
    {
        m_plant_alert_state = next_state;
        (void)ble_tes_plant_alert_set(&m_tes, &m_plant_alert_state);
    }
}


/**@brief Pressure sensor event handler.
 */
static void drv_pressure_evt_handler(drv_pressure_evt_t const * p_event)
{
    switch (p_event->type)
    {
        case DRV_PRESSURE_EVT_DATA:
        {
            if (p_event->mode == DRV_PRESSURE_MODE_BAROMETER)
            {
                ble_tes_pressure_t pressure;
                pressure_conv_data(drv_pressure_get(),&pressure);
                (void)ble_tes_pressure_set(&m_tes, &pressure);

            }
        }
        break;

        case DRV_PRESSURE_EVT_ERROR:
            APP_ERROR_CHECK_BOOL(false);
            break;

        default:
            break;
    }
}



/**@brief Humidity sensor event handler.
 */
static void drv_humidity_evt_handler(drv_humidity_evt_t event)
{
    uint32_t err_code;

    if (event == DRV_HUMIDITY_EVT_DATA)
    {
        ble_tes_temperature_t temp;
        ble_tes_humidity_t humid;

        float temperature = drv_humidity_temp_get();
        uint16_t humidity = drv_humidity_get();

        temperature_conv_data(temperature, &temp);
        humidity_conv_data(humidity, &humid);


        bool dry_air = humidity < M_PLANT_DRY_AIR_HUMIDITY_PERCENT;
        bool overwater = humidity > M_PLANT_OVERWATER_HUMIDITY_PERCENT;

        bool temperature_alert = (temperature > M_PLANT_HIGH_TEMP_THRESHOLD_CELSIUS) || (temperature < M_PLANT_LOW_TEMP_THRESHOLD_CELSIUS);
        if (m_tes.is_plant_alert_notif_enabled)
        {
            plant_alert_condition_set(BLE_TES_PLANT_ALERT_DRY_AIR_MASK, dry_air);
            plant_alert_condition_set(BLE_TES_PLANT_ALERT_TEMPERATURE_MASK, temperature_alert);
        }
        /*
        * Sound alert for possible watering issue.
        */
       if (dry_air && !m_dry_air_sound_active)
       {
        m_dry_air_sound_active = true;

        NRF_LOG_WARNING("SMARTLEAF ALERT: Dry air / possible water issue\r\n");

        // Blue LED warning
        (void)m_ui_led_set(0, 0, 100);

        // Lower beep = possible water issue
        smartleaf_play_beep_pattern(SMARTLEAF_DRY_AIR_BEEP_FREQ_HZ, 2);
       }
       else if (!dry_air)
       {
        m_dry_air_sound_active = false;
       }
       /*
       * Sound alert for possible overwatering.
       */
      if (overwater && !m_overwater_sound_active)
      {
        m_overwater_sound_active = true;
        NRF_LOG_WARNING("SMARTLEAF ALERT: Possible overwatering / too much moisture\r\n");
        // Cyan LED warning
        (void)m_ui_led_set(0, 100, 100);
        // Very low beep = overwater warning
        smartleaf_play_beep_pattern(SMARTLEAF_OVERWATER_BEEP_FREQ_HZ, 3);
       }
       else if (!overwater)
       {
        m_overwater_sound_active = false;
       }
       /*
         * Keep this part from the original code.
         * It is needed for gas sensor calibration.
         */
        if (m_calib_gas_sensor == true)
        {
            err_code = calibrate_gas_sensor(humidity, temperature);
            APP_ERROR_CHECK(err_code);
            m_calib_gas_sensor = false;
        }

        /*
         * These two blocks are needed for the Thingy app graphs.
         * Without them, temperature and humidity will not update.
         */
        if (m_get_temperature == true)
        {
            err_code = ble_tes_temperature_set(&m_tes, &temp);
            APP_ERROR_CHECK(err_code);
            m_get_temperature = false;
        }

        if (m_get_humidity == true)
        {
            err_code = ble_tes_humidity_set(&m_tes, &humid);
            APP_ERROR_CHECK(err_code);
            m_get_humidity = false;
        }
    }
    else
    {
        APP_ERROR_CHECK_BOOL(false);
    }
}

/**@brief Gas sensor data handler.
 */
static void drv_gas_data_handler(drv_gas_sensor_data_t const * p_data)
{
    if (p_data != NULL)
    {
        ble_tes_gas_t data;
        data.eco2_ppm = p_data->ec02_ppm;
        data.tvoc_ppb = p_data->tvoc_ppb;

        NRF_LOG_DEBUG("gas_data_handler eCO2:, %d, - TVOC:, %d,\r\n", p_data->ec02_ppm,
                                                                      p_data->tvoc_ppb);
        /*
         * SmartLeaf VOC alert:
         * High TVOC may indicate poor air quality, mold/decay, or stress.
         */
        bool high_voc = data.tvoc_ppb > M_PLANT_HIGH_TVOC_THRESHOLD_PPB;

        if (high_voc && !m_voc_alert_sound_active)
        {
            m_voc_alert_sound_active = true;

            NRF_LOG_WARNING("SMARTLEAF ALERT: High VOC / possible plant decay. TVOC=%d ppb\r\n",
                            data.tvoc_ppb);

            // Purple LED warning: red + blue
            (void)m_ui_led_set(100, 0, 100);

            // Sharp high beep = VOC / possible dying warning
            smartleaf_play_alert_sound(SMARTLEAF_VOC_ALERT_BEEP_FREQ_HZ);
        }
        else if (!high_voc)
        {
            m_voc_alert_sound_active = false;
        }

        (void)ble_tes_gas_set(&m_tes, &data);

        #if defined (ENV_DEBUG)
            uint32_t err_code;
            uint16_t dummy_baseline;
            uint16_t dummy_raw_adc;
            uint8_t dummy_current;
            err_code = drv_gas_sensor_baseline_get(&dummy_baseline);
            APP_ERROR_CHECK(err_code);
            err_code = drv_gas_sensor_raw_data_get(&dummy_current, &dummy_raw_adc);
            APP_ERROR_CHECK(err_code);
        #endif
    }
}


/**@brief Color sensor data handler.
 */
static void drv_color_data_handler(drv_color_data_t const * p_data)
{
    (void)drv_ext_light_off(DRV_EXT_RGB_LED_SENSE);

    if (p_data != NULL)
    {
        ble_tes_color_t data;
        NRF_LOG_DEBUG("color_data_handler r: %d - g: %d - b: %d - c: %d\r\n", p_data->red,
                                                                              p_data->green,
                                                                              p_data->blue,
                                                                              p_data->clear);
        data.red   = p_data->red;
        data.green = p_data->green;
        data.blue  = p_data->blue;
        data.clear = p_data->clear;


        bool low_light_sample = data.clear <= M_PLANT_LOW_LIGHT_CLEAR_THRESHOLD;
        NRF_LOG_INFO("LIGHT DEBUG: clear=%d threshold=%d low_sample=%d count=%d active=%d\r\n",
             data.clear,
             M_PLANT_LOW_LIGHT_CLEAR_THRESHOLD,
             low_light_sample,
             m_low_light_count,
             m_low_light_sound_active);

        if (low_light_sample)
        {
            if (m_low_light_count < 255)
            {
                m_low_light_count++;
            }
        }
        else
        {
            m_low_light_count = 0;
            m_low_light_sound_active = false;
        }

        bool low_light = m_low_light_count >= M_PLANT_LOW_LIGHT_COUNT_LIMIT;

        if (m_tes.is_plant_alert_notif_enabled)
        {
            plant_alert_condition_set(BLE_TES_PLANT_ALERT_LOW_LIGHT_MASK, low_light);
        }
        if (low_light && !m_low_light_sound_active)
        {
            m_low_light_sound_active = true;
            NRF_LOG_WARNING("Low light alert! Clear=%d R=%d G=%d B=%d\r\n",
                data.clear,
                data.red,
                data.green,
                data.blue);

                (void)m_ui_led_set(100, 0, 0); // RED LED warning
                smartleaf_play_beep_pattern(SMARTLEAF_LOW_LIGHT_BEEP_FREQ_HZ, 1);

            }
            (void)ble_tes_color_set(&m_tes, &data);
    }
}


/**@brief Function for handling temperature timer timeout event.
 *
 * @details This function will read the temperature at the configured rate.
 */
static void temperature_timeout_handler(void * p_context)
{
    uint32_t err_code;
    m_get_temperature = true;

    // Read temperature from humidity sensor.
    err_code = drv_humidity_sample();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting temperature sampling.
 */
static uint32_t temperature_start(void)
{
    uint32_t err_code;

    if (m_tempature_active)
    {
        return NRF_SUCCESS;
    }

    m_get_temperature = true;
    m_temp_humid_for_ble_transfer = true;

    err_code = drv_humidity_enable();
    RETURN_IF_ERROR(err_code);

    err_code = drv_humidity_sample();
    RETURN_IF_ERROR(err_code);

    err_code = app_timer_start(temperature_timer_id,
                               APP_TIMER_TICKS(m_p_config->temperature_interval_ms),
                               NULL);
    RETURN_IF_ERROR(err_code);

    m_tempature_active = true;

    return NRF_SUCCESS;
}


/**@brief Function for stopping temperature sampling.
 */
static uint32_t temperature_stop(bool disable_drv)
{
    uint32_t err_code;
    m_get_temperature = false;

    if (!m_tempature_active)
    {
        if (disable_drv && !m_humidity_active && !m_temp_humid_for_gas_calibration)
        {
            m_temp_humid_for_ble_transfer = false;
            return drv_humidity_disable();
        }

        return NRF_SUCCESS;
    }

    err_code = app_timer_stop(temperature_timer_id);
    RETURN_IF_ERROR(err_code);

    m_tempature_active = false;

    if (disable_drv)
    {
        m_temp_humid_for_ble_transfer = false;

        if (!m_temp_humid_for_gas_calibration) // If used by the gas sensor, do not turn off.
        {
            return drv_humidity_disable();
        }
        else
        {
            return NRF_SUCCESS;
        }
    }
    else
    {
        return NRF_SUCCESS;
    }
}


/**@brief Function for handling pressure timer timout event.
 *
 * @details This function will read the pressure at the configured rate.
 */
static void pressure_timeout_handler(void * p_context)
{
    uint32_t err_code;

    err_code = drv_pressure_sample();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting pressure sampling.
 */
static uint32_t pressure_start(void)
{
    uint32_t err_code;

    if (m_pressure_active)
    {
        return NRF_SUCCESS;
    }

    err_code = drv_pressure_enable();
    APP_ERROR_CHECK(err_code);

    err_code = drv_pressure_sample();
    APP_ERROR_CHECK(err_code);


    err_code = app_timer_start(pressure_timer_id,
                           APP_TIMER_TICKS(m_p_config->pressure_interval_ms),
                           NULL);
    RETURN_IF_ERROR(err_code);

    m_pressure_active = true;
    return NRF_SUCCESS;
}


/**@brief Function for stopping pressure sampling.
 */
static uint32_t pressure_stop(void)
{
    uint32_t err_code;

    if (m_pressure_active){
        return NRF_SUCCESS;
    }

    err_code = app_timer_stop(pressure_timer_id);
    RETURN_IF_ERROR(err_code);

    m_pressure_active = false;

    return drv_pressure_disable();
}

/**@brief Function for handling gas sensor calibration.
 *
 * @details This function will read the humidity and temperature at the configured rate.
 */
static void gas_calib_timeout_handler(void * p_context)
{
    uint32_t err_code;
    uint16_t gas_baseline = 0;

    if (m_gas_state == GAS_STATE_WARMUP)
    {
        err_code = gas_load_baseline_flash(&gas_baseline);
        APP_ERROR_CHECK(err_code);

        if (gas_baseline == 0)
        {
            NRF_LOG_WARNING("No valid baseline stored in flash. Baseline not written to gas sensor.\r\n");
        }
        else
        {
            err_code = drv_gas_sensor_baseline_set(gas_baseline);
            APP_ERROR_CHECK(err_code);
        }

        m_gas_state = GAS_STATE_ACTIVE;

        (void)app_timer_stop(gas_calib_timer_id);

    }
    else if (m_gas_state == GAS_STATE_ACTIVE)
    {
        // For later implementation of gas sensor humidity and temperature calibration.
    }
    else
    {
        // Should never happen.
    }
}


/**@brief Sends the sampled humidity and temperature to the gas sensor for calibration.
 *
 * @note Not currently used.
 */
static uint32_t calibrate_gas_sensor(uint16_t humid, float temp)
{
    uint32_t err_code;

    if (m_temp_humid_for_gas_calibration) // Check that the gas sensor is still enabled.
    {
        uint16_t rh_ppt    = humid * 10;
        int32_t temp_mdeg = (int32_t)(temp * 1000.0f);

        NRF_LOG_DEBUG("Calibrating gas sensor: humid out %d [ppt], temp out: %d [mdeg C]\r\n", rh_ppt, temp_mdeg);

        err_code = drv_gas_sensor_calibrate_humid_temp(rh_ppt, temp_mdeg);
        RETURN_IF_ERROR(err_code);

        return NRF_SUCCESS;
    }
    else
    {
        return NRF_SUCCESS; // Do nothing.
    }
}


/**@brief Stops the humidity and temperature sensor, used to calibrate the gas sensor.
 *
 * @note Not currently used.
 */
static uint32_t humidity_temp_stop_for_gas_calibration(void)
{
    uint32_t err_code;

    m_temp_humid_for_gas_calibration = false;

    if (m_temp_humid_for_ble_transfer)
    {
        // The temprature or humidity is being transferred over BLE. Do nothing.
    }
    else
    {
        err_code = drv_humidity_disable();
        RETURN_IF_ERROR(err_code);
    }

    return app_timer_stop(gas_calib_timer_id);
}


/**@brief Function for handling humidity timer timout event.
 *
 * @details This function will read the humidity at the configured rate.
 */
static void humidity_timeout_handler(void * p_context)
{
    uint32_t err_code;
    m_get_humidity = true;

    // Sample humidity sensor.
    err_code = drv_humidity_sample();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting humidity sampling.
 */
static uint32_t humidity_start(void)
{
    uint32_t err_code;

    if (m_humidity_active)
    {
        return NRF_SUCCESS;
    }

    m_get_humidity = true;
    m_temp_humid_for_ble_transfer = true;

    err_code = drv_humidity_enable();
    RETURN_IF_ERROR(err_code);

    err_code = drv_humidity_sample();
    RETURN_IF_ERROR(err_code);

    err_code = app_timer_start(humidity_timer_id,
                           APP_TIMER_TICKS(m_p_config->humidity_interval_ms),
                           NULL);

    RETURN_IF_ERROR(err_code);

    m_humidity_active = true;

    return NRF_SUCCESS;
}


/**@brief Function for stopping humidity sampling.
 */
static uint32_t humidity_stop(bool disable_drv)
{
    uint32_t err_code;

    m_get_humidity = false;

    if (!m_humidity_active)
    {
        if (disable_drv && !m_tempature_active && !m_temp_humid_for_gas_calibration)
        {
            m_temp_humid_for_ble_transfer = false;
            return drv_humidity_disable();
        }
        return NRF_SUCCESS;
    }

    err_code = app_timer_stop(humidity_timer_id);
    RETURN_IF_ERROR(err_code);

    m_humidity_active = false;

    if (disable_drv)
    {
        m_temp_humid_for_ble_transfer = false;

        if (!m_temp_humid_for_gas_calibration) // If used by the gas sensor, do not turn off.
        {
            return drv_humidity_disable();
        }
        else
        {
            return NRF_SUCCESS;
        }
    }
    else
    {
        return NRF_SUCCESS;
    }
}

/**@brief Loads the gas sensor baseline values from flash storage.
 */
static uint32_t gas_load_baseline_flash(uint16_t * p_gas_baseline)
{
    // No explicit flash load performed here, since this is done by the m_environment module at init and stored in m_p_config.
    switch(m_p_config->gas_interval_mode)
    {
        case GAS_MODE_250MS:
            *p_gas_baseline = m_p_baseline->mode_250ms;
            NRF_LOG_DEBUG("Gas sensor baseline loaded from flash, value 0x%04x, mode: GAS_MODE_250MS \r\n", *p_gas_baseline);
        break;

        case GAS_MODE_1S:
            *p_gas_baseline = m_p_baseline->mode_1s;
            NRF_LOG_DEBUG("Gas sensor baseline loaded from flash, value 0x%04x, mode: GAS_MODE_1S \r\n", *p_gas_baseline);
        break;

        case GAS_MODE_10S:
            *p_gas_baseline = m_p_baseline->mode_10s;
            NRF_LOG_DEBUG("Gas sensor baseline loaded from flash, value 0x%04x, mode: GAS_MODE_10S \r\n", *p_gas_baseline);
        break;

        case GAS_MODE_60S:
            *p_gas_baseline = m_p_baseline->mode_60s;
            NRF_LOG_DEBUG("Gas sensor baseline loaded from flash, value 0x%04x, mode: GAS_MODE_60S \r\n", *p_gas_baseline);
        break;

        default:
            return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}


/**@brief Stores the gas sensor baseline values to flash storage.
 */
static uint32_t gas_store_baseline_flash(uint16_t baseline)
{
    uint32_t err_code;

    switch(m_p_config->gas_interval_mode)
    {
        case GAS_MODE_250MS:
            m_p_baseline->mode_250ms = baseline;
            NRF_LOG_DEBUG("Gas sensor baseline stored to flash, value 0x%04x, mode: GAS_MODE_250MS\r\n", baseline);
        break;

        case GAS_MODE_1S:
            m_p_baseline->mode_1s = baseline;
            NRF_LOG_DEBUG("Gas sensor baseline stored to flash, value 0x%04x, mode: GAS_MODE_1S\r\n", baseline);
        break;

        case GAS_MODE_10S:
            m_p_baseline->mode_10s = baseline;
            NRF_LOG_DEBUG("Gas sensor baseline stored to flash, value 0x%04x, mode: GAS_MODE_10S\r\n", baseline);
        break;

        case GAS_MODE_60S:
            m_p_baseline->mode_60s = baseline;
            NRF_LOG_DEBUG("Gas sensor baseline stored to flash, value 0x%04x, mode: GAS_MODE_60S\r\n", baseline);
        break;

        default:
            return NRF_ERROR_INVALID_STATE;
    }

    err_code = m_env_flash_baseline_store(m_p_baseline); // Store new baseline values to flash.
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


static uint32_t gas_start(void)
{
    NRF_LOG_DEBUG("Gas start: mode: 0x%x \r\n", m_p_config->gas_interval_mode);

    uint32_t err_code;
    drv_gas_sensor_mode_t mode;

    switch (m_p_config->gas_interval_mode)
    {
        case GAS_MODE_250MS:
            mode = DRV_GAS_SENSOR_MODE_250MS;
            break;
        case GAS_MODE_1S:
            mode = DRV_GAS_SENSOR_MODE_1S;
            break;
        case GAS_MODE_10S:
            mode = DRV_GAS_SENSOR_MODE_10S;
            break;
        case GAS_MODE_60S:
            mode = DRV_GAS_SENSOR_MODE_60S;
            break;
        default:
            mode = DRV_GAS_SENSOR_MODE_10S;
            break;
    }

    err_code = drv_gas_sensor_start(mode);
    RETURN_IF_ERROR(err_code);

    m_gas_state = GAS_STATE_WARMUP;

    return app_timer_start(gas_calib_timer_id,
                           APP_TIMER_TICKS(M_GAS_BASELINE_WRITE_MS),
                           NULL);
}


static uint32_t gas_stop(void)
{
    uint32_t err_code;
    uint16_t baseline;

    if (m_gas_state == GAS_STATE_ACTIVE)
    {
        err_code = humidity_temp_stop_for_gas_calibration();
        RETURN_IF_ERROR(err_code);

        err_code = drv_gas_sensor_baseline_get(&baseline);
        RETURN_IF_ERROR(err_code);

        err_code = gas_store_baseline_flash(baseline);
        RETURN_IF_ERROR(err_code);
    }

    m_gas_state = GAS_STATE_IDLE;

    return drv_gas_sensor_stop();
}


/**@brief Function for handling color timer timeout event.
 *
 * @details This function will read the color at the configured rate.
 */
static void color_timeout_handler(void * p_context)
{
    uint32_t                    err_code;
    // drv_ext_light_rgb_intensity_t color;

    (void)drv_ext_light_off(DRV_EXT_RGB_LED_SENSE);

    err_code = drv_color_sample();
    APP_ERROR_CHECK(err_code);
}


static uint32_t color_start(void)
{
    uint32_t                    err_code;
    drv_ext_light_rgb_intensity_t color;

    if( m_color_active)
    {
        return NRF_SUCCESS;
    }

    // color.r = 0;
    // color.g = 0;
    // color.b = 0;

    // (void)drv_ext_light_rgb_intensity_set(DRV_EXT_RGB_LED_SENSE, &color);

    err_code = drv_color_start();
    APP_ERROR_CHECK(err_code);

    (void)drv_ext_light_off(DRV_EXT_RGB_LED_SENSE);

    err_code = drv_color_sample();
    APP_ERROR_CHECK(err_code);

    err_code =  app_timer_start(color_timer_id,
                           APP_TIMER_TICKS(m_p_config->color_interval_ms),
                           NULL);
    RETURN_IF_ERROR(err_code);

    m_color_active = true;

    return NRF_SUCCESS;
}


static uint32_t color_stop(void)
{
    uint32_t err_code;

    if (!m_color_active)
    {
        return NRF_SUCCESS;
    }

    err_code = app_timer_stop(color_timer_id);
    APP_ERROR_CHECK(err_code);

    m_color_active = false;

    (void)drv_ext_light_off(DRV_EXT_RGB_LED_SENSE);

    err_code = drv_color_stop();
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}


static uint32_t config_verify(ble_tes_config_t * p_config)
{
    uint32_t err_code;

    if ( (p_config->temperature_interval_ms < BLE_TES_CONFIG_TEMPERATURE_INT_MIN)    ||
         (p_config->temperature_interval_ms > BLE_TES_CONFIG_TEMPERATURE_INT_MAX)    ||
         (p_config->pressure_interval_ms < BLE_TES_CONFIG_PRESSURE_INT_MIN)          ||
         (p_config->pressure_interval_ms > BLE_TES_CONFIG_PRESSURE_INT_MAX)          ||
         (p_config->humidity_interval_ms < BLE_TES_CONFIG_HUMIDITY_INT_MIN)          ||
         (p_config->humidity_interval_ms > BLE_TES_CONFIG_HUMIDITY_INT_MAX)          ||
         (p_config->color_interval_ms < BLE_TES_CONFIG_COLOR_INT_MIN)                ||
         (p_config->color_interval_ms > BLE_TES_CONFIG_COLOR_INT_MAX)                ||
         (p_config->gas_interval_mode < BLE_TES_CONFIG_GAS_MODE_MIN)                 ||
         ((int)p_config->gas_interval_mode > (int)BLE_TES_CONFIG_GAS_MODE_MAX))
    {
        err_code = m_env_flash_config_store((ble_tes_config_t *)&m_default_config);
        APP_ERROR_CHECK(err_code);
    }

    return NRF_SUCCESS;
}


/**@brief Function for applying the configuration.
 *
 */
static uint32_t config_apply(ble_tes_config_t * p_config)
{
    uint32_t err_code;

    NULL_PARAM_CHECK(p_config);

    (void)temperature_stop(false);
    (void)pressure_stop();
    (void)humidity_stop(true);
    (void)color_stop();

    if ((p_config->temperature_interval_ms > 0) &&
        (m_tes.is_temperature_notif_enabled) )
    {
        err_code = temperature_start();
        APP_ERROR_CHECK(err_code);
    }

    if ((p_config->pressure_interval_ms > 0) &&
        (m_tes.is_pressure_notif_enabled) )
    {
        err_code = pressure_start();
        APP_ERROR_CHECK(err_code);
    }

    if ((p_config->humidity_interval_ms > 0) &&
        (m_tes.is_humidity_notif_enabled || m_tes.is_plant_alert_notif_enabled) )
    {
        err_code = humidity_start();
        APP_ERROR_CHECK(err_code);
    }

    if ((p_config->color_interval_ms > 0) )
        //(m_tes.is_color_notif_enabled || m_tes.is_plant_alert_notif_enabled) )
    {
        err_code = color_start();
        APP_ERROR_CHECK(err_code);
    }

    return NRF_SUCCESS;
}


/**@brief Function for handling event from the Thingy Environment Service.
 *
 * @details This function will process the data received from the Thingy Environment BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_tes    Thingy Environment Service structure.
 * @param[in] evt_type Thingy Environment Service event type.
 * @param[in] p_data   Event data.
 * @param[in] length   Length of the data.
 */
static void ble_tes_evt_handler( ble_tes_t        * p_tes,
                                 ble_tes_evt_type_t evt_type,
                                 uint8_t          * p_data,
                                 uint16_t           length)
{
    uint32_t err_code;

    switch (evt_type)
    {
        case BLE_TES_EVT_NOTIF_TEMPERATURE:
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_NOTIF_TEMPERATURE: %d\r\n", p_tes->is_temperature_notif_enabled);
            if (p_tes->is_temperature_notif_enabled)
            {
                err_code = temperature_start();
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                err_code = temperature_stop((p_tes->is_humidity_notif_enabled == false) && (p_tes->is_plant_alert_notif_enabled == false));
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_TES_EVT_NOTIF_PRESSURE:
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_NOTIF_PRESSURE: %d\r\n", p_tes->is_pressure_notif_enabled);
            if (p_tes->is_pressure_notif_enabled)
            {
                err_code = pressure_start();
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                err_code = pressure_stop();
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_TES_EVT_NOTIF_HUMIDITY:
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_NOTIF_HUMIDITY: %d\r\n", p_tes->is_humidity_notif_enabled);
            if (p_tes->is_humidity_notif_enabled)
            {
                err_code = humidity_start();
                APP_ERROR_CHECK(err_code);
            }
            else
            {   if (!p_tes->is_plant_alert_notif_enabled) // If plant alert is not enabled, the humidity sensor can be turned off. Otherwise, keep it on to monitor the plant alert condition.
                {
                    err_code = humidity_stop(p_tes->is_temperature_notif_enabled == false);
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BLE_TES_EVT_NOTIF_GAS:
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_NOTIF_GAS: %d\r\n", p_tes->is_gas_notif_enabled);
            if (p_tes->is_gas_notif_enabled)
            {
                err_code = gas_start();
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                err_code = gas_stop();
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_TES_EVT_NOTIF_COLOR:
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_NOTIF_COLOR: %d\r\n", p_tes->is_color_notif_enabled);
            if (p_tes->is_color_notif_enabled)
            {
                err_code = color_start();
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // if (!p_tes->is_plant_alert_notif_enabled) // If plant alert is not enabled, the color sensor can be turned off. Otherwise, keep it on to monitor the plant alert condition.
                // {
                //     err_code = color_stop();
                //     APP_ERROR_CHECK(err_code);
                // }
                
            }
            break;

        case BLE_TES_EVT_NOTIF_PLANT_ALERT:
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_NOTIF_PLANT_ALERT: %d\r\n", p_tes->is_plant_alert_notif_enabled);
            if (p_tes->is_plant_alert_notif_enabled)
            {
                m_plant_alert_state = 0;

                if (m_p_config->humidity_interval_ms > 0)
                {
                    err_code = humidity_start();
                    APP_ERROR_CHECK(err_code);
                }
                if (m_p_config->color_interval_ms > 0)
                {
                    err_code = color_start();
                    APP_ERROR_CHECK(err_code);
                }

                (void)ble_tes_plant_alert_set(&m_tes, &m_plant_alert_state);

#if defined(ENV_DEBUG)
                /* Force a plant alert notification for debug/testing. */
                plant_alert_condition_set(BLE_TES_PLANT_ALERT_LOW_LIGHT_MASK, true);
#endif
            }
            else{
                m_plant_alert_state = 0;

                if (!p_tes->is_humidity_notif_enabled)
                {
                    err_code = humidity_stop(p_tes->is_temperature_notif_enabled == false);
                    APP_ERROR_CHECK(err_code);
                }

                if (!p_tes->is_color_notif_enabled)
                {
                    err_code = color_stop();
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;



        case BLE_TES_EVT_CONFIG_RECEIVED:
        {
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_CONFIG_RECEIVED: %d\r\n", length);
            APP_ERROR_CHECK_BOOL(length == sizeof(ble_tes_config_t));

            err_code = m_env_flash_config_store((ble_tes_config_t *)p_data);
            APP_ERROR_CHECK(err_code);

            err_code = config_apply((ble_tes_config_t *)p_data);
            APP_ERROR_CHECK(err_code);
        }
        break;

        default:
            break;

    }
}


/**@brief Function for initializing the Thingy Environment Service.
 *
 * @details This callback function will be called from the ble handling module to initialize the Thingy Environment service.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
static uint32_t environment_service_init(bool major_minor_fw_ver_changed)
{
    uint32_t              err_code;
    ble_tes_temperature_t temperature = {.integer = 0, .decimal = 0};
    ble_tes_pressure_t    pressure    = {.integer = 0, .decimal = 0};
    ble_tes_humidity_t    humidity    = 0;
    ble_tes_color_t       color       = {.red = 0, .green = 0, .blue = 0, .clear = 0};
    ble_tes_gas_t         gas         = {.eco2_ppm = 0, .tvoc_ppb = 0};
    ble_tes_plant_alert_t plant_alert = 0;
    ble_tes_init_t        tes_init;

    /**@brief Load configuration from flash. */
    err_code = m_env_flash_init(&m_default_config, &m_p_config, &m_default_baseline, &m_p_baseline);
    RETURN_IF_ERROR(err_code);

    if (major_minor_fw_ver_changed)
    {
        err_code = m_env_flash_config_store(&m_default_config);
        APP_ERROR_CHECK(err_code);

        err_code = m_env_flash_baseline_store(&m_default_baseline);
        APP_ERROR_CHECK(err_code);
    }

    err_code = config_verify(m_p_config);
    RETURN_IF_ERROR(err_code);

    memset(&tes_init, 0, sizeof(tes_init));

    tes_init.p_init_temperature = &temperature;
    tes_init.p_init_pressure = &pressure;
    tes_init.p_init_humidity = &humidity;
    tes_init.p_init_color = &color;
    tes_init.p_init_gas = &gas;
    tes_init.p_init_plant_alert = &plant_alert;
    tes_init.p_init_config = m_p_config;
    tes_init.evt_handler = ble_tes_evt_handler;

    NRF_LOG_INFO("Init: ble_tes_init \r\n");
    err_code = ble_tes_init(&m_tes, &tes_init);
    RETURN_IF_ERROR(err_code);

    (void)config_apply(m_p_config);

    return NRF_SUCCESS;
}


/**@brief Function for passing the BLE event to the Thingy Environment service.
 *
 * @details This callback function will be called from the BLE handling module.
 *
 * @param[in] p_ble_evt    Pointer to the BLE event.
 */
static void environment_on_ble_evt(ble_evt_t * p_ble_evt)
{
    ble_tes_on_ble_evt(&m_tes, p_ble_evt);

    if (p_ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED)
    {
        uint32_t err_code;
        err_code = m_environment_stop();
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the humidity/temperature sensor
 */
static uint32_t humidity_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    ret_code_t               err_code = NRF_SUCCESS;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    drv_humidity_init_t    init_params =
    {
        .twi_addr            = HTS221_ADDR,
        .pin_int             = HTS_INT,
        .p_twi_instance      = p_twi_instance,
        .p_twi_cfg           = &twi_config,
        .evt_handler         = drv_humidity_evt_handler
    };

    err_code = drv_humidity_init(&init_params);

    return err_code;
}


static uint32_t pressure_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    drv_pressure_init_t init_params;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.twi_addr                = LPS22HB_ADDR;
    init_params.pin_int                 = LPS_INT;
    init_params.p_twi_instance          = p_twi_instance;
    init_params.p_twi_cfg               = &twi_config;
    init_params.evt_handler             = drv_pressure_evt_handler;
    init_params.mode                    = DRV_PRESSURE_MODE_BAROMETER;

    return drv_pressure_init(&init_params);
}


static uint32_t gas_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    uint32_t       err_code;
    drv_gas_init_t init_params;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.p_twi_instance = p_twi_instance;
    init_params.p_twi_cfg      = &twi_config;
    init_params.twi_addr       = CCS811_ADDR;
    init_params.data_handler   = drv_gas_data_handler;

    err_code = drv_gas_sensor_init(&init_params);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


static uint32_t color_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    uint32_t err_code;
    drv_color_init_t init_params;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.p_twi_instance = p_twi_instance;
    init_params.p_twi_cfg      = &twi_config;
    init_params.twi_addr       = BH1745_ADDR;
    init_params.data_handler   = drv_color_data_handler;

    err_code = drv_color_init(&init_params);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


uint32_t m_environment_start(void)
{
    return NRF_SUCCESS;
}


uint32_t m_environment_stop(void)
{
    uint32_t err_code;

    err_code = temperature_stop(false);
    APP_ERROR_CHECK(err_code);

    err_code = pressure_stop();
    APP_ERROR_CHECK(err_code);

    err_code = humidity_stop(true);
    APP_ERROR_CHECK(err_code);

    err_code = color_stop();
    APP_ERROR_CHECK(err_code);

    err_code = gas_stop();
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}


uint32_t m_environment_init(m_ble_service_handle_t * p_handle, m_environment_init_t * p_params)
{
    uint32_t err_code;

    NULL_PARAM_CHECK(p_handle);
    NULL_PARAM_CHECK(p_params);

    NRF_LOG_INFO("Init: \r\n");

    p_handle->ble_evt_cb = environment_on_ble_evt;
    p_handle->init_cb    = environment_service_init;

    /**@brief Init drivers */
    err_code = pressure_sensor_init(p_params->p_twi_instance);
    APP_ERROR_CHECK(err_code);

    err_code = humidity_sensor_init(p_params->p_twi_instance);
    APP_ERROR_CHECK(err_code);

    err_code = gas_sensor_init(p_params->p_twi_instance);
    APP_ERROR_CHECK(err_code);

    err_code = color_sensor_init(p_params->p_twi_instance);
    APP_ERROR_CHECK(err_code);

    /**@brief Init application timers */
    err_code = app_timer_create(&temperature_timer_id, APP_TIMER_MODE_REPEATED, temperature_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&pressure_timer_id, APP_TIMER_MODE_REPEATED, pressure_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&humidity_timer_id, APP_TIMER_MODE_REPEATED, humidity_timeout_handler);
    RETURN_IF_ERROR(err_code);

    err_code = app_timer_create(&color_timer_id, APP_TIMER_MODE_REPEATED, color_timeout_handler);
    RETURN_IF_ERROR(err_code);

    err_code = app_timer_create(&gas_calib_timer_id, APP_TIMER_MODE_REPEATED, gas_calib_timeout_handler);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}
