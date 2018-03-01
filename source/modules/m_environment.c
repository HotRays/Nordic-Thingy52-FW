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
#include "app_timer.h"
#include "pca20020.h"
#include "nrf_delay.h"
#include "fstorage.h"
#include "m_ui.h"
#define  NRF_LOG_MODULE_NAME "m_env         "
#include "nrf_log.h"
#include "macros_common.h"

static void temperature_timeout_handler(void * p_context); ///< Temperature handler, forward declaration.
static void humidity_timeout_handler(void * p_context);    ///< Humidity handler, forward declaration.

static ble_tes_t              m_tes;            ///< Structure to identify the Thingy Environment Service.
static ble_tes_config_t     * m_p_config;       ///< Configuraion pointer.
static const ble_tes_config_t m_default_config = ENVIRONMENT_CONFIG_DEFAULT; ///< Default configuraion.

static bool        m_get_humidity                   = false;
static bool        m_get_temperature                = false;
static bool        m_temp_humid_for_ble_transfer    = false;    ///< Set when humidity or temperature is requested over BLE.

APP_TIMER_DEF(temperature_timer_id);
APP_TIMER_DEF(humidity_timer_id);

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


/**@brief Function for converting the humidity sample.
 */
static void humidity_conv_data(uint8_t humid, ble_tes_humidity_t * p_out_humid)
{
   *p_out_humid = (uint8_t)humid;
   NRF_LOG_DEBUG("humidity_conv_data: Relative Humidty: ,%d,%%\r\n", humid);
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

    return NRF_SUCCESS;
}


/**@brief Function for stopping temperature sampling.
 */
static uint32_t temperature_stop(bool disable_drv)
{
    uint32_t err_code;
    m_get_temperature = false;

    err_code = app_timer_stop(temperature_timer_id);
    RETURN_IF_ERROR(err_code);

    if (disable_drv)
    {
        m_temp_humid_for_ble_transfer = false;
        return drv_humidity_disable();
    }
    else
    {
        return NRF_SUCCESS;
    }
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

    m_get_humidity = true;
    m_temp_humid_for_ble_transfer = true;

    err_code = drv_humidity_enable();
    RETURN_IF_ERROR(err_code);

    err_code = drv_humidity_sample();
    RETURN_IF_ERROR(err_code);

    return app_timer_start(humidity_timer_id,
                           APP_TIMER_TICKS(m_p_config->humidity_interval_ms),
                           NULL);
}


/**@brief Function for stopping humidity sampling.
 */
static uint32_t humidity_stop(bool disable_drv)
{
    uint32_t err_code;

    m_get_humidity = false;

    err_code = app_timer_stop(humidity_timer_id);
    RETURN_IF_ERROR(err_code);

    if (disable_drv)
    {
        m_temp_humid_for_ble_transfer = false;
        return drv_humidity_disable();
    }
    else
    {
        return NRF_SUCCESS;
    }
}

static uint32_t config_verify(ble_tes_config_t * p_config)
{
    uint32_t err_code;

    if ( (p_config->temperature_interval_ms < BLE_TES_CONFIG_TEMPERATURE_INT_MIN)    ||
         (p_config->temperature_interval_ms > BLE_TES_CONFIG_TEMPERATURE_INT_MAX)    ||
         (p_config->humidity_interval_ms < BLE_TES_CONFIG_HUMIDITY_INT_MIN)          ||
         (p_config->humidity_interval_ms > BLE_TES_CONFIG_HUMIDITY_INT_MAX))
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
    (void)humidity_stop(true);

    if ((p_config->temperature_interval_ms > 0) &&
        (m_tes.is_temperature_notif_enabled) )
    {
        err_code = temperature_start();
        APP_ERROR_CHECK(err_code);
    }

    if ((p_config->humidity_interval_ms > 0) &&
        (m_tes.is_humidity_notif_enabled) )
    {
        err_code = humidity_start();
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
                err_code = temperature_stop(p_tes->is_humidity_notif_enabled == false);
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
            {
                err_code = humidity_stop(p_tes->is_temperature_notif_enabled == false);
                APP_ERROR_CHECK(err_code);
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
        {
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_: %d\r\n", evt_type);
        }
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
    ble_tes_humidity_t    humidity    = 0;
    ble_tes_init_t        tes_init;

    /**@brief Load configuration from flash. */
    err_code = m_env_flash_init(&m_default_config, &m_p_config);
    RETURN_IF_ERROR(err_code);

    if (major_minor_fw_ver_changed)
    {
        err_code = m_env_flash_config_store(&m_default_config);
        APP_ERROR_CHECK(err_code);

        // err_code = m_env_flash_baseline_store(&m_default_baseline);
        // APP_ERROR_CHECK(err_code);
    }

    err_code = config_verify(m_p_config);
    RETURN_IF_ERROR(err_code);

    memset(&tes_init, 0, sizeof(tes_init));

    tes_init.p_init_temperature = &temperature;
    tes_init.p_init_humidity = &humidity;
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


uint32_t m_environment_start(void)
{
    return NRF_SUCCESS;
}


uint32_t m_environment_stop(void)
{
    uint32_t err_code;

    err_code = temperature_stop(false);
    APP_ERROR_CHECK(err_code);

    err_code = humidity_stop(true);
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
    // err_code = pressure_sensor_init(p_params->p_twi_instance);
    // APP_ERROR_CHECK(err_code);

    err_code = humidity_sensor_init(p_params->p_twi_instance);
    APP_ERROR_CHECK(err_code);

    // err_code = gas_sensor_init(p_params->p_twi_instance);
    // APP_ERROR_CHECK(err_code);

    // err_code = color_sensor_init(p_params->p_twi_instance);
    // APP_ERROR_CHECK(err_code);

    /**@brief Init application timers */
    err_code = app_timer_create(&temperature_timer_id, APP_TIMER_MODE_REPEATED, temperature_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // err_code = app_timer_create(&pressure_timer_id, APP_TIMER_MODE_REPEATED, pressure_timeout_handler);
    // APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&humidity_timer_id, APP_TIMER_MODE_REPEATED, humidity_timeout_handler);
    RETURN_IF_ERROR(err_code);

    // err_code = app_timer_create(&color_timer_id, APP_TIMER_MODE_REPEATED, color_timeout_handler);
    // RETURN_IF_ERROR(err_code);

    // err_code = app_timer_create(&gas_calib_timer_id, APP_TIMER_MODE_REPEATED, gas_calib_timeout_handler);
    // RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}
