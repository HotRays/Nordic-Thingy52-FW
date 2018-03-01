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

#include "m_ui.h"
#include "ble_uis.h"
#include "m_ui_flash.h"
#include "app_error.h"
#include "app_button.h"
#include <stdlib.h>
#include <string.h>
#include "pca20020.h"
#include "nrf_drv_gpiote.h"
#include "app_scheduler.h"
#include "app_timer.h"

#define  NRF_LOG_MODULE_NAME "m_ui          "
#include "nrf_log.h"
#include "macros_common.h"

static ble_uis_led_t     * mp_config_ui;
static ble_uis_t           m_uis;
static const ble_uis_led_t m_default_config_connected    = UI_CONFIG_DEFAULT_CONNECTED;
static const ble_uis_led_t m_default_config_disconnected = UI_CONFIG_DEFAULT_DISCONNECTED;
static const ble_uis_led_t m_default_config_error        = UI_CONFIG_DEFAULT_ERROR;



/**@brief Sends commands to drv_ext_light for changing led configurations.
 *
 * @param[in] p_config_ui       Contains all settings for the LED according to the BLE characteristics.
 * @param[in] p_color_only      Contains data from program calls (not BLE) if only color is to be changed.
 *
 * @Note If both p_config_ui and p_color_only == NULL, the static config will be loaded. This may be written to over BLE.
 * if only p_color_only == NULL, p_config_ui will be used, if only p_config_ui == NULL, p_color_only will be used.
 * Supplied intensities will be converted according to the current mode.
 *
 * @return NRF_SUCCESS
 * @return M_IU_STATUS_CODE_INVALID_PARAM
 * @return Other codes from the underlying drivers
 */
static ret_code_t led_set(ble_uis_led_t const * const p_config_ui)
{
    ble_uis_led_t  conf_ui;

    // Either load config from supplied conf_ui parameter or get from static variable.
    if ((p_config_ui == NULL)) // No config supplied, use static config.
    {
        conf_ui = *mp_config_ui;
    }
    else if(p_config_ui == NULL) // p_color_only supplied. Use the supplied colors according to the mode in mp_config_ui.
    {
        conf_ui = *mp_config_ui;
        switch(conf_ui.mode)
        {
            case BLE_UIS_LED_MODE_OFF:
                // No need to copy data as status = OFF.
                break;

            default: // Should never be reached.
                return M_IU_STATUS_CODE_INVALID_PARAM;
        }
    }
    else
    {
        return M_IU_STATUS_CODE_INVALID_PARAM;
    }

    if (mp_config_ui->mode == BLE_UIS_LED_MODE_OFF)
    {
        conf_ui.mode = BLE_UIS_LED_MODE_OFF;         // If LED configured as off in the static config, let the LED remain off when BLE is disconnected as well.
        NRF_LOG_INFO("Mode: BLE_UIS_LED_MODE_OFF \r\n");
        return NRF_SUCCESS;
    }
    else
    {
        return M_IU_STATUS_CODE_INVALID_PARAM;
    }
}


/**@brief Function for passing the BLE event to the UI service.
 *
 * @details This callback function will be called from the BLE handling module.
 *
 * @param[in] p_ble_evt    Pointer to the BLE event.
 */
static void thingy_ui_on_ble_evt(ble_evt_t * p_ble_evt)
{
    ret_code_t err_code;

    if (p_ble_evt == NULL)
    {
        return;
    }

    ble_uis_on_ble_evt(&m_uis, p_ble_evt);

    switch(p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED: // Upon reconnect, read last stored configuraion.
            NRF_LOG_INFO("BLE connected\r\n");
            err_code = led_set(mp_config_ui);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_GAP_EVT_DISCONNECTED: // When disconnecting, revert to the default LED config.
            NRF_LOG_INFO("BLE disonnected\r\n");
            err_code = m_ui_led_set_event(M_UI_BLE_DISCONNECTED);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for handling LED write events UI Service.
 *
 * @param[in] p_wss     UI Service structure.
 * @param[in] new_state Value of the RGB LED.
 */
static void ble_uis_led_write_handler(ble_uis_t * p_uis, ble_uis_led_t * input)
{
    ret_code_t         err_code;

    /* If anything has changed, store the new values to flash. */
    if (mp_config_ui != input)
    {
        (void)m_ui_flash_config_store(input);
    }

    err_code = led_set(NULL);

    if (err_code != NRF_ERROR_NOT_SUPPORTED && err_code != NRF_SUCCESS)
    {
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling PIN write events UI Service.
 *
 * @param[in] p_wss     UI Service structure.
 * @param[in] pin       Value of the pins.
 */
static void ble_uis_pin_write_handler(ble_uis_t * p_uis, ble_uis_pin_t * pin)
{

    NRF_LOG_INFO("ble_uis_pin_write_handler: MOS_1 %d, MOS_2 %d, MOS_3 %d, MOS_4 %d\r\n",
                                                                              pin->mos_1,
                                                                              pin->mos_2,
                                                                              pin->mos_3,
                                                                              pin->mos_4);

   if (pin->mos_1)
   {
       nrf_gpio_pin_set(MOS_1);
   }
   else
   {
       nrf_gpio_pin_clear(MOS_1);
   }

   if (pin->mos_2)
   {
       nrf_gpio_pin_set(MOS_2);
   }
   else
   {
       nrf_gpio_pin_clear(MOS_2);
   }

   if (pin->mos_3)
   {
       nrf_gpio_pin_set(MOS_3);
   }
   else
   {
       nrf_gpio_pin_clear(MOS_3);
   }

   if (pin->mos_4)
   {
       nrf_gpio_pin_set(MOS_4);
   }
   else
   {
       nrf_gpio_pin_clear(MOS_4);
   }
}


/**@brief Function for initializing the UI Service.
 *
 * @details This callback function will be called from the ble handling module to initialize the UI service.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
static ret_code_t thingy_ui_service_init(bool major_minor_fw_ver_changed)
{
    uint32_t        err_code;
    ble_uis_init_t  uis_init;

    /**@brief Load configuration from flash. */
    err_code = m_ui_flash_init(&m_default_config_connected, &mp_config_ui);
    RETURN_IF_ERROR(err_code);
    
    if (major_minor_fw_ver_changed)
    {
        err_code = m_ui_flash_config_store(&m_default_config_connected);
        APP_ERROR_CHECK(err_code);
    }

    uis_init.p_init_led        = mp_config_ui;
    uis_init.init_pin.mos_1    = 0;
    uis_init.init_pin.mos_2    = 0;
    uis_init.init_pin.mos_3    = 0;
    uis_init.init_pin.mos_4    = 0;
    uis_init.led_write_handler = ble_uis_led_write_handler;
    uis_init.pin_write_handler = ble_uis_pin_write_handler;

    return ble_uis_init(&m_uis, &uis_init);
}


static void button_evt_handler(uint8_t pin_no, uint8_t button_action)
{
    uint32_t err_code;

    if (pin_no == BUTTON)
    {
        //If Thingy is connected to a Central.
        if (m_uis.conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            err_code = ble_uis_on_button_change(&m_uis, button_action);
            if (err_code != NRF_ERROR_INVALID_STATE
                && err_code != NRF_SUCCESS)
            {
                APP_ERROR_CHECK(err_code);
            }
        }
    }
}


ret_code_t m_ui_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    return led_set(NULL);
}


ret_code_t m_ui_led_set_event(ui_led_events event_code)
{
    ret_code_t err_code;

    if (event_code == M_UI_BLE_CONNECTED)
    {
        err_code = led_set(&m_default_config_connected);
        RETURN_IF_ERROR(err_code);
    }
    else if (event_code == M_UI_BLE_DISCONNECTED)
    {
        err_code = led_set(&m_default_config_disconnected);
        RETURN_IF_ERROR(err_code);
    }
    else
    {
        err_code = led_set(&m_default_config_error);
        RETURN_IF_ERROR(err_code);
    }
    return NRF_SUCCESS;
}


static ret_code_t button_init(void)
{
    ret_code_t err_code;

    /* Configure gpiote for the sensors data ready interrupt. */
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        RETURN_IF_ERROR(err_code);
    }

    static const app_button_cfg_t button_cfg =
    {
        .pin_no         = BUTTON,
        .active_state   = APP_BUTTON_ACTIVE_LOW,
        .pull_cfg       = NRF_GPIO_PIN_PULLUP,
        .button_handler = button_evt_handler
    };

    err_code = app_button_init(&button_cfg, 1, APP_TIMER_TICKS(50));
    RETURN_IF_ERROR(err_code);

    return app_button_enable();
}


uint32_t m_ui_init(m_ble_service_handle_t * p_handle, m_ui_init_t * p_params)
{
    uint32_t                        err_code;
    //lint --e{651} Potentially confusing initializer

    NULL_PARAM_CHECK(p_handle);

    p_handle->ble_evt_cb = thingy_ui_on_ble_evt;
    p_handle->init_cb    = thingy_ui_service_init;

    err_code = button_init();
    APP_ERROR_CHECK(err_code);
    
    nrf_gpio_cfg_output(MOS_1);
    nrf_gpio_cfg_output(MOS_2);
    nrf_gpio_cfg_output(MOS_3);
    nrf_gpio_cfg_output(MOS_4);
    nrf_gpio_pin_clear(MOS_1);
    nrf_gpio_pin_clear(MOS_2);
    nrf_gpio_pin_clear(MOS_3);
    nrf_gpio_pin_clear(MOS_4);

    return NRF_SUCCESS;
}
