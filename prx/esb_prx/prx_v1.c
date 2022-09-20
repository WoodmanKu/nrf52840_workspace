
#include "nrf_esb.h"

#include <stdbool.h>
#include <stdint.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "boards.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_drv_gpiote.h"
#include "nrfx_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// timer
#include "app_timer.h"
#include "nrf_drv_clock.h"

// usb cdc
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "app_uart.h"

APP_TIMER_DEF(m_repeated_timer_id);

// leds
#define LED_ON 0
#define LED_OFF 1
#define LED_USB_RESUME (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN (BSP_BOARD_LED_5) //changed from led 1 to 5
#define LED_CDC_ACM_RX (BSP_BOARD_LED_2)
#define LED_CDC_ACM_TX (BSP_BOARD_LED_3)

// define usb
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE 0
#define CDC_ACM_COMM_EPIN NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE 1
#define CDC_ACM_DATA_EPIN NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT NRF_DRV_USBD_EPOUT1

//Enable power usb detection
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

#define READ_SIZE 1
static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];

/** @brief CDC_ACM class instance */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);
// usb define ended

uint8_t led_nr;

int count;

nrf_esb_payload_t rx_payload;

static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

static void repeated_timer_handler(void *p_context)
{
    size_t size = sprintf(m_tx_buffer, "fps: %u\r\n", count);

    app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, size);

    if (count >= 1 && count < 70)
    {
        nrf_gpio_pin_write(LED_1, 0);
        nrf_gpio_pin_write(LED_2, 1);
        nrf_gpio_pin_write(LED_3, 1);
        nrf_gpio_pin_write(LED_4, 1);
    }
    else if (count >= 70 && count < 330)
    {
        nrf_gpio_pin_write(LED_1, 0);
        nrf_gpio_pin_write(LED_2, 0);
        nrf_gpio_pin_write(LED_3, 1);
        nrf_gpio_pin_write(LED_4, 1);
    }
    else if (count >= 330 && count < 590)
    {
        nrf_gpio_pin_write(LED_1, 0);
        nrf_gpio_pin_write(LED_2, 0);
        nrf_gpio_pin_write(LED_3, 0);
        nrf_gpio_pin_write(LED_4, 1);
    }
    else if (count > 590)
    {
        nrf_gpio_pin_write(LED_1, 0);
        nrf_gpio_pin_write(LED_2, 0);
        nrf_gpio_pin_write(LED_3, 0);
        nrf_gpio_pin_write(LED_4, 0);
    }
    else
    {
        nrf_gpio_pin_write(LED_1, 1);
        nrf_gpio_pin_write(LED_2, 1);
        nrf_gpio_pin_write(LED_3, 1);
        nrf_gpio_pin_write(LED_4, 1);
    }



    count = 0;
}

static void create_timers()
{
    ret_code_t err_code = app_timer_create(&m_repeated_timer_id, APP_TIMER_MODE_REPEATED, repeated_timer_handler);
    APP_ERROR_CHECK(err_code);
}

void nrf_esb_event_handler(nrf_esb_evt_t const *p_event)
{
    switch (p_event->evt_id)
    {
    case NRF_ESB_EVENT_TX_SUCCESS:
        break;
    case NRF_ESB_EVENT_TX_FAILED:
        break;
    case NRF_ESB_EVENT_RX_RECEIVED:
        count++;
        if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
        {
            //nothing so far
        }
        break;
    }
}

// just sure the high-frequency clock is running
void clocks_start(void)
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
        ;
}

void gpio_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);

    // make sure leds are off
    nrf_gpio_pin_write(LED_1, 1);
    nrf_gpio_pin_write(LED_2, 1);
    nrf_gpio_pin_write(LED_3, 1);
    nrf_gpio_pin_write(LED_4, 1);
}

uint32_t esb_init(void)
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};
    nrf_esb_config_t nrf_esb_config = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode = NRF_ESB_MODE_PRX;
    nrf_esb_config.event_handler = nrf_esb_event_handler;
    nrf_esb_config.selective_auto_ack = true;

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return err_code;
}

/** @brief Function for initializing the nrf_log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/** @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const *p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
    case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
    {
        // indicate it's opened
        bsp_board_led_on(LED_CDC_ACM_OPEN);

        /*Setup first transfer*/
        ret_code_t err_code = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                               m_rx_buffer,
                                               READ_SIZE);
        UNUSED_VARIABLE(err_code);
        break;
    }

    case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
        bsp_board_led_off(LED_CDC_ACM_OPEN);
        break;

    case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
        break;

    case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
    {
        ret_code_t err_code;
        /*Get amount of data transfered*/
            do{
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                NRF_LOG_INFO("RX: size: %lu char: %c", size, m_rx_buffer[0]);

                /* Fetch data until internal buffer is empty */
                err_code = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            m_rx_buffer,
                                            READ_SIZE);
            } while (err_code == NRF_SUCCESS);

            bsp_board_led_invert(LED_CDC_ACM_RX);
            break;
    }
    default:
        break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
    case APP_USBD_EVT_DRV_SUSPEND:
        break;

    case APP_USBD_EVT_DRV_RESUME:
        break;

    case APP_USBD_EVT_STARTED:
        break;

    case APP_USBD_EVT_STOPPED:
        app_usbd_disable();
        // bsp_board_led_off(); //(tbc)
        break;

    case APP_USBD_EVT_POWER_DETECTED:
        if (!nrf_drv_usbd_is_enabled())
        {
            app_usbd_enable();
        }
        break;

    case APP_USBD_EVT_POWER_REMOVED:
    {
        app_usbd_stop();
    }
    break;

    case APP_USBD_EVT_POWER_READY:
    {
        app_usbd_start();
    }
    break;

    default:
        break;
    }
}

// USB CODE END

int main(void)
{
    ret_code_t err_code;

    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler};

    gpio_init();
    log_init();

    // start the timer
    lfclk_request();
    app_timer_init();
    create_timers();

    err_code = app_timer_start(m_repeated_timer_id, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // end of timer and log

    app_usbd_serial_num_generate();
    err_code = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(err_code);

    app_usbd_class_inst_t const *class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    err_code = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(err_code);

    // esb start
    clocks_start();

    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_esb_start_rx();
    APP_ERROR_CHECK(err_code);

    if (USBD_POWER_DETECTION) {
        err_code = app_usbd_power_events_enable();
        APP_ERROR_CHECK(err_code);
    }
    else {
        app_usbd_enable();
        app_usbd_start();
    }
    APP_ERROR_CHECK(err_code);

    for (;;) {
        if(nrf_esb_is_idle()){
            nrf_esb_start_rx();
        }
        while (app_usbd_event_queue_process()){
            //nothing to do wor
        }
         
    }
}

/*
abandoned codes

while (true)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            __WFE();
        }
    }

*/
