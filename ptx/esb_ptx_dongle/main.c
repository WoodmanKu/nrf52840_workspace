/*
60Hz_v3

try:
1. try init timer during looping
2. use ppi channel to simulate power down
3. no more selfish talk

*/

#include "boards.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_beta.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_util.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_rng.h"
#include "bsp.h"

#define RANDOM_BUFF_SIZE    16      /**< Random numbers buffer size. */

/** @brief Function for getting vector of random numbers.
 *
 * @param[out] p_buff       Pointer to unit8_t buffer for storing the bytes.
 * @param[in]  length       Number of bytes to take from pool and place in p_buff.
 *
 * @retval     Number of bytes actually placed in p_buff.
 */

uint8_t beta_id = 0;
uint32_t rf_channel;  //present rf channel used by esb
bool beta_id_flag; //false = haven't got a id yet, btw got id means "in queue"

static nrf_esb_payload_t tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);

static nrf_esb_payload_t rx_payload; //the command from master

static int16_t num_of_beta = 20;  //(20) the total no. of beta we wanna connect

const int16_t timeslot = 400;        // or so called "tick"
static const nrf_drv_timer_t TIMER_ESB = NRF_DRV_TIMER_INSTANCE(0); //for sending loop
static const nrf_drv_timer_t TIMER_ESB_ENTRY = NRF_DRV_TIMER_INSTANCE(1); //for entry loop
bool entry_flagger = true; // if true, try to enter/re-enter the queue
static volatile int8_t state;             // state of the sending looping
static volatile bool entry_flag = true;      // in entry loop, when flag is true, send package (used to replace delay)

//set up ppi channel
static nrf_ppi_channel_t m_ppi_channel1;

// for "re-entry"
static volatile int16_t count_fail = 0; // the no. of sending failure
static volatile int16_t count_success = 0;
static volatile int16_t count_send = 0;    // count how many time it has entered the sending state 0
static volatile int16_t check_now = 60;       // check and reset the count_fail for each 60 frames
static const int16_t entry_threshold = 3; // the fail-threshold used when entry flagger = true;

void nrf_esb_event_handler(nrf_esb_evt_t const *p_event)
{
    switch (p_event->evt_id)
    {
    case NRF_ESB_EVENT_TX_SUCCESS:
        // Toggle tx success LEDs.
        (void)bsp_board_led_on(BSP_BOARD_LED_0);
        (void)bsp_board_led_off(BSP_BOARD_LED_2);

        // this is so important!!! without it would double the frame rate (sending two identical packages, so far dun know why)
        (void)nrf_esb_flush_tx();
        count_success++;

        break;
    case NRF_ESB_EVENT_TX_FAILED:
        (void)bsp_board_led_on(BSP_BOARD_LED_2);
        (void)bsp_board_led_off(BSP_BOARD_LED_0);

        // clear buffer
        (void)nrf_esb_flush_tx();
        count_fail++; // if fail too many times, re-entry the queue

    case NRF_ESB_EVENT_RX_RECEIVED:
        while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
        {
            if (rx_payload.data[1] == rf_channel && rx_payload.pipe == 1)
            {
                nrf_esb_stop_rx();
                rf_channel = rx_payload.data[2];
                nrf_esb_set_rf_channel(rx_payload.data[2]);

                //enter the entry_loop
                nrf_drv_timer_disable(&TIMER_ESB);
                entry_flagger = true;
                //(tbc)
            }
        }
        break;
    }
}

// just make sure the high-frequency clock is running for esb
void clocks_start(void)
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

/**
 * @brief Handler for timer_esb events.
 */
void timer_esb_event_handler(nrf_timer_event_t event_type, void *p_context)
{
    switch (event_type)
    {
    case NRF_TIMER_EVENT_COMPARE0:
        state++;
        count_send++;
        break;

    default:
        // Do nothing.
        break;
    }
}

void timer_esb_entry_event_handler(nrf_timer_event_t event_type, void *p_context){  
    entry_flag = true;
}

void gpio_init(void)
{
    nrf_gpio_range_cfg_output(8, 15);
    bsp_board_init(BSP_INIT_LEDS);
}

// void VDD_nRF_init() {
// //this will work without the softdevice
// //set the VDD_nRF (operate and output voltage) to 1.8V for ICM 20948 to use 
// if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) !=
//         (UICR_REGOUT0_VOUT_1V8 << UICR_REGOUT0_VOUT_Pos))
//     {
//         NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
//         while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

//         NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
//                             (UICR_REGOUT0_VOUT_1V8 << UICR_REGOUT0_VOUT_Pos);  

//         NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
//         while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

//         NVIC_SystemReset();
//     }

// //make sure dcdc have been enabled
//     NRF_POWER->DCDCEN = 1;
//     NRF_POWER->DCDCEN0 = 1;
// }

uint32_t esb_init(void)
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};
    nrf_esb_config_t nrf_esb_config = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay = 180;
    nrf_esb_config.retransmit_count = 0;
    nrf_esb_config.bitrate = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler = nrf_esb_event_handler;
    nrf_esb_config.mode = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack = false;
    nrf_esb_config.tx_mode = NRF_ESB_TXMODE_MANUAL;

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, NRF_ESB_PIPE_COUNT);
    VERIFY_SUCCESS(err_code);

    return err_code;
}

void write_package(void)
{
    nrf_esb_skip_tx();
    tx_payload.noack = false; // (tbc)
    tx_payload.data[1] = beta_id;
    tx_payload.data[2] = 0x80; // 0x80 = 1000 0000, indicate it is from beta, later maybe add rssi to this field
    if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
    {
        tx_payload.data[3]++;
    }
    else
    {
        // nothing so far
        bsp_board_led_off(BSP_BOARD_LED_0);
    }
}

static uint8_t random_vector_generate(uint8_t * p_buff, uint8_t size, int upper, int lower)
{
    uint32_t err_code;
    uint8_t  available;
    uint32_t output;

    nrf_drv_rng_bytes_available(&available);
    uint8_t length = MIN(size, available);

    err_code = nrf_drv_rng_rand(p_buff, length);
    APP_ERROR_CHECK(err_code);

    output = ((* p_buff) % (upper - lower + 1)) + lower;

    return output;
}

/**
 * @brief Configure TIMER_ESB for sending loop
 */
static void timer_esb_init(void){
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG; //using default frequency 16MHz
    ret_code_t err_code = nrf_drv_timer_init(&TIMER_ESB, &timer_cfg, timer_esb_event_handler);
    APP_ERROR_CHECK(err_code);

    uint32_t time_us = timeslot;
    uint32_t time_ticks = nrf_drv_timer_us_to_ticks(&TIMER_ESB, time_us);
    nrf_drv_timer_extended_compare(
        &TIMER_ESB, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}

/**
 * @brief Configure TIMER_ESB_ENTRY for entry loop
 */
static void timer_esb_entry_init(void){
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_1MHz; //use 31250 for ms tick, instead of 16 MHz , seems can save power
    ret_code_t err_code = nrf_drv_timer_init(&TIMER_ESB_ENTRY, &timer_cfg, timer_esb_entry_event_handler);
    APP_ERROR_CHECK(err_code);

    uint32_t time_us = 31600; //~32 fps , hard code, tbc
    uint32_t time_ticks = nrf_drv_timer_us_to_ticks(&TIMER_ESB_ENTRY, time_us);
    nrf_drv_timer_extended_compare(&TIMER_ESB_ENTRY, NRF_TIMER_CC_CHANNEL1, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}

int main(void)
{
    ret_code_t err_code;

    gpio_init();

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    clocks_start();

    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

    //init timers, this part can be put into looping (require timer_uninit)
    timer_esb_init();
    timer_esb_entry_init();

    // init the random number generator
    err_code = nrf_drv_rng_init(NULL);
    APP_ERROR_CHECK(err_code);
    
    uint8_t p_buff[RANDOM_BUFF_SIZE];
    
    while (true)
    {
        int8_t num_of_state = (num_of_beta << 1); //total number of state
        int16_t entry_delay = timeslot * ((num_of_state << 1) - 1); //control the frame rate in the entry loop for lowering the affect towards other beta in sending-loop
        int8_t reset = num_of_state - 1;  //the state to reset back to state 0
        int8_t target_frame_rate = (2500/num_of_state) ;  //min,20 beta => 64Hz ;;  max,8 beta => 167Hz  for stable connection
        int8_t fail_threshold = (2500/num_of_state)/60;  // if package fail >= success + target frame rate/60, try to re-enter the queue
        volatile uint32_t unique_delay = random_vector_generate(p_buff, RANDOM_BUFF_SIZE,200,100);

        // try to enter the queue
        int count_try = 0;
        nrf_drv_timer_enable(&TIMER_ESB_ENTRY);

        //entry-loop, actually the packages would also be transmitted in this loop at ~80Hz (out of queue)
        while (entry_flagger == true)
        {
            if(entry_flag){
            nrf_drv_timer_pause(&TIMER_ESB_ENTRY);
            nrf_drv_timer_clear(&TIMER_ESB_ENTRY);

            entry_flag = false;
            nrf_esb_stop_rx();
            nrf_esb_flush_rx();

            write_package();
            nrf_esb_start_tx();
            count_try++;

            // check the count_fail/success for each 10 frames
            if ((count_try % 5) == 0)
            {
                if (count_fail >= entry_threshold)  //5 fail in 10 transmission == invalid
                {
                    // try another timeslot
                    nrf_delay_us(unique_delay);
                } else if (count_success > count_fail) //valid timeslot , let's go!!
                {
                    if(beta_id_flag == false){
                        beta_id = random_vector_generate(p_buff, 1,255,1); //randomly generate an id for the beta in range[1,255]
                        beta_id_flag = true;
                    }
                    state = 0;
                    nrf_drv_timer_enable(&TIMER_ESB);
                    entry_flagger = false; // indicate that it has entered the queue
                    count_try = 0;
                }
                //reset fail & success counts
                count_fail = 0;
                count_success = 0;
            }

            //if try for 0.5 sec still not in queue, regenerate the delay
            if ((count_try % (target_frame_rate >> 2)) == 0){
                unique_delay = random_vector_generate(p_buff, RANDOM_BUFF_SIZE,200,100);
            }

            //if try for 4 sec still not in queue, just stop for [0.05,0.1] sec
            if ((count_try % ((target_frame_rate >> 1) << 2)) == 0 && count_try > target_frame_rate){
                nrf_delay_us(unique_delay*500);
            }

            nrf_esb_suspend();
            nrf_esb_flush_tx();
            nrf_esb_start_rx();

            bsp_board_led_invert(BSP_BOARD_LED_1);
            nrf_drv_timer_resume(&TIMER_ESB_ENTRY);
        }
            
    }

        //sending_loop (in queue)
        while((nrf_drv_timer_is_enabled(&TIMER_ESB)) && (entry_flagger == false))
        {
            bsp_board_led_off(BSP_BOARD_LED_1);
            if (count_send > (check_now << 4))
                {
                    bsp_board_led_invert(BSP_BOARD_LED_3);
                    if (count_fail >= (count_success + fail_threshold)){
                        nrf_esb_stop_rx();
                        nrf_drv_timer_disable(&TIMER_ESB);
                        entry_flagger = true;
                        continue;
                    }

                    //reset all count
                    count_send = 0;
                    count_success = 0;
                    count_fail = 0;
                }

            // in diff state do~ :
            if(state == reset) {
                state = 0; //reset state
                continue;
            }

            switch (state) {
            case 0: {
                nrf_esb_start_tx();
                break;
            }
            case 1: {
                //stop sending package and clear the tx buffer
                nrf_esb_suspend();
                nrf_esb_flush_tx();
                nrf_esb_start_rx();
                break;
            }
            // default => idle-state or rx-state
            default: {
                break;
            }
          }
            
            //read sensor data and write the package 2 state before sending it
            if(state == (reset - 2)){
                nrf_esb_stop_rx();
                nrf_esb_flush_rx();
                write_package();
                continue;
            }

        }

        bsp_board_leds_off(); //damn......error
        
    }
}