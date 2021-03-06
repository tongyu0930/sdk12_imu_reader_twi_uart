/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "app_mpu.h"


#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */


void uart_error_handle(app_uart_evt_t * p_event)
{
	switch (p_event->evt_type)
	    {
	        case APP_UART_COMMUNICATION_ERROR:
	            APP_ERROR_HANDLER(p_event->data.error_communication);
	            break;

	        case APP_UART_FIFO_ERROR:
	            APP_ERROR_HANDLER(p_event->data.error_code);
	            break;

	        default:
	            break;
	    }
}


/**@brief Function for initializing the UART.
 */
static void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
          {
              RX_PIN_NUMBER,
              TX_PIN_NUMBER,
              RTS_PIN_NUMBER,
              CTS_PIN_NUMBER,
              APP_UART_FLOW_CONTROL_ENABLED,
              false,
              UART_BAUDRATE_BAUDRATE_Baud115200
          };

        APP_UART_FIFO_INIT(&comm_params,
                             UART_RX_BUF_SIZE,
                             UART_TX_BUF_SIZE,
                             uart_error_handle,
                             APP_IRQ_PRIORITY_LOWEST,
                             err_code);

        APP_ERROR_CHECK(err_code);
}


void mpu_setup(void)
{
    ret_code_t ret_code;
    ret_code = mpu_init();
    APP_ERROR_CHECK(ret_code); // Check for errors in return value

    // Setup and configure the MPU with intial values
    mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); // Load default values
    p_mpu_config.smplrt_div = 19;   // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G; // Set accelerometer full scale range to 2G
    ret_code = mpu_config(&p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(ret_code); // Check for errors in return value


    mpu_magn_config_t p_magnetometer_conf;
    ret_code = mpu_magnetometer_init(&p_magnetometer_conf);
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code;
    bsp_board_leds_init();
    uart_init();
    printf("\033[2J\033[;HMPU simple example. Compiled @ %s\r\n", __TIME__);
    mpu_setup();
    accel_values_t acc_values;
    gyro_values_t gyro_values;
    magn_values_t  p_magnetometer_values;
    int sample_number = 0;

    while (true)
        {
            err_code = mpu_read_accel(&acc_values); // 没有彻底弄懂register读取原理，2进制怎么就变成十进制了？
            APP_ERROR_CHECK(err_code);
            err_code = mpu_read_gyro(&gyro_values);
            APP_ERROR_CHECK(err_code);
            err_code = mpu_read_magnetometer(&p_magnetometer_values, NULL); // 还不能够读取数据
            // %06d 意思为小于6位的整型，不足6位前面补零。2的16次方＝65536（5位），6位integer最开头那一位为正负号。
            printf("\033[3;1HSample # %d\r\naX: %06d\r\naY: %06d\r\naZ: %06d\r\ngX: %06d\r\ngY: %06d\r\ngZ: %06d\r\nmX: %06d\r\nmY: %06d\r\nmZ: %06d\r\n",\
            		++sample_number, acc_values.x, acc_values.y, acc_values.z, \
					gyro_values.x, gyro_values.y, gyro_values.z, \
					p_magnetometer_values.x, p_magnetometer_values.y, p_magnetometer_values.z);

            if(p_magnetometer_values.x != 0){nrf_gpio_pin_toggle(LED_1);}
            nrf_delay_ms(500);
        }
}


