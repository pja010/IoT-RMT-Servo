/* LEDC (LED Controller) basic example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_MIN               (250) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_DUTY_MAX               (950)
#define LEDC_FREQUENCY          (50) // Frequency in Hertz.
#define MAX_ANGLE (180)

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = LEDC_DUTY_MIN,//0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}


static int angle_to_duty(int angle){
    return (int)(LEDC_DUTY_MIN + (float)angle / MAX_ANGLE * (LEDC_DUTY_MAX - LEDC_DUTY_MIN));
}


void app_main(void)
{
    // Set the LEDC peripheral configuration
    example_ledc_init();
    vTaskDelay(10);

    while(1){
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, angle_to_duty(0));
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(100);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, angle_to_duty(90)); //819
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(100);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, angle_to_duty(180)); //819
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(100);
    }

}
