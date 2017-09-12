/* Timer group-hardware timer example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

*/

#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/adc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/gpio.h"

#define TIMER_INTR_SEL		TIMER_INTR_LEVEL  	// Timer level interrupt
#define TIMER_GROUP    		TIMER_GROUP_0     	// Test on timer group 0
#define TIMER_DIVIDER  		2               	// Hardware timer clock divider
#define TIMER_INTERVAL0_SEC	(0.001)  			// test interval for timer 0 [1000usec/1.0msec(0.001),100usec/0.1msec(0.0001),8.5usec/0.0085msec(0.00001)]
#define	TIMER_CPU_CLOCK		160000000L			// CPU Clock 160Mhz
#define TIMER_SCALE    		(TIMER_BASE_CLK / TIMER_DIVIDER)  // used to calculate counter value
#define TIMER_SCALE160 		(TIMER_CPU_CLOCK / TIMER_DIVIDER) // used to calculate counter value

xSemaphoreHandle semaphore;

static bool toggle21 = 0x01;
static bool toggle04 = 0x01;

//
// Interrput Timer Group0-timer0
//
void interrupt_timer_group0_isr(void *para)
{
	toggle04 ^= 0x01;
    gpio_set_level(GPIO_NUM_4, toggle04);

    TIMERG0.hw_timer[0].update 	= 1;
    TIMERG0.int_clr_timers.t0 	= 1;

	xSemaphoreGiveFromISR(semaphore, NULL);

	TIMERG0.hw_timer[0].config.alarm_en = 1;
}

//
// Initialize Timer Group0-timer0
//
static void init_tg0_timer0()
{
	int timer_group 	= TIMER_GROUP_0;
    int timer_idx 		= TIMER_0;

	// Configure timer
	timer_config_t config;
    config.alarm_en 	= 1;
    config.auto_reload 	= 1;
    config.counter_dir 	= TIMER_COUNT_UP;
    config.divider 		= TIMER_DIVIDER;
    config.intr_type 	= TIMER_INTR_SEL;
    config.counter_en 	= TIMER_PAUSE;
	timer_init(timer_group, timer_idx, &config);

    timer_pause(timer_group, timer_idx);
    timer_set_alarm_value(timer_group, timer_idx, TIMER_INTERVAL0_SEC * TIMER_SCALE);

    timer_enable_intr(timer_group, timer_idx);
    timer_isr_register(timer_group, timer_idx, interrupt_timer_group0_isr, (void*) timer_idx, ESP_INTR_FLAG_LOWMED, NULL);

    timer_start(timer_group, timer_idx);
}

//
// Initialize GPIO
//
static void init_gpio_config()
{
    gpio_config_t gpio_config_struct;
    gpio_config_struct.pin_bit_mask  = 0x00000000;
    gpio_config_struct.pin_bit_mask |= 0x00200010;
    gpio_config_struct.mode = GPIO_MODE_OUTPUT;
    gpio_config_struct.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config_struct.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config_struct.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio_config_struct);
}

//
// Timer0 Debug Routine
//
void timer0_main_ctrl()
{
    while(true){
        xSemaphoreTake(semaphore, portMAX_DELAY);
        toggle21 ^= 0x01;
        gpio_set_level(GPIO_NUM_21, toggle21);
    }
}

//
// Main Routine
//
void app_main()
{
    init_gpio_config();
    vSemaphoreCreateBinary(semaphore);
    xTaskCreate(timer0_main_ctrl, "timer0_main_ctrl", 2048, NULL, 5, NULL);
    init_tg0_timer0();
}
