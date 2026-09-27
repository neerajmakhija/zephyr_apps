/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define SW0_NODE  DT_ALIAS(sw0)

#define STACK_SIZE 		512
#define TASK_PRIORITY	5

K_SEM_DEFINE(button_sem, 0, 1);
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static struct gpio_callback button_cb_data;

static void green_led_blink_thread(void *p1 , void *p2 , void *p3)
{
	const struct gpio_dt_spec *led = p1;
	int 				 period_ms = POINTER_TO_INT(p2);
	ARG_UNUSED(p3);

	if (!gpio_is_ready_dt(led)) {
		return;
	}
	gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
	while (1) 
	{
		gpio_pin_toggle_dt(led);
		k_msleep(period_ms);
	}
}

static void blue_led_blink_thread(void *p1 , void *p2 , void *p3)
{
	const struct gpio_dt_spec *led = p1;
	int 				 period_ms = POINTER_TO_INT(p2);
	ARG_UNUSED(p3);
	int paused = 0;

	if (!gpio_is_ready_dt(led)) {
		return;
	}
	gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
	while (1) 
	{
		if(k_sem_take(&button_sem, K_MSEC(period_ms)) != 0)
		{
			if(!paused)
				gpio_pin_toggle_dt(led);
		}
		else {
			paused = !paused;
			gpio_pin_set_dt(led, 0);
		}

	}
}

static void debounce_handler(struct k_work *work)
{
    if (gpio_pin_get_dt(&button) == 1) {     /* still pressed after settling */
        k_sem_give(&button_sem);
    }
}

static K_WORK_DELAYABLE_DEFINE(debounce_work, debounce_handler);

static void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	k_work_reschedule(&debounce_work, K_MSEC(30));   /* each bounce restarts the 30 ms */
}

K_THREAD_DEFINE( blink_led_green,
				 STACK_SIZE,
				 green_led_blink_thread,
				 &led_green,
				 INT_TO_POINTER(500),
				 NULL,
				 TASK_PRIORITY,
				 0,
				 0);

K_THREAD_DEFINE( blink_led_blue,
				 STACK_SIZE,
				 blue_led_blink_thread,
				 &led_blue,
				 INT_TO_POINTER(200),
				 NULL,
				 TASK_PRIORITY,
				 0,
				 0);
int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	return 0;
}
