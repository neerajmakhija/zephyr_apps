/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(blinky_play, LOG_LEVEL_DBG);

/* ------------------------------------------------------------------ */
/* Devicetree                                                          */
/* ------------------------------------------------------------------ */
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define SW0_NODE  DT_ALIAS(sw0)

static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec button    = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static struct gpio_callback button_cb_data;

static const struct device *const temp_dev = DEVICE_DT_GET(DT_ALIAS(die_temp0));
static const struct device *const vref_dev = DEVICE_DT_GET(DT_ALIAS(volt_sensor0));

/* ------------------------------------------------------------------ */
/* Thread config                                                       */
/* ------------------------------------------------------------------ */
#define STACK_SIZE          512
#define TASK_PRIORITY       5

#define SENSOR_STACK_SIZE   1024
#define SENSOR_PRIORITY     7        /* lower priority than the LEDs */
#define SENSOR_PERIOD_MS    1000

K_SEM_DEFINE(button_sem, 0, 1);

/* ------------------------------------------------------------------ */
/* zbus: reading message, channel, listener                            */
/* ------------------------------------------------------------------ */
struct env_reading {
	uint32_t uptime_ms;
	int32_t  temp_cdeg;      /* corrected die temp, 0.01 °C */
	int32_t  vdda_mv;
};

static void log_listener_cb(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(log_listener, log_listener_cb);

ZBUS_CHAN_DEFINE(chan_reading,              /* name                   */
		 struct env_reading,        /* message type           */
		 NULL,                      /* validator              */
		 NULL,                      /* user data              */
		 ZBUS_OBSERVERS(log_listener),
		 ZBUS_MSG_INIT(0));         /* initial value: all 0   */

/* Runs synchronously inside the publisher's thread (the sensor thread) */
static void log_listener_cb(const struct zbus_channel *chan)
{
	const struct env_reading *r = zbus_chan_const_msg(chan);

	LOG_INF("t=%u ms  temp %d.%02d C  VDDA %d mV",
		r->uptime_ms, r->temp_cdeg / 100, abs(r->temp_cdeg % 100),
		r->vdda_mv);
}

/* ------------------------------------------------------------------ */
/* Die temperature correction for VDDA != 3.3 V                        */
/* ------------------------------------------------------------------ */
#define TS_CAL1 (*(volatile const uint16_t *)0x1FFF7A2C)   /* 30 °C  @ 3.3 V */
#define TS_CAL2 (*(volatile const uint16_t *)0x1FFF7A2E)   /* 110 °C @ 3.3 V */

static int32_t die_temp_correct(int32_t t_rep_mdeg, int32_t vdda_mv)
{
	int64_t cal1 = TS_CAL1;
	int64_t span = (int64_t)TS_CAL2 - cal1;             /* counts per 80 °C */

	int64_t raw_x1000      = cal1 * 1000 + (int64_t)(t_rep_mdeg - 30000) * span / 80;
	int64_t raw_corr_x1000 = raw_x1000 * vdda_mv / 3300;

	return 30000 + (int32_t)((raw_corr_x1000 - cal1 * 1000) * 80 / span);
}

static int read_sensor_mv_mdeg(const struct device *dev, enum sensor_channel ch,
			       int32_t *out_milli)
{
	struct sensor_value v;
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, ch, &v);
	}
	if (rc == 0) {
		*out_milli = v.val1 * 1000 + v.val2 / 1000;
	}
	return rc;
}

/* ------------------------------------------------------------------ */
/* Sensor thread                                                       */
/* ------------------------------------------------------------------ */
static void sensor_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	if (!device_is_ready(temp_dev) || !device_is_ready(vref_dev)) {
		LOG_ERR("temp or vref sensor not ready");
		return;
	}

	int64_t next = k_uptime_get();

	while (1) {
		int32_t t_rep_mdeg, vdda_mv;

		if (read_sensor_mv_mdeg(temp_dev, SENSOR_CHAN_DIE_TEMP, &t_rep_mdeg) == 0 &&
		    read_sensor_mv_mdeg(vref_dev, SENSOR_CHAN_VOLTAGE, &vdda_mv) == 0) {

			struct env_reading msg = {
				.uptime_ms = (uint32_t)k_uptime_get(),
				.temp_cdeg = die_temp_correct(t_rep_mdeg, vdda_mv) / 10 + CONFIG_ENVNODE_TEMP_OFFSET_CDEG,
				.vdda_mv   = vdda_mv,
			};

			LOG_DBG("raw temp %d m°C", t_rep_mdeg);

			if (zbus_chan_pub(&chan_reading, &msg, K_MSEC(100)) != 0) {
				LOG_WRN("zbus publish failed");
			}
		} else {
			LOG_WRN("sensor read failed");
		}

		/* Absolute wake-up time: the period does not drift even though
		 * reading + publishing takes time each loop. */
		next += SENSOR_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}
}

K_THREAD_DEFINE(sensor_svc, SENSOR_STACK_SIZE, sensor_thread,
		NULL, NULL, NULL, SENSOR_PRIORITY, 0, 0);

/* ------------------------------------------------------------------ */
/* LEDs and button (unchanged logic)                                   */
/* ------------------------------------------------------------------ */
static void green_led_blink_thread(void *p1, void *p2, void *p3)
{
	const struct gpio_dt_spec *led = p1;
	int period_ms = POINTER_TO_INT(p2);
	ARG_UNUSED(p3);

	if (!gpio_is_ready_dt(led)) {
		return;
	}
	gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
	while (1) {
		gpio_pin_toggle_dt(led);
		k_msleep(period_ms);
	}
}

static void blue_led_blink_thread(void *p1, void *p2, void *p3)
{
	const struct gpio_dt_spec *led = p1;
	int period_ms = POINTER_TO_INT(p2);
	ARG_UNUSED(p3);
	int paused = 0;

	if (!gpio_is_ready_dt(led)) {
		return;
	}
	gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
	while (1) {
		if (k_sem_take(&button_sem, K_MSEC(period_ms)) != 0) {
			if (!paused) {
				gpio_pin_toggle_dt(led);
			}
		} else {
			paused = !paused;
			LOG_INF("button short press, paused=%d", paused);
			gpio_pin_set_dt(led, 0);
		}
	}
}

static void debounce_handler(struct k_work *work)
{
	if (gpio_pin_get_dt(&button) == 1) {
		LOG_DBG("debounce fired");
		k_sem_give(&button_sem);
	}
}
static K_WORK_DELAYABLE_DEFINE(debounce_work, debounce_handler);

static void button_pressed(const struct device *dev, struct gpio_callback *cb,
			   uint32_t pins)
{
	k_work_reschedule(&debounce_work, K_MSEC(30));
}

K_THREAD_DEFINE(blink_led_green, STACK_SIZE, green_led_blink_thread,
		&led_green, INT_TO_POINTER(500), NULL, TASK_PRIORITY, 0, 0);

K_THREAD_DEFINE(blink_led_blue, STACK_SIZE, blue_led_blink_thread,
		&led_blue, INT_TO_POINTER(200), NULL, TASK_PRIORITY, 0, 0);


static int cmd_env_show(const struct shell *sh, size_t argc, char **argv)
{
	struct env_reading r;

	/* Copy the latest message out of the channel */
	int rc = zbus_chan_read(&chan_reading, &r, K_MSEC(100));
	if (rc) {
		shell_error(sh, "read failed: %d", rc);
		return rc;
	}
	shell_print(sh, "t=%u ms  temp %d.%02d C  VDDA %d mV",
		r.uptime_ms, r.temp_cdeg / 100, abs(r.temp_cdeg % 100),
		r.vdda_mv);
	return 0;
}
SHELL_STATIC_SUBCMD_SET_CREATE(env_cmds,
	SHELL_CMD(show, NULL, "Show latest sensor reading", cmd_env_show),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(env, &env_cmds, "Environment sensor commands", NULL);

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("button device %s not ready", button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("configure button failed: %d", ret);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("configure button interrupt failed: %d", ret);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	ret = gpio_add_callback(button.port, &button_cb_data);
	if (ret != 0) {
		LOG_ERR("add button callback failed: %d", ret);
	}

	return 0;
}