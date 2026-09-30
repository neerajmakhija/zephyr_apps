#include <stdarg.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/logging/log.h>
#include "envnode.h"

LOG_MODULE_REGISTER(hmi, LOG_LEVEL_INF);

static const struct device *const nx_uart = DEVICE_DT_GET(DT_ALIAS(nextion_uart));

/* The channel is defined in main.c; here we only declare it */
ZBUS_CHAN_DECLARE(chan_reading);

/* A subscriber has its own queue and runs in its own thread, so slow
 * UART writes never block the sensor thread that publishes. */
ZBUS_SUBSCRIBER_DEFINE(hmi_sub, 4);
ZBUS_CHAN_ADD_OBS(chan_reading, hmi_sub, 3);   /* attach without touching main.c */

/* Send one Nextion command: printf-style text + FF FF FF terminator */
static void nx_send(const char *fmt, ...)
{
	char buf[64];
	va_list ap;

	va_start(ap, fmt);
	int n = vsnprintk(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	if (n < 0 || n >= (int)sizeof(buf)) {
		LOG_WRN("nextion command too long");
		return;
	}
	for (int i = 0; i < n; i++) {
		uart_poll_out(nx_uart, buf[i]);
	}
	for (int i = 0; i < 3; i++) {
		uart_poll_out(nx_uart, 0xFF);
	}
}

static void hmi_thread(void *p1, void *p2, void *p3)
{
	const struct zbus_channel *chan;
	struct env_reading r;

	if (!device_is_ready(nx_uart)) {
		LOG_ERR("nextion uart not ready");
		return;
	}

	k_msleep(500);            /* give the display time to boot */
	nx_send("page 0");
	LOG_INF("nextion ready");

	while (zbus_sub_wait(&hmi_sub, &chan, K_FOREVER) == 0) {
		if (chan != &chan_reading ||
		    zbus_chan_read(&chan_reading, &r, K_MSEC(50)) != 0) {
			continue;
		}

		nx_send("tTemp.txt=\"%d.%d\"", r.temp_cdeg / 100,
			abs(r.temp_cdeg % 100) / 10);

		uint32_t s = r.uptime_ms / 1000;
		nx_send("tUptime.txt=\"up %u:%02u:%02u \"", s / 3600, (s / 60) % 60, s % 60);
	}
}

K_THREAD_DEFINE(hmi, 1024, hmi_thread, NULL, NULL, NULL, 8, 0, 0);