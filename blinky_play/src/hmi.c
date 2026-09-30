#include <stdarg.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/logging/log.h>
#include "envnode.h"

LOG_MODULE_REGISTER(hmi, LOG_LEVEL_INF);

static const struct device *const nx_uart = DEVICE_DT_GET(DT_ALIAS(nextion_uart));

/* ---- Page and component IDs (must match the Nextion build sheet) ---- */
#define PG_DASHBOARD   0
#define PG_SETTINGS    1
#define PG_PROV        2
#define PG_RESET       3

#define TOUCH(pg, comp)  (((pg) << 8) | (comp))
#define BTN_PROV         TOUCH(PG_SETTINGS, 9)
#define BTN_CANCEL       TOUCH(PG_PROV, 6)
#define BTN_CONFIRM      TOUCH(PG_RESET, 4)

/* ---- Shared state ---- */
static atomic_t cur_page = ATOMIC_INIT(PG_DASHBOARD);
K_MUTEX_DEFINE(nx_tx_lock);                 /* two threads send commands */
K_MSGQ_DEFINE(nx_rx_q, 1, 128, 1);          /* bytes from ISR to rx thread */

ZBUS_CHAN_DECLARE(chan_reading);
ZBUS_SUBSCRIBER_DEFINE(hmi_sub, 4);
ZBUS_CHAN_ADD_OBS(chan_reading, hmi_sub, 3);

/* ------------------------------------------------------------------ */
/* TX                                                                  */
/* ------------------------------------------------------------------ */
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

	k_mutex_lock(&nx_tx_lock, K_FOREVER);
	for (int i = 0; i < n; i++) {
		uart_poll_out(nx_uart, buf[i]);
	}
	for (int i = 0; i < 3; i++) {
		uart_poll_out(nx_uart, 0xFF);
	}
	k_mutex_unlock(&nx_tx_lock);
}

/* ------------------------------------------------------------------ */
/* RX: ISR pushes bytes, a thread assembles and handles frames          */
/* ------------------------------------------------------------------ */
static void nx_isr(const struct device *dev, void *user_data)
{
	uint8_t c;

	uart_irq_update(dev);
	while (uart_irq_rx_ready(dev)) {
		if (uart_fifo_read(dev, &c, 1) == 1) {
			(void)k_msgq_put(&nx_rx_q, &c, K_NO_WAIT);   /* drop if full */
		}
	}
}

static void on_touch(uint8_t page, uint8_t comp)
{
	switch (TOUCH(page, comp)) {
	case BTN_PROV:
		LOG_INF("provisioning requested");
		/* Later: send CMD_START_PROV to the ESP32 and wait for its ACK */
		nx_send("page prov");
		nx_send("tStatus.txt=\"Radio not connected yet\"");
		break;
	case BTN_CANCEL:
		LOG_INF("provisioning cancelled");
		nx_send("page settings");
		break;
	case BTN_CONFIRM:
		LOG_WRN("factory reset requested (not implemented yet)");
		nx_send("page dashboard");
		break;
	default:
		/* Navigation buttons: the display already changed page itself */
		break;
	}
}

static void handle_frame(const uint8_t *f, size_t n)
{
	if (n == 4 && f[0] == 0x65) {                 /* touch event */
		uint8_t page = f[1], comp = f[2], pressed = f[3];

		LOG_INF("touch page %u comp %u %s", page, comp,
			pressed ? "press" : "release");
		if (!pressed) {
			on_touch(page, comp);
			nx_send("sendme");                    /* ask: which page now? */
		}
	} else if (n == 2 && f[0] == 0x66) {          /* reply to sendme */
		atomic_set(&cur_page, f[1]);
		LOG_INF("current page %u", f[1]);
	} else if (n == 1 && f[0] == 0x88) {
		LOG_INF("display ready");
	} else if (n == 1) {
		LOG_WRN("display error code 0x%02x", f[0]);
	} else {
		LOG_HEXDUMP_DBG(f, n, "unhandled frame");
	}
}

static void hmi_rx_thread(void *p1, void *p2, void *p3)
{
	uint8_t frame[16];
	size_t len = 0;
	int ff = 0;
	uint8_t c;

	while (k_msgq_get(&nx_rx_q, &c, K_FOREVER) == 0) {
		if (len >= sizeof(frame)) {               /* garbage: resync */
			LOG_WRN("rx frame overflow, resync");
			len = 0;
			ff = 0;
		}
		frame[len++] = c;
		ff = (c == 0xFF) ? ff + 1 : 0;

		if (ff == 3) {                            /* FF FF FF = end */
			handle_frame(frame, len - 3);
			len = 0;
			ff = 0;
		}
	}
}

/* ------------------------------------------------------------------ */
/* Display updates from sensor readings                                */
/* ------------------------------------------------------------------ */
static void hmi_thread(void *p1, void *p2, void *p3)
{
	const struct zbus_channel *chan;
	struct env_reading r;

	if (!device_is_ready(nx_uart)) {
		LOG_ERR("nextion uart not ready");
		return;
	}

	uart_irq_callback_user_data_set(nx_uart, nx_isr, NULL);
	uart_irq_rx_enable(nx_uart);

	k_msleep(500);                                /* display boot time */
	nx_send("bkcmd=2");                           /* report errors only */
	nx_send("page dashboard");
	atomic_set(&cur_page, PG_DASHBOARD);
	nx_send("tSub.txt=\"Room temp (est.)\"");
	LOG_INF("nextion ready");

	while (zbus_sub_wait(&hmi_sub, &chan, K_FOREVER) == 0) {
		if (chan != &chan_reading ||
		    zbus_chan_read(&chan_reading, &r, K_MSEC(50)) != 0) {
			continue;
		}
		if (atomic_get(&cur_page) != PG_DASHBOARD) {
			continue;                             /* fields not on screen */
		}

		nx_send("tTemp.txt=\"%d.%d\"", r.temp_cdeg / 100,
			abs(r.temp_cdeg % 100) / 10);

		uint32_t s = r.uptime_ms / 1000;
		nx_send("tUptime.txt=\"up %u:%02u:%02u \"", s / 3600, (s / 60) % 60, s % 60);
	}
}

K_THREAD_DEFINE(hmi,    1024, hmi_thread,    NULL, NULL, NULL, 8, 0, 0);
K_THREAD_DEFINE(hmi_rx, 1024, hmi_rx_thread, NULL, NULL, NULL, 6, 0, 0);