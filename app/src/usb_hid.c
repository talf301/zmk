/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <device.h>
#include <init.h>

#include <usb/usb_device.h>
#include <usb/class/usb_hid.h>

#include <zmk/usb.h>
#include <zmk/hid.h>
#include <zmk/keymap.h>
#include <zmk/event_manager.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static const struct device *hid_dev;

// Define workqueue for sending USB data.
K_THREAD_STACK_DEFINE(usb_hid_wq_stack, CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE);
struct k_work_q usb_hid_wq;

struct usb_hid_msg {
	uint8_t data[255];
	uint8_t len;
};

// Keep track of the number of consecutive HID writes failures so messages can
// be dropped if they continuously fail to send.
static int usb_hid_failed;

K_MSGQ_DEFINE(usb_hid_msgq, sizeof(struct usb_hid_msg), 8, 1);

void usb_hid_work_handler(struct k_work *work) {
	struct usb_hid_msg msg;
	while (k_msgq_peek(&usb_hid_msgq, &msg) == 0) {
		// Attempt to write HID message. If it fails, retry with up to a total
		// of three attempts. Reattempt after 10ms or when the USB HID interrupt
		// IN endpoint is ready, whichever comes first.
		int err = hid_int_ep_write(hid_dev, msg.data, msg.len, NULL);
		if (err) {
			usb_hid_failed++;
			if (usb_hid_failed < 3) {
				k_work_reschedule_for_queue(&usb_hid_wq, k_work_delayable_from_work(work), K_MSEC(10));
				return;
			} else {
				LOG_ERR("dropped HID message due to %d consecutive failures", usb_hid_failed);
			}
		}

		// Remove message from message queue and reset failure count.
		k_msgq_get(&usb_hid_msgq, &msg, K_NO_WAIT);
		usb_hid_failed = 0;
	}
}

K_WORK_DELAYABLE_DEFINE(usb_hid_work, usb_hid_work_handler);

static void in_ready_cb(const struct device *dev) {
	k_work_reschedule_for_queue(&usb_hid_wq, &usb_hid_work, K_NO_WAIT);
}

static const struct hid_ops ops = {
    .int_in_ready = in_ready_cb,
};

int zmk_usb_hid_send_report(const uint8_t *report, size_t len) {
    switch (zmk_usb_get_status()) {
    case USB_DC_SUSPEND:
        return usb_wakeup_request();
    case USB_DC_ERROR:
    case USB_DC_RESET:
    case USB_DC_DISCONNECTED:
    case USB_DC_UNKNOWN:
        return -ENODEV;
    default:
		{
			struct usb_hid_msg msg = {.len = len};
			memcpy(&msg.data, report, len);
			if (k_msgq_put(&usb_hid_msgq, &msg, K_NO_WAIT)) {
				LOG_ERR("failed to add HID message to queue");
			} else {
				// Add to queue. This uses "schedule" rather than "reschedule"
				// to keep the existing delay if the work item is already in the
				// queue such as following a USB HID write failure.
				k_work_schedule_for_queue(&usb_hid_wq, &usb_hid_work, K_NO_WAIT);
			}
			return 0;
		}
    }
}

uint8_t zmk_slicemk_data[RAWHID_TX_SIZE+1] = {0x0a};

void zmk_slicemk_set_layer(uint8_t layer) {
	zmk_slicemk_data[1] = layer;
	zmk_usb_hid_send_report(zmk_slicemk_data, sizeof(zmk_slicemk_data));
}

static void zmk_slicemk_set_key_bit(uint8_t *data, int i, bool b) {
    int byte = i / 8;
    int bit = i % 8;
    if (b) {
        data[byte] |= BIT(bit);
    } else {
        data[byte] &= ~BIT(bit);
    }
}

void zmk_slicemk_set_key_state(int key, bool pressed) {
	zmk_slicemk_set_key_bit(&zmk_slicemk_data[2], key, pressed);
	zmk_usb_hid_send_report(zmk_slicemk_data, sizeof(zmk_slicemk_data));
}

static int zmk_usb_hid_init(const struct device *_arg) {
    hid_dev = device_get_binding("HID_0");
    if (hid_dev == NULL) {
        LOG_ERR("Unable to locate HID device");
        return -EINVAL;
    }

    usb_hid_register_device(hid_dev, zmk_hid_report_desc, sizeof(zmk_hid_report_desc), &ops);
    usb_hid_init(hid_dev);

	k_work_queue_start(&usb_hid_wq, usb_hid_wq_stack, K_THREAD_STACK_SIZEOF(usb_hid_wq_stack), 5, NULL);

    return 0;
}

SYS_INIT(zmk_usb_hid_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
