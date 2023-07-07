/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#ifndef _USB_MICROPHONE_H_
#define _USB_MICROPHONE_H_

#include "tusb.h"
#include "usbd_mic2ch.h"

#define BYTES_PER_SAMPLE (CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)

#ifndef SAMPLE_BUFFER_SIZE
#define SAMPLE_BUFFER_SIZE (CFG_TUD_AUDIO_EP_SZ_IN / CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX)
#endif

typedef void (*usb_microphone_tx_ready_handler_t)(void);

void usb_microphone_init(uint32_t sample_rate);
void usb_microphone_set_tx_ready_handler(usb_microphone_tx_ready_handler_t handler);
void usb_microphone_task();
uint16_t usb_microphone_write(const void * data, uint16_t len);

#endif
