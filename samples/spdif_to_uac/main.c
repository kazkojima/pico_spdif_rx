/*------------------------------------------------------/
/ Copyright (c) 2023, Kaz Kojima
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"

#include "usb_microphone.h"

#include "spdif_rx.h"

#define PIN_PICO_SPDIF_RX_DATA 15
#define DAC_ZERO 1

static bool decode_flg = false;
volatile static bool uac_setup_flg = false;
volatile static bool uac_cancel_flg = false;
typedef struct { uint8_t b[3]; } s24_t;
s24_t sample_buffer[SAMPLE_BUFFER_SIZE];

inline void set_s24(s24_t *s, int32_t x)
{
  s->b[0] = x >> 8;
  s->b[1] = x >> 16;
  s->b[2] = x >> 24;
}

inline int32_t get_s24(s24_t s)
{ return (int32_t)(((s.b[2] << 16) + (s.b[1] << 8) + s.b[0]) << 8); }

void spdif_rx_read(s24_t *samples, size_t sample_count)
{
  static bool mute_flag = true;

  uint32_t fifo_count = spdif_rx_get_fifo_count();
  if (spdif_rx_get_state() == SPDIF_RX_STATE_STABLE) {
    if (mute_flag && fifo_count >= sample_count) {
      mute_flag = false;
    }
  } else {
    mute_flag = true;
  }

  //printf("mute %d fifo %ld sc %d\n", mute_flag, fifo_count, sample_count);

  if (mute_flag) {
    for (int i = 0; i < sample_count / 2; i++) {
      set_s24(&samples[2*i+0], DAC_ZERO);
      set_s24(&samples[2*i+1], DAC_ZERO);
    }
  } else {
    uint32_t total_count = sample_count;
    int i = 0;
    uint32_t read_count = 0;
    uint32_t* buff;
    while (read_count < total_count) {
      uint32_t get_count = spdif_rx_read_fifo(&buff, total_count - read_count);
      for (int j = 0; j < get_count / 2; j++) {
	set_s24(&samples[2*i+0], (int32_t) ((buff[j*2+0] & 0x0ffffff0) << 4));
	set_s24(&samples[2*i+1], (int32_t) ((buff[j*2+1] & 0x0ffffff0) << 4));
	i++;
      }
      read_count += get_count;
    }
  }
}

void measure_freqs(void) {
  uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
  uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
  uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
  uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
  uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
  uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
  uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
  uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

  printf("pll_sys  = %dkHz\n", f_pll_sys);
  printf("pll_usb  = %dkHz\n", f_pll_usb);
  printf("rosc     = %dkHz\n", f_rosc);
  printf("clk_sys  = %dkHz\n", f_clk_sys);
  printf("clk_peri = %dkHz\n", f_clk_peri);
  printf("clk_usb  = %dkHz\n", f_clk_usb);
  printf("clk_adc  = %dkHz\n", f_clk_adc);
  printf("clk_rtc  = %dkHz\n", f_clk_rtc);

  // Can't measure clk_ref / xosc as it is the ref
}


void on_stable_func(spdif_rx_samp_freq_t samp_freq)
{
  // callback function should be returned as quick as possible
  uac_setup_flg = true;
  uac_cancel_flg = false;
}

void on_lost_stable_func()
{
  // callback function should be returned as quick as possible
  uac_cancel_flg = true;
}

void on_usb_microphone_tx_ready()
{
  // Callback from TinyUSB library when all data is ready
  // to be transmitted.
  //
  // Read new samples into local buffer.
  spdif_rx_read(sample_buffer, SAMPLE_BUFFER_SIZE);
  // Write local buffer to the USB microphone
  usb_microphone_write(sample_buffer, sizeof(sample_buffer));
}

int main()
{
  stdio_init_all();

  // initialize the USB microphone interface
  usb_microphone_init();
  usb_microphone_set_tx_ready_handler(on_usb_microphone_tx_ready);

  //measure_freqs();

  spdif_rx_config_t config = {
    .data_pin = PIN_PICO_SPDIF_RX_DATA,
    .pio_sm = 0,
    .dma_channel0 = 2,
    .dma_channel1 = 3,
    .alarm = 0,
    .flags = SPDIF_RX_FLAGS_ALL
  };

#if 1
  spdif_rx_start(&config);
  spdif_rx_set_callback_on_stable(on_stable_func);
  spdif_rx_set_callback_on_lost_stable(on_lost_stable_func);
#endif

  while (true) {
    usb_microphone_task();
  }

  return 0;
}
