/*------------------------------------------------------/
/ Copyright (c) 2023, Kaz Kojima
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "hardware/clocks.h"
#include "hardware/pio.h"

#include "spdif_rx.h"

#define UDP_TARGET_ADDR "10.253.253.8"
#define UDP_PORT 5991

#define PIN_PICO_SPDIF_RX_DATA 15
#define DAC_ZERO 1

typedef struct { uint8_t b[4]; } se32_t;
se32_t sample_buffer[192*2];
int n_samples = 48*2; // Dummy
uint32_t sample_rate;
volatile static bool udp_setup_flg = false;
volatile static bool udp_cancel_flg = false;

const int SOFT_START_COUNT = 8*1000; // 8s

const uint LED_PIN = CYW43_WL_GPIO_LED_PIN;

inline void set_se32(se32_t *s, uint32_t x)
{
  s->b[0] = x >> 0;
  s->b[1] = x >> 8;
  s->b[2] = x >> 16;
  s->b[3] = x >> 24;
}

void spdif_rx_read(se32_t *samples, size_t sample_count)
{
  static bool mute_flag = true;
  static int soft_start;

  uint32_t fifo_count = spdif_rx_get_fifo_count();
  if (spdif_rx_get_state() == SPDIF_RX_STATE_STABLE) {
    if (mute_flag && fifo_count >= sample_count) {
      mute_flag = false;
      soft_start = SOFT_START_COUNT;
    }
  } else {
    mute_flag = true;
  }

  //printf("mute %d fifo %ld sc %d\n", mute_flag, fifo_count, sample_count);

  if (mute_flag || soft_start > 0) {
    for (int i = 0; i < sample_count / 2; i++) {
      set_se32(&samples[2*i+0], DAC_ZERO);
      set_se32(&samples[2*i+1], DAC_ZERO);
    }
    if (soft_start > 0) {
      soft_start--;
      // to avoid fifo overflow
      uint32_t* trash;
      spdif_rx_read_fifo(&trash, fifo_count);
    }
  } else {
    uint32_t total_count = sample_count;
    int i = 0;
    uint32_t read_count = 0;
    uint32_t* buff;
    while (read_count < total_count) {
      uint32_t get_count = spdif_rx_read_fifo(&buff, total_count - read_count);
      for (int j = 0; j < get_count / 2; j++) {
	set_se32(&samples[2*i+0], ((buff[j*2+0] & 0x0ffffff0) << 4));
	set_se32(&samples[2*i+1], ((buff[j*2+1] & 0x0ffffff0) << 4));
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
  udp_setup_flg = true;
  udp_cancel_flg = false;
  n_samples = (samp_freq/1000) * 2;
  sample_rate = samp_freq;
}

void on_lost_stable_func()
{
  // callback function should be returned as quick as possible
  udp_cancel_flg = true;
}

int main()
{
  stdio_init_all();

  if (cyw43_arch_init()) {
    printf("failed to initialise\n");
    return 1;
  }

  cyw43_arch_enable_sta_mode();

  printf("Connecting to Wi-Fi...\n");
  if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
    printf("failed to connect.\n");
    return 1;
  } else {
    printf("Connected.\n");
  }

  //measure_freqs();

  spdif_rx_config_t config = {
    .data_pin = PIN_PICO_SPDIF_RX_DATA,
    .pio_sm = 0,
    .dma_channel0 = 2,
    .dma_channel1 = 3,
    .alarm = 0,
    .flags = SPDIF_RX_FLAGS_ALL
  };

  spdif_rx_start(&config);
  spdif_rx_set_callback_on_stable(on_stable_func);
  spdif_rx_set_callback_on_lost_stable(on_lost_stable_func);

  while (!udp_setup_flg) {
    sleep_ms(10);
  }

  cyw43_arch_gpio_put(LED_PIN, 1);

  struct udp_pcb* pcb = udp_new();

  ip_addr_t addr;
  ipaddr_aton(UDP_TARGET_ADDR, &addr);

  while (true) {
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, n_samples * 4, PBUF_RAM);
    spdif_rx_read((se32_t *)p->payload, n_samples);
    err_t er = udp_sendto(pcb, p, &addr, UDP_PORT);
    pbuf_free(p);
    if (er != ERR_OK) {
      printf("Failed to send UDP packet! error=%d", er);
    } else {
      //printf("Sent packet\n");
    }
#if PICO_CYW43_ARCH_POLL
    cyw43_arch_poll();
#endif
  }

  return 0;
}
