/*
 * TinyBee I2S output streamer.
 * Adapted in 2026 from Marlin's ESP32 I2S stepper stream (GPL-3.0),
 * originally Copyright (c) 2020 MarlinFirmware.
 */
#include "tinybee_i2s.h"

#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include <esp_heap_caps.h>
#include <esp_intr_alloc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <rom/lldesc.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>
#include <soc/io_mux_reg.h>
#include <soc/i2s_struct.h>

#include "robot_config.h"

namespace {
constexpr size_t kBufferCount = 4;
constexpr size_t kSamplesPerBuffer = 64;
constexpr size_t kBufferBytes = kSamplesPerBuffer * sizeof(uint32_t);

struct DmaState {
  uint32_t* buffers[kBufferCount]{};
  lldesc_t* descriptors[kBufferCount]{};
  QueueHandle_t queue = nullptr;
};

DmaState dma;
I2SSampleCallback sample_callback = nullptr;
portMUX_TYPE i2s_lock = portMUX_INITIALIZER_UNLOCKED;

void configureMatrixOutput(int pin, uint32_t signal) {
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
  gpio_set_direction(gpio_num_t(pin), GPIO_MODE_OUTPUT);
  gpio_matrix_out(pin, signal, false, false);
}

void resetFifo() {
  portENTER_CRITICAL(&i2s_lock);
  I2S0.conf.rx_fifo_reset = 1; I2S0.conf.rx_fifo_reset = 0;
  I2S0.conf.tx_fifo_reset = 1; I2S0.conf.tx_fifo_reset = 0;
  portEXIT_CRITICAL(&i2s_lock);
}

void stopI2S() {
  portENTER_CRITICAL(&i2s_lock);
  I2S0.out_link.stop = 1;
  I2S0.conf.tx_start = 0;
  I2S0.int_clr.val = I2S0.int_st.val;
  portEXIT_CRITICAL(&i2s_lock);
}

void startI2S() {
  portENTER_CRITICAL(&i2s_lock);
  I2S0.lc_conf.in_rst = 1; I2S0.lc_conf.in_rst = 0;
  I2S0.lc_conf.out_rst = 1; I2S0.lc_conf.out_rst = 0;
  I2S0.conf.tx_reset = 1; I2S0.conf.tx_reset = 0;
  I2S0.conf.rx_reset = 1; I2S0.conf.rx_reset = 0;
  I2S0.int_clr.val = 0xFFFFFFFF;
  I2S0.out_link.start = 1;
  I2S0.conf.tx_start = 1;
  portEXIT_CRITICAL(&i2s_lock);
}

void IRAM_ATTR dmaInterrupt(void*) {
  BaseType_t higher_priority_woken = pdFALSE;
  if (I2S0.int_st.out_eof) {
    lldesc_t* finished = reinterpret_cast<lldesc_t*>(I2S0.out_eof_des_addr);
    uint32_t* buffer = reinterpret_cast<uint32_t*>(const_cast<uint8_t*>(finished->buf));
    if (xQueueIsQueueFullFromISR(dma.queue)) {
      uint32_t* discarded = nullptr;
      xQueueReceiveFromISR(dma.queue, &discarded, &higher_priority_woken);
    }
    xQueueSendFromISR(dma.queue, &buffer, &higher_priority_woken);
  }
  I2S0.int_clr.val = I2S0.int_st.val;
  if (higher_priority_woken == pdTRUE) portYIELD_FROM_ISR();
}

void producerTask(void*) {
  uint32_t* buffer = nullptr;
  for (;;) {
    if (xQueueReceive(dma.queue, &buffer, portMAX_DELAY) == pdTRUE && buffer) {
      for (size_t sample = 0; sample < kSamplesPerBuffer; ++sample)
        buffer[sample] = sample_callback ? sample_callback() : 0;
    }
  }
}
}

bool tinybeeI2SBegin(I2SSampleCallback callback) {
  sample_callback = callback;
  periph_module_enable(PERIPH_I2S0_MODULE);

  dma.queue = xQueueCreate(kBufferCount, sizeof(uint32_t*));
  if (!dma.queue) return false;

    // First allocate every buffer and descriptor.
    for (size_t i = 0; i < kBufferCount; ++i) {
        dma.buffers[i] = static_cast<uint32_t*>(
            heap_caps_calloc(
                kSamplesPerBuffer,
                sizeof(uint32_t),
                MALLOC_CAP_DMA
            )
        );

        dma.descriptors[i] = static_cast<lldesc_t*>(
            heap_caps_calloc(
                1,
                sizeof(lldesc_t),
                MALLOC_CAP_DMA
            )
        );

        if (!dma.buffers[i] || !dma.descriptors[i]) {
            return false;
        }
    }

    // Only link the descriptors after all of them exist.
    for (size_t i = 0; i < kBufferCount; ++i) {
        for (size_t sample = 0; sample < kSamplesPerBuffer; ++sample) {
            dma.buffers[i][sample] =
                sample_callback ? sample_callback() : 0;
        }

        lldesc_t* descriptor = dma.descriptors[i];

        descriptor->owner  = 1;
        descriptor->eof    = 1;
        descriptor->sosf   = 0;
        descriptor->length = kBufferBytes;
        descriptor->size   = kBufferBytes;
        descriptor->buf =
            reinterpret_cast<uint8_t*>(dma.buffers[i]);
        descriptor->offset = 0;

        descriptor->empty = reinterpret_cast<uint32_t>(
            dma.descriptors[(i + 1) % kBufferCount]
        );
    }

  I2S0.out_link.addr = reinterpret_cast<uint32_t>(dma.descriptors[0]);
  stopI2S();
  resetFifo();

  I2S0.conf.tx_reset = 1; I2S0.conf.tx_reset = 0;
  I2S0.conf.rx_reset = 1; I2S0.conf.rx_reset = 0;
  I2S0.lc_conf.in_rst = 1; I2S0.lc_conf.in_rst = 0;
  I2S0.lc_conf.out_rst = 1; I2S0.lc_conf.out_rst = 0;

  I2S0.lc_conf.check_owner = 0;
  I2S0.lc_conf.out_loop_test = 0;
  I2S0.lc_conf.out_auto_wrback = 0;
  I2S0.lc_conf.out_data_burst_en = 0;
  I2S0.lc_conf.outdscr_burst_en = 0;
  I2S0.lc_conf.out_no_restart_clr = 0;
  I2S0.lc_conf.indscr_burst_en = 0;
  I2S0.lc_conf.out_eof_mode = 1;

  I2S0.conf2.lcd_en = 0;
  I2S0.conf2.camera_en = 0;
  I2S0.pdm_conf.pcm2pdm_conv_en = 0;
  I2S0.pdm_conf.pdm2pcm_conv_en = 0;
  I2S0.fifo_conf.dscr_en = 1;
  I2S0.conf_chan.tx_chan_mod = 0;
  I2S0.fifo_conf.tx_fifo_mod = 0;
  I2S0.conf.tx_mono = 0;
  I2S0.conf_chan.rx_chan_mod = 0;
  I2S0.fifo_conf.rx_fifo_mod = 0;
  I2S0.conf.rx_mono = 0;
  I2S0.conf.tx_start = 0;
  I2S0.conf.rx_start = 0;
  I2S0.conf.tx_msb_right = 1;
  I2S0.conf.tx_right_first = 1;
  I2S0.conf.tx_slave_mod = 0;
  I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
  I2S0.pdm_conf.rx_pdm_en = 0;
  I2S0.pdm_conf.tx_pdm_en = 0;
  I2S0.conf.tx_short_sync = 0;
  I2S0.conf.rx_short_sync = 0;
  I2S0.conf.tx_msb_shift = 0;
  I2S0.conf.rx_msb_shift = 0;

  // 160 MHz / 10 / 2 / 32 = 250 kHz word rate.
  I2S0.clkm_conf.clka_en = 0;
  I2S0.clkm_conf.clkm_div_num = 10;
  I2S0.clkm_conf.clkm_div_a = 0;
  I2S0.clkm_conf.clkm_div_b = 0;
  I2S0.sample_rate_conf.tx_bck_div_num = 2;

  I2S0.int_ena.out_eof = 1;
  I2S0.int_ena.out_dscr_err = 0;
  I2S0.int_ena.out_total_eof = 0;
  I2S0.int_ena.out_done = 0;

  intr_handle_t handle = nullptr;
  if (esp_intr_alloc(ETS_I2S0_INTR_SOURCE, 0, dmaInterrupt, nullptr, &handle) != ESP_OK)
    return false;
  esp_intr_enable(handle);

  if (xTaskCreatePinnedToCore(producerTask, "i2s-motion", 4096, nullptr, 20,
                              nullptr, 0) != pdPASS)
    return false;

  configureMatrixOutput(config::I2S_DATA_PIN, I2S0O_DATA_OUT23_IDX);
  configureMatrixOutput(config::I2S_BCK_PIN, I2S0O_BCK_OUT_IDX);
  configureMatrixOutput(config::I2S_WS_PIN, I2S0O_WS_OUT_IDX);
  startI2S();
  return true;
}
