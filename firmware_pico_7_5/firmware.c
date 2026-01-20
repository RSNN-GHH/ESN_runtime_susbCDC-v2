#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "tusb.h"
#include "soft_spi.h"
#include "firmware.h"
#include "pio_spi.h"

#define COUNT_PIN 1

static uint32_t count_win_us = 1000;
static uint32_t frame_size16 = 64;

const uint8_t DAC_CH[8] = {0x00, 0x02, 0x04, 0x06, 0x07, 0x05, 0x03, 0x01};
pio_spi_inst_t SPI0 = {.pio = pio0, .sm = 0};



// 向 CDC 写二进制（最多 64 字节一包）
static void cdc_write_bin(const uint8_t *buf, size_t len) {
    for (size_t offset = 0; offset < len; offset += 64) {
        size_t chunk = len - offset > 64 ? 64 : len - offset;
        tud_cdc_write(buf + offset, chunk);
        tud_cdc_write_flush();
        tud_task();
        // 给出一点空隙，让 USB 事务有机会被提交
        sleep_ms(1);
    }
}


// 从 CDC 读精确 len 字节（阻塞，直到读满）
static void cdc_read_exact(uint8_t *buf, size_t len) {
    size_t idx = 0;
    while (idx < len) {
        tud_task();
        if (tud_cdc_available()) {
            int n = tud_cdc_read(buf + idx, len - idx);
            if (n > 0) {
                idx += n;
            }
        }
    }
}

// SPI 输出函数（保持不变）
static inline void Osc_output(uint16_t data) {
    uint8_t wr[3] = { ch_osc, 0x00, 0x00 };
    wr[1] = (data >> 8) & 0xFF;
    wr[2] = data & 0xFF;
    soft_spi_transfer_3bytes(wr);
    sleep_us(100);
}

// 在 count_win_us 窗口内计数脉冲
static uint16_t count_pulse(void) {
    uint16_t pulse = 0;
    uint32_t start = time_us_32();
    while (time_us_32() - start < count_win_us) {
        if (gpio_get(COUNT_PIN)) {
            while (gpio_get(COUNT_PIN)) {
                tight_loop_contents();
            }
            pulse++;
        }
    }
    return pulse;
}

static inline void dac8568_write_data(pio_spi_inst_t *spi, uint32_t data)
{
    static uint8_t data_array[4];
    data_array[0] = (uint8_t)(data >> 24);
    data_array[1] = (uint8_t)((data >> 16) & 0xFF);
    data_array[2] = (uint8_t)((data >> 8) & 0xFF);
    data_array[3] = (uint8_t)(data & 0xFF);
    pio_spi_write8_blocking(spi, data_array, 4);
}

static inline void dac8568_write_channel(pio_spi_inst_t *spi, uint8_t channel, uint16_t data)
{
    // 缩放到 32768~65535
    uint16_t data_scaled = (uint16_t)(((uint32_t)data * 32767) / 65535 + 32768);
    uint32_t txdata = 0x03000000;
    txdata |= (uint32_t)(channel&0x0F)<<20;
    txdata |= (uint32_t)data_scaled<<4;
    dac8568_write_data(spi, txdata);
}

void dac8568_init(pio_spi_inst_t *spi)
{
	sleep_us(100);
	dac8568_write_data(spi, 0x07000000);	/* 复位 */
	sleep_us(100);
	dac8568_write_data(spi, 0x040000FF);	/* 通道全部上电 */
	dac8568_write_data(spi, 0x090A0000);	/* 使用内部参考 */
}

void init_GATE0(void) {
    float clkdiv = 2.0f;  // 1 MHz @ 125 clk_sys
    uint cpha0_prog_offs = pio_add_program(SPI0.pio, &spi_cpha1_cs_program);
    pio_spi_cs_init(SPI0.pio, SPI0.sm, cpha0_prog_offs, 8, clkdiv, 1, 0, 8, 7, 1);
    dac8568_init(&SPI0);
}

// 握手：读 8 字节参数，清残留并回 ACK
static void init_spiking_module(void) {
    uint8_t buf[8];
    // 1) 读 frame_size16 (uint32) + count_win_us (uint32)
    cdc_read_exact(buf, 8);
    frame_size16 = ((uint32_t)buf[0]) | ((uint32_t)buf[1] << 8)
                 | ((uint32_t)buf[2] << 16)| ((uint32_t)buf[3] << 24);
    count_win_us = ((uint32_t)buf[4]) | ((uint32_t)buf[5] << 8)
                 | ((uint32_t)buf[6] << 16)| ((uint32_t)buf[7] << 24);

    // 2) 清空残留 CDC 数据
    while (tud_cdc_available()) {
        uint8_t dump[64];
        tud_cdc_read(dump, sizeof(dump));
        tud_task();
    }

    // 3) 回 ACK （同样 8 字节，LC 字节序）
    uint8_t ack[8];
    ack[0] = frame_size16 & 0xFF;
    ack[1] = (frame_size16 >> 8) & 0xFF;
    ack[2] = (frame_size16 >> 16) & 0xFF;
    ack[3] = (frame_size16 >> 24) & 0xFF;
    ack[4] = count_win_us & 0xFF;
    ack[5] = (count_win_us >> 8) & 0xFF;
    ack[6] = (count_win_us >> 16) & 0xFF;
    ack[7] = (count_win_us >> 24) & 0xFF;
    cdc_write_bin(ack, 8);
}

static void spiking_io(uint16_t *data_to_w,
                       uint16_t *data_read,
                       uint32_t frame_size) {
    for (uint32_t i = 0; i < frame_size; i++) {
        // Osc_output(data_to_w[i]);
        dac8568_write_channel(&SPI0, DAC_CH[0], data_to_w[i]);
        data_read[i] = count_pulse();
    }
}

int main() {
    // 初始化时钟、CDC、GPIO、SPI、看门狗
    set_sys_clock_khz(300000, true);
    stdio_init_all();
    tusb_init();
    gpio_init(COUNT_PIN);
    gpio_set_dir(COUNT_PIN, GPIO_IN);
    soft_spi_init();
    init_GATE0();
    dac8568_write_channel(&SPI0, DAC_CH[0], 0);
    watchdog_enable(2000, 1);

    // 握手
    init_spiking_module();

    // 动态分配：数据缓冲
    uint16_t *data_to_w = malloc(frame_size16 * sizeof(uint16_t));
    uint16_t *data_read = malloc(frame_size16 * sizeof(uint16_t));
    uint8_t *recv_buf = malloc(frame_size16 * 2);

    while (1) {
        tud_task();
        // 1) 读 frame_size16 * 2 字节
        cdc_read_exact(recv_buf, frame_size16 * 2);
        // 2) 解析成 uint16 数组（小端）
        for (uint32_t i = 0; i < frame_size16; i++) {
            data_to_w[i] = recv_buf[2*i] | (recv_buf[2*i+1] << 8);
        }
        // 3) 执行输出 + 计数
        spiking_io(data_to_w, data_read, frame_size16);

        // 4) 打包并回传二进制结果
        for (uint32_t i = 0; i < frame_size16; i++) {
            recv_buf[2*i]   = data_read[i] & 0xFF;
            recv_buf[2*i+1] = (data_read[i] >> 8) & 0xFF;
        }
        tud_task();
        cdc_write_bin(recv_buf, frame_size16 * 2);

        watchdog_update();
    }
}
