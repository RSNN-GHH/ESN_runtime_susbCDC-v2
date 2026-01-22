#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "tusb.h"
#include "soft_spi.h"
#include "pio_spi.h"
#include "firmware.h"


const uint8_t v[4] = {0x00, 0x02, 0x04, 0x06}; // 器件通道选择
const uint8_t comp_ref = 0x00;          // 比较器参考电压通道
const uint8_t o[4] = {36, 37, 34, 35}; //  输出通道选择
static uint32_t count_win_us = 1000;
static uint32_t frame_size16 = 64;

pio_spi_inst_t SPI0 = {.pio = pio0, .sm = 0};
pio_spi_inst_t SPI1 = {.pio = pio0, .sm = 1};



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
	uint32_t txdata = 0x03000000;
	txdata |= (uint32_t)(channel&0x0F)<<20;
	txdata |= (uint32_t)data<<4;
	dac8568_write_data(spi, txdata);
}

void dac8568_init(pio_spi_inst_t *spi)
{
	sleep_us(1000);
	dac8568_write_data(spi, 0x07000000);	/* 复位 */
	sleep_us(1000);
	dac8568_write_data(spi, 0x040000FF);	/* 通道全部上电 */
	dac8568_write_data(spi, 0x090A0000);	/* 使用内部参考 */
}



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
// static inline void Osc_output(uint16_t data) {
//     uint8_t wr[3] = { ch_osc, 0x00, 0x00 };
//     wr[1] = (data >> 8) & 0xFF;
//     wr[2] = data & 0xFF;
//     soft_spi_transfer_3bytes(wr);
//     sleep_us(100);
// }

// 在 count_win_us 窗口内计数脉冲
static uint16_t count_pulse(uint8_t count_pin) {
    uint16_t pulse = 0;
    uint32_t start = time_us_32();
    while (time_us_32() - start < count_win_us) {
        if (gpio_get(count_pin)) {
            while (gpio_get(count_pin)) {
                tight_loop_contents();
            }
            pulse++;
        }
    }
    return pulse;
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
        dac8568_write_channel(&SPI1, v[0], data_to_w[i]); // DAC1 输出电压
        sleep_us(100);
        data_read[i] = count_pulse(o[0]);
        // Osc_output(0);
        dac8568_write_channel(&SPI1, v[0], 0); // DAC1 清零
        sleep_us(100);
    }
}


int main() {
    // 初始化时钟、CDC、GPIO、SPI、看门狗
    set_sys_clock_khz(300000, true);
    stdio_init_all();
    tusb_init();
    // 初始化计数引脚
    for (int i = 0; i < 4; i++) {
        gpio_init(o[i]);
        gpio_set_dir(o[i], GPIO_IN);
    }

    // 按照原始代码的方式初始化 PIO SPI
    float clkdiv = 2.0f;
    // 为两个DAC分别添加PIO程序
    uint cpha0_prog_offs = pio_add_program(SPI0.pio, &spi_cpha1_cs_program);
    uint cpha1_prog_offs = pio_add_program(SPI1.pio, &spi_cpha1_cs_program);
    // 按照原始代码方式初始化：pin_sck, pin_mosi, pin_miso
    // SPI0: GPIO8(SCK), GPIO7(MOSI/DIN), GPIO1(MISO不用)
    pio_spi_cs_init(SPI0.pio, SPI0.sm, cpha0_prog_offs, 8, clkdiv, 1, 0, 8, 7, 1);
    // SPI1: GPIO11(SCK), GPIO10(MOSI/DIN), GPIO1(MISO不用)  
    pio_spi_cs_init(SPI1.pio, SPI1.sm, cpha1_prog_offs, 8, clkdiv, 1, 0, 29, 28, 1);
    // 初始化DAC芯片
    sleep_ms(100);
    dac8568_init(&SPI0);
    dac8568_init(&SPI1);
    sleep_ms(100);
    //比较器电压
    dac8568_write_channel(&SPI0, comp_ref, 65536/10); 

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
