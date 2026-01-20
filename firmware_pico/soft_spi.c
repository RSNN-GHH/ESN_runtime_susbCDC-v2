#include "pico/stdlib.h"
#include "soft_spi.h"
#include <stdio.h>

// 初始化软件 SPI
void soft_spi_init() {
    gpio_init(DRAIN_SCK_PIN);
    gpio_set_dir(DRAIN_SCK_PIN, GPIO_OUT);
    gpio_put(DRAIN_SCK_PIN, 0);

    gpio_init(DRAIN_MOSI_PIN);
    gpio_set_dir(DRAIN_MOSI_PIN, GPIO_OUT);
    gpio_put(DRAIN_MOSI_PIN, 0);

    gpio_init(DRAIN_CS_PIN);
    gpio_set_dir(DRAIN_CS_PIN, GPIO_OUT);
    gpio_put(DRAIN_CS_PIN, 1); // 默认拉高片选

}

// 通过软件 SPI 发送一个字节
void __time_critical_func(soft_spi_transfer)(uint8_t data_out) {
    gpio_put(DRAIN_CS_PIN, 0);
    for (int i = 0; i < 8; i++) {
        // 发送数据位（从最高位到最低位）
        gpio_put(DRAIN_MOSI_PIN, (data_out & 0x80) != 0);
        gpio_put(DRAIN_SCK_PIN, 1);
        gpio_put(DRAIN_SCK_PIN, 0);
    }
    gpio_put(DRAIN_CS_PIN, 1); 
}

// 通过软件 SPI 连续发送3个字节
void __time_critical_func(soft_spi_transfer_3bytes)(uint8_t *data_out) {
    gpio_put(DRAIN_CS_PIN, 0);
    for (int i = 0; i < 24; i++) {
        // 发送数据位（从最高位到最低位）
        gpio_put(DRAIN_MOSI_PIN, (data_out[i / 8] & (0x80 >> (i % 8))) != 0);
        gpio_put(DRAIN_SCK_PIN, 1);
        gpio_put(DRAIN_SCK_PIN, 0);
    }
    gpio_put(DRAIN_CS_PIN, 1);
}
