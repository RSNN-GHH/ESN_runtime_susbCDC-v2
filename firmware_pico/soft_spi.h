#define DRAIN_SCK_PIN  34   
#define DRAIN_MOSI_PIN 33   
#define DRAIN_CS_PIN   35   

void soft_spi_init();
void soft_spi_transfer(uint8_t data);
void soft_spi_transfer_3bytes(uint8_t *data_out);
void soft_spi1_init();
uint8_t soft_spi1_slave_transfer(uint8_t data_out);