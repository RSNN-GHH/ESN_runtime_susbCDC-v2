// SPI Defines for spi1 to ADC
#define ADC_SPI_PORT    spi1
#define ADC_MISO    44
#define ADC_CS      45
#define ADC_SCK     46
#define ADC_MOSI    47
#define ADC_RST     43
#define RST		    0x8500	// ADC复位

//UART define
#define UART_ID uart1
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// dac8532 程序寄存器
#define ch_drain   0x30    // DRAIN DAC channel
#define ch_osc   0x34

// 程序寄存器
#define AUTO_SEQ_EN							0x01	// 自动扫描排序控制寄存器
#define CH_PWR_DN			 					0x02	// 通道掉电寄存器
#define FEATURE_SELECT 					0x03	// 器件特性选择控制寄存器
#define CH0_INPUT_RANGE 				0x05	// 通道0输入范围选择寄存器
#define CH1_INPUT_RANGE 				0x06	// 通道1输入范围选择寄存器
#define CH2_INPUT_RANGE 				0x07	// 通道2输入范围选择寄存器
#define CH3_INPUT_RANGE 				0x08	// 通道3输入范围选择寄存器
#define CH4_INPUT_RANGE 				0x09	// 通道4输入范围选择寄存器
#define CH5_INPUT_RANGE 				0x0A	// 通道5输入范围选择寄存器
#define CH6_INPUT_RANGE 				0x0B	// 通道6输入范围选择寄存器
#define CH7_INPUT_RANGE 				0x0C	// 通道7输入范围选择寄存器
#define MAN_CH_0			0xC000	// 选择通道0输入
#define MAN_CH_1			0xC400	// 选择通道1输入
#define MAN_CH_2			0xC800	// 选择通道2输入
#define MAN_CH_3			0xCC00	// 选择通道3输入
#define MAN_CH_4			0xD000	// 选择通道4输入
#define MAN_CH_5			0xD400	// 选择通道5输入
#define MAN_CH_6			0xD800	// 选择通道6输入
#define MAN_CH_7			0xDC00	// 选择通道7输入

// MUX
#define MUX_EN 36
#define MUX_ADD0 37
#define MUX_ADD1 38
#define MUX_ADD2 39

// 输入范围（VREF = 4.096V）
#define VREF_B_25							0x00	// 通道输入范围±2.5*VREF
#define VREF_B_125							0x01	// 通道输入范围±1.25*VREF
#define VREF_B_0625							0x02	// 通道输入范围±0.625*VREF
#define VREF_U_25							0x05	// 通道输入范围2.5*VREF
#define VREF_U_125							0x06	// 通道输入范围1.25*VREF