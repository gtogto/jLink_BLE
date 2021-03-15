#ifndef PERIPHERAL_H_INCLUDED
#define PERIPHERAL_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define SPI_BAUDRATE_LOW                    1000 /**Set baudrate to 1 MBaud/s (1000kHz)*/
#define SPI_BAUDRATE_MIDDLE                 4000 /**Set baudrate to 4 MBaud/s (4000kHz)*/ ///dylee
#define SPI_BAUDRATE_HIGH                   8000 /**Set baudrate to 8 MBaud/s (8000kHz)*/

#define I2C_FREQ_STD                        100 /**SCL frequency in kHZ*/
#define I2C_FREQ_250                        250 /**SCL frequency in kHZ*/
#define I2C_FREQ_FAST                       400 /**SCL frequency in kHZ*/

#define LF_OSC_FREQ                         32768UL

#define ADC_VDD_channel     0
#define ADC_BATT_channel     1

#define  RTC_delay_instance     2

#define  RX_required_timer      0
#define  delay_CC_reg           1
#define  deep_sleep_CC_reg      2
#define  PB_CC_reg              3

#define  WDT_RTC_instance       1
#define  WDT_RTC_chan           0
#define  WDT_RTC_NFC_chan       1

#define USER_PB_SHORT_PRESS_TICK                (USER_PB_SHORTPRESS_LIMIT * 32768)/1000
#define USER_PB_LONG_PRESS_TICK                 (USER_PB_LONGPRESS_LIMIT * 32768)/1000
#define USER_PB_VLONG_PRESS_TICK                (USER_PB_VLONGPRESS_LIMIT * 32768)/1000

#define MPU9250 	(1<<0)
#define MMA8453		(1<<1)
#define LIS2DH12    (1<<2)

#define PREDEF_MAC_PAGE         0x73000

 /// Read commands
 #define READ_ARRAY                     0x0B
 #define READ_APPAY_50M                 0x03
 #define DUAL_OUT_READ                  0x3B
 #define DUAL_IO_READ                   0xBB
 #define QUAD_OUT_READ                  0x6B
 #define QUAD_IO_READ                   0xEB
 #define DUAL_READ_MODE_RESET           0xFFFF
 #define QUAD_READ_MODE_RESET           0xFF

 ///Program and Erase Commands
 #define BLOCK_ERASE_4K                 0x20
 #define BLOCK_ERASE_32K                0x52
 #define BLOCK_ERASE_64K                0xD8
 #define CHIP_ERASE                     0xC7     ////0x60
 #define BYTE_PROGRAM                   0x02

 ///Protection Commands
 #define WRITE_ENABLE                   0x06
 #define WRITE_DISABLE                  0x04

 ///Security Commands
 #define ERASE_SECURITY_REG_PAGE        0x44
 #define PROGRAM_SECURITY_REG_PAGE      0x42
 #define READ_SECURITY_REG_PAGE         0x48

 ///Status Register Commands
 #define READ_STATUS_REG_BYTE_1         0x05
 #define READ_STATUS_REG_BYTE_2         0x35
 #define WRITE_STATUS_REG               0x01
 #define WRITE_ENABLE_VOL_STATUS_REG    0x50

 ///Miscellaneous Commands
 #define READ_MAN_DEVICE_ID             0x9F
 #define READ_ID                        0x90
 #define DEEP_POWER_DOWN                0xB9
 #define RESUME_DEEP_POWER_DOWN         0xAB




#include "app_uart.h"

int _write(int file, char *ptr, int len) ;
uint8_t body[256];
unsigned char spi_txd[1024] ;
unsigned char spi_rxd[1024] ;

void GPIO_IRQ_handler(uint32_t pin);
void uart_error_handle(app_uart_evt_t * p_event) ;
void UART_Init(void) ;
void UART_Uninit(void);      ///dylee
void GPIOs_LED_Uninit(void); ///dylee
void ADC_Uninit(void);       ///dylee


volatile bool PB_short_pressed_3t;
volatile bool PB_long_pressed;
volatile bool PB_very_long_pressed;
volatile bool TAG_shutdown;
volatile bool CHG_active;
volatile bool charging_just_ended;
volatile bool charging_just_started;
volatile bool NFC_field_active;
volatile bool NFC_should_be_check;
volatile uint8_t RTCCNT_MSByte;	//RTC counter most significant byte - RTC_cnt have only 24 bites
volatile bool rtcDelayComplete[3][4]; //
volatile uint16_t baudrate;

void GPIOs_init(void);
int flash_write_spi(uint32_t address, uint32_t bodylength,uint8_t *bodyBuffer);
int flash_read_spi(uint32_t address, uint32_t readlength,uint8_t *readBuffer);
void flash_Block_Erase_spi(uint32_t address);
void flash_erase_spi(void);
void flash_write_enable_spi(void);
void flash_Block_Chip_Erase(void)  ;

uint8_t I2CDRV_readFromRegister(uint8_t device_addr, uint8_t reg_addr);
uint8_t I2CDRV_burst_readFromRegister(uint8_t device_addr, uint8_t reg_addr,uint8_t length, uint8_t *data );
void I2CDRV_writeToRegister(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
uint8_t I2CDRV_burst_writeToRegister(uint8_t device_addr, uint8_t reg_addr,uint8_t length, uint8_t *data );

void I2C_config(uint16_t freq_kHz);
void I2C_enable(void);      ///dylee
void I2C_disable_(void);    ///dylee
void I2C_disable(void);


void FLASH_CS_CLR(void);
void FLASH_CS_SET(void);

void Flash_SPI_pins_enable(void); ///dylee
void Flash_SPI_pins_disable(void);
void UWB_SPI_pins_enable(void);
void UWB_SPI_pins_disable(void);
uint16_t SPI_init(uint16_t freq_kHz);
uint16_t SPI_Flash_init(uint16_t freq_kHz);  ///dylee


void UWB_IRQ_enable(void);
void UWB_IRQ_disable(void);
void GPIO_IRQ_enable(void);
void GPIO_IRQ_disable(void);
void ACC_IRQ_disable(void);
void ACC_IRQ_enable(void);
void ACC_IRQ_pin_dis(void);
void PB_IRQ_enable(void);
void PB_IRQ_disable(void);
void CHG_IRQ_enable(void);
void CHG_IRQ_disable(void);
void do_SWResetMCU(uint8_t errnum);
bool ext_irq_init(void);
bool GPIO_PinInGet(uint32_t pin);
bool get_PB_state(void);
bool get_CHG_status(void);
bool get_ACC_INT_pin_status(void);


void RED_ON(void);
void RED_OFF(void);
void GREEN_ON(void);
void GREEN_OFF(void);
void ORANGE_ON(void);
void ORANGE_OFF(void);

void DW_RST_ON(void);
void DW_RST_OFF(void);
void DW_CS_CLR(void);
void DW_CS_SET(void);

void NFC_PWR_ON(void);
void NFC_PWR_OFF(void);

void ADC_RDIV_PULL_UP(void);
void ADC_RDIV_PULL_DOWN(void);

void ADC_MEAS_PIN_BYPASS(void);
void ADC_MEAS_PIN_PULL_UP(void);



void clock_configuration(void);
void rtc_configuration(void);
void watchdog_init(uint32_t rr_us);
bool wdog_get_state(void);
void WDOG_Feed(void);
void WDOG_enable(bool en);


uint32_t get_RTC_counter(uint8_t RTC_instance);

void ADC_init(void);
uint16_t ADC_one_shot_measure(uint8_t channel);
uint8_t meas_battery_voltage_raw(void);

void RTC_CC_set(uint8_t RTC_instace,uint32_t channel, uint32_t val, bool IRQ_en);

void configure_ram_retention(void);

void DebugMsg(uint8_t printrole, char *Str, ...);

void UWB_IRQ_disable(void);
void UWB_IRQ_enable(void);
void PB_IRQ_disable(void);
void PB_IRQ_enable(void);
void ACC_IRQ_disable(void);
void ACC_IRQ_enable(void);
void ACC_IRQ_pin_dis(void);
void GPIO_IRQ_enable(void);
void GPIO_IRQ_disable(void);
void RTCs_IRQ_disable(void);
void RTCs_IRQ_enable(void);
void RTC2_IRQ_disable(void);
void RTC2_IRQ_enable(void);

void PB_LDO_SET(void);

uint32_t write_bytes_to_user_page(uint16_t addr, uint8_t* data, size_t len);


void calculate_crc16(uint16_t* crc, uint8_t* data, size_t length);




#endif /* PERIPHERAL_H_INCLUDED */
