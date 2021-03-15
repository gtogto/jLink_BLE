#ifndef PORT_DEF_H_INCLUDED
#define PORT_DEF_H_INCLUDED


#define    ADC_RDIV     23
#define    ADC_MEAS     30
#define    I2C_SDA     29
#define    I2C_SCL     28
///#define    BUZZER      13
#define    PUSHBUTTON  1 ///2
#define    NFC_FD      26
///#define    USER_LED       30
#define    USER_PB_LED1  12//22    ///DWM TEIA MODULE  RED
///#define    USER_PB_LED    31
#define    USER_LED1     13//14    ///DWM TEIA MODULE  GREEN
#define    CHG_STATUS  15
#define    EXT_ACC_IRQ 2

#define    LORA_RESET     13
#define    LORA_WAKEUP    26
#define    NFC_PWR     8
#define    TESTPIN     27

#define    ACC_IRQ      25

#define    SPI_MOSI     20
#define    SPI_MISO     18
#define    SPI_SCK      16
#define    SPI_CS       17

#define    SPI_MOSI_FLASH      6
#define    SPI_MISO_FLASH      7
#define    SPI_SCK_FLASH       4
#define    SPI_CS_FLASH        3     ///2

#define DW_RST      24
#define DW_IRQ      19

#define PB_LDO      2


/*
#define     LED9        30
#define     LED10       31
#define     LED11       22
#define     LED12       14
*/

#define     DBG_SWO
#define     DBG_SWDIO


#endif /* PORT_DEF_H_INCLUDED */
