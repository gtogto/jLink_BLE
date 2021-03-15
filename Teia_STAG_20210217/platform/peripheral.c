
#include "peripheral.h"
#include "setting.h"
#include "port_def.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_power.h"
#include "nrf_drv_wdt.h"
#include "nrf_drv_saadc.h"
#include "nrf_nvmc.h"
#include <string.h>
#include <stdarg.h>
#include "SEGGER_RTT.h"
#include "timing.h"
#include "../Teia_var.h"

#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "../drivers/radios/DWM/deca_device_api.h"

volatile bool TAG_SHUTDOWNED = false;

//############################################################################################
//                    Uart function
//############################################################################################
int _write(int file, char *ptr, int len)
{
	int i=0;
	uint8_t cr;
	for(i=0 ; i<len ; i++) {
		cr = *ptr++;
		while(app_uart_put(cr) != NRF_SUCCESS);
	}
	return len;
}

void uart_error_handle(app_uart_evt_t * p_event)
{
	if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR) APP_ERROR_HANDLER(p_event->data.error_communication);
	else if (p_event->evt_type == APP_UART_FIFO_ERROR) APP_ERROR_HANDLER(p_event->data.error_code);
}

void UART_Uninit(void)
{
    	app_uart_close();
}

void UART_Init(void)
{
	uint32_t err_code;
	const app_uart_comm_params_t comm_params = { 11,  5,  0xFFFFFFFF,  0xFFFFFFFF,   0, false,  UART_BAUDRATE_BAUDRATE_Baud115200};
	APP_UART_INIT(&comm_params,uart_error_handle,  APP_IRQ_PRIORITY_LOW,  err_code);
	APP_ERROR_CHECK(err_code);
 }
//############################################################################################

void GPIO_IRQ_handler(uint32_t pin)
{
	switch(pin)    {
		case DW_IRQ:
			do{
			dwt_isr();
			} while (GPIO_PinInGet(DW_IRQ));
			break;
		case ACC_IRQ://printf("ACC..\r\n") ;
			int_flag.motion_action_detected = true;
			break;
		default:
			break;
	}
}
//############################################################################################
void RED_ON(void)
{
    	nrf_gpio_pin_clear(USER_PB_LED1);
	nrf_gpio_pin_set(USER_LED1);
}
void RED_OFF(void)
{
    	nrf_gpio_pin_set(USER_PB_LED1);
	nrf_gpio_pin_set(USER_LED1);		
}


void GREEN_ON(void)
{
    	nrf_gpio_pin_clear(USER_LED1);
	nrf_gpio_pin_set(USER_PB_LED1);
}
void GREEN_OFF(void)
{
    	nrf_gpio_pin_set(USER_LED1);
	nrf_gpio_pin_set(USER_PB_LED1);
}

void ORANGE_ON(void)
{
    	nrf_gpio_pin_clear(USER_LED1);
	nrf_gpio_pin_clear(USER_PB_LED1);
}
void ORANGE_OFF(void)
{
    	nrf_gpio_pin_set(USER_LED1);
	nrf_gpio_pin_set(USER_PB_LED1);
}


void DW_RST_ON(void)
{
    	nrf_gpio_pin_clear(DW_RST);
    	nrf_gpio_cfg_output(DW_RST);
}

void DW_RST_OFF(void)
{
    	nrf_gpio_cfg_input(DW_RST,NRF_GPIO_PIN_NOPULL);
}

void DW_CS_CLR(void)
{
    	nrf_gpio_pin_clear(SPI_CS);
}

void DW_CS_SET(void)
{
    	nrf_gpio_pin_set(SPI_CS);
}

void FLASH_CS_CLR(void)
{
    	nrf_gpio_pin_clear(SPI_CS_FLASH);
}

void FLASH_CS_SET(void)
{
    	nrf_gpio_pin_set(SPI_CS_FLASH);
}

void ADC_RDIV_PULL_DOWN(void)
{
    	nrf_gpio_pin_clear(ADC_RDIV);
}

void ADC_RDIV_PULL_UP(void)
{
    	nrf_gpio_pin_set(ADC_RDIV);
}

void ADC_MEAS_PIN_PULL_UP(void)
{
   	// NRF_SAADC->CH[1].CONFIG |= 2;
    	nrf_gpio_cfg_input(ADC_MEAS,NRF_GPIO_PIN_PULLUP);
}

void ADC_MEAS_PIN_BYPASS(void)
{
    	//NRF_SAADC->CH[1].CONFIG &= ~(11UL);
    	nrf_gpio_cfg_input(ADC_MEAS,NRF_GPIO_PIN_NOPULL);
}

bool GPIO_PinInGet(uint32_t pin)
{
    	return (bool)nrf_gpio_pin_read(pin);
}


void NFC_PWR_ON(void)
{
    	nrf_gpio_pin_set(NFC_PWR);
    	I2C_config(I2C_FREQ_STD);       ///nt3h2111 work properly at @100kHz
}

void NFC_PWR_OFF(void)
{
    	nrf_gpio_pin_clear(NFC_PWR);
    	I2C_config(I2C_FREQ_FAST);       ///all other chips can work @400kHz
}

void PB_LDO_SET(void)
{
    	nrf_gpio_pin_set(PB_LDO);
}

bool get_PB_state(void)
{
    return nrf_gpio_pin_read(PUSHBUTTON);
}

bool get_CHG_status(void)
{
    return (nrf_gpio_pin_read(CHG_STATUS));
}

bool get_ACC_INT_pin_status(void)
{
    return (nrf_gpio_pin_read(ACC_IRQ));
}

bool get_FD_state(void)
{
    return nrf_gpio_pin_read(NFC_FD);
}

void UWB_GPIO_init(void)
{
	nrf_gpio_cfg_output(SPI_MOSI);
	nrf_gpio_cfg_output(SPI_SCK);
	nrf_gpio_cfg_output(SPI_CS);
	nrf_gpio_cfg_input(SPI_MISO,NRF_GPIO_PIN_NOPULL);
}

void Flash_GPIO_init(void)
{
	nrf_gpio_cfg_output(SPI_MOSI_FLASH);
	nrf_gpio_cfg_output(SPI_SCK_FLASH);
	nrf_gpio_cfg_output(SPI_CS_FLASH);
	nrf_gpio_cfg_input(SPI_MISO_FLASH,NRF_GPIO_PIN_NOPULL);
}

void GPIOs_init(void)
{
	 //set LEDs as output
	nrf_gpio_cfg_output(LORA_WAKEUP);
	nrf_gpio_pin_set(USER_LED1);
	nrf_gpio_cfg_output(USER_LED1);
	nrf_gpio_cfg(USER_LED1,NRF_GPIO_PIN_DIR_OUTPUT,NRF_GPIO_PIN_INPUT_DISCONNECT,NRF_GPIO_PIN_NOPULL,GPIO_PIN_CNF_DRIVE_H0H1,NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_pin_set(USER_PB_LED1);
	nrf_gpio_cfg_output(USER_PB_LED1);
	nrf_gpio_cfg(USER_PB_LED1,NRF_GPIO_PIN_DIR_OUTPUT,NRF_GPIO_PIN_INPUT_DISCONNECT,NRF_GPIO_PIN_NOPULL,GPIO_PIN_CNF_DRIVE_H0H1,NRF_GPIO_PIN_NOSENSE);

	//configure SPI pins
	nrf_gpio_cfg_output(SPI_MOSI);
	nrf_gpio_cfg_output(SPI_SCK);
	nrf_gpio_cfg_output(SPI_CS);
	nrf_gpio_cfg_input(SPI_MISO,NRF_GPIO_PIN_NOPULL);

	//configure ACC irq pin as input
	nrf_gpio_cfg_input(ACC_IRQ,NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(PB_LDO,NRF_GPIO_PIN_PULLUP);

	//configure PUSHBUTTON as pin as input with internal pullup
	//nrf_gpio_cfg_input(PUSHBUTTON,NRF_GPIO_PIN_PULLUP);

	//configure charger status pin as input without pull resistor
	//nrf_gpio_cfg_input(CHG_STATUS,NRF_GPIO_PIN_PULLUP);

	nrf_gpio_cfg_input(DW_IRQ,NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_input_disconnect(DW_RST);
	nrf_gpio_input_disconnect(9);

	//configure ADC_RDIV
	nrf_gpio_cfg_output(ADC_RDIV);
	nrf_gpio_pin_set(ADC_RDIV);

	nrf_gpio_cfg_input(ADC_MEAS,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(CHG_STATUS,NRF_GPIO_PIN_PULLUP);
}
//############################################################################################
//                    I2C peripheral function
//############################################################################################
#define I2C_TRANSFER_DONE   0               /**< I2C instance returned status OK. */
#define I2C_DEVICE_NOT_RESP 1               /**< I2C instance returned status ERR. */
#define TWI_INSTANCE_ID     1               /**< I2C instance index. */
#define I2C_FREQ_100k       (26738688)      /**< I2C frequency = 100 kHz. */
#define I2C_FREQ_250k       (67108864)      /**< I2C frequency = 250 kHz. */
#define I2C_FREQ_400k       (104857600)     /**< I2C frequency = 400 kHz. */

/* TWI instance. */
static const nrf_drv_twi_t i2c = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static uint8_t i2c_transfer_complete = false;

void I2C_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
	switch (p_event->type)    {
		case NRF_DRV_TWI_EVT_DONE:
			i2c_transfer_complete = true;
			break;
		default:
			i2c_transfer_complete = true;
		break;
	}
}

void I2C_enable(void)
{
    	nrf_drv_twi_enable(&i2c);
}

void I2C_config(uint16_t freq_kHz)
{
	nrf_gpio_cfg_output(I2C_SCL);
	nrf_gpio_cfg_input(I2C_SDA,NRF_GPIO_PIN_NOPULL);

	for(uint8_t i = 0; i<9; i++)   {
		nrf_gpio_pin_clear(I2C_SCL);
		delay_us(5);
		nrf_gpio_pin_set(I2C_SCL);
		delay_us(5);
	}
	nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
	i2c_config.scl = I2C_SCL;
	i2c_config.sda = I2C_SDA;

	if(freq_kHz == 100) i2c_config.frequency = I2C_FREQ_100k;
	else if(freq_kHz == 250) i2c_config.frequency = I2C_FREQ_250k;
	else if(freq_kHz == 400) i2c_config.frequency = I2C_FREQ_400k;
	nrf_drv_twi_init(&i2c,&i2c_config,I2C_handler,NULL);
	nrf_drv_twi_enable(&i2c);
}

void I2C_disable(void)
{
	nrf_drv_twi_disable(&i2c);
	nrf_gpio_cfg_input(I2C_SCL,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(I2C_SDA,NRF_GPIO_PIN_NOPULL);
}

void I2C_disable_(void)
{
    	nrf_drv_twi_disable(&i2c);
}

uint8_t I2CDRV_readFromRegister(uint8_t device_addr, uint8_t reg_addr)
{
	uint8_t reg_data;

	i2c_transfer_complete = false;
	nrf_drv_twi_tx(&i2c,device_addr,&reg_addr,1,true);
	while(!i2c_transfer_complete);
	i2c_transfer_complete = false;
	nrf_drv_twi_rx(&i2c,device_addr,&reg_data,1);
	while(!i2c_transfer_complete);
	return reg_data;
}
uint8_t I2CDRV_burst_readFromRegister(uint8_t device_addr, uint8_t reg_addr,uint8_t length, uint8_t *data )
{
	uint8_t ret = 0;
	i2c_transfer_complete = false;
	nrf_drv_twi_tx(&i2c,device_addr,&reg_addr,1,true);
	while(!i2c_transfer_complete);
	i2c_transfer_complete = false;
	nrf_drv_twi_rx(&i2c,device_addr,data,length);
	while(!i2c_transfer_complete);
	return ret;
}

uint8_t I2CDRV_burst_writeToRegister(uint8_t device_addr, uint8_t reg_addr,uint8_t length, uint8_t *data )
{
	uint8_t ret = 0;
	uint8_t i2c_data[17];
	i2c_data[0] = reg_addr;
	memcpy(&i2c_data[1],data, length);
	i2c_transfer_complete = false;
	nrf_drv_twi_tx(&i2c,device_addr,i2c_data,length+1,false);
	while(!i2c_transfer_complete);
	return ret;
}

void I2CDRV_writeToRegister(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
	uint8_t tx_buff[2];
	tx_buff[0] = reg_addr;
	tx_buff[1] = data;
	i2c_transfer_complete = false;
	nrf_drv_twi_tx(&i2c,device_addr,tx_buff,2,false);
	while(!i2c_transfer_complete);
}
//############################################################################################



//############################################################################################
//                    SPI peripheral function
//############################################################################################
#define SPI_INSTANCE                        0 /**< SPI instance index. */
#define SPI_DEFAULT_CONFIG_IRQ_PRIORITY     6 /**<Lowest priority. */

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
static bool spi_transfer_complete = false;

void Flash_SPI_pins_enable(void) //==========================================================
{
	nrf_gpio_pin_clear(SPI_MOSI_FLASH);
	nrf_gpio_cfg_output(SPI_MOSI_FLASH);
	nrf_gpio_pin_clear(SPI_SCK_FLASH);
	nrf_gpio_cfg_output(SPI_SCK_FLASH);
	nrf_gpio_pin_set(SPI_CS_FLASH);
	nrf_gpio_cfg_output(SPI_CS_FLASH);
	nrf_gpio_cfg_input(SPI_MISO_FLASH,NRF_GPIO_PIN_NOPULL);
}
void UWB_SPI_pins_enable(void) //============================================================
{
	nrf_gpio_pin_clear(SPI_MOSI);
	nrf_gpio_cfg_output(SPI_MOSI);
	nrf_gpio_pin_clear(SPI_SCK);
	nrf_gpio_cfg_output(SPI_SCK);
	nrf_gpio_pin_set(SPI_CS);
	nrf_gpio_cfg_output(SPI_CS);
	nrf_gpio_cfg_input(SPI_MISO,NRF_GPIO_PIN_NOPULL);
}
void UWB_SPI_pins_disable(void) //===========================================================
{
	nrf_gpio_cfg_input(SPI_MOSI,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(SPI_CS,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(SPI_MISO,NRF_GPIO_PIN_NOPULL);
}
void Flash_SPI_pins_disable(void) //=========================================================
{
	nrf_gpio_cfg_input(SPI_MOSI_FLASH,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(SPI_CS_FLASH,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(SPI_MISO_FLASH,NRF_GPIO_PIN_NOPULL);
}

void spi_event_handler(nrf_drv_spi_evt_t const * p_event) //=================================
{
    	spi_transfer_complete  = true;
}

uint16_t Flash_SPI_init(uint16_t freq_kHz) //================================================
{
	nrf_drv_spi_uninit(&spi);
	uint16_t ret = 0;
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.miso_pin = SPI_MISO_FLASH;
	spi_config.mosi_pin = SPI_MOSI_FLASH;
	spi_config.sck_pin = SPI_SCK_FLASH;
	spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
	if      (freq_kHz>= 8000)  {spi_config.frequency = NRF_DRV_SPI_FREQ_8M;  ret = 8000;}
	else if (freq_kHz>= 4000)  {spi_config.frequency = NRF_DRV_SPI_FREQ_4M;  ret = 4000;}
	else if (freq_kHz>= 2000)  {spi_config.frequency = NRF_DRV_SPI_FREQ_2M;  ret = 2000;}
	else if (freq_kHz>= 1000)  {spi_config.frequency = NRF_DRV_SPI_FREQ_1M;  ret = 1000;}
	else if (freq_kHz>= 500)   {spi_config.frequency = NRF_DRV_SPI_FREQ_500K;ret = 500;}
	else if (freq_kHz>= 250)   {spi_config.frequency = NRF_DRV_SPI_FREQ_250K;ret = 250;}
	else                       {spi_config.frequency = NRF_DRV_SPI_FREQ_125K;ret = 125;}
	nrf_drv_spi_init(&spi,&spi_config,spi_event_handler);
	return ret;
}

uint16_t SPI_init(uint16_t freq_kHz) //======================================================
{
	nrf_drv_spi_uninit(&spi);
	uint16_t ret = 0;
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.miso_pin = SPI_MISO;
	spi_config.mosi_pin =  SPI_MOSI;
	spi_config.sck_pin = SPI_SCK;
	spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
	if      (freq_kHz>= 8000)  {spi_config.frequency = NRF_DRV_SPI_FREQ_8M;  ret = 8000;}
	else if (freq_kHz>= 4000)  {spi_config.frequency = NRF_DRV_SPI_FREQ_4M;  ret = 4000;}
	else if (freq_kHz>= 2000)  {spi_config.frequency = NRF_DRV_SPI_FREQ_2M;  ret = 2000;}
	else if (freq_kHz>= 1000)  {spi_config.frequency = NRF_DRV_SPI_FREQ_1M;  ret = 1000;}
	else if (freq_kHz>= 500)   {spi_config.frequency = NRF_DRV_SPI_FREQ_500K;ret = 500;}
	else if (freq_kHz>= 250)   {spi_config.frequency = NRF_DRV_SPI_FREQ_250K;ret = 250;}
	else                       {spi_config.frequency = NRF_DRV_SPI_FREQ_125K;ret = 125;}
	nrf_drv_spi_init(&spi,&spi_config,spi_event_handler);
	return ret;
}

void flash_write_enable_spi(void) //========================================================
{
	uint8_t temp;
	uint8_t write_enable_cmd;
	write_enable_cmd = 0x06;
	FLASH_CS_CLR();
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, &write_enable_cmd,1 ,&temp, 0);
	while(spi_transfer_complete == false);
	FLASH_CS_SET();
	delay_us(20);
}

void flash_Block_Erase_spi(uint32_t address) //==============================================
{
	uint8_t headerBuffer[3];
	uint8_t temp;
	uint8_t Block_erase_cmd;
	Block_erase_cmd = 0x20;
	headerBuffer[0] = (uint8_t)((address >> 16) & 0xff);
	headerBuffer[1] = (uint8_t)((address >> 8) & 0xff);
	headerBuffer[2] = (uint8_t)(address & 0xff);
	flash_write_enable_spi();
	FLASH_CS_CLR();
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, &Block_erase_cmd,1 ,&temp, 0);
	while(spi_transfer_complete == false);
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, headerBuffer,3 ,&temp, 0);
	while(spi_transfer_complete == false);
	FLASH_CS_SET();
	delay_and_sleep(500, 1, true);
	//delay_ms(500);
}

void flash_erase_spi(void)//================================================================
{
	uint8_t temp;
	uint8_t erase_cmd;
	erase_cmd = 0x60;
	flash_write_enable_spi();
	FLASH_CS_CLR();
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, &erase_cmd,1 ,&temp, 0);
	while(spi_transfer_complete == false);
	FLASH_CS_SET();
	//delay_ms(10000);
}

int flash_read_spi(uint32_t address, uint32_t readlength,uint8_t *readBuffer) //=============
{
	uint8_t headerBuffer[4];
	uint8_t temp;
	uint8_t buff[4];
	uint8_t read_cmd;

	read_cmd = 0x0B;
	headerBuffer[0] = (uint8_t)((address >> 16) & 0xff);
	headerBuffer[1] = (uint8_t)((address >> 8) & 0xff);
	headerBuffer[2] = (uint8_t)(address & 0xff);
	headerBuffer[3] = 0x00;

	FLASH_CS_CLR();
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, &read_cmd,1 ,&temp, 0);
	while(spi_transfer_complete == false);

	/*Send header via SPI*/
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, headerBuffer,4 ,&temp, 0);
	while(spi_transfer_complete == false);

	/*Read data via SPI*/
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, buff,4,readBuffer,readlength);
		while(spi_transfer_complete == false);

	FLASH_CS_SET();
	//delay_ms(100);
	return 0;
}

int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint32_t readlength,uint8_t *readBuffer)
{
	uint8_t temp;
	uint8_t buff[4];

	DW_CS_CLR();
	/*Send header via SPI*/
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, headerBuffer,headerLength,&temp,0);
	while(spi_transfer_complete == false);

	/*Read data via SPI*/
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, buff,4,readBuffer,readlength);
	while(spi_transfer_complete == false);
	DW_CS_SET();

	return 0;
}

int flash_write_spi(uint32_t address, uint32_t bodylength, uint8_t *bodyBuffer)//=====================
{
	uint8_t headerBuffer[3];
	uint8_t temp;
	uint8_t write_cmd;
	write_cmd = 0x02;

	headerBuffer[0] = (uint8_t)((address >> 16) & 0xff);
	headerBuffer[1] = (uint8_t)((address >> 8) & 0xff);
	headerBuffer[2] = (uint8_t)(address & 0xff);

	flash_write_enable_spi();

	FLASH_CS_CLR();
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, &write_cmd,1 ,&temp, 0);
	while(spi_transfer_complete == false);

	/*Send header via SPI*/
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, headerBuffer,3,&temp,0);
	while(spi_transfer_complete == false);

	/*Send data via SPI*/
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, bodyBuffer,bodylength,&temp,0);
	while(spi_transfer_complete == false);

	FLASH_CS_SET();
	delay_and_sleep(400, 1, true);
	//delay_ms(400);
	return 0;
}

int writetospi(uint16_t headerLength, uint8_t *headerBuffer, uint32_t bodylength, uint8_t *bodyBuffer)
{

	uint8_t temp;

	DW_CS_CLR();
	/*Send header via SPI*/
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, headerBuffer,headerLength,&temp,0);
	while(spi_transfer_complete == false);

	/*Send data via SPI*/
	spi_transfer_complete = false;
	nrf_drv_spi_transfer(&spi, bodyBuffer,bodylength,&temp,0);
	while(spi_transfer_complete == false);
	DW_CS_SET();
	return 0;
}

//############################################################################################

void clock_configuration(void)
{

	uint32_t err_code;
	err_code = nrf_drv_clock_init();
	APP_ERROR_CHECK(err_code);
	if(!nrf_drv_clock_lfclk_is_running())
	{
	    	nrf_drv_clock_lfclk_request(NULL);
	}
}

//############################################################################################
//                    RTC  function
//############################################################################################
const nrf_drv_rtc_t rtc0 = NRF_DRV_RTC_INSTANCE(0); /**<Definition of RTC0 instance.*/
const nrf_drv_rtc_t rtc1 = NRF_DRV_RTC_INSTANCE(1); /**<Definition of RTC1 instance.*/
const nrf_drv_rtc_t rtc2 = NRF_DRV_RTC_INSTANCE(2); /**<Definition of RTC2 instance.*/

#define PB_PRS      (0)     /**<Pushbutton pressed flag.*/
#define PB_RLS      (1)     /**<Pushbutton released flag.*/
#define PB_TIMEOUT  (2)     /**<Pushbutton timeout flag.*/


static void PB_handler(uint8_t flag)
{
    static uint32_t PB_prs_timestamp = 0;
    static uint32_t PB_rls_timestamp = 0;
    static uint8_t  short_press_cnt = 0;
    static bool  timer_long = false;
    static bool  timer_short = false;
    static bool  timer_very_long = false;

    if(flag == PB_PRS){
        PB_prs_timestamp = get_RTC_timestamp(true);
        RTC_CC_set(RTC_delay_instance,PB_CC_reg,((PB_prs_timestamp + USER_PB_LONG_PRESS_TICK)&0x00FFFFFF),true);
        timer_long = true;
        if(((PB_prs_timestamp - PB_rls_timestamp) >= USER_PB_LONG_PRESS_TICK) || ((PB_prs_timestamp - PB_rls_timestamp) < USER_PB_SHORT_PRESS_TICK))
        {
            short_press_cnt = 0;
        }
        if(short_press_cnt >= 2){

            RTC_CC_set(RTC_delay_instance,PB_CC_reg,((PB_prs_timestamp + USER_PB_SHORT_PRESS_TICK)&0x00FFFFFF),true);
            timer_long = false;
            timer_short = true;
            timer_very_long = false;
            short_press_cnt = 0;
        }

    }
    else if(flag == PB_RLS){
        PB_rls_timestamp = get_RTC_timestamp(true);
        RTC_CC_set(RTC_delay_instance,PB_CC_reg,0,false);
        timer_long = false;
        if((PB_rls_timestamp-PB_prs_timestamp) >= USER_PB_SHORT_PRESS_TICK)
        {
            short_press_cnt++;
        }
        else
        {
            short_press_cnt = 0;
        }
    }
    else if(flag == PB_TIMEOUT) {
        if(timer_long) {
            PB_long_pressed = true;
            timer_long = false;
            if(!TAG_SHUTDOWNED)
            {
                RTC_CC_set(RTC_delay_instance,PB_CC_reg,((PB_prs_timestamp + USER_PB_VLONG_PRESS_TICK)&0x00FFFFFF),true);
                timer_very_long = true;

            }
        }
        else if(timer_short) {
            PB_short_pressed_3t = true;
        }
        else if(timer_very_long) {
            PB_very_long_pressed = true;
        }
    }

}


void _RTC2_evt_handler(nrf_drv_rtc_int_type_t int_type)
{
    switch(int_type)
    {
        case(NRF_DRV_RTC_INT_COMPARE0):
            rtcDelayComplete[2][0]= true;
            NRF_RTC2->EVENTS_COMPARE[0] = 0;
            break;

        case (NRF_DRV_RTC_INT_COMPARE1):
            rtcDelayComplete[2][1]= true;
            NRF_RTC2->EVENTS_COMPARE[1] = 0;
            break;

        case (NRF_DRV_RTC_INT_COMPARE2):
            rtcDelayComplete[2][2]= true;
            NRF_RTC2->EVENTS_COMPARE[2] = 0;
            break;

        case (NRF_DRV_RTC_INT_COMPARE3):
            rtcDelayComplete[2][3]= true;
            NRF_RTC2->EVENTS_COMPARE[3] = 0;
            PB_handler(PB_TIMEOUT);
            int_flag.wake_up = true;
            break;


        case (NRF_DRV_RTC_INT_OVERFLOW):
            RTCCNT_MSByte++;
            NRF_RTC2->EVENTS_OVRFLW = 0;
            break;

        default:
            break;
    }
    // Clear all other events (also unexpected ones)
    NRF_RTC2->EVENTS_TICK       = 0;
}


void rtc_configuration(void)
{
    uint32_t err_code;

    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    err_code = nrf_drv_rtc_init(&rtc2, &config,_RTC2_evt_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_rtc_enable(&rtc2);                                      //enable rtc2 counter

    nrf_drv_rtc_counter_clear(&rtc2);                               //clear RTC counter value
    nrf_drv_rtc_overflow_enable(&rtc2,true);
    RTCCNT_MSByte = 0UL;                                            //clear RTC MSByte
}


uint32_t get_RTC_counter(uint8_t RTC_instance)
{
    uint32_t ret = ~0UL;
    if(RTC_instance == 0)    {
        ret = nrf_drv_rtc_counter_get(&rtc0);
    }
    else if(RTC_instance == 1)    {
        ret = nrf_drv_rtc_counter_get(&rtc1);
    }
    else if(RTC_instance == 2)    {
        ret = nrf_drv_rtc_counter_get(&rtc2);
    }
    return ret;
}

void RTC_CC_set(uint8_t RTC_instance,uint32_t channel, uint32_t val, bool IRQ_en)
{
    if(RTC_instance == 0)    {
        NRF_RTC0->EVENTS_COMPARE[channel] = 0;
        nrf_drv_rtc_cc_set(&rtc0,channel,val,IRQ_en);
    }
    else if(RTC_instance == 1)    {
        NRF_RTC1->EVENTS_COMPARE[channel] = 0;
        nrf_drv_rtc_cc_set(&rtc1,channel,val,IRQ_en);
    }
    else if(RTC_instance == 2)    {
        NRF_RTC2->EVENTS_COMPARE[channel] = 0;
        nrf_drv_rtc_cc_set(&rtc2,channel,val,IRQ_en);
    }

}


//############################################################################################
//                    GPIO_IRQ_handler function
//############################################################################################

void _GPIO_IRQ_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    //static bool PB_released = true;

    if(nrf_gpio_pin_read(DW_IRQ))  GPIO_IRQ_handler((uint32_t)DW_IRQ);

    /*if(!nrf_gpio_pin_read(PUSHBUTTON))
    {

        if(PB_released)
        {
            GPIO_IRQ_handler((uint32_t)PUSHBUTTON);
            PB_handler(PB_PRS);
            int_flag.wake_up = true;
            PB_released = false;

        }
    }
    else
    {
        if(!PB_released)
        {
            PB_released = true;
            PB_handler(PB_RLS);
            int_flag.wake_up =true;
        }
    }*/

    if(nrf_gpio_pin_read(ACC_IRQ))
    {
        GPIO_IRQ_handler((uint32_t)ACC_IRQ);
    }
}

//############################################################################################
//                    ext_irq_init function
//############################################################################################
bool ext_irq_init(void)
{
    uint32_t err_code = 0UL;
    if(!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
    }
    else return(false);


    // init interupt pin DW_IRQ//
    if(!err_code)
    {
        nrf_drv_gpiote_in_config_t gpiote_in_config;
        gpiote_in_config.hi_accuracy = false;
        gpiote_in_config.is_watcher = false;
        gpiote_in_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;
        gpiote_in_config.pull = NRF_GPIO_PIN_PULLDOWN;
        err_code = nrf_drv_gpiote_in_init(DW_IRQ, &gpiote_in_config, _GPIO_IRQ_handler);
        nrf_drv_gpiote_in_event_enable(DW_IRQ, true);
    }
    else return(false);


    // init interupt pin ACCELEROMETER//
    if(!err_code)
    {
        nrf_drv_gpiote_in_config_t gpiote_in_config;
        gpiote_in_config.hi_accuracy = false;                                                   //low power mode
        gpiote_in_config.is_watcher = false;
        gpiote_in_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;                                    //Nordic workaround for low power mode IRQ
        gpiote_in_config.pull = NRF_GPIO_PIN_PULLDOWN;                                          //ACC interrupt pin is active high
        err_code = nrf_drv_gpiote_in_init(ACC_IRQ, &gpiote_in_config, _GPIO_IRQ_handler);       //init pushbutton interrupt request
        nrf_drv_gpiote_in_event_enable(ACC_IRQ,false);                                          //accelerometer interrupt must be disabled after reset

    }
    else return(false);


    // init interupt pin PUSHBUTTON//
    /*if(!err_code)
    {
        nrf_drv_gpiote_in_config_t gpiote_in_config;
        gpiote_in_config.hi_accuracy = false;                                                    //low power mode
        gpiote_in_config.is_watcher = false;
        gpiote_in_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;                                    //Nordic workaround for low power mode IRQ
        gpiote_in_config.pull = NRF_GPIO_PIN_PULLUP;                                            //second terminal of pushbutton is grounded
        err_code = nrf_drv_gpiote_in_init(PUSHBUTTON, &gpiote_in_config, _GPIO_IRQ_handler);    //init PB interrupt request
        nrf_drv_gpiote_in_event_enable(PUSHBUTTON,false);                                       //PB interrupt must be disabled after reset

        PB_long_pressed = false;
        PB_short_pressed_3t = false;                                                            //clear PB pressed variables after reset - just in case

    }
    else return(false);*/

    return (true);

}

void UWB_IRQ_disable(void)
{
	nrf_drv_gpiote_in_event_disable(DW_IRQ);
}

void UWB_IRQ_enable(void)
{
	nrf_drv_gpiote_in_event_enable(DW_IRQ, true);
}

void PB_IRQ_disable(void)
{
	nrf_drv_gpiote_in_event_disable(PUSHBUTTON);
}

void PB_IRQ_enable(void)
{
	nrf_drv_gpiote_in_event_enable(PUSHBUTTON, true);
}

void ACC_IRQ_disable(void)
{
	nrf_drv_gpiote_in_event_disable(ACC_IRQ);
}


void ACC_IRQ_enable(void)
{
	nrf_drv_gpiote_in_event_enable(ACC_IRQ, true);
}

void ACC_IRQ_pin_dis(void)
{
    nrf_gpio_cfg_default(ACC_IRQ);
}

void CHG_IRQ_enable(void)
{
	nrf_drv_gpiote_in_event_enable(CHG_STATUS, true);
}

void CHG_IRQ_disable(void)
{
	nrf_drv_gpiote_in_event_disable(CHG_STATUS);
}


void GPIO_IRQ_enable(void)
{
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

void GPIO_IRQ_disable(void)
{
    NVIC_DisableIRQ(GPIOTE_IRQn);
}

void RTCs_IRQ_disable(void)
{
    NVIC_DisableIRQ(RTC1_IRQn);
    NVIC_DisableIRQ(RTC2_IRQn);
}

void RTCs_IRQ_enable(void)
{
    NVIC_EnableIRQ(RTC1_IRQn);
    NVIC_EnableIRQ(RTC2_IRQn);
}

void RTC2_IRQ_disable(void)
{
    NVIC_DisableIRQ(RTC2_IRQn);
}

void RTC2_IRQ_enable(void)
{
    NVIC_EnableIRQ(RTC2_IRQn);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*******************************************RAM retention setting************************************************************************//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define NRF52_ONRAM1_OFFRAM1  	POWER_RAM_POWER_S0POWER_On << POWER_RAM_POWER_S0POWER_Pos | POWER_RAM_POWER_S1POWER_On  << POWER_RAM_POWER_S1POWER_Pos      \
                                    | POWER_RAM_POWER_S0RETENTION_On  << POWER_RAM_POWER_S0RETENTION_Pos | POWER_RAM_POWER_S1RETENTION_On  << POWER_RAM_POWER_S1RETENTION_Pos;

#define NRF52_ONRAM1_OFFRAM0    POWER_RAM_POWER_S0POWER_On << POWER_RAM_POWER_S0POWER_Pos | POWER_RAM_POWER_S1POWER_On << POWER_RAM_POWER_S1POWER_Pos      \
                                    | POWER_RAM_POWER_S0RETENTION_Off << POWER_RAM_POWER_S0RETENTION_Pos | POWER_RAM_POWER_S1RETENTION_Off << POWER_RAM_POWER_S1RETENTION_Pos;

#define NRF52_ONRAM0_OFFRAM0    POWER_RAM_POWER_S0POWER_Off<< POWER_RAM_POWER_S0POWER_Pos | POWER_RAM_POWER_S1POWER_Off<< POWER_RAM_POWER_S1POWER_Pos;

//=======================================================================

void configure_ram_retention(void)
{
		// Configure nRF52 RAM retention parameters. Set for System On 64kB RAM retention
		NRF_POWER->RAM[0].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[1].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[2].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[3].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[4].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[5].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[6].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[7].POWER = NRF52_ONRAM1_OFFRAM0;
}

//=======================================================================

void _RTC1_evt_handler(nrf_drv_rtc_int_type_t int_type)
{
    switch(int_type)
    {
        case(NRF_DRV_RTC_INT_COMPARE0):                 //wdt stage
            NRF_RTC1->EVENTS_COMPARE[0] = 0;
            NVIC_SystemReset();
            break;
        case(NRF_DRV_RTC_INT_COMPARE1):                 //nfc field detection stage
            NRF_RTC1->EVENTS_COMPARE[1] = 0;
            NFC_should_be_check = true;
            int_flag.wake_up = true;
            break;



        default:
            break;
    }

}



bool WDOG_running = false;  /**<State of WDOG timer (true if WDOG is enabled)*/
uint32_t timeout_tcks = 0;  /**<Timeout of WDOG represented as number of RTC ticks.*/
//=======================================================================

void watchdog_init(uint32_t rr_us)
{
    uint32_t err_code;
    nrf_drv_rtc_uninit(&rtc1);
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.interrupt_priority = 1;                                  //highest priority //TODO chceck colision with softdevice
    err_code = nrf_drv_rtc_init(&rtc1, &config,_RTC1_evt_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_rtc_enable(&rtc1);                                      //enable rtc2 counter


    nrf_drv_rtc_counter_clear(&rtc1);                               //clear RTC counter value
    timeout_tcks = convertTime2Ticks(rr_us*2UL + WD_GUARD_INTERVAL_US);
    uint32_t rtccnt_val = get_RTC_counter(WDT_RTC_instance);
    RTC_CC_set(WDT_RTC_instance,WDT_RTC_chan, (rtccnt_val + timeout_tcks) & RTC_TOP, true);
    WDOG_running = true;
}

//=======================================================================

bool wdog_get_state(void)
{
    return WDOG_running;
}
//=======================================================================

void WDOG_Feed(void)
{
    if(WDOG_running)
    {
        uint32_t rtccnt_val = get_RTC_counter(WDT_RTC_instance);
        RTC_CC_set(WDT_RTC_instance,WDT_RTC_chan, (rtccnt_val + timeout_tcks) & RTC_TOP, true);
    }

}

//=======================================================================

void WDOG_enable(bool en)
{
    if(en)
    {
        uint32_t rtccnt_val = get_RTC_counter(WDT_RTC_instance);
        RTC_CC_set(WDT_RTC_instance,WDT_RTC_chan, (rtccnt_val + timeout_tcks) & RTC_TOP, true);
        WDOG_running = true;

    }
    else
    {
       nrf_drv_rtc_cc_disable(&rtc1,WDT_RTC_chan);
       WDOG_running = false;
    }

}



/* Hardware WDOG is not used becouse of inappropriate properties for this application. RTC is used instead of that.
void watchdog_init(uint32_t rr_us)
{
    nrf_drv_wdt_config_t wdt_config = NRF_DRV_WDT_DEAFULT_CONFIG ;
    float timeout_ms = 2*(float)rr_us/1000.f;                               //set timeout to 2 times of Refresh rate

     wdt_config.reload_value = (uint32_t)timeout_ms;

     nrf_drv_wdt_init(&wdt_config,_WDOG_IRQ_handler);

     nrf_drv_wdt_channel_alloc(NRF_WDT_RR0);
     nrf_drv_wdt_enable();
}


void WDOG_Feed(void)
{
	nrf_drv_wdt_feed();
}

bool wdog_get_state(void)
{
    return nrf_wdt_started();
}
*/

//=======================================================================

void _ADC_IRQ_handler(nrf_drv_saadc_evt_t const * p_event)
{

}

void ADC_Uninit(void)
{
    nrf_drv_saadc_uninit();

    nrf_drv_saadc_channel_uninit(ADC_VDD_channel);

    ///dylee delay_ms(100);
}
//=======================================================================

void ADC_init(void)
{
    nrf_drv_saadc_config_t adc_conf= NRF_DRV_SAADC_DEFAULT_CONFIG;
    nrf_saadc_channel_config_t adc_ch0_conf = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SAADC_CH_PSELP_PSELP_VDD);
    adc_ch0_conf.gain = NRF_SAADC_GAIN1_6;
    adc_ch0_conf.acq_time = NRF_SAADC_ACQTIME_40US;
    nrf_drv_saadc_init(&adc_conf,_ADC_IRQ_handler);

    nrf_drv_saadc_channel_init(ADC_VDD_channel,&adc_ch0_conf);

    nrf_saadc_channel_config_t adc_ch1_conf = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);
    adc_ch1_conf.gain = NRF_SAADC_GAIN1_4;
    adc_ch1_conf.acq_time = NRF_SAADC_ACQTIME_40US;
    nrf_drv_saadc_channel_init(ADC_BATT_channel,&adc_ch1_conf);


}

//=======================================================================

uint16_t ADC_one_shot_measure(uint8_t channel)
{
    nrf_saadc_value_t meas_value = 0;
    nrf_drv_saadc_sample_convert(channel,&meas_value);
    return (uint16_t)meas_value;
}

//=======================================================================

uint8_t meas_battery_voltage_raw(void)
{
    uint8_t raw_voltage;
//#if PLATFORM == TAG_2_IMU
    ADC_RDIV_PULL_DOWN();                                                           //battery is connected via resistive divider - second resistor must be tied to ground
    ADC_MEAS_PIN_BYPASS();                                                          //
    delay_and_sleep(1000,false,true);                                                //wait while bias capacitor (100pF) is not charged
    raw_voltage = ADC_one_shot_measure(ADC_BATT_channel);   //measure voltage on battery
    raw_voltage = (uint8_t)((float)(raw_voltage-1)*0.98f);

    ADC_MEAS_PIN_PULL_UP();                                                         //meas pin must be pulled up to protect against high voltage
    ADC_RDIV_PULL_UP();                                                             //battery is connected via resistive divider - second resistor be in high Z state to energy saving
#if 0
    raw_voltage = ADC_one_shot_measure(ADC_VDD_channel);   //measure voltage on battery
    raw_voltage = (uint8_t)((float)raw_voltage*0.96f);

#endif
    return raw_voltage;
}


#define PAGE_LENGTH             0x400UL
#define USER_PAGE 		        0x74C00
//=======================================================================

uint32_t write_bytes_to_user_page(uint16_t addr, uint8_t* data, size_t len)
{
    if((addr+len) < PAGE_LENGTH)
    {
        uint8_t temp[PAGE_LENGTH];

        memcpy((void*)temp,(void*)USER_PAGE,PAGE_LENGTH);
        nrf_nvmc_page_erase(USER_PAGE);

        memcpy((void*)(temp+addr),(void*)data,len);
        nrf_nvmc_write_bytes(USER_PAGE,temp,PAGE_LENGTH);
    }
    return 1;

}

//=======================================================================
void do_SWResetMCU(uint8_t errnum)
{
	if(errnum)
	{
		//DebugMsg(RELEASE, "\ne%d", errnum);
		delay_and_sleep(200, 1, true);
	}
	NVIC_SystemReset();
}


