
#include <stdlib.h>
#include <string.h>

#include "Teia_routines.h"
#include "../platform/peripheral.h"
#include "../platform/timing.h"
#include "../platform/DWM_routines.h"
#include "../Teia_var.h"
#include "nrf.h"
#include "LIS2DH12.h"
#include "nrf_peripherals.h"
#include "../drivers/radios/DWM/deca_device_api.h"

#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_common.h"
#include "app_util_platform.h"
#include "../../Teia_STAG_20210217/main.h"
#include "app_uart.h"


const uint8_t at_cmd_msg[AT_SEND_CMD_LENGTH] = {'A','T','+','L','O','R','A','_','M','S','G','W','=','1',',','<'};
//**************************************************************************> System_Init
void System_Init(void)
{
    	clock_configuration();
    	rtc_configuration();
    	GPIOs_init();
		
    	//UART_Init() ;
    	UWB_SPI_pins_enable();
    	baudrate = SPI_init(SPI_BAUDRATE_LOW);
    	I2C_config(100);
    	delay_ms(10);

   	UWB_SPI_pins_enable();
    	DWM1000_initAndDoHwResetProcedure();
    	DWM1000_init();

    	ACC_Init();
}
//**************************************************************************> ACC_Init
void ACC_Init(void)
{
    //lis2dh12_calib();
    lis2dh12_WOM_setting(LIS2DH12_ODR25_Hz);
}

//**************************************************************************> ADC_Battery_check
void ADC_Battery_check(void)
{

        int temp, tmp ;
        ADC_init();
        check_batteryStatus(true);
        ADC_Uninit();
        battery = systemValues.actual_batt_voltage_raw;
        temp = (int)battery ;
        tmp = temp /100 ; temp = temp - tmp*100 ;
        battery_amount[0] = (uint8_t)tmp + 48 ;
        tmp = temp/10 ;  temp = temp - tmp*10 ;
        battery_amount[1] = (uint8_t)tmp + 48 ;
        battery_amount[2] = (uint8_t)temp + 48 ;
  	//printf("battery = %d\r\n",battery);

}
//**************************************************************************> LORA_Delay_time
void LORA_CMD_Delay_time(void)
{
    	uint32_t timestamp_start,timestamp_end ;
    	timestamp_start = get_RTC_timestamp(true);
    	while(!flag_lora_rx_OK){
        	timestamp_end = get_RTC_timestamp(true);
        	if((timestamp_end-timestamp_start) > USER_PB_2s_wait){
            		break ;
        	}
    	}
    	delay_us(200);
}
//**************************************************************************> LORA_Delay_time
void LORA_DATA_Delay_time(void)
{
    	uint32_t timestamp_start,timestamp_end ;
    	timestamp_start = get_RTC_timestamp(true);
    	while(!flag_lora_rx_OK){
        	timestamp_end = get_RTC_timestamp(true);
        	if((timestamp_end-timestamp_start) >USER_PB_2s_wait ){
            		break ;
        	}
    	}
    	delay_us(200);
}
//**************************************************************************> Send_LoRa_Wakeup_Status
void Send_LoRa_Ack(uint8_t cmd)  // added 2020.09.22
{
    	int i ;
	mode_lora_send = LORA_DATA_SEND_MODE ;
	flag_lora_rx_OK = 0 ;
    	for( i = 0 ; i < AT_SEND_CMD_LENGTH ; i++) while(app_uart_put(at_cmd_msg[i]) != NRF_SUCCESS);
    	for( i = 0 ; i < 12 ; i++)  while(app_uart_put(UWB_MACid[i]) != NRF_SUCCESS);
    	while(app_uart_put(cmd) != NRF_SUCCESS);
    	for( i = 0 ; i < 1 ; i++)  while(app_uart_put(battery_amount[i]) != NRF_SUCCESS);
    	while(app_uart_put('>') != NRF_SUCCESS);while(app_uart_put('\r') != NRF_SUCCESS);  while(app_uart_put('\n') != NRF_SUCCESS);
	LORA_DATA_Delay_time(); 
    	mode_lora_send = LORA_RECEIVE_MODE ;

}
//**************************************************************************> Send_LoRa_Wakeup_Status
void Send_LoRa_Echo(void)  // added 2020.09.22
{
    	int i ;
	mode_lora_send = LORA_DATA_SEND_MODE ;
	flag_lora_rx_OK = 0 ;
    	for( i = 0 ; i < AT_SEND_CMD_LENGTH ; i++) while(app_uart_put(at_cmd_msg[i]) != NRF_SUCCESS);
    	for( i = 1 ; i < 90 ; i++)  while(app_uart_put(flash_rd_Buf[i]) != NRF_SUCCESS);
	while(app_uart_put('\r') != NRF_SUCCESS);  while(app_uart_put('\n') != NRF_SUCCESS);
	LORA_DATA_Delay_time(); 
    	mode_lora_send = LORA_RECEIVE_MODE ;

}
//*************************************************************************************
void Send_Flash_Data_to_SC(uint8_t num) 
{
	int i ;

	mode_lora_send = LORA_DATA_SEND_MODE ;
	flag_lora_rx_OK = 0 ;
	for( i = 0 ; i < AT_SEND_CMD_LENGTH ; i++) while(app_uart_put(at_cmd_msg[i]) != NRF_SUCCESS);
	for( i = 0 ; i < 12 ; i++)  while(app_uart_put(UWB_MACid[i]) != NRF_SUCCESS);
    	while(app_uart_put('E') != NRF_SUCCESS);	
    	for( i = 0 ; i < 75 ; i++)  while(app_uart_put( process_info[ (75 * (num-1)) + i ]) != NRF_SUCCESS);
    	while(app_uart_put('>') != NRF_SUCCESS);while(app_uart_put('\r') != NRF_SUCCESS);  while(app_uart_put('\n') != NRF_SUCCESS);
	LORA_DATA_Delay_time(); 
    	mode_lora_send = LORA_RECEIVE_MODE ;

}
//**************************************************************************> Send_LoRa_Upload
void Send_LoRa_Upload(void)
{
    	int i ;
	mode_lora_send = LORA_DATA_SEND_MODE ;
	flag_lora_rx_OK = 0 ;
    	for( i = 0 ; i < AT_SEND_CMD_LENGTH ; i++) while(app_uart_put(at_cmd_msg[i]) != NRF_SUCCESS);
    	for( i = 1 ; i < 89 ; i++)  while(app_uart_put(flash_rd_Buf[i]) != NRF_SUCCESS);
    	while(app_uart_put('>') != NRF_SUCCESS);while(app_uart_put('\r') != NRF_SUCCESS);  while(app_uart_put('\n') != NRF_SUCCESS);
	LORA_DATA_Delay_time(); 
    	mode_lora_send = LORA_RECEIVE_MODE ;
}
//**************************************************************************> LORA_Setting
void LORA_Setting(void)
{
	UART_Init() ;
	mode_lora_send = 	LORA_CMD_SEND_MODE ;	
	flag_lora_rx_OK = false;
    	printf("AT+LORA_SCODEW=BB\r\n"); LORA_CMD_Delay_time(); flag_lora_rx_OK = false;
    	printf("AT+LORA_APPSKEYW=0123456789ABCDEF0123456789ABCDEF\r\n"); LORA_CMD_Delay_time();flag_lora_rx_OK = false;
	printf("AT+LORA_DEVIDW=07\r\n"); LORA_CMD_Delay_time();flag_lora_rx_OK = false;
    	printf("AT+LORA_BW=9\r\n");      LORA_CMD_Delay_time();flag_lora_rx_OK = false;
    	printf("AT+LORA_PWRS=13\r\n");   LORA_CMD_Delay_time();flag_lora_rx_OK = false;
    	printf("AT+LORA_SFS=12\r\n");    LORA_CMD_Delay_time();flag_lora_rx_OK = false;
    	printf("AT+LORA_FREQS=26\r\n");  LORA_CMD_Delay_time();flag_lora_rx_OK = false;
    	printf("AT+LORA_COMPRESSW=2\r\n"); LORA_CMD_Delay_time();flag_lora_rx_OK = false;
	//    printf("AT+LORA_AESW=ON\r\n");   LORA_CMD_Delay_time(); flag_lora_rx_OK = false;
    	printf("AT+LORA_BRIEF=1\r\n");  LORA_CMD_Delay_time(); flag_lora_rx_OK = false;
    	mode_lora_send = LORA_RECEIVE_MODE ;
	UART_Uninit();		
}
//**************************************************************************> Convert_Macaddress
void Convert_Macaddress(void)
{
    uint8_t k,tmp ;
    for(tmp = 0 ;tmp < 6 ; tmp++){
        k = (uint8_t)((LMSParameters.MACaddress >> (8*tmp) ) & 0xFF) ;
        UWB_MACid[tmp*2] = (k >> 4) & 0x0f ;
        UWB_MACid[tmp*2+1] = k & 0x0f ;
    }
    for(tmp = 0 ;tmp < 12 ; tmp++){
        if( UWB_MACid[tmp] < 10 ) UWB_MACid[tmp] = UWB_MACid[tmp]+48 ;
        else UWB_MACid[tmp] = UWB_MACid[tmp]+55 ;
    }
    //for(k=0;k<12;k++) {   printf("%c\r\n",UWB_MACid[k]) ;}
}
//**************************************************************************> LORA_Wakeup
void LORA_Wakeup(void)
{
    nrf_gpio_pin_clear(LORA_WAKEUP);delay_us(5);
    nrf_gpio_pin_set(LORA_WAKEUP);  delay_ms(1) ;
    UART_Init(); delay_ms(1) ;
    flag_lora_enable = true;
    //printf("AT\r\n"); LORA_Delay_time();
}
//**************************************************************************> LORA_Sleep
void LORA_Sleep(void)
{
    printf("AT+LORA_SLEEP=\r\n");
    delay_ms(2);
    UART_Uninit();
    flag_lora_enable = false ;
}
//**************************************************************************> 

void start_UWB_TX(void)
{
	dwt_starttx(DWT_START_TX_IMMEDIATE); // send the frame
}
//**************************************************************************> UWB_SPI_init
void UWB_SPI_init(void)
{
    Flash_SPI_pins_disable() ;
    UWB_SPI_pins_enable();
    baudrate = SPI_init(SPI_BAUDRATE_LOW);
}
//**************************************************************************> MEMORY_SPI_init
void MEMORY_SPI_init(void)
{
    UWB_SPI_pins_disable() ;
    Flash_SPI_pins_enable();
    baudrate = Flash_SPI_init(SPI_BAUDRATE_LOW);
}
//**************************************************************************>configure_UWB
void UWB_Reconfiguration(void)
{
	UWB_SPI_pins_enable();
	DWM_Setparameter() ; //DWM_reconfigure();
    	GPIO_IRQ_enable();
	UWB_IRQ_enable();
}
//**************************************************************************>UWB_radio_wake_up
void UWB_radio_wake_up(void)
{
	DMW1000_wake_up();
}
//**************************************************************************>UWB_radio_enter_sleep
void UWB_radio_enter_sleep(void)
{
	DM1000_enter_sleep();
}

//**************************************************************************>
void send_Blink(void)
{
	extern volatile bool blink_sent;

	UWB_SPI_init();

	UWB_radio_wake_up();

	dwt_writetxdata(prepared_blink[0] ,(uint8_t *) &prepared_blink[1], 0); // write the frame data
	dwt_writetxfctrl(prepared_blink[0] , 0);   	                           //write length of message
	start_UWB_TX();
	while(!blink_sent);
	blink_sent = false;
	UWB_radio_enter_sleep();
	UWB_SPI_pins_disable();
}
void send_Register(void)
{
	extern volatile bool blink_sent;
	UWB_SPI_pins_enable();
	UWB_radio_wake_up();
	//prepareRegistMsg(&systemValues);
	start_UWB_TX();
	while(!blink_sent);
	blink_sent = false;
	UWB_radio_enter_sleep();
	UWB_SPI_pins_disable();
}
//**************************************************************************>
void check_batteryStatus(bool first_measure)
{
	if ((systemValues.need_to_measure_battery == MEAS_BATT_AFTER_RR) || (first_measure))	{
		systemValues.actual_batt_voltage_raw = meas_battery_voltage_raw();
		systemValues.need_to_measure_battery = 0;
		if(first_measure) systemValues.lowbattcount = SLEEP_AFTER_LOW_BATT_MEASURE - 1;
		if (systemValues.actual_batt_voltage_raw <= MIN_ALLOWED_BATT_VOLTAGE)    systemValues.lowbattcount++;
		else systemValues.lowbattcount = 0;
	}
	systemValues.need_to_measure_battery++;
}
//**************************************************************************>
void shutdown_tag(void)
{
	UWB_IRQ_disable();
	PB_IRQ_disable();
	ACC_IRQ_disable();
	RTCs_IRQ_disable();
	CHG_IRQ_disable();
	DM1000_enter_deep_sleep();
	DM1000_enter_sleep();
	//set_sensors_sleep();
}
//**************************************************************************>
#define PRF_16      (0)
#define PRF_64      (1)

#define DR_110k     (0)
#define DR_850k     (1)
#define DR_6M8      (2)

#define PREAM_L_2048 (2)
#define PREAM_L_1024 (3)
#define PREAM_L_256  (5)
#define PREAM_L_128  (6)

#define nSFD_STD     (0)
#define nSFD_nSTD    (1)

void Set_LMS_Parameters(void)
{
	bool STD = true;

	// add eeprom data reading routine


	// if there is not any data in eeprom, then set parameter to default value
	LMSParameters.init_state = INITParameters.init_state ;
	LMSParameters.channel = INITParameters.channel  ;
	LMSParameters.data_rate = INITParameters.data_rate  ;
	LMSParameters.RF_profile = INITParameters.RF_profile ;
	LMSParameters.preamble = INITParameters.preamble ;
	LMSParameters.prf = INITParameters.prf ;
	LMSParameters.preamCode = INITParameters.preamCode ;
	LMSParameters.nSfd = INITParameters.nSfd ;
	LMSParameters.use_random_deviation = INITParameters.use_random_deviation ;
	LMSParameters.refresh_rate_ms = INITParameters.refresh_rate_ms ;
	LMSParameters.no_motion_refresh_rate = INITParameters.no_motion_refresh_rate ;
	LMSParameters.RX_period_ms = INITParameters.RX_period_ms ;
	LMSParameters.RX_duration_ms = INITParameters.RX_duration_ms ;
	LMSParameters.tx_pwr_level = INITParameters.tx_pwr_level ;

	if(LMSParameters.prf == PRF_16)	{
		if     ((LMSParameters.channel == 1) && (LMSParameters.preamCode == 1)) STD = true;
		else if((LMSParameters.channel == 2) && (LMSParameters.preamCode == 3)) STD = true;
		else if((LMSParameters.channel == 3) && (LMSParameters.preamCode == 5)) STD = true;
		else if((LMSParameters.channel == 4) && (LMSParameters.preamCode == 7)) STD = true;
		else if((LMSParameters.channel == 5) && (LMSParameters.preamCode == 3)) STD = true;
		else if((LMSParameters.channel == 7) && (LMSParameters.preamCode == 7)) STD = true;
		else STD = false;

		if(STD)	{
		    if      ((LMSParameters.data_rate == DR_110k) && (LMSParameters.preamble == PREAM_L_1024) && (LMSParameters.nSfd == nSFD_nSTD)) LMSParameters.RF_profile = 0;
		    else if ((LMSParameters.data_rate == DR_850k) && (LMSParameters.preamble == PREAM_L_256) && (LMSParameters.nSfd == nSFD_nSTD)) LMSParameters.RF_profile = 1;
		    else if ((LMSParameters.data_rate == DR_6M8) && (LMSParameters.preamble == PREAM_L_128) && (LMSParameters.nSfd == nSFD_STD)) LMSParameters.RF_profile = 2;
		    else if ((LMSParameters.data_rate == DR_110k) && (LMSParameters.preamble == PREAM_L_2048) && (LMSParameters.nSfd == nSFD_nSTD)) LMSParameters.RF_profile = 6;
		    else STD = false;
		}
	}
	else if(LMSParameters.prf == PRF_64){
		if     ((LMSParameters.channel == 1) && (LMSParameters.preamCode == 9))     STD = true;
		else if((LMSParameters.channel == 2) && (LMSParameters.preamCode == 10)) STD = true;
		else if((LMSParameters.channel == 3) && (LMSParameters.preamCode == 11)) STD = true;
		else if((LMSParameters.channel == 4) && (LMSParameters.preamCode == 20)) STD = true;
		else if((LMSParameters.channel == 5) && (LMSParameters.preamCode == 12)) STD = true;
		else if((LMSParameters.channel == 7) && (LMSParameters.preamCode == 17)) STD = true;
		else STD = false;

		if(STD)
		{
		    if      ((LMSParameters.data_rate == DR_110k) && (LMSParameters.preamble == PREAM_L_1024) && (LMSParameters.nSfd == nSFD_nSTD)) LMSParameters.RF_profile = 3;
		    else if ((LMSParameters.data_rate == DR_850k) && (LMSParameters.preamble == PREAM_L_256) && (LMSParameters.nSfd == nSFD_nSTD)) LMSParameters.RF_profile = 4;
		    else if ((LMSParameters.data_rate == DR_6M8) && (LMSParameters.preamble == PREAM_L_128) && (LMSParameters.nSfd == nSFD_STD)) LMSParameters.RF_profile = 5;
		    else if ((LMSParameters.data_rate == DR_110k) && (LMSParameters.preamble == PREAM_L_2048) && (LMSParameters.nSfd == nSFD_nSTD)) LMSParameters.RF_profile = 7;
		    else STD = false;
		}
	}

	if(!STD) LMSParameters.RF_profile = 9;

}

void Reset_LMS_Parameters(void)
{

	INITParameters.init_state 	= SETTING_INIT_CHECK;
	INITParameters.channel = UWB_CHANNEL ;
	INITParameters.data_rate = 1;
	INITParameters.RF_profile = 4;
	INITParameters.preamble = 5;
	INITParameters.prf = 1;
	INITParameters.preamCode = UWB_PREAMCODE;
	INITParameters.nSfd = 1;
	INITParameters.use_random_deviation = 1;
	INITParameters.refresh_rate_ms = 100;
	INITParameters.no_motion_refresh_rate = 0;
	INITParameters.RX_period_ms = 0;
	INITParameters.RX_duration_ms = 1000;
	INITParameters.tx_pwr_level = 0x0UL;

       // add eeprom write routine....

}
//*************************************************************************************
bool Read_Flash_Block_Data(uint32_t block) 
{
	int i ;
	MEMORY_SPI_init();
	flash_read_spi(BLOCK_SIZE * block , PROCESS_INFO_SIZE, process_info);
	for( i=0;i<PROCESS_INFO_SIZE ;i++){
		if( (process_info[i] < 0x20)||(process_info[i] > 0x7E)) break ;
	}
	if( i == PROCESS_INFO_SIZE ) return true ;
	else return false ;
}
//*************************************************************************************
void Erase_Flash_All(void ) 
{
	int i ;
	MEMORY_SPI_init();
	for(i=0;i<PROCESS_SIZE;i++){
		flash_Block_Erase_spi(BLOCK_SIZE * i);
		//flash_read_spi(BLOCK_SIZE * i , PROCESS_INFO_SIZE, process_info);
		//for(int j=0;j<PROCESS_INFO_SIZE;j++) if( process_info[j] != 0xFF) RED_ON() ;
	}	
}
//*************************************************************************************
bool Verify_Flash_Data(uint8_t num) 
{
	int i ;
	for( i =0 ;i < 75 ;i++){
		if( LoRa_RXD_Buf[i+14] != process_info[ (75 * (num-1)) + i ]) break ;
	}
	if( i == 75 )	return true ;
	else return false ;							
}
//*************************************************************************************
void LED_Blink(uint8_t  color, uint32_t ontime )
{
	switch( color ){
		case RED:
			RED_ON() ; delay_and_sleep(ontime, 1, true);	RED_OFF() ;
			break;
		case GREEN:
			GREEN_ON() ; delay_and_sleep(ontime, 1, true);	GREEN_OFF() ;
			break ;
		case ORANGE:
			ORANGE_ON() ; delay_and_sleep(ontime, 1, true);	ORANGE_OFF() ;
			break;
		case BLACK:
			ORANGE_OFF() ;
			break;
	}
}
//*************************************************************************************
bool LoRa_Processor(void)
{
	uint32_t i;

	if( flag_lora_data_rx_OK ){
		flag_lora_data_rx_OK = false ;	

		if( LoRa_RXD_Buf[13] == LORA_RX_CMD_CONNECT ){	
		//================================================ Connection
			ADC_Battery_check();
			Send_LoRa_Ack(ACK_BATTERY) ;
			flash_write_enable = true  ;  // write command control
			timestamp_lora_start = get_RTC_timestamp(true);
			lora_rx_waittime = WAIT_TIME_300ms ;
			 
		}
		else if( LoRa_RXD_Buf[13] == LORA_RX_CMD_LORA_PERIOD ){	
		//================================================= Lora
			if( ( LoRa_RXD_Buf[14] >0x2F ) && ( LoRa_RXD_Buf[14] <0x3A) ){
				lora_period = LoRa_RXD_Buf[15] -0x30 ;						
				ADC_Battery_check();
				Send_LoRa_Ack(ACK_BATTERY) ;
				return true ;
			}
		}	
		else if( LoRa_RXD_Buf[13] == LORA_RX_CMD_UWB_PERIOD ){						
		//================================================= UWB 
			if( ( LoRa_RXD_Buf[14] >0x2F ) && ( LoRa_RXD_Buf[14] <0x3A) ){
				uwb_period = LoRa_RXD_Buf[15] -0x30 ;
				ADC_Battery_check();
				Send_LoRa_Ack(ACK_BATTERY) ;
				return true ;
			}
		}
		else if( LoRa_RXD_Buf[13] == LORA_RX_CMD_LED_ONOFF ){	
		//=================================================  LED on/off
			if( ( LoRa_RXD_Buf[14] >0x2F ) && ( LoRa_RXD_Buf[14] <0x3A) ){
				led_onoff = LoRa_RXD_Buf[15] -0x30 ;
				ADC_Battery_check();
				Send_LoRa_Ack(ACK_BATTERY) ;
				return true ;
			}
		}				
		else if( LoRa_RXD_Buf[13] == LORA_RX_CMD_WRITE ){	 
		//==============================================   Write processing
			uint8_t total, current ;
			total = LoRa_RXD_Buf[14] - 0x30;
			current = LoRa_RXD_Buf[15]  - 0x30  ;
			if( ( total  == PROCESS_PACKET_NUM )&&(current < PROCESS_PACKET_NUM ) ){
				for( i = 0 ; i< 75 ;i++)	process_info[ (current-1)*75+i] = LoRa_RXD_Buf[i+14]  ;						
				for(i=0 ; i<90 ; i++)flash_rd_Buf[i] = LoRa_RXD_Buf[i]; flash_rd_Buf[13] = 'V' ;
				Send_LoRa_Echo() ;
				timestamp_lora_start = get_RTC_timestamp(true);
				lora_rx_waittime = WAIT_TIME_500ms ;				
			}
			else if( ( total  == PROCESS_PACKET_NUM )&&(current  == PROCESS_PACKET_NUM ) ){
				for( i = 0 ; i< 75 ;i++)	process_info[ 2*75+i] = LoRa_RXD_Buf[i+14]  ;		
				if( flash_write_enable ){
					flash_write_enable = false ;
					MEMORY_SPI_init();
					flash_write_spi(BLOCK_SIZE * page_cnt++ , PROCESS_INFO_SIZE, process_info);
					page_cnt_start = page_cnt ;
					erase_state_machine = SEND_QUIT ;
				}
				for(i=0 ; i<90 ; i++)flash_rd_Buf[i] = LoRa_RXD_Buf[i]; flash_rd_Buf[13] = 'V' ;
				Send_LoRa_Echo() ;
				return true ;
			}
			else{
				Send_LoRa_Ack(ACK_ERROR) ;		
				timestamp_lora_start = get_RTC_timestamp(true);
				lora_rx_waittime = WAIT_TIME_500ms ;				
			}						
		}
		else if( LoRa_RXD_Buf[13] == LORA_RX_CMD_ERASE ){ 
		//===============================================   Erase command
			ADC_Battery_check();
			bat_initial_value = battery ;
	
			if( Read_Flash_Block_Data( page_cnt - (page_cnt_start--)) ) {
				erase_state_machine = VERIFY_PACKET1 ;
				Send_Flash_Data_to_SC(VERIFY_PACKET1) ;
				flag_flash_send_finish = false ;
				timestamp_lora_start = get_RTC_timestamp(true);
				lora_rx_waittime = WAIT_TIME_1000ms ;					
			}
			else {
				flag_flash_send_finish = true ;	
				flag_charging_area = true ;  // charging
				bat_average_value = 0 ; // charging
				bat_check_cnt = 0 ; // charging
				erase_state_machine = SEND_QUIT ;
				Erase_Flash_All() ;
				Send_LoRa_Ack(ACK_QUIT) ;
				return true ;
			}
		}
		else if( LoRa_RXD_Buf[13] == LORA_RX_DATA_ERASE ){ 
		//===============================================    flash verifying
			switch(erase_state_machine){
				case VERIFY_PACKET1:
					//-------------------------------------------------
					if(Verify_Flash_Data(VERIFY_PACKET1)){
						erase_state_machine = VERIFY_PACKET2;
						Send_Flash_Data_to_SC(VERIFY_PACKET2) ;
					}
					else Send_Flash_Data_to_SC(VERIFY_PACKET1) ; 	
					timestamp_lora_start = get_RTC_timestamp(true);
					lora_rx_waittime = WAIT_TIME_1000ms ;										
					break;
					
				case VERIFY_PACKET2:
					//-------------------------------------------------
					if(Verify_Flash_Data(VERIFY_PACKET2)){
						erase_state_machine = VERIFY_PACKET3;
						Send_Flash_Data_to_SC(VERIFY_PACKET3) ;
					}
					else	Send_Flash_Data_to_SC(VERIFY_PACKET2) ;
					timestamp_lora_start = get_RTC_timestamp(true);
					lora_rx_waittime = WAIT_TIME_1000ms ;					
					break;
					
				case VERIFY_PACKET3:
					//-------------------------------------------------
					if(Verify_Flash_Data(VERIFY_PACKET3)){
						if(page_cnt_start == 0 ){
							flag_flash_send_finish = true ;
						}
						else{
							if( Read_Flash_Block_Data( page_cnt - (page_cnt_start--))  ) {
								erase_state_machine = VERIFY_PACKET1 ;
								Send_Flash_Data_to_SC(VERIFY_PACKET1) ;
								flag_flash_send_finish = false ;
							}
							else flag_flash_send_finish = true ;		
						}
					}
					else	Send_Flash_Data_to_SC(VERIFY_PACKET3) ;
					timestamp_lora_start = get_RTC_timestamp(true);
					lora_rx_waittime = WAIT_TIME_1000ms ;					
					break;
					
				case SEND_QUIT:
					break;
			}

			if( flag_flash_send_finish ){
				flag_flash_send_finish = false ;
				flag_charging_area = true ; // charging
				bat_average_value = 0 ; // charging
				bat_check_cnt = 0 ; // charging
				erase_state_machine = SEND_QUIT ;
				Erase_Flash_All() ;
				Send_LoRa_Ack(ACK_QUIT) ;
				return true ;			
			}
		}
	}
	return false ;				

}










