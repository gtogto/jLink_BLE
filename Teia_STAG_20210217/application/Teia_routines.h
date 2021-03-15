#ifndef  LMS_ROUTINE_H
#define  LMS_ROUTINE_H

#include <stdbool.h>
#include <stdint.h>

//#define SAVE_DEFAULT

void Variable_Reset(void) ;
bool Verify_Flash_Data(uint8_t num) ;
void Erase_Flash_All(void ) ;
bool Read_Flash_Block_Data(uint32_t block) ;
bool LoRa_Processor(void) ;
void LED_Blink(uint8_t  color, uint32_t ontime );

void System_Init(void) ;
void UWB_Reconfiguration(void);
void Set_LMS_Parameters(void) ;
void Reset_LMS_Parameters(void) ;
void start_UWB_TX(void) ;

void Flash_GPIO_init(void);
void UWB_GPIO_init(void);
void UWB_SPI_init(void);
void MEMORY_SPI_init(void);

void LORA_Setting(void);
void LORA_Sleep(void);
void LORA_Delay_time(void);
void LORA_SW_RESET(void);
void LORA_Wakeup(void);

void Send_LoRa_Ack(uint8_t cmd);
void Send_LoRa_Echo(void) ;
void Send_LoRa_Upload(void) ;
void Send_Flash_Data_to_SC(uint8_t num) ;

void Flash_SPI_pins_enable(void);
uint16_t Flash_SPI_init(uint16_t freq_kHz);
void flash_update(void) ;

void UWB_radio_wake_up(void);
void init_TDOA_params(void);
void configure_UWB(void);
bool RX_handler(bool first_rx);
void send_Blink(void);
void send_Register(void);
void TAG_sleep_STD(void);
void TAG_sleep_with_AHRS(void);
void button_action_handler(void);
uint8_t no_motion_handler(void);
void configure_sensors(void);
void check_batteryStatus(bool first_measure);
bool check_NFC_EEPROM(void);
void resetToDefault(void);
void loadSetting(void);
void shutdown_tag(void);
void sleep_while_charging(void);
bool check_new_setting(bool after_start_check);
void enter_bootloader(void);
void save_default_config(void);
void UWB_radio_enter_sleep(void);   
void save_new_config(void);

void ADC_Battery_check(void);
void Convert_Macaddress(void);
void ReConvert_Macaddress(void);

void ACC_Init(void);
void Acc_compare(void);
void acc_boundary_data(void);
//void init_Blinks(systemValues_t *systemValues);

//uint16_t Battery_counter ;

bool first_blink;
uint16_t on_start_infoblinks;
uint16_t count_msg1;
uint16_t count_msg2;
uint16_t count_msg3;




#endif
