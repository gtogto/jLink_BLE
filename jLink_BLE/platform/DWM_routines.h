#ifndef DWM_ROUTINES_H_INCLUDED
#define DWM_ROUTINES_H_INCLUDED






#include <stdint.h>
#include <stdbool.h>


#define DWM_WAKEUP_LINE_ACTIVE_TIME 			550 //us
#define DWM_WAKEUP_LINE_ACTIVE_TICKS_RTC		21 //ticks
#define DWM_WAKEUP_XTAL_STAB_TIME		 		4 //ms
#define DWM_WAKEUP_LINE_ACTIVE_LONG_TIME 		1000 //us

#define PRF_16      (0)
#define PRF_64      (1)

#define DR_110k     (0)
#define DR_850k     (1)
#define DR_6M8      (2)

#define PREAM_L_2048 (2)
#define PREAM_L_1024 (3)
#define PREAM_L_256  (5)
#define PREAM_L_128  (6)



//void read_otp_params(void);


volatile bool blink_sent;

void  DWM_Setparameter(void);

void DWM1000_init(void);
void DWM1000_initAndDoHwResetProcedure(void);
uint32_t DWM_reconfigure(void);
void DMW1000_wake_up(void);
void DM1000_enter_sleep(void);
void DM1000_enter_deep_sleep(void);

//bool setUWB_RX_and_wait(systemValues_t *sysVal, bool rx_after_start);
//bool waitForConf(uint32_t sysconfbits, systemValues_t *sysVal, bool rx_after_start);
//void read_otp_params(void);
//uint32_t get_OTP_pwr_level(void);
//uint8_t get_OTP_PG_delay(void);
//void load_TX_params_from_OTP(void);


//This function is only for test purposes - comment it after debug
//void set_CFMode(void);
#endif /* DWM_ROUTINES_H_INCLUDED */
