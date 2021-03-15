#ifndef TIMING_H_INCLUDED
#define TIMING_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>
#include "sdk_config.h"

#define RTC_FREQUENCY       RTC_DEFAULT_CONFIG_FREQUENCY

#define ONE_SEC_RTC_TCKS	32768UL
#define TWO_SEC_RTC_TCKS	65536UL
#define FIVE_SEC_RTC_TCKS	163840UL

#define RR_DURING_CHARGE_TCKS                   ((RR_DURING_CHARGE_MS *RTC_FREQUENCY)/1000UL)

#define RTC_TOP             0x00FFFFFFUL  //maximum value of RTC counter



void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void delay_and_sleep(uint32_t t, bool is_ms,bool use_deep_sleep);
uint32_t get_RTC_timestamp(bool r_32bit);
uint32_t get_RTCelapsedCount(uint32_t earlyCNT);


uint8_t set_sleep_time(uint32_t tcks, uint32_t early);
void deep_sleep_RTC_IRQ_disable(void);
uint32_t convertTime2Ticks(uint32_t time_us);
float convertTicks2Time(uint32_t ticks);
uint32_t delay_and_sleep_tck(uint32_t ticks, uint32_t early, bool use_deep_sleep);
int32_t calculate_deviation (uint32_t deviation_max);
int32_t calculate_dev(uint32_t deviation_max);

#endif /* TIMING_H_INCLUDED */
