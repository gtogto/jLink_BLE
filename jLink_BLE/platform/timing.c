

#include "timing.h"
#include "peripheral.h"
#include "nrf_drv_rtc.h"
#include "nrf_delay.h"
#include "../Teia_var.h"
#include <stdlib.h>

//================================================================================
void delay_us(uint32_t us)
{
    nrf_delay_us(us);
}

//================================================================================
void delay_ms(uint32_t ms)
{
    nrf_delay_ms(ms);
}

//================================================================================
void delay_and_sleep(uint32_t t, bool is_ms,bool use_deep_sleep)
{
    rtcDelayComplete[RTC_delay_instance][delay_CC_reg] = false;

    uint32_t time_us = (is_ms) ? (t * 1000UL) : (t);
    uint32_t ticks = convertTime2Ticks(time_us);

    if(ticks<= 1)
    {
        nrf_delay_us(time_us);
        return;
    }

    uint32_t rtccnt = (get_RTC_counter(RTC_delay_instance)) & RTC_TOP;
    RTC_CC_set(RTC_delay_instance,delay_CC_reg,(rtccnt+ticks)& RTC_TOP,true);

    if(use_deep_sleep)
    {
        while(rtcDelayComplete[RTC_delay_instance][delay_CC_reg] == false)
        {
            __WFI();
        }
    }
    else
    {
        while(!rtcDelayComplete[RTC_delay_instance][delay_CC_reg])__NOP();
    }
    return;

}

//================================================================================
uint32_t delay_and_sleep_tck(uint32_t ticks, uint32_t early, bool use_deep_sleep)
{
    rtcDelayComplete[RTC_delay_instance][delay_CC_reg] = false;
    uint32_t rtccnt = get_RTC_timestamp(true);
    uint32_t elapsedTicks = (rtccnt - early);
    uint32_t ticks_in_sleep = 0;
    if(elapsedTicks >= ticks)
    {
        return ticks_in_sleep;
    }
    else
    {
        uint32_t remainingTicks = ticks - elapsedTicks;

        if(remainingTicks<= 2UL)
        {
            nrf_delay_us(30*ticks);
            return 1UL;
        }
        else if(remainingTicks >= RTC_TOP)
        {
            RTC_CC_set(RTC_delay_instance,delay_CC_reg,(rtccnt-1) & RTC_TOP,true);
            remainingTicks -= RTC_TOP;
            ticks_in_sleep += RTC_TOP;
        }
        else
        {
            RTC_CC_set(RTC_delay_instance,delay_CC_reg,(rtccnt + remainingTicks) & RTC_TOP,true);
            ticks_in_sleep += remainingTicks;
            remainingTicks = 0;
        }

        while((!rtcDelayComplete[RTC_delay_instance][delay_CC_reg]) && (int_flag.wake_up != true))
        {
            if(use_deep_sleep) __WFI();
            else               __NOP();
        }

        if(remainingTicks) delay_and_sleep_tck(remainingTicks,get_RTC_timestamp(true),use_deep_sleep);
        else return ticks_in_sleep;
    }
    return ticks_in_sleep;

}
//================================================================================
/*
uint32_t RTC_set_CC_reg(uint32_t ticks, uint32_t early, bool use_deep_sleep)
{
    rtcDelayComplete[RTC_delay_instance][waiting_CC_reg] = false;
    uint32_t rtccnt = get_RTC_timestamp(true);
    uint32_t elapsedTicks = (rtccnt - early);
    uint32_t ticks_in_sleep = 0;
    if(elapsedTicks >= ticks)
    {
        return ticks_in_sleep;
    }
    else
    {
        uint32_t remainingTicks = ticks - elapsedTicks;

        if(remainingTicks<= 2UL)
        {
            nrf_delay_us(30*ticks);
            return 1UL;
        }
        else if(remainingTicks >= RTC_TOP)
        {
            RTC_CC_set(RTC_delay_instance,delay_CC_reg,(rtccnt-1) & RTC_TOP,true);
            remainingTicks -= RTC_TOP;
            ticks_in_sleep += RTC_TOP;
        }
        else
        {
            RTC_CC_set(RTC_delay_instance,delay_CC_reg,(rtccnt + remainingTicks) & RTC_TOP,true);
            ticks_in_sleep += remainingTicks;
            remainingTicks = 0;
        }

        while(!rtcDelayComplete[RTC_delay_instance][delay_CC_reg])
        {
            if(use_deep_sleep) __WFI();
            else               __NOP();
        }

        if(remainingTicks) delay_and_sleep_tck(remainingTicks,get_RTC_timestamp(true),use_deep_sleep);
        else return ticks_in_sleep;
    }
    return ticks_in_sleep;

}

*/

//================================================================================
uint32_t get_RTC_timestamp(bool r_32bit)
{
	uint32_t RTC_ticks;
	RTC_ticks = get_RTC_counter(RTC_delay_instance);
	if(r_32bit)
	RTC_ticks |= (RTCCNT_MSByte << 24);

	return(RTC_ticks);
}

//================================================================================
uint32_t get_RTCelapsedCount(uint32_t earlyCNT)
{
	uint32_t cnt = get_RTC_timestamp(true);

    return (cnt - earlyCNT);
}

//================================================================================
uint8_t set_sleep_time(uint32_t tcks, uint32_t early)
{
    uint8_t ret;
	if(tcks <= 1) return 0;
	else
    {
        if(tcks == 2) ret = 1;
        else ret = 2;
        RTC_CC_set(RTC_delay_instance,deep_sleep_CC_reg,get_RTC_counter(RTC_delay_instance)+tcks,true);
    }
	return ret;

}

//================================================================================
 void deep_sleep_RTC_IRQ_disable(void)
{
        RTC_CC_set(RTC_delay_instance,deep_sleep_CC_reg,0,false);
}

//================================================================================
uint32_t convertTime2Ticks(uint32_t time_us)
{
	uint32_t ticks = 0;
	ticks = (uint32_t) ((((float) RTC_FREQUENCY * (float) time_us) / 1000000.0)+0.5f);
	return ticks;
}

//================================================================================
float convertTicks2Time(uint32_t ticks)
{
	float time = 0;
	time = (float) ((float) ticks/(float) RTC_FREQUENCY);
	return time;
}

//================================================================================
int32_t calculate_deviation (uint32_t deviation_max)
{
    if(tdoaParameters.use_random_deviation)
    {
        int32_t deviation = rand()% ((deviation_max << 1)+1);
        deviation -= deviation_max;
        return deviation;
    }
    else return 0UL;

}

//================================================================================
uint32_t uniform_rand(uint32_t max_num)
{
    if (max_num == 0) return 0;
    if (max_num == RAND_MAX) return rand();
    if (max_num > RAND_MAX) return ~0UL;
    uint32_t width = RAND_MAX / max_num;
    uint32_t rand_max = width * max_num;
    uint32_t random = rand()% rand_max;
    random = random / width;
    return random;
}

//================================================================================
int32_t calculate_dev(uint32_t deviation_max)
{
    if(tdoaParameters.use_random_deviation)
    {
        int32_t deviation = uniform_rand(deviation_max << 1);
        deviation -= deviation_max;
        return deviation;
    }
    else return 0UL;

}
//================================================================================

