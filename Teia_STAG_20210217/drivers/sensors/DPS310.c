/**
 * @file DPS310.c
 * @brief DSP310 barometer library source
 * @date 23.1.2017
 * @author prachar.petr
 */
 /***************************************************************************//**
 * @addtogroup drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup sensors
 * @{
 ******************************************************************************/

 /***************************************************************************//**
 * @addtogroup barometer_driver
 * @{
 ******************************************************************************/


#include <stdbool.h>
#include <stdint.h>
#include "../../platform/timing.h"
#include "../../platform/peripheral.h"
#include "../../TEIA_var.h"
#include "DPS310.h"


//VARIABLES//
volatile float copm_scale_factors[8] =   { 524288.f,\
										   1572864.f,\
										   3670016.f,\
										   7864320.f,\
										   253952.f,\
										   516096.f,\
										   1040384.f,\
										   2088960.f };

volatile float c0, c1, c00, c10, c01, c11, c20, c21, c30;  // Calibration Coefficients

volatile uint8_t kT;		//comp_scale_factor index for temperature;
volatile uint8_t kP;		//comp_scale_factor index for pressure;

volatile uint8_t PRS_meas_time;

void dps310_read_data(uint32_t *PSR, uint8_t *TMP)
{
	uint8_t raw_data[6];
	uint32_t temp;
	uint8_t reg;

	int32_t pressure_raw = 0;
	int32_t temperature_raw =0;

	static float PSR_SC = 0.f;
	static float TMP_SC = 0.f;

	volatile static float pressure = 0.f;
	volatile static float temperature = 0.f;

	reg = I2CDRV_readFromRegister(DPS310_ADDR,DPS310_MEAS_CFG);

	if(((reg & DPS310_TMP_RDY_MASK)!=0) && ((reg & DPS310_PRS_RDY_MASK)!=0)) //check if both pressure and temperature is ready to read
	{
		I2CDRV_burst_readFromRegister(DPS310_ADDR,0x00,6,raw_data);														//read RAW pressure and temperature data
		temp = (((uint32_t)raw_data[0])<<16) | (((uint32_t)raw_data[1])<<8) | (((uint32_t)raw_data[2]));				//calc corrected pressure and temp value
		pressure_raw = (temp < (1UL << 23)) ? (int32_t)temp : ((int32_t)temp - (1UL << 24));							//
		temp = (((uint32_t)raw_data[3])<<16) | (((uint32_t)raw_data[4])<<8) | (((uint32_t)raw_data[5]));				//
		temperature_raw = (temp < (1UL << 23)) ? (int32_t)temp : ((int32_t)temp - (1UL << 24));							//
		PSR_SC = ((float)pressure_raw / copm_scale_factors[kP]);														//
		TMP_SC = ((float)temperature_raw / copm_scale_factors[kT]);														//
		temperature = c0 + c1*TMP_SC;
		pressure = c00 + PSR_SC * (c10  + PSR_SC * (c20 + PSR_SC * c30)) + TMP_SC *c01+TMP_SC*PSR_SC*(c11+PSR_SC*c21);	//
		pressure*=100;																									//
	}
	else if((reg & DPS310_PRS_RDY_MASK)!=0)			//check if pressure is ready to read
	{
		I2CDRV_burst_readFromRegister(DPS310_ADDR,0x00,3,raw_data);															//read RAW pressure data
		temp = (((uint32_t)raw_data[0])<<16) | (((uint32_t)raw_data[1])<<8) | (((uint32_t)raw_data[2]));					//calc corrected pressure
		pressure_raw = (temp < (1UL << 23)) ? (int32_t)temp : ((int32_t)temp - (1UL << 24));
		PSR_SC = ((float)pressure_raw / copm_scale_factors[kP]);
		pressure = c00 + PSR_SC * (c10  + PSR_SC * (c20 + PSR_SC * c30)) + TMP_SC *c01+TMP_SC*PSR_SC*(c11+PSR_SC*c21);
		pressure*=100;
	}
	else if((reg & DPS310_TMP_RDY_MASK)!=0)			//check if temperature is ready to read
	{
		I2CDRV_burst_readFromRegister(DPS310_ADDR,0x03,3,&raw_data[3]);										//read RAW temperature data
		temp = (((uint32_t)raw_data[3])<<16) | (((uint32_t)raw_data[4])<<8) | (((uint32_t)raw_data[5]));	//calc corrected temperature
		temperature_raw = (temp < (1UL << 23)) ? (int32_t)temp : ((int32_t)temp - (1UL << 24));
		TMP_SC = ((float)temperature_raw / copm_scale_factors[kT]);
		temperature = c0 + c1*TMP_SC;
	}


	*TMP = (uint8_t)((temperature+40)*2);
	*PSR = (uint32_t)pressure;



	//DebugMsg(DEVELOP,"P = %7d Pa   T = %4d °C",(uint32_t)pressure,(uint32_t)temperature);
}


bool dps310_test(void)
{
	uint8_t reg=0;
	reg = I2CDRV_readFromRegister(DPS310_ADDR,DPS_WHO_AM_I_REG);
	if (reg== DPS_WHO_AM_I_VAL) return true;
	else return false;
}

void dps310_set_standby(void)
{
	uint8_t reg=0;
	reg = I2CDRV_readFromRegister(DPS310_ADDR,DPS310_MEAS_CFG);
	I2CDRV_writeToRegister(DPS310_ADDR,DPS310_MEAS_CFG,(reg & ~DPS310_BOTH_CONT_MEAS));
	}

void dps310_set_active(void)
{
	uint8_t reg=0;
	reg = I2CDRV_readFromRegister(DPS310_ADDR,DPS310_MEAS_CFG);
	I2CDRV_writeToRegister(DPS310_ADDR,DPS310_MEAS_CFG,(reg | DPS310_BOTH_CONT_MEAS));
}

void dps310_set_measure(uint8_t TMP_rate,uint8_t TMP_osr,uint8_t PRS_rate,uint8_t PRS_osr, bool continous)
{
	uint8_t reg=0;
	//reg = I2CDRV_readFromRegister(DPS310_ADDR,DPS310_TMP_CFG);
    reg |= TMP_rate;
    reg |= TMP_osr;
    reg |= (0x80 & I2CDRV_readFromRegister(DPS310_ADDR,DPS310_COEF_SRCE));			//use internal temperature sensor - base on datasheet recognition
    //reg&= 0x7F;
	I2CDRV_writeToRegister(DPS310_ADDR,DPS310_TMP_CFG,reg);

	reg = I2CDRV_readFromRegister(DPS310_ADDR,DPS310_PRS_CFG);
	reg &= ~DPS310_CFG_PRC_MASK;
    reg &= ~DPS310_CFG_RATE_MASK;
    reg |= PRS_rate;
    reg |= PRS_osr;
	I2CDRV_writeToRegister(DPS310_ADDR,DPS310_PRS_CFG,reg);

	reg = I2CDRV_readFromRegister(DPS310_ADDR,DPS310_MEAS_CFG);
	reg &= ~DPS310_MEAS_CTRL_MASK;
	if(continous) reg |= DPS310_BOTH_CONT_MEAS;
	I2CDRV_writeToRegister(DPS310_ADDR,DPS310_MEAS_CFG,reg);


	kT = TMP_osr;
	kP = PRS_osr;

	reg = I2CDRV_readFromRegister(DPS310_ADDR,DPS310_CFG_REG);
	if(TMP_osr > DPS310_OSR_8)
	{
		reg |= DPS310_T_SHIFT_bit;
	}
	else
		reg &= ~DPS310_T_SHIFT_bit;

	if(PRS_osr > DPS310_OSR_8)
	{
		reg |= DPS310_P_SHIFT_BIT;
	}
	else
		reg &= ~DPS310_P_SHIFT_BIT;

	I2CDRV_writeToRegister(DPS310_ADDR,DPS310_CFG_REG,reg);
}


void dps310_read_coefs(void)
{
	uint8_t data[18];
	int32_t temp;
	uint8_t reg;
	uint8_t i=1;
	do
	{
		reg = I2CDRV_readFromRegister(DPS310_ADDR,0x08) & DPS310_COEF_RDY_MASK;
		delay_us(10000);
		i++;
	}while((!reg)&&(i));
	if(!i && !reg) sensors_params.BARO_mounted = false;

	I2CDRV_burst_readFromRegister(DPS310_ADDR,DPS310_COEF,18,data);

	temp = ((((uint32_t)data[0] << 4) | ((data[1] >> 4)&0x0F))) & 0x0FFF ;
	temp = (temp < (1UL << 11)) ? temp : (temp - (1UL << 12));
	c0 = (float)temp;
	c0 = c0/2;

	temp = (((uint32_t)data[1] << 8) | (uint32_t)data[2]) & 0x0FFF;
	temp = (temp < (1UL << 11)) ? temp : (temp - (1UL << 12));
	c1 = (float)temp;

	temp = (((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4)) & 0x000FFFFFUL;
	temp = (temp < (1UL << 19)) ? temp : (temp - (1UL << 20));
	c00 = (float)temp;

	temp = (((uint32_t)data[5] << 16) | ((uint32_t)data[6] << 8) | ((uint32_t)data[7])) & 0x000FFFFFUL;
	temp = (temp < (1UL << 19)) ? temp : (temp - (1UL << 20));
	c10 = (float)temp;

	temp = (((uint32_t)data[8] << 8) | ((uint32_t)data[9] )) & 0xFFFFUL ;
	temp = (temp < (1UL << 15)) ? temp : (temp - (1UL << 16));
	c01 = (float)temp;

	temp = (((uint32_t)data[10] << 8) | ((uint32_t)data[11] )) & 0xFFFFUL ;
	temp = (temp < (1UL << 15)) ? temp : (temp - (1UL << 16));
	c11 = (float)temp;

	temp = (((uint32_t)data[12] << 8) | ((uint32_t)data[13] )) & 0xFFFFUL ;
	temp = (temp < (1UL << 15)) ? temp : (temp - (1UL << 16));
	c20 = (float)temp;

	temp = (((uint32_t)data[14] << 8) | ((uint32_t)data[15] )) & 0xFFFFUL ;
	temp = (temp < (1UL << 15)) ? temp : (temp - (1UL << 16));
	c21 = (float)temp;

	temp = (((uint32_t)data[16] << 8) | ((uint32_t)data[17] )) & 0xFFFFUL ;
	temp = (temp < (1UL << 15)) ? temp : (temp - (1UL << 16));
	c30 = (float)temp;

}

void dps310_init(uint8_t setting)
{
	uint8_t TMP_rate;
	uint8_t TMP_osr;
	uint8_t PRS_rate;
	uint8_t PRS_osr;
	uint8_t rate = DPS310_RATE_1HZ;

    sensors_params.BARO_cont_mode = true;

	if(tdoaParameters.refresh_rate_ms <= 250)        rate = DPS310_RATE_8HZ;
	else if (tdoaParameters.refresh_rate_ms <= 500)  rate = DPS310_RATE_4HZ;
	else if (tdoaParameters.refresh_rate_ms <= 1000) rate = DPS310_RATE_2HZ;
	else if (tdoaParameters.refresh_rate_ms <= 2000) rate = DPS310_RATE_1HZ;
	else
    {
        rate = DPS310_RATE_1HZ;
        sensors_params.BARO_cont_mode = false;
    }

    PRS_rate = rate;
	TMP_rate = rate;
    PRS_meas_time = 4;
	switch (setting)
	{
					case 1:
						PRS_osr  = DPS310_OSR_16;
						//PRS_rate = DPS310_RATE_1HZ;
						//TMP_rate = DPS310_RATE_1HZ;
						PRS_meas_time = 6;
						break;

					case 2:
						PRS_osr  = DPS310_OSR_64;
						PRS_meas_time = 28;
						//PRS_rate = DPS310_RATE_4HZ;
						//TMP_rate = DPS310_RATE_4HZ;
						break;
					case 3:
						PRS_osr  = DPS310_OSR_128;
						PRS_meas_time = 105;
						//PRS_rate = DPS310_RATE_4HZ;
						//PRS_rate = 0x30;
						//TMP_rate = DPS310_RATE_4HZ;
						break;
					default:
						PRS_osr  = DPS310_OSR_128;
						PRS_meas_time = 105;
						//PRS_rate = 0x00;
						//TMP_rate = 0x00;
					    break;
	}
	TMP_osr = 0x00;			//temperature over sampling ratio is always zero

	//chip error workaround
	I2CDRV_writeToRegister(DPS310_ADDR,0x0E,0xA5);
	I2CDRV_writeToRegister(DPS310_ADDR,0x0F,0x96);
	I2CDRV_writeToRegister(DPS310_ADDR,0x62,0x02);
	I2CDRV_writeToRegister(DPS310_ADDR,0x0E,0x00);
	I2CDRV_writeToRegister(DPS310_ADDR,0x0F,0x00);
	/*end of chip error workaround*/

	dps310_set_measure(TMP_rate, TMP_osr, PRS_rate,PRS_osr,sensors_params.BARO_cont_mode);
}

void dps310_meas_prs(void)
{
    uint8_t reg;
    reg = I2CDRV_readFromRegister(DPS310_ADDR,DPS310_MEAS_CFG);
	reg |= DPS310_PRS_SINGLE_MEAS;
	I2CDRV_writeToRegister(DPS310_ADDR,DPS310_MEAS_CFG,reg);
	delay_and_sleep(PRS_meas_time,true,true);
	while(!(I2CDRV_readFromRegister(DPS310_ADDR,DPS310_MEAS_CFG)&DPS310_PRS_RDY_MASK));
}

void dps310_meas_tmp(void)
{
    uint8_t reg;
    reg = I2CDRV_readFromRegister(DPS310_ADDR,DPS310_MEAS_CFG);
	reg |= DPS310_TMP_SINGLE_MEAS;
	I2CDRV_writeToRegister(DPS310_ADDR,DPS310_MEAS_CFG,reg);
	delay_and_sleep(4,true,true);
	while(!(I2CDRV_readFromRegister(DPS310_ADDR,DPS310_MEAS_CFG)&DPS310_TMP_RDY_MASK));
}

void dps310_reset(void)
{
    delay_and_sleep(4,true,true);
    uint8_t reg;
    reg = I2CDRV_readFromRegister(DPS310_ADDR,0x0C);
    reg &= 0xF0;
    reg |= 0b00001001;
    I2CDRV_writeToRegister(DPS310_ADDR,0x0C,reg);
}

/** @} (end group barometer_driver) */
/** @} (end group sensors) */
/** @} (end group drivers) */

