/***************************************************************************//**
 * @file mpu9250.c
 * @brief MPU-9250 9-axis IMU sensor library source
 ******************************************************************************/
/* Includes */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "mpu9250.h"


#include "../../platform/timing.h"
#include "../../platform/peripheral.h"

#include "../../application/sensors_routines.h"
#include "../../TEIA_var.h"


#define PI 3.14159265358979323846f

 /***************************************************************************//**
 * @addtogroup drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup sensors
 * @{
 ******************************************************************************/

 /***************************************************************************//**
 * @addtogroup ACC_GYRO_MAG_driver
 * @{
 ******************************************************************************/





static float gyroSense;
static float accSense;
static float magSense;

static void InvertMatrix(double m[16], double *invOut);

/* Functions */

/**************************************************************************//**
 * @brief  Test communication with mpu9250
 *****************************************************************************/

bool mpu9250_test(void)
{
	uint8_t ID = I2CDRV_readFromRegister(MPU9250_ADDR,WHO_AM_I_REG);
	if (ID == WHO_AM_I_VAL) //0x3A
    {
		return true; //GPIO_PinOutSet(USER_LED);
	}
	else
    {
		return false; //GPIO_PinOutClear(USER_LED);
	}
}

/**************************************************************************//**
 * @brief  Reset mpu9250
 *****************************************************************************/

void mpu9250_reset(void)
{
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay_and_sleep(100000,false,false);
}
/**************************************************************************//**
 * @brief  Initialize mpu9250 into default setting - enable ACC, GYRO, and MAGNETOMETER and set clock source
 *****************************************************************************/

void mpu9250_init_all(void)
{
	//mpu9250_reset();

	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_1,0x08);	//internal osc

	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_2,0xC0);	//enable all axes for acc and gyro

	/**magnetometer setting*/
	mpu9250_bypass_I2C(); 							// connect primary and auxillary I2C together
	delay_and_sleep(200000,false,false);
	uint8_t ODR;
    if(sensors_params.AHRS_enable) ODR = AK8963_8_Hz;
	else
    {
        sensors_params.MAG_cont_mode = true;
        if(tdoaParameters.refresh_rate_ms >= 1000)
        {
            ODR = AK8963_SINGLE_MEAS;
            sensors_params.MAG_cont_mode = false;

        }
        else if(tdoaParameters.refresh_rate_ms >= 125) ODR = AK8963_8_Hz;
        else ODR = AK8963_100_Hz;
    }
    AK8963_init(ODR,AK8963_RES_16b);
	/**end of magnetometer setting*/


	I2CDRV_writeToRegister(MPU9250_ADDR,CONF,0x01);			//disable FSYNC mode and set LP filter to bandwidth = 184 kHz (for Fs=1kHz)

	uint8_t reg = I2CDRV_readFromRegister(MPU9250_ADDR,GYRO_CONF);
	I2CDRV_writeToRegister(MPU9250_ADDR,GYRO_CONF, ((reg & 0x1C) | 0x00));  	//clear gyro self-test bits and set internal sample rate to 1 kHz

	reg = I2CDRV_readFromRegister(MPU9250_ADDR,ACC_CONF1);
	I2CDRV_writeToRegister(MPU9250_ADDR,ACC_CONF1, ((reg & 0x1F) | 0x00));

	reg = I2CDRV_readFromRegister(MPU9250_ADDR,ACC_CONF2);
	I2CDRV_writeToRegister(MPU9250_ADDR,ACC_CONF2, ((reg & 0xF0) | 0x00));  	//clear acc self-test bits and set internal sample rate to 1 kHz



	I2CDRV_writeToRegister(MPU9250_ADDR,SMPLRT_DIV, 0x04);						//internal sample rate will by divided by 5, Fs=200 Hz
}


/**************************************************************************//**
 * @brief  Initialize mpu9250_accelerometer
 *****************************************************************************/

void mpu9250_init_acc(void)
{
	//mpu9250_reset();

	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_1,0x08);	//internal osc enable

	uint8_t reg = I2CDRV_readFromRegister(MPU9250_ADDR,PWR_MGMT_2);
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_2, reg & 0xC7);   //enable acc (all axes)

	reg = I2CDRV_readFromRegister(MPU9250_ADDR,CONF);
	reg &= 0x80;
	reg |= 0x01;
	I2CDRV_writeToRegister(MPU9250_ADDR,CONF,reg);			//disable FSYNC mode and set LP filter to bandwidth = 184 kHz (for Fs=1kHz)

	reg = I2CDRV_readFromRegister(MPU9250_ADDR,ACC_CONF1);
	I2CDRV_writeToRegister(MPU9250_ADDR,ACC_CONF1, ((reg & 0x1F) | 0x00));

	reg = I2CDRV_readFromRegister(MPU9250_ADDR,ACC_CONF2);
	I2CDRV_writeToRegister(MPU9250_ADDR,ACC_CONF2, ((reg & 0xF0) | 0x00));  	//clear acc self-test bits and set internal sample rate to 1 kHz

	I2CDRV_writeToRegister(MPU9250_ADDR,SMPLRT_DIV, 0x04);						//internal sample rate will by divided by 5, Fs=200 Hz
}

/**************************************************************************//**
 * @brief  Initialize mpu9250_magnetometer
 *****************************************************************************/

void mpu9250_init_mag(void)
{
	mpu9250_bypass_I2C(); 							// connect primary and auxillary I2C together
	delay_us(200000);
	uint8_t ODR;
	if(sensors_params.AHRS_enable) ODR = AK8963_8_Hz;
	else
	sensors_params.MAG_cont_mode = true;
	{
		if(tdoaParameters.refresh_rate_ms >= 1000)
		{
			ODR = AK8963_SINGLE_MEAS;
			sensors_params.MAG_cont_mode = false;

		}
		else if(tdoaParameters.refresh_rate_ms >= 125) ODR = AK8963_8_Hz;
		else ODR = AK8963_100_Hz;
	}
	AK8963_init(ODR,AK8963_RES_16b);
}

/**************************************************************************//**
 * @brief  Initialize mpu9250_gyroscope
 *****************************************************************************/

void mpu9250_init_gyro(void)
{
	//mpu9250_reset();

	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_1,0x08);	//internal osc

	uint8_t reg = I2CDRV_readFromRegister(MPU9250_ADDR,PWR_MGMT_2);
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_2, reg & 0xF8);

	reg = I2CDRV_readFromRegister(MPU9250_ADDR,CONF);
	reg &= 0x80;
	reg |= 0x01;
	I2CDRV_writeToRegister(MPU9250_ADDR,CONF,reg);			//disable FSYNC mode and set LP filter to bandwidth = 184 kHz (for Fs=1kHz)

	reg = I2CDRV_readFromRegister(MPU9250_ADDR,GYRO_CONF);
	I2CDRV_writeToRegister(MPU9250_ADDR,GYRO_CONF, ((reg & 0x1C) | 0x00));  	//clear gyro self-test bits and set internal sample rate to 1 kHz

	I2CDRV_writeToRegister(MPU9250_ADDR,SMPLRT_DIV, 0x04);						//internal sample rate will by divided by 5, Fs=200 Hz
}

void mpu9250_dis_gyro(void)
{
	uint8_t reg = I2CDRV_readFromRegister(MPU9250_ADDR,PWR_MGMT_2);
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_2, reg | 0x07);
}

void mpu9250_en_gyro(void)
{
	uint8_t reg = I2CDRV_readFromRegister(MPU9250_ADDR,PWR_MGMT_2);
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_2, reg & 0xF8);
}

void mpu9250_dis_acc(void)
{
	uint8_t reg = I2CDRV_readFromRegister(MPU9250_ADDR,PWR_MGMT_2);
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_2, reg | 0x38);
}

void mpu9250_en_acc(void)
{
	uint8_t reg = I2CDRV_readFromRegister(MPU9250_ADDR,PWR_MGMT_2);
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_2, reg & 0xC7);
}

void mpu9250_dis_mag(void)
{

	I2CDRV_writeToRegister(AK8963_ADDR,AK8963_CTRL1,AK8963_PWR_DOWN);
}

void mpu9250_en_mag(void)
{
	uint8_t ODR;
	if(tdoaParameters.refresh_rate_ms >= 1000)
    {
        ODR = AK8963_SINGLE_MEAS;
    }
	else if(tdoaParameters.refresh_rate_ms >= 125) ODR = AK8963_8_Hz;
	else ODR = AK8963_100_Hz;

	AK8963_init(ODR,AK8963_RES_16b);

}

void mpu9250_disable(void)
{
	I2CDRV_writeToRegister(AK8963_ADDR,AK8963_CTRL1,AK8963_PWR_DOWN);
	//I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_2,0xFF);
	//I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_1,0x4F);
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_2,0x3F);
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_1,0x48);
}

void mpu9250_enable_all(void)
{
	mpu9250_en_mag();
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_2,0x00);
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_1,0x00);
}

void mpu9250_clk_enable(void)
{
    I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_1,0x00);
}

void mpu9250_set_sample_rate(uint8_t rate)
{
	I2CDRV_writeToRegister(MPU9250_ADDR,SMPLRT_DIV, rate);
}

void mpu9250_WOM(uint8_t LP_ODR, uint8_t threshold)
{
	uint8_t reg = I2CDRV_readFromRegister(MPU9250_ADDR,PWR_MGMT_1);
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_1, reg & 0x8F);			//clr sleep, cycle and gyro_stadby

	reg = I2CDRV_readFromRegister(MPU9250_ADDR,PWR_MGMT_2);
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_2, (reg & 0xC0)|0x00);		//all acc axes enable, all gyro axis disaable  !!!

	reg = I2CDRV_readFromRegister(MPU9250_ADDR,ACC_CONF2);
	I2CDRV_writeToRegister(MPU9250_ADDR,ACC_CONF1,(reg & 0xF0) | 0x01 );  	//accel - LP filter BW = 184 Hz

	I2CDRV_writeToRegister(MPU9250_ADDR,INT_PIN_CFG,0x02);					//0x02 INT pin is active only for 50us   //0x22 INT pin is active until INT src reg is not readed

	I2CDRV_writeToRegister(MPU9250_ADDR,INT_ENABLE,0x40);					//enable Wake on Motiont interupt

	reg = I2CDRV_readFromRegister(MPU9250_ADDR,ACCEL_INTEL_CTRL);
	I2CDRV_writeToRegister(MPU9250_ADDR,ACCEL_INTEL_CTRL,reg | 0xC0);		//enable transient mode -  Compare the current sample with the previous sample


	I2CDRV_writeToRegister(MPU9250_ADDR,WOM_THR,threshold);

	reg = I2CDRV_readFromRegister(MPU9250_ADDR,LP_ACCEL_ODR);
	reg &= 0xF0;
	reg |= LP_ODR;
	I2CDRV_writeToRegister(MPU9250_ADDR,LP_ACCEL_ODR,reg);
}

uint8_t mpu9250_get_int_src(void)
{
	return I2CDRV_readFromRegister(MPU9250_ADDR,INT_STATUS);
}

/**************************************************************************//**
 * @brief  Test communication with mpu9250's magnetometer
 *****************************************************************************/

bool AK8963_test(void)
{
	uint8_t ID = I2CDRV_readFromRegister(AK8963_ADDR,0x00);
	if (ID == 0x48) {  //value of who am i register
		return true;
	} else {
		return false;
	}
}

/**************************************************************************//**
 * @brief  Read of acceleration raw data from mpu9250
 *****************************************************************************/
void mpu9250_read_raw_acc_data() //read measured acceleration
{
		uint8_t data[6];
		I2CDRV_burst_readFromRegister(MPU9250_ADDR,ACCEL_XOUT_H, 6, data);
		sensors_data.acc_data.x_axis = ((((int16_t) data[0]) << 8) | data[1]);
		sensors_data.acc_data.y_axis = ((((int16_t) data[2]) << 8) | data[3]);
		sensors_data.acc_data.z_axis = ((((int16_t) data[4]) << 8) | data[5]);
}

/**************************************************************************//**
 * @brief  Read of acceleration raw data from mpu9250
 *****************************************************************************/
void mpu9250_read_corr_acc_data() //read measured acceleration
{
		uint8_t data[6];
		I2CDRV_burst_readFromRegister(MPU9250_ADDR,ACCEL_XOUT_H, 6, data);
		sensors_data.acc_data.x_axis = ((((int16_t) data[0]) << 8) | data[1]) - sensors_data.acc_offsets_SC.x_axis;
		sensors_data.acc_data.y_axis = ((((int16_t) data[2]) << 8) | data[3]) - sensors_data.acc_offsets_SC.y_axis;
		sensors_data.acc_data.z_axis = ((((int16_t) data[4]) << 8) | data[5]) - sensors_data.acc_offsets_SC.z_axis;
}


/**************************************************************************//**
 * @brief  Read of acceleration corrected data from mpu9250
 *****************************************************************************/
/*void mpu9250_read_corr_acc_data(corr_data_9ax_t *corr_data) //read measured acceleration
{
		uint8_t data[6];
		I2CDRV_burst_readFromRegister(MPU9250_ADDR,ACCEL_XOUT_H, 6, data);
		int16_t xaxis = ((((int16_t) data[0]) << 8) | data[1]);
		int16_t yaxis = ((((int16_t) data[2]) << 8) | data[3]);
		int16_t zaxis = ((((int16_t) data[4]) << 8) | data[5]);

		corr_data->x_acc=(float)xaxis*accSense;
		corr_data->y_acc=(float)yaxis*accSense;
		corr_data->z_acc=(float)zaxis*accSense;
}
*/


/**************************************************************************//**
 * @brief  Read of gyro raw data from mpu9250
 *****************************************************************************/
void mpu9250_read_raw_gyro_data(void) //read measured acceleration
{
		uint8_t data[6];
		I2CDRV_burst_readFromRegister(MPU9250_ADDR,GYRO_XOUT_H, 7, data);
		sensors_data.gyro_data.x_axis = ((((int16_t) data[0]) << 8) | data[1]);
		sensors_data.gyro_data.y_axis = ((((int16_t) data[2]) << 8) | data[3]);
		sensors_data.gyro_data.z_axis = ((((int16_t) data[4]) << 8) | data[5]);
}


/**************************************************************************//**
 * @brief  Read of gyro raw data from mpu9250
 *****************************************************************************/
void mpu9250_read_corr_gyro_data(void) //read measured acceleration
{
		uint8_t data[6];
		I2CDRV_burst_readFromRegister(MPU9250_ADDR,GYRO_XOUT_H, 7, data);
		sensors_data.gyro_data.x_axis = ((((int16_t) data[0]) << 8) | data[1])-sensors_data.gyro_offsets_SC.x_axis;
		sensors_data.gyro_data.y_axis = ((((int16_t) data[2]) << 8) | data[3])-sensors_data.gyro_offsets_SC.y_axis;
		sensors_data.gyro_data.z_axis = ((((int16_t) data[4]) << 8) | data[5])-sensors_data.gyro_offsets_SC.z_axis;
}


/**************************************************************************//**
 * @brief  Read of magnetometer raw data from AK8963
 *****************************************************************************/
void AK8963_read_raw_mag_data() //read measured magnetic field
{
		uint8_t data[7];

		I2CDRV_burst_readFromRegister(AK8963_ADDR,MAG_XOUT_L, 7, data);
		sensors_data.mag_data.x_axis = ((((int16_t) data[1]) << 8) | data[0]);
		sensors_data.mag_data.y_axis = ((((int16_t) data[3]) << 8) | data[2]);
		sensors_data.mag_data.z_axis = ((((int16_t) data[5]) << 8) | data[4]);
}


/**************************************************************************//**
 * @brief  Read of magnetometer corrected data from AK8963
 *****************************************************************************/
void AK8963_read_corr_mag_data() //read measured magnetic field
{
		uint8_t data[7];

		I2CDRV_burst_readFromRegister(AK8963_ADDR,MAG_XOUT_L, 7, data);
		sensors_data.mag_data.x_axis = ((((int16_t) data[1]) << 8) | data[0])- sensors_data.mag_offsets.x_axis;
		sensors_data.mag_data.y_axis = ((((int16_t) data[3]) << 8) | data[2])- sensors_data.mag_offsets.y_axis;
		sensors_data.mag_data.z_axis = ((((int16_t) data[5]) << 8) | data[4])- sensors_data.mag_offsets.z_axis;
}


/**************************************************************************//**
 * @brief  Read 9DOF raw data from mpu9250
 *****************************************************************************/
void mpu9250_read_raw_data(int_data_9ax_t *meas_data)
{
		uint8_t data[14];
		I2CDRV_burst_readFromRegister(MPU9250_ADDR,ACCEL_XOUT_H, 14, data);
		meas_data->x_acc = ((((int16_t) data[0]) << 8) | data[1]);
		meas_data->y_acc = ((((int16_t) data[2]) << 8) | data[3]);
		meas_data->z_acc = ((((int16_t) data[4]) << 8) | data[5]);

		meas_data->x_gyro = ((((int16_t) data[8]) << 8) | data[9]);
		meas_data->y_gyro = ((((int16_t) data[10]) << 8) | data[11]);
		meas_data->z_gyro = ((((int16_t) data[12]) << 8) | data[13]);

		I2CDRV_burst_readFromRegister(AK8963_ADDR,MAG_XOUT_L, 7, data);
		meas_data->x_mag = ((((int16_t) data[1]) << 8) | data[0]);
		meas_data->y_mag = ((((int16_t) data[3]) << 8) | data[2]);
		meas_data->z_mag = ((((int16_t) data[5]) << 8) | data[4]);
}

/**************************************************************************//**
 * @brief  Read 6DOF raw data from mpu9250
 *****************************************************************************/
void mpu9250_read_raw_data_IMU(int_data_6ax_t *meas_data)
{
		uint8_t data[14];
		I2CDRV_burst_readFromRegister(MPU9250_ADDR,ACCEL_XOUT_H, 14, data);
		meas_data->x_acc = ((((int16_t) data[0]) << 8) | data[1]);
		meas_data->y_acc = ((((int16_t) data[2]) << 8) | data[3]);
		meas_data->z_acc = ((((int16_t) data[4]) << 8) | data[5]);

		meas_data->x_gyro = ((((int16_t) data[8]) << 8) | data[9]);
		meas_data->y_gyro = ((((int16_t) data[10]) << 8) | data[11]);
		meas_data->z_gyro = ((((int16_t) data[12]) << 8) | data[13]);
}

void mpu9250_IMU_calib(void)
{
	uint16_t i=0;
	int_data_6ax_t meas_data;
	int32_t acc_xoffs=0, acc_yoffs=0, acc_zoffs=0;
	int32_t gyro_xoffs=0, gyro_yoffs=0, gyro_zoffs=0;

	int16_t gravity;

	mpu9250_init_all();
	mpu9250_set_full_scales(GYRO_FS_250dps, ACC_FS_2g);

	if      (acc_full_scale == ACC_FS_2g) gravity=0x3FFF;
	else if (acc_full_scale == ACC_FS_4g) gravity=0x1FFF;
	else if (acc_full_scale == ACC_FS_8g) gravity=0x0FFF;
	else  /*(acc_full_scale ==ACC_FS_16g)*/ gravity=0x07FF;
	gravity*=-1;





	for (i = 0; i < 2048; i++)
	{
		mpu9250_read_raw_data_IMU(&meas_data);
		acc_xoffs += (int32_t)meas_data.x_acc;
		acc_yoffs += (int32_t)meas_data.y_acc;
		acc_zoffs += (int32_t)(meas_data.z_acc+gravity);
		gyro_xoffs+= (int32_t)meas_data.x_gyro;
		gyro_yoffs+= (int32_t)meas_data.y_gyro;
		gyro_zoffs+= (int32_t)meas_data.z_gyro;
		WDOG_Feed();
	}

	sensors_data.acc_offsets.x_axis =(int16_t)(acc_xoffs  >> 11);
	sensors_data.acc_offsets.y_axis= (int16_t)(acc_yoffs  >> 11);
	sensors_data.acc_offsets.z_axis =(int16_t)(acc_zoffs  >> 11);
	sensors_data.gyro_offsets.x_axis = (int16_t)(gyro_xoffs >> 11);
	sensors_data.gyro_offsets.y_axis = (int16_t)(gyro_yoffs >> 11);
	sensors_data.gyro_offsets.z_axis = (int16_t)(gyro_zoffs >> 11);
}

/**************************************************************************//**
 * @brief  Read 9DOF raw data from mpu9250
 *****************************************************************************/
void mpu9250_read_corr_data(corr_data_9ax_t *corr_data)
{
	int_data_9ax_t raw_data;
	mpu9250_read_raw_data(&raw_data);
	corr_data->x_acc=(float)(raw_data.x_acc-sensors_data.acc_offsets_SC.x_axis)*accSense;
	corr_data->y_acc=(float)(raw_data.y_acc-sensors_data.acc_offsets_SC.y_axis)*accSense;
	corr_data->z_acc=(float)(raw_data.z_acc-sensors_data.acc_offsets_SC.z_axis)*accSense;

	corr_data->x_gyro=(float)(raw_data.x_gyro-sensors_data.gyro_offsets_SC.x_axis)*gyroSense;
	corr_data->y_gyro=(float)(raw_data.y_gyro-sensors_data.gyro_offsets_SC.y_axis)*gyroSense;
	corr_data->z_gyro=(float)(raw_data.z_gyro-sensors_data.gyro_offsets_SC.z_axis)*gyroSense;

	corr_data->x_mag=(float)(raw_data.x_mag-sensors_data.mag_offsets.x_axis)*magSense;
	corr_data->y_mag=(float)(raw_data.y_mag-sensors_data.mag_offsets.y_axis)*magSense;
	corr_data->z_mag=(float)(raw_data.z_mag-sensors_data.mag_offsets.z_axis)*magSense;
}

/**************************************************************************//**
 * @brief  Read 6DOF raw data from mpu9250
 *****************************************************************************/
void mpu9250_read_corr_data_6DOF(corr_data_9ax_t *corr_data)
{
	int_data_6ax_t raw_data;
	mpu9250_read_raw_data_IMU(&raw_data);
	corr_data->x_acc=(float)(raw_data.x_acc-sensors_data.acc_offsets.x_axis)*accSense;
	corr_data->y_acc=(float)(raw_data.y_acc-sensors_data.acc_offsets.y_axis)*accSense;
	corr_data->z_acc=(float)(raw_data.z_acc-sensors_data.acc_offsets.z_axis)*accSense;

	corr_data->x_gyro=(float)(raw_data.x_gyro-sensors_data.gyro_offsets.x_axis)*gyroSense;
	corr_data->y_gyro=(float)(raw_data.y_gyro-sensors_data.gyro_offsets.y_axis)*gyroSense;
	corr_data->z_gyro=(float)(raw_data.z_gyro-sensors_data.gyro_offsets.z_axis)*gyroSense;
}





/**************************************************************************//**
 * @brief  Bypass the auxilary I2C to main I2C mpu9250
 ******************************************************************************/
void mpu9250_bypass_I2C(void)
{
		uint8_t data;
		data = I2CDRV_readFromRegister(MPU9250_ADDR,BYPASS_INC_CONF);
		I2CDRV_writeToRegister(MPU9250_ADDR,BYPASS_INC_CONF,data|0x02);
}

void AK8963_init(uint8_t mode,bool resolution)
{

	if(resolution)
	{
		I2CDRV_writeToRegister(AK8963_ADDR,AK8963_CTRL1,0x10|mode);
		magSense = 0.15f;
	}
	else
	{
		I2CDRV_writeToRegister(AK8963_ADDR,AK8963_CTRL1,0x00|mode);
		magSense = 0.6f;
	}

}

static void AK8963_start_meas(void)
{
    uint8_t reg;
    reg = I2CDRV_readFromRegister(AK8963_ADDR,AK8963_CTRL1);
    reg &= 0xF0;
    reg |= AK8963_SINGLE_MEAS;
    I2CDRV_writeToRegister(AK8963_ADDR,AK8963_CTRL1,reg);
}

static bool AK8963_data_ready(void)
{
    uint8_t reg = I2CDRV_readFromRegister(AK8963_ADDR,AK8963_STATUS1);
    if(reg & AK8963_DATA_RDY) return true;
    else return false;
}

void AK8963_do_meas(void)
{
    AK8963_start_meas();
    delay_and_sleep(20,true,true);
    while(!AK8963_data_ready());
}

void mpu9250_set_full_scales(uint8_t gyro_fs, uint8_t acc_fs)
{
	uint8_t data=I2CDRV_readFromRegister(MPU9250_ADDR,ACC_CONF1);
	I2CDRV_writeToRegister(MPU9250_ADDR,ACC_CONF1,(data&ACC_FS_mask)|acc_fs);
	data=I2CDRV_readFromRegister(MPU9250_ADDR,GYRO_CONF);
	I2CDRV_writeToRegister(MPU9250_ADDR,GYRO_CONF,(data&GYRO_FS_mask)|gyro_fs);

	if     (gyro_fs==GYRO_FS_250dps) gyroSense= 250.0f/32768.f;
	else if(gyro_fs==GYRO_FS_500dps) gyroSense= 500.0f/32768.f;
	else if(gyro_fs==GYRO_FS_1000dps)gyroSense=1000.0f/32768.f;
	else if(gyro_fs==GYRO_FS_2000dps)gyroSense=2000.0f/32768.f;

	gyroSense*= (PI/180);

	if     (acc_fs==ACC_FS_2g) accSense = 2000.0f/32768;
	else if(acc_fs==ACC_FS_4g) accSense = 4000.0f/32768;
	else if(acc_fs==ACC_FS_8g) accSense = 8000.0f/32768;
	else if(acc_fs==ACC_FS_16g)accSense =16000.0f/32768;

	acc_full_scale = acc_fs;
}

void mpu9250_set_LPmode(void)
{
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_1,0b00100000); //cycle mode
}

void mpu9250_set_normal_mode(void)
{
	I2CDRV_writeToRegister(MPU9250_ADDR,PWR_MGMT_1,0b00000000);
}

void mag_calib_AK8963(uint8_t duration_s,uint8_t delay_s)
{
    uint32_t i=0,j=0,k=0;
    int32_t mag_data[4];

    uint32_t time_cs = (uint32_t)duration_s * 100UL; //time in centiseconds
    uint32_t wait_time = convertTime2Ticks(10000);		 //time between two magnetometer acquisition = 10000 us

    AK8963_init(AK8963_100_Hz,AK8963_RES_16b);
    double vect[4];
    double mat2vect[16];

    mag_data[3] = 1;


    for(i = 0; i < delay_s; i++)
    {
    	LED1_TOGGLE();
    	delay_and_sleep(1000,true,false);
    }
    LED1_OFF();

    for(j=0;j<16;j++)
    {
        mat2vect[j]=0;
        if(j<4) vect[j] = 0;
    }

    for(i=0;i<time_cs;i++)
    {
    	uint32_t timestamp = get_RTC_timestamp(true);
    	if(!(i % 5)) LED1_TOGGLE();
        AK8963_read_raw_mag_data();
        mag_data[0]=(int32_t)sensors_data.mag_data.x_axis;
        mag_data[1]=(int32_t)sensors_data.mag_data.y_axis;
        mag_data[2]=(int32_t)sensors_data.mag_data.z_axis;

        double Y = (double)(mag_data[0]*mag_data[0]+mag_data[1]*mag_data[1]+mag_data[2]*mag_data[2]);
        for(j=0;j<4;j++)
        {
            for(k=0;k<4;k++)
            {
                mat2vect[4*j+k]+= (double)mag_data[j]*((double)mag_data[k]);
            }
            vect[j] += (double)mag_data[j]*Y;
        }
        WDOG_Feed();
        delay_and_sleep_tck(wait_time,timestamp,false);
    }

    InvertMatrix(mat2vect,mat2vect);

    double BetaX= mat2vect[0]*vect[0]+mat2vect[1]*vect[1]+mat2vect[2]*vect[2]+mat2vect[3]*vect[3];
    double BetaY= mat2vect[4]*vect[0]+mat2vect[5]*vect[1]+mat2vect[6]*vect[2]+mat2vect[7]*vect[3];
    double BetaZ= mat2vect[8]*vect[0]+mat2vect[9]*vect[1]+mat2vect[10]*vect[2]+mat2vect[11]*vect[3];
    double Beta = mat2vect[12]*vect[0]+mat2vect[13]*vect[1]+mat2vect[14]*vect[2]+mat2vect[15]*vect[3];
    double flux_density = (Beta + pow(BetaX/2,2.f) +  pow(BetaY/2,2.f) +  pow(BetaZ/2,2.f));
    flux_density = (sqrt(flux_density));

    sensors_data.mag_offsets.x_axis = ((int16_t)BetaX+1)>>1;
    sensors_data.mag_offsets.y_axis = ((int16_t)BetaY+1)>>1;
    sensors_data.mag_offsets.z_axis = ((int16_t)BetaZ+1)>>1;
    sensors_data.geo_mag_flux = (uint16_t)flux_density;



    return;
}


static void InvertMatrix(double m[16], double *invOut)
{
    double inv[16], det;
    uint16_t i;

    inv[0] = m[5]  * m[10] * m[15] -
             m[5]  * m[11] * m[14] -
             m[9]  * m[6]  * m[15] +
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] -
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
              m[4]  * m[11] * m[14] +
              m[8]  * m[6]  * m[15] -
              m[8]  * m[7]  * m[14] -
              m[12] * m[6]  * m[11] +
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
             m[4]  * m[11] * m[13] -
             m[8]  * m[5] * m[15] +
             m[8]  * m[7] * m[13] +
             m[12] * m[5] * m[11] -
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] -
               m[8]  * m[6] * m[13] -
               m[12] * m[5] * m[10] +
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
              m[1]  * m[11] * m[14] +
              m[9]  * m[2] * m[15] -
              m[9]  * m[3] * m[14] -
              m[13] * m[2] * m[11] +
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
             m[0]  * m[11] * m[14] -
             m[8]  * m[2] * m[15] +
             m[8]  * m[3] * m[14] +
             m[12] * m[2] * m[11] -
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
              m[0]  * m[11] * m[13] +
              m[8]  * m[1] * m[15] -
              m[8]  * m[3] * m[13] -
              m[12] * m[1] * m[11] +
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
              m[0]  * m[10] * m[13] -
              m[8]  * m[1] * m[14] +
              m[8]  * m[2] * m[13] +
              m[12] * m[1] * m[10] -
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
             m[1]  * m[7] * m[14] -
             m[5]  * m[2] * m[15] +
             m[5]  * m[3] * m[14] +
             m[13] * m[2] * m[7] -
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
              m[0]  * m[7] * m[14] +
              m[4]  * m[2] * m[15] -
              m[4]  * m[3] * m[14] -
              m[12] * m[2] * m[7] +
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
              m[0]  * m[7] * m[13] -
              m[4]  * m[1] * m[15] +
              m[4]  * m[3] * m[13] +
              m[12] * m[1] * m[7] -
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
               m[0]  * m[6] * m[13] +
               m[4]  * m[1] * m[14] -
               m[4]  * m[2] * m[13] -
               m[12] * m[1] * m[6] +
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
              m[1] * m[7] * m[10] +
              m[5] * m[2] * m[11] -
              m[5] * m[3] * m[10] -
              m[9] * m[2] * m[7] +
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
             m[0] * m[7] * m[10] -
             m[4] * m[2] * m[11] +
             m[4] * m[3] * m[10] +
             m[8] * m[2] * m[7] -
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
               m[0] * m[7] * m[9] +
               m[4] * m[1] * m[11] -
               m[4] * m[3] * m[9] -
               m[8] * m[1] * m[7] +
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
              m[0] * m[6] * m[9] -
              m[4] * m[1] * m[10] +
              m[4] * m[2] * m[9] +
              m[8] * m[1] * m[6] -
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return;

    det = 1.0 / det;

    for(i=0;i<16;i++) *(invOut+i) = inv[i] * det;


}

/** @} (end group ACC_GYRO_MAG_driver) */
/** @} (end group sensors) */
/** @} (end group drivers) */

