/***************************************************************************//**
 * @file mpu9250.h
 * @brief MPU-9250 9-axis IMU sensor library header
 ******************************************************************************/

#ifndef MPU9250_H_
#define MPU9250_H_

#include <stdint.h>
#include <stdbool.h>
//#include "sensors_routines.h"

#define MPU9250_ADDR 0x68
#define WHO_AM_I_REG 0x75
#define WHO_AM_I_VAL 0x71
#define ACCEL_XOUT_H 0x3B         //mpu9250
//#define ACCEL_XOUT_H 0x2D           //icm-20948
#define GYRO_XOUT_H  0x43			//mpu9250
//#define GYRO_XOUT_H  0x33
#define BYPASS_INC_CONF 0x37


#define SMPLRT_DIV       0x19
#define CONF           	 0x1A
#define GYRO_CONF     	 0x1B
#define ACC_CONF1     	 0x1C
#define ACC_CONF2    	 0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F


#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define MOT_DETECT_CTRL  0x69

#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A

#define	WOM_MASK		 0x40

#define ACCEL_INTEL_CTRL 0x69

#define ACC_FS_mask  0xE7
#define ACC_FS_2g    0x00
#define ACC_FS_4g    0x08
#define ACC_FS_8g    0x10
#define ACC_FS_16g   0x18

#define GYRO_FS_mask	0xE7
#define GYRO_FS_250dps  0x00
#define GYRO_FS_500dps  0x08
#define GYRO_FS_1000dps 0x10
#define GYRO_FS_2000dps 0x18

#define AK8963_ADDR          0x0C
#define AK8963_WHO_AM_I_REG  0x00
#define AK8963_WHO_AM_I_VAL  0x48
#define AK8963_STATUS1       0x02

#define MAG_XOUT_L 			0x03
#define MAG_XOUT_H 			0x04
#define MAG_YOUT_L 			0x05
#define MAG_YOUT_H 			0x06
#define MAG_ZOUT_L 			0x07
#define MAG_ZOUT_H 			0x08


#define DR_200				0x04
#define DR_100				0x09
#define DR_50				0x13
#define DR_25				0x27
#define DR_12_5			    0x4F
#define DR_6_25				0x9F
#define DR_3_125			0xFF

#define LP_ODR_250			0x0A
#define LP_ODR_125			0x09
#define LP_ODR_62_5			0x08
#define	LP_ODR_31_25		0x07
#define LP_ODR_15_63		0x06
#define LP_ODR_7_81			0x05
#define LP_ODR_3_91			0x04
#define LP_ODR_1_95			0x03
#define LP_ODR_0_98			0x02
#define LP_ODR_0_49			0x01
#define LP_ODR_0_24			0x00



#define AK8963_STATUS1      0x02
#define AK8963_STATUS2      0x09
#define AK8963_CTRL1		0x0A
#define AK8963_CTRL2		0x0B

#define AK8963_DATA_RDY         0x01
#define AK8963_PWR_DOWN		    0x00
#define AK8963_SINGLE_MEAS		0x01
#define AK8963_8_Hz				0x02
#define AK8963_100_Hz			0x06
#define AK8963_EXT_TRIG			0x04

#define AK8963_RES_14b  false
#define AK8963_RES_16b  true


typedef struct {
	int16_t x_acc;
	int16_t y_acc;
	int16_t z_acc;

	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;

	int16_t x_mag;
	int16_t y_mag;
	int16_t z_mag;
} int_data_9ax_t;

typedef struct {
	int16_t x_acc;
	int16_t y_acc;
	int16_t z_acc;

	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;

} int_data_6ax_t;

typedef struct {
	float x_acc;
	float y_acc;
	float z_acc;

	float x_gyro;
	float y_gyro;
	float z_gyro;

	float x_mag;
	float y_mag;
	float z_mag;
} corr_data_9ax_t;




volatile static uint8_t acc_full_scale;



bool mpu9250_test(void);

void mpu9250_init_all(void);
void mpu9250_init_acc(void);
void mpu9250_init_mag(void);
void mpu9250_init_gyro(void);
void mpu9250_dis_gyro(void);
void mpu9250_en_gyro(void);
void mpu9250_dis_acc(void);
void mpu9250_en_acc(void);
void mpu9250_dis_mag(void);
void mpu9250_en_mag(void);
void mpu9250_disable(void);
void mpu9250_enable_all(void);
void mpu9250_clk_enable(void);
void mpu9250_set_sample_rate(uint8_t rate);
void mpu9250_WOM(uint8_t LP_ODR, uint8_t threshold);
uint8_t mpu9250_get_int_src(void);
void mpu9250_read_raw_acc_data();
//void mpu9250_read_corr_acc_data(corr_data_9ax_t *corr_data);
void mpu9250_read_corr_acc_data(void);
void mpu9250_read_raw_gyro_data(void);
void mpu9250_read_corr_gyro_data(void);
void mpu9250_read_corr_data(corr_data_9ax_t *corr_data);
void mpu9250_bypass_I2C(void);
void mpu9250_read_raw_data(int_data_9ax_t *meas_data);
void mpu9250_set_full_scales(uint8_t gyro_fs, uint8_t acc_fs);
void mpu9250_IMU_calib(void);
void mpu9250_read_corr_data_6DOF(corr_data_9ax_t *corr_data);
void mpu9250_set_LPmode(void);
void mpu9250_set_normal_mode(void);
uint8_t AK8963_I2C_readFromRegister(uint8_t reg);
void AK8963_I2C_writeToRegister(uint8_t reg, uint8_t data) ;
void AK8963_init(uint8_t mode,bool resolution);
void AK8963_read_corr_mag_data(void);   //read measured magnetic field
void AK8963_read_raw_mag_data(void); //read measured magnetic field
void AK8963_do_meas(void);

bool AK8963_test(void);

void mag_calib_AK8963(uint8_t duration_s,uint8_t delay_s);


#define		ACC_GUARD_TIME		(1UL)  //guard time - acc and mcu clock can be different, becouse of it guard time is added when acc mode is switched



#endif /* MPU9250_H_ */
