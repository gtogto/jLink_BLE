

#include "LIS2DH12.h"
#include "../../Teia_var.h"
#include "../../platform/peripheral.h"

 /***************************************************************************//**
 * @addtogroup drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup sensors
 * @{
 ******************************************************************************/

 /***************************************************************************//**
 * @addtogroup accelerometer_driver
 * @{
 ******************************************************************************/

#define LIS2DH12_ADDR       0x19

#define STATUS_REG_AUX      0x07

#define OUT_TEMP_L          0x0C
#define OUT_TEMP_H          0x0D

#define WHO_AM_I_REG        0x0F
#define WHO_AM_I_VAL        0x33

#define CTRL_REG0           0x1E
#define TEMP_CFG_REG        0x1F
#define CTRL_REG1           0x20
#define CTRL_REG2           0x21
#define CTRL_REG3           0x22
#define CTRL_REG4           0x23
#define CTRL_REG5           0x24
#define CTRL_REG6           0x25

#define REFERENCE           0x26
#define STATUS_REG          0x27

#define OUT_X_L             0x28
#define OUT_X_H             0x29

#define OUT_Y_L             0x2A
#define OUT_Y_H             0x2B

#define OUT_Z_L             0x2C
#define OUT_Z_H             0x2D

#define FIFO_CTRL_REG       0x2E
#define FIFO_SRC_REG        0x2F

#define INT1_CFG            0x30
#define INT1_SRC            0x31
#define INT1_THS            0x32
#define INT1_DURATION       0x33

#define INT2_CFG            0x34
#define INT2_SRC            0x35
#define INT2_THS            0x36
#define INT2_DURATION       0x37

#define CLICK_CFG           0x38
#define CLICK_SRC           0x39
#define CLICK_THS           0x3A
#define TIME_LIMIT          0x3B
#define TIME_LATENCY        0x3C
#define TIME_WINDOW         0x3D

#define ACT_THS             0x3E
#define ACT_DUR             0x3F

#define LPen                (1<<3)
#define HRen                (1<<3)

#define ACC_FS_2g    0x00
#define ACC_FS_4g    0x10
#define ACC_FS_8g    0x20
#define ACC_FS_16g   0x30


#define LOW_POWER        LIS2DH12_LP_MODE
#define NORMAL           LIS2DH12_NORMAL_MODE
#define HIGH_RESOLUTION  LIS2DH12_HR_MODE

#define I2CDRV_burst_readFromRegister(x,y,z,a);  I2CDRV_burst_readFromRegister(x,y|0x80,z,a);

volatile uint8_t ctrl_reg1_cache;
volatile static uint8_t meas_mode;
volatile static uint8_t full_scale;

bool lis2dh12_test(void)
{
    uint8_t ID = I2CDRV_readFromRegister(LIS2DH12_ADDR,WHO_AM_I_REG);
    if(ID == WHO_AM_I_VAL) return true;
    else return false;
}

void lis2dh12_init(void)
{
    // CTRL_REG0 - disconnect internal pull-ups SDO/SA0 - see datasheet p.33
    uint8_t reg = 0b10010000;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG0,reg);

    // TEMP_CFG_REG - disable temperature sensor - see datasheet p.34
    reg = 0x00;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,TEMP_CFG_REG,reg);

    // CTRL_REG1 - low-power mode + disable all axes + power down mode - see datasheet p.34
    reg = 0b00001000;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG1,reg);
    ctrl_reg1_cache = 0b00001000;                           //save default value for CTRL_REG1;

    // CTRL_REG2 - High pass filter - normal mode, cutoff = ODR/200, output data bypassed, data for interrupt filtered - see datasheet p.36
    reg = 0b10100011;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG2,reg);


    // CTRL_REG3 - Interrupt generation - click disable, IA interrupt enable on pin INT1, data available int disable,FIFO ints disable - see datasheet p.36
    reg = 0b01100000;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG3,reg);

    // CTRL_REG4 - continous update data, big endian,FS+-8g, low power mode, self test disable, SPI - 4wire  - see datasheet p.37
    reg = 0b0010000;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG4,reg);
    meas_mode = LOW_POWER;

    //CTRL_REG5 - Normal mode,FIFO disable, latch INT1 request in internal register unil read, 4D orientation interupt disable
    reg = 0b00101000;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG5,reg);

    //CTRL_REG6 - All interupts on pin INT2 disable
    reg = 0b00000000;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG6,reg);

}



void lis2dh12_set_Standby(void)
{
    ctrl_reg1_cache = I2CDRV_readFromRegister(LIS2DH12_ADDR,CTRL_REG1);     //save state of the register before replace with standby value
    // CTRL_REG1 - low-power mode + disable all axes + power down mode - see datasheet p.34
    uint8_t reg = 0b00001000;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG1,reg);

}

void lis2dh12_set_Active(void)
{

    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG1,ctrl_reg1_cache);        // return CTRL_REG1 to state before standby mode
}

void lis2dh12_LP_mode(uint32_t refresh_rate)
{
    uint8_t ODR;

    if(refresh_rate >= 200) ODR = LIS2DH12_ODR10_Hz;
    else if(refresh_rate >= 80) ODR = LIS2DH12_ODR25_Hz;
    else if(refresh_rate >= 40) ODR = LIS2DH12_ODR50_Hz;
    else if(refresh_rate >= 20) ODR = LIS2DH12_ODR100_Hz;
    else ODR = LIS2DH12_ODR200_Hz;

    ctrl_reg1_cache = I2CDRV_readFromRegister(LIS2DH12_ADDR,CTRL_REG1);     //save state of the register before replace with standby value
    uint8_t reg = 0b00001111 | ODR;                                         //Low-power mode + all axis enable | ODR
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG1,reg);
}


void lis2dh12_WOM_setting(uint8_t ODR)
{
	//set datarate
	uint8_t reg = 0b00001111 | ODR;                                         //Low-power mode + all axis enable | ODR
	I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG1,reg);
	reg = 0b10000001;                                                         //HP normal mode + HP enable only for INT1 | HP_cutoff
	I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG2,reg);
	reg = 0b01000000;                                                         //only IA1 interrupt enable on pin INT1
	I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG3,reg);
	reg = I2CDRV_readFromRegister(LIS2DH12_ADDR,CTRL_REG5);
	reg &= 0b11110111;                                                      //latch interrupt request INT1 until SRC is read
	I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG5,reg);
	reg = 0b00101010;                                                       //ENABLE interrupt on high acceleration in any axis
	I2CDRV_writeToRegister(LIS2DH12_ADDR,INT1_CFG,reg);
	/*   reg = I2CDRV_readFromRegister(LIS2DH12_ADDR,CTRL_REG4);
	reg &= 0b00110000;
	reg >>= 4;*/
	I2CDRV_writeToRegister(LIS2DH12_ADDR,INT1_THS,0x30);
	// I2CDRV_writeToRegister(LIS2DH12_ADDR,ACT_THS,reg);
	I2CDRV_writeToRegister(LIS2DH12_ADDR,INT1_DURATION,0x01);           //set minimal duration of INT statement to 1/ODR
	//  I2CDRV_writeToRegister(LIS2DH12_ADDR,ACT_DUR,0x01);           //set minimal duration of INT statement to 1/ODR
}

uint8_t lis2dh12_get_int_src(void)
{
    return(I2CDRV_readFromRegister(LIS2DH12_ADDR,INT1_SRC));

}

void lis2dh12_set_ODR(uint8_t ODR)
{
    uint8_t reg = I2CDRV_readFromRegister(LIS2DH12_ADDR,CTRL_REG1);
    reg &= 0x0F;
    reg |= (ODR & 0xF0);
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG1,reg);
    ctrl_reg1_cache = reg;
}

void lis2dh12_set_all_axes_active(void)
{
    uint8_t reg = I2CDRV_readFromRegister(LIS2DH12_ADDR,CTRL_REG1);
    reg |= 0x07;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG1,reg);
    ctrl_reg1_cache = reg;

}

void lis2dh12_set_FS(uint8_t FS)
{
    uint8_t reg = I2CDRV_readFromRegister(LIS2DH12_ADDR,CTRL_REG4);
    reg &= 0b11001111;
    reg |= (FS & 0b00110000);
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG4,reg);
}

void lis2dh12_set_precision(uint8_t mode)
{
    uint8_t reg = I2CDRV_readFromRegister(LIS2DH12_ADDR,CTRL_REG1);
    if(mode == LOW_POWER) reg |= LPen;
    else reg &= ~LPen;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG1,reg);
    ctrl_reg1_cache = reg;

    reg = I2CDRV_readFromRegister(LIS2DH12_ADDR,CTRL_REG4);
    if(mode == HIGH_RESOLUTION) reg |= HRen;
    else reg &= ~HRen;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG4,reg);

    meas_mode = mode;
}

void lisdh12_interrupt_init(void)
{
    uint8_t reg = 0b00001111;  ///0b00001000;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG1,reg);

    reg = 0b10000001;                                                         //HP normal mode + HP enable only for INT1 | HP_cutoff
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG2,reg);

    reg = 0b01000000;                                                         //only IA1 interrupt enable on pin INT1
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG3,reg);

    reg = I2CDRV_readFromRegister(LIS2DH12_ADDR,CTRL_REG5);
    reg &= 0b11110111;                                                      //latch interrupt request INT1 until SRC is read
    I2CDRV_writeToRegister(LIS2DH12_ADDR,CTRL_REG5,reg);

    reg = 0b00101010;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,INT1_CFG,reg);
    //reg = 0b01111111;
    //I2CDRV_writeToRegister(LIS2DH12_ADDR,INT1_THS,reg);
    reg = 0b01111111;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,INT1_THS,reg);

    reg = 0b01111111;
    I2CDRV_writeToRegister(LIS2DH12_ADDR,INT1_SRC,reg);
    I2CDRV_writeToRegister(LIS2DH12_ADDR,INT1_DURATION,0x01);
}


/**************************************************************************//**
 * @brief  Read of acceleration raw data from lis2dh12
 *****************************************************************************/
void lis2dh12_read_raw_acc_data(uint8_t scale) //read measured acceleration
{
		uint8_t data[6];
		I2CDRV_burst_readFromRegister(LIS2DH12_ADDR,OUT_X_L, 6, data);

        sensors_data.acc_data.x_axis = ((((int16_t) data[1]) << 8) | data[0]);
        sensors_data.acc_data.x_axis >>= 6;

        sensors_data.acc_data.y_axis = ((((int16_t) data[3]) << 8) | data[2]);
        sensors_data.acc_data.y_axis >>= 6;

        sensors_data.acc_data.z_axis = ((((int16_t) data[5]) << 8) | data[4]);
        sensors_data.acc_data.z_axis >>= 6;

        //convert acc measure into acceleration in [mg]
        switch(scale)                  //acc full scale is stored in MPU format
        {
            case(ACC_FS_16g):
                sensors_data.acc_data.x_axis <<=3;
                sensors_data.acc_data.y_axis <<=3;
                sensors_data.acc_data.z_axis <<=3;
                break;

            case(ACC_FS_8g):
                sensors_data.acc_data.x_axis <<=2;
                sensors_data.acc_data.y_axis <<=2;
                sensors_data.acc_data.z_axis <<=2;
                break;

            case(ACC_FS_4g):
                sensors_data.acc_data.x_axis <<=1;
                sensors_data.acc_data.y_axis <<=1;
                sensors_data.acc_data.z_axis <<=1;
                break;
            default:
                break;

        }


}


/**************************************************************************//**
 * @brief  Read of acceleration corrected data from lis2dh12
 *****************************************************************************/
void lis2dh12_read_corr_acc_data(void) //read measured acceleration
{
		lis2dh12_read_raw_acc_data(ACC_FS_4g);
		sensors_data.acc_data.x_axis -= sensors_data.acc_offsets.x_axis;
		sensors_data.acc_data.y_axis -= sensors_data.acc_offsets.y_axis;
		sensors_data.acc_data.z_axis -= sensors_data.acc_offsets.z_axis;
}


/**************************************************************************//**
 * @brief  Read of acceleration corrected data from lis2dh12 and and convert it to acceleration in g
 *****************************************************************************/
void lis2dh12_read_acceleration(float *xaxis_f,float *yaxis_f,float *zaxis_f)
{
	lis2dh12_read_corr_acc_data();

	*xaxis_f = (float)sensors_data.acc_data.x_axis / 1000.f;
	*yaxis_f = (float)sensors_data.acc_data.y_axis / 1000.f;
	*zaxis_f = (float)sensors_data.acc_data.z_axis / 1000.f;

}



void lis2dh12_calib(void)
{
	uint16_t i=0;
	int32_t acc_xoffs=0, acc_yoffs=0, acc_zoffs=0;

	int16_t gravity = 1000;

	lis2dh12_init();
	lis2dh12_set_FS(ACC_FS_2g);
	lis2dh12_set_ODR(LIS2DH12_ODR100_Hz);
	lis2dh12_set_precision(HIGH_RESOLUTION);
    lis2dh12_set_all_axes_active();                                             //all accelerometer axes should be measured
    lis2dh12_set_Active();                                                      //switch acc to active mode


	for (i = 0; i < 2048; i++)
	{
		lis2dh12_read_raw_acc_data(ACC_FS_2g);
		acc_xoffs += (int32_t) (sensors_data.acc_data.x_axis);
		acc_yoffs += (int32_t) (sensors_data.acc_data.y_axis);
		acc_zoffs += (int32_t) (sensors_data.acc_data.z_axis - gravity);
		WDOG_Feed();
	}

	sensors_data.acc_offsets.x_axis =(int16_t)(acc_xoffs  >> 11);
	sensors_data.acc_offsets.y_axis =(int16_t)(acc_yoffs  >> 11);
	sensors_data.acc_offsets.z_axis =(int16_t)(acc_zoffs  >> 11);
}

/** @} (end group accelerometer_driver) */
/** @} (end group sensors) */
/** @} (end group drivers) */



