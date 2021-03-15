#ifndef LIS2DH12_H_INCLUDED
#define  LIS2DH12_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>


//ODR
#define LIS2DH12_PWR_DOWN           (0<<4)    /**<Power down mode*/
#define LIS2DH12_ODR1_Hz            (1<<4)    /**<Output data rate =  1 Hz*/
#define LIS2DH12_ODR10_Hz           (2<<4)    /**<Output data rate =  10 Hz*/
#define LIS2DH12_ODR25_Hz           (3<<4)    /**<Output data rate =  25 Hz*/
#define LIS2DH12_ODR50_Hz           (4<<4)    /**<Output data rate =  50 Hz*/
#define LIS2DH12_ODR100_Hz          (5<<4)    /**<Output data rate =  100 Hz*/
#define LIS2DH12_ODR200_Hz          (6<<4)    /**<Output data rate =  200 Hz*/
#define LIS2DH12_ODR400_Hz          (7<<4)    /**<Output data rate =  400 Hz*/
#define LIS2DH12_ODR1620_Hz         (8<<4)    /**<Output data rate =  1620 Hz*/
#define LIS2DH12_ODR5376_Hz         (9<<4)    /**<Output data rate =  5376 Hz*/
#define LIS2DH12_ODR1344_Hz         (9<<4)    /**<Output data rate =  1344 Hz*/



//high pass filter cutt-off frequency
#define LIS2DH12_HP_AGGRESSIVE    (0 << 5)    /**<HPF fc = ODR/50 */
#define LIS2DH12_HP_STRONG        (1 << 5)    /**<HPF fc = ODR/100*/
#define LIS2DH12_HP_MEDIUM        (2 << 5)    /**<HPF fc = ODR/200*/
#define LIS2DH12_HP_LIGHT         (3 << 5)    /**<HPF fc = ODR/400*/

//full-scale range of acc
#define LIS2DH12_FS_2g         (0 << 4)    /**<Full scale = +-2 g */
#define LIS2DH12_FS_4g         (1 << 4)    /**<Full scale = +-4 g */
#define LIS2DH12_FS_8g         (2 << 4)    /**<Full scale = +-8 g */
#define LIS2DH12_FS_16g        (3 << 4)    /**<Full scale = +-16 g */

#define LIS2DH12_LP_MODE        (0)        /**<low power mode */
#define LIS2DH12_NORMAL_MODE    (1)        /**<normal mode */
#define LIS2DH12_HR_MODE        (2)        /**<high resolution */


bool lis2dh12_test(void);
void lis2dh12_init(void);


void lis2dh12_set_Standby(void);
void lis2dh12_set_Active(void);
void lis2dh12_LP_mode(uint32_t refresh_rate);

uint8_t lis2dh12_get_int_src(void);
void lis2dh12_WOM_setting(uint8_t ODR);
void lis2dh12_set_ODR(uint8_t ODR);
void lis2dh12_set_FS(uint8_t FS);
void lis2dh12_set_all_axes_active(void);
void lis2dh12_set_precision(uint8_t mode);

void lis2dh12_read_raw_acc_data(uint8_t scale);
void lis2dh12_read_corr_acc_data(void);
void lis2dh12_read_acceleration(float *xaxis_f,float *yaxis_f,float *zaxis_f);

void lis2dh12_calib(void);
void lisdh12_interrupt_init(void);

#endif /* LIS2DH12_H_INCLUDED */
