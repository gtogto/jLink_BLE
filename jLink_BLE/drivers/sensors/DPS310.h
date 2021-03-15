/**
 * @file DPS310.h
 * @brief DSP310 barometer library header
 * @date 23.1.2017
 * @author prachar.petr
 */

#ifndef DPS310_H_
#define DPS310_H_

#include <stdint.h>
#include <stdbool.h>

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup DPS310
 * @{
 ******************************************************************************/


#define DPS310_ADDR 	0x77

#define DPS_WHO_AM_I_REG 	0x0D
#define DPS_WHO_AM_I_VAL	0x10

#define DPS310_PSR_B2		0x00
#define DPS310_PSR_B1		0x01
#define DPS310_PSR_B0		0x02
#define DPS310_TMP_B2		0x03
#define DPS310_TMP_B1		0x04
#define DPS310_TMP_B0		0x05

#define DPS310_PRS_CFG				0x06
#define DPS310_TMP_CFG				0x07
#define DPS310_MEAS_CFG				0x08
#define DPS310_CFG_REG				0x09
#define DPS310_INT_STS				0x0A
#define DPS310_FIFO_STS				0x0B
#define DPS310_RESET				0x0C

#define DPS310_COEF					0x10
#define DPS310_COEF_SRCE			0x28

#define DPS310_CFG_RATE_MASK		0x70
#define DPS310_CFG_PRC_MASK			0x07

#define DPS310_RATE_1HZ				0x00
#define DPS310_RATE_2HZ				0x10
#define DPS310_RATE_4HZ				0x20
#define DPS310_RATE_8HZ				0x30
#define DPS310_RATE_16HZ			0x40
#define DPS310_RATE_32HZ			0x50
#define DPS310_RATE_64HZ			0x60
#define DPS310_RATE_128HZ			0x70

#define DPS310_OSR_1				0x00
#define DPS310_OSR_2				0x01
#define DPS310_OSR_4				0x02
#define DPS310_OSR_8 				0x03
#define DPS310_OSR_16 				0x04
#define DPS310_OSR_32   			0x05
#define DPS310_OSR_64   			0x06
#define DPS310_OSR_128	 			0x07

#define DPS310_COEF_RDY_MASK		0x80
#define DPS310_SENS_RDY_MASK		0x40
#define DPS310_TMP_RDY_MASK			0x20
#define DPS310_PRS_RDY_MASK			0x10

#define DPS310_MEAS_CTRL_MASK		0x07

#define DPS310_PRS_SINGLE_MEAS		0x01
#define DPS310_TMP_SINGLE_MEAS		0x02
#define DPS310_PRS_CONT_MEAS		0x05
#define DPS310_TMP_CONT_MEAS		0x06
#define DPS310_BOTH_CONT_MEAS		0x07

#define DPS310_P_SHIFT_BIT			0x04
#define DPS310_T_SHIFT_bit			0x08

void dps310_read_data(uint32_t *PSR, uint8_t *TMP);
bool dps310_test(void);
void dps310_set_standby(void);
void dps310_set_active(void);
void dps310_init(uint8_t setting);
void dps310_meas_prs(void);
void dps310_meas_tmp(void);
void dps310_set_measure(uint8_t TMP_rate,uint8_t TMP_osr,uint8_t PRS_rate,uint8_t PRS_osr, bool continous);
void dps310_read_coefs(void);
void dps310_reset(void);


/** @} (end group DPS310) */
/** @} (end group Drivers) */

#endif /* DPS310_H_ */
