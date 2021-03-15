 /**
 * @file NT3H2111.c
 * @brief Driver for EEPROM with NFC and I2C interface
 * @date 6/2018
 * @author prachar.petr
 */

#ifndef NT3H2111_H_INCLUDED
#define NT3H2111_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>

#define NT3H2111_ADDR     0x55


#define NT3H_WHO_AM_I_REG   0x00
#define NT3H_WHO_AM_I_VAL   0x04

#define USER_MEMORY_BASE 1
#define USER_MEMORY_TOP  55

typedef struct {
        uint8_t static_lock_B[3];
        uint8_t dynamic_lock_B[3];
        uint8_t auth0;
        uint8_t acces;
        uint8_t pwd[4];
        uint8_t pack[2];
        uint8_t pt_i2c;
        uint8_t config_regs[8];
        uint8_t session_regs[8];
} ntag_setting_t;


bool nt3h2111_test(void);
bool nt3h2111_read_data(uint8_t start_addr, uint8_t* data, uint16_t length_B);
bool nt3h2111_write_data(uint8_t start_addr, uint8_t* data, uint16_t length_B);
bool nt3h2111_write_block(uint8_t block_addr, uint8_t* data, bool is_setting);

bool nt3h2111_ndef_format(void);



#endif /* NT3H2111_H_INCLUDED */
