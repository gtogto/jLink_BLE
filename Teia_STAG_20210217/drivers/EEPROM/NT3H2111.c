/**
 * @file NT3H2111.c
 * @brief Driver for EEPROM with NFC and I2C interface
 * @date 6/2018
 * @author prachar.petr
 */

 /***************************************************************************//**
 * @addtogroup drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup memory
 * @{
 ******************************************************************************/

 /***************************************************************************//**
 * @addtogroup EEPROM
 * @{
 ******************************************************************************/

#include "NT3H2111.h"
#include "../../platform/peripheral.h"
//#include "my_memcpy.h"
#include <string.h>
#include "timing.h"






bool nt3h2111_test(void)
{
	uint8_t page[16]={0};
	I2CDRV_burst_readFromRegister(NT3H2111_ADDR,0,16,page);
	if (page[0]== NT3H_WHO_AM_I_VAL) return true;
	else return false;
}

/*static inline*/ bool nt3h2111_read_block(uint8_t block_addr, uint8_t* data)
{
    bool ret = I2CDRV_burst_readFromRegister(NT3H2111_ADDR,block_addr,16,data);
    delay_ms(50);
    return ret;
}


/*static inline*/  bool nt3h2111_write_block(uint8_t block_addr, uint8_t* data, bool is_setting)
{
    if(((block_addr >= USER_MEMORY_BASE) && (block_addr <= USER_MEMORY_TOP)) || is_setting)
    {
        bool ret = I2CDRV_burst_writeToRegister(NT3H2111_ADDR,block_addr,16,data);
        delay_ms(50);
        return ret;
    }
    else return true;
}

bool nt3h2111_write_data(uint8_t start_addr, uint8_t* data, uint16_t length_B)
{
    uint8_t ret = 0;
    uint8_t num_of_blocks = length_B >> 4;                  //calculate number of blocks that will be full used
    uint8_t l_mod_16 = (uint8_t)(length_B & 0x000F);        //calculate modulo 16 of length in bytes -

    for(uint8_t i = 0; i < num_of_blocks; i++)
    {
        ret += nt3h2111_write_block(start_addr + i,data + (i << 4),false);  //write block into memory
    }

    if(l_mod_16)                                                    //if some data rest, copy it to next block and remaining bytes fill with zero
    {
        uint8_t buff[20] ;
        memcpy(&buff[0],&data[num_of_blocks*16],(size_t)l_mod_16);              //copy rest of data into array
        for(uint8_t i = l_mod_16 ; i < 16; i++) buff[i] = 0x00;     //fill rest of array with zeros
        ret += nt3h2111_write_block(start_addr + num_of_blocks,buff,false);      //
    }
    if(ret) return true;
    else return false;
}

bool nt3h2111_read_data(uint8_t start_addr, uint8_t* data, uint16_t length_B)
{
    uint8_t ret = 0;
    uint8_t num_of_blocks = length_B >> 4;                  //calculate number of blocks that will be full used
    uint8_t l_mod_16 = (uint8_t)(length_B & 0x000F);        //calculate modulo 16 of length in bytes -

    for(uint8_t i = 0; i < num_of_blocks; i++)
    {
        ret += nt3h2111_read_block(start_addr + i,data + (i << 4));  //write block into memory
    }

    if(l_mod_16)                                                    //if some data rest, copy it to next block and remaining bytes fill with zero
    {
        uint8_t buff[16];
        ret += nt3h2111_read_block(start_addr + num_of_blocks,buff);
        memcpy(data+(num_of_blocks*16),buff,l_mod_16);                  //copy rest of data into array
    }
    if(ret) return true;
    else return false;
}


bool read_config_regs(ntag_setting_t* config)
{
    uint8_t mem_page[16];
    nt3h2111_read_block(0,mem_page);

    nt3h2111_read_block(0x38,mem_page);
    memcpy(&(config->dynamic_lock_B),&mem_page[8],3);
    config->auth0 = mem_page[15];

    nt3h2111_read_block(0x39,mem_page);
    config->acces = mem_page[0];
    config->pt_i2c = mem_page[12];

    nt3h2111_read_block(0x3A,mem_page);
    memcpy(&(config->config_regs),mem_page,sizeof(config->config_regs));

    nt3h2111_read_block(0xFE,mem_page);
    memcpy(&(config->session_regs),mem_page,sizeof(config->session_regs));

    return 0;
}

bool nt3h2111_unlock_memory(uint8_t start_addr)
{
    uint8_t mem_page[16];


    nt3h2111_read_block(0x38,mem_page);
    mem_page[15]=0xFF;
    nt3h2111_write_block(0x38,mem_page,true);
    delay_ms(10);

    nt3h2111_read_block(0x39,mem_page);
    mem_page[12]=0;
    nt3h2111_write_block(0x39,mem_page,true);
    delay_ms(10);

    nt3h2111_read_block(0x3A,mem_page);

    mem_page[0] = 1;
    mem_page[1] = 1;
    nt3h2111_write_block(0x3A,mem_page,true);
    delay_ms(10);

    nt3h2111_read_block(0xFE,mem_page);

    mem_page[0] = 1;
    mem_page[1] = 1;
    nt3h2111_write_block(0xFE,mem_page,true);
    delay_ms(10);
    return 0;
}

bool nt3h2111_ndef_format(void)
{
    bool ret = false;
    uint8_t mem_page[16];


    ret = nt3h2111_read_block(0x00,mem_page);
    mem_page[0] = 0x55 << 1;

    mem_page[12] = 0xE1;
    mem_page[13] = 0x10;
    mem_page[14] = 0x6D;
    mem_page[15] = 0x0F;
    ret = nt3h2111_write_block(0x00,mem_page,true);
    delay_ms(10);

    for(uint8_t i = 0; i <16; i++) mem_page[i] = 0;
    mem_page[0] = 0x03;
    mem_page[1] = 0x03;
    mem_page[2] = 0xD0;

    ret = nt3h2111_write_block(0x01,mem_page,false);
    return ret;

}


//0x3, 0xe, 0xd1, 0x1, 0xa, 0x55, 0x1, 0x73, 0x65, 0x77, 0x69, 0x6f, 0x2e, 0x6e, 0x65, 0x74, 0xfe


/** @} (end group EEPROM) */
/** @} (end group memory) */
/** @} (end group drivers) */
