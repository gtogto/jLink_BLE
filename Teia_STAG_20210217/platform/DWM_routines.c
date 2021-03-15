

#include "DWM_routines.h"
#include "peripheral.h"
#include "timing.h"
#include "../Teia_var.h"
#include <string.h>
#include "../drivers/radios/DWM/deca_device_api.h"
#include "../drivers/radios/DWM/deca_regs.h"
#include "../../../main.h"

//========================================================================================
void Dwt_Txcallback(const dwt_callback_data_t *txd)
{
	blink_sent = true;

}
//========================================================================================
void Dwt_Rxcallback(const dwt_callback_data_t *rxd)
{

	if( rxd->event == DWT_SIG_RX_OKAY ){
		printf("%d = ",rxd->datalength);
		for(uint16 k=0;k<(rxd->datalength);k++){
	    		printf("%02x ", rxd->data[k]);
		}
		printf("\r\n") ;
	}
	else{
		printf("UWB_RX_ERROR\r\n") ;
	}
}
//========================================================================================
static void dw_restore_from_sleep(void)
{
	dwt_setinterrupt(DWT_INT_TFRS, 1); //re-enable the TX/RX interrupts
}
//========================================================================================
void DWM1000_initAndDoHwResetProcedure(void)
{
    	DW_RST_ON();
	delay_and_sleep(6, 1, true);
	DW_RST_OFF();
	delay_and_sleep(6, 1, true);
	UWB_SPI_pins_enable();
	DW_CS_CLR();
	delay_and_sleep(1, 1, true);
	DW_CS_SET();
	delay_and_sleep(6, 1, true);
}

//========================================================================================
static uint32_t init_DWM1000_communication(void)
{
	if(baudrate != SPI_BAUDRATE_LOW) baudrate = SPI_init(SPI_BAUDRATE_LOW);
	dwt_softreset(); 	delay_ms(100);
	uint32_t   devID = dwt_readdevid();
	if (DWT_DEVICE_ID != devID) {
		DWM1000_initAndDoHwResetProcedure();
		dwt_softreset();
		DW_CS_CLR();
		delay_and_sleep(100, 1, true);
		DW_CS_SET();
		delay_and_sleep(100, 1, true);
		devID = dwt_readdevid();
		if (DWT_DEVICE_ID != devID) return (-1);
	}
	if (dwt_initialise(DWT_LOADUCODE ) != DWT_SUCCESS)  return (-1);
	baudrate = SPI_init(SPI_BAUDRATE_HIGH);
	devID = dwt_readdevid();
	if (DWT_DEVICE_ID != devID) return (-1);
	return 0;
}

const static uint16_t CRC16_table[] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
	};

void calculate_crc16(uint16_t* crc, uint8_t* data, size_t length)
{
	*crc = 0x0000;
	for(uint8_t i=0; i < length-2; i++) *crc = (*crc >> 8) ^ CRC16_table[(*crc^(uint16_t)data[i]) & 0x00ff];
}

//========================================================================================
void DWM1000_init(void)
{
	int32_t result = DWT_ERROR;

	result = init_DWM1000_communication();

	if(result == DWT_ERROR){
		do_SWResetMCU(1);
        	DWM1000_initAndDoHwResetProcedure();
		dwt_softreset();
	}
	else{
		uint32_t lotid = dwt_getlotid() & 0xfffffff;
		uint32_t partid = dwt_getpartid() & 0xfffff;
		LMSParameters.MACaddress = ((uint64_t) lotid << 16) | (uint64_t) partid;
        	uint8_t predef_MAC[6] = {0};
		uint16_t MAC_CRC = 0;
		uint16_t calc_CRC = 0xFFFF;
		memcpy((void*)&predef_MAC,(void*)PREDEF_MAC_PAGE, sizeof(predef_MAC));
		memcpy((void*)&MAC_CRC,(void*)(PREDEF_MAC_PAGE + sizeof(predef_MAC)),sizeof(MAC_CRC));
		calculate_crc16(&calc_CRC,predef_MAC, sizeof(predef_MAC)+2);
		if(MAC_CRC == calc_CRC)	{
			memcpy((void*)&LMSParameters.MACaddress,(void*)&predef_MAC,sizeof(predef_MAC));
			LMSParameters.MACaddress &= MAC_ADDR_48b_MASK;
		}
		dwt_configuresleep(DWT_LOADUCODE | DWT_PRESRV_SLEEP | DWT_CONFIG | DWT_TANDV,DWT_WAKE_CS | DWT_SLP_EN);
		dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT| DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO/* | DWT_INT_RXPTO*/),1);
		dwt_setcallbacks(Dwt_Txcallback, Dwt_Rxcallback); //NULL );
		//dwt_enableRXTXtimeMesure();
		//mslee read_otp_params();
		//mslee dwt_entersleep();
	}
}

//========================================================================================
uint32_t ret=0;
dwt_config_t dwt_Config;

void DWM_Setparameter(void)
{
	dwt_txconfig_t dwt_Txconfig;
	dwt_Config.chan = LMSParameters.channel;
	dwt_Config.phrMode = DWT_PHRMODE_STD;
	dwt_Config.sfdTO = DWT_SFDTOC_DEF;
	dwt_Config.dataRate = LMSParameters.data_rate;

	static const uint8_t preambleCfg[8] = {
	DWT_PLEN_4096,DWT_PLEN_2048,DWT_PLEN_1536,	DWT_PLEN_1024, DWT_PLEN_512,DWT_PLEN_256, DWT_PLEN_128, DWT_PLEN_64, };
	dwt_Config.txPreambLength = preambleCfg[LMSParameters.preamble];
	switch (dwt_Config.txPreambLength) {
        	case DWT_PLEN_4096:
        	case DWT_PLEN_2048:
        	case DWT_PLEN_1536:
            		dwt_Config.rxPAC = DWT_PAC64;
            		break;
            	//-----------------------------------
        	case DWT_PLEN_1024:
            		dwt_Config.rxPAC = DWT_PAC32;
            		break;
            	//-----------------------------------
        	case DWT_PLEN_512:
        	case DWT_PLEN_256:
            		dwt_Config.rxPAC = DWT_PAC16;
            		break;
            	//-----------------------------------
        	case DWT_PLEN_128:
        	case DWT_PLEN_64:
            		dwt_Config.rxPAC = DWT_PAC8;
            		break;
       }
  	static const uint8_t prfCfg[2] = {	DWT_PRF_16M,DWT_PRF_64M,};
	dwt_Config.prf = prfCfg[ LMSParameters.prf];
	dwt_Config.txCode =  LMSParameters.preamCode;
	dwt_Config.rxCode =  LMSParameters.preamCode;
	dwt_Config.nsSFD = LMSParameters.nSfd;
	dwt_configure(&dwt_Config) ;
	const uint8_t txPGdelayConfig[8] ={	0x0, 	0xc9,0xc2,0xc5,0x95,0xc0,0x0,  0x93 	};
    	if(LMSParameters.tx_PG_delay == 0UL)  dwt_Txconfig.PGdly = txPGdelayConfig[ dwt_Config.chan];
    	else dwt_Txconfig.PGdly = LMSParameters.tx_PG_delay;
    	dwt_Txconfig.PGdly = txPGdelayConfig[dwt_Config.chan];
	if(dwt_Config.dataRate == DWT_BR_6M8)     dwt_setsmarttxpower(1);
	else			dwt_setsmarttxpower(0);
	dwt_Txconfig.power = LMSParameters.tx_pwr_level;
	dwt_configuretxrf(&dwt_Txconfig);
}
//=====================================================================
void DMW1000_wake_up(void)
{
	UWB_SPI_pins_enable();
	DW_CS_CLR();
	delay_and_sleep(1,1,0);
	DW_CS_SET();
    	delay_and_sleep(3, 1, true);
	dw_restore_from_sleep();
}
//=====================================================================
void DM1000_enter_sleep(void)
{
    dwt_entersleep();
}
#if 0
//=====================================================================
void DM1000_enter_deep_sleep(void)
{
	//shut off radio
	dwt_forcetrxoff();
	delay_ms(20);
	dwt_configuresleep(DWT_LOADUCODE | DWT_PRESRV_SLEEP | DWT_CONFIG | DWT_TANDV,DWT_WAKE_CS | DWT_SLP_EN);
	delay_ms(20);
	dwt_entersleep();
	UWB_SPI_pins_disable();
}

//=============================================================================
void initTdoaParameters(tdoaParameters_t* data, bool defaultValues)
{
	data->this_tag_MACaddress = 0; // TODO ??
	if(defaultValues == true)
	{
		data->channel = CHANNEL_DEFAULT;
		data->data_rate = DR_MODE_DEFAULT;
		data->preamble = PREAMBLE_DEFAULT;
		data->prf = PRF_DEFAULT;
		data->preamCode = PREAM_CODE_DEFAULT;
		data->nSfd = NSFD;
		data->use_random_deviation = RANDOM_DEVIATION_DEFAULT;
		data->frequency_of_Eblink = MCR_LEVEL_DEFAULT;
		data->motion_control_mode = MCR_LEVEL_DEFAULT;
		data->refresh_rate_ms = REFRESH_RATE_DEFAULT;
		data->no_motion_refresh_rate = NO_MOT_REFRESH_RATE_DEFAULT;
		data->tx_pwr_level = TX_LEVEL_DEFAULT;
	}
	else
	{
		data->channel = 0;
		data->data_rate = 0;
		data->preamble = 0;
		data->prf = 0;
		data->preamCode = 0;
		data->nSfd = 0;
		data->use_random_deviation = 0;
		data->frequency_of_Eblink = 0;
		data->motion_control_mode = 0;
		data->refresh_rate_ms = 0;
		data->tx_pwr_level = 0;
	}
}



void Dwt_Rxcallback_conf(const dwt_callback_data_t *rxd)
{
	receivedFcode = 0;

	/*******************************************************************************
	 * SYNC msg
	 */
	if (rxd->event == DWT_SIG_RX_OKAY && rxd->datalength == sizeof(tdoa_uwb_conf_msg))
	//if (rxd->event == DWT_SIG_RX_OKAY)
	{
		dwt_readrxdata(&receivedFcode, 1, 0);

		if (receivedFcode == UWB_FCODE_CONF)
        {
			//fcode is SYNC
			dwt_readrxdata((uint8_t*) &confRxBuff, conf_packet_len, 0);

			uint64_t destAddr;
			memcpy(&(destAddr), &(confRxBuff.destAddr),
					sizeof(confRxBuff.destAddr));

			destAddr &= ~(0xFFFF000000000000);

			if (destAddr == TAG_BROADCAST_ADDR	|| destAddr == tdoaParameters.this_tag_MACaddress)
			{
				confState = true;
			}
			else
			{
				confState = false; //syncState = SYNC_TIMED_OUT;
				//re-enable receiver
				receivedFcode = 0;
				dwt_forcetrxoff();
				dwt_rxenable(0);
			}

		}
		else
        {
			//fcode is not valid for SYNC - (probably it is POLL due to same length)
			confState = false; //syncState = SYNC_TIMED_OUT;
			//re-enable receiver
			receivedFcode = 0;
			dwt_forcetrxoff();
			dwt_rxenable(0);
		} //end of if(syncedFcode == RTLS_DEMO_MSG_SANCH_SYNC)

		/*******************************************************************************
		 * RESPONSE msg or BLID RESPONSE msg
		 */
	}
	else
    {
		//Any other message was captured
		confState = false;
		//re-enable receiver
		receivedFcode = 0;
		dwt_forcetrxoff();
		dwt_rxenable(0);

	} //end of main if

}
#endif

#if 0  // Temp
/**
 * @brief Function for set the DWM into RX state and wait for new config, if new config is received its chcket if is not corrupted and stored
 * @return Return true if new config was received
 */
bool setUWB_RX_and_wait(systemValues_t *sysVal, bool rx_after_start)
{
	uint32_t sysconfbits = dwt_read32bitoffsetreg(SYS_CFG_ID, 0);
	dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering
	bool ret = (waitForConf(sysconfbits, sysVal, rx_after_start));
	return ret;
}
uint32_t DWM_reconfigure(void)
{
	dwt_txconfig_t dwt_Txconfig;

	dwt_Config.chan = 5; //tdoaParameters.channel; // >7 , == 6,  == 0 then error
	dwt_Config.phrMode = DWT_PHRMODE_STD;
	dwt_Config.sfdTO = DWT_SFDTOC_DEF; //default value
	dwt_Config.dataRate = 1; // tdoaParameters.data_rate;

	static const uint8_t preambleCfg[8] = {
		DWT_PLEN_4096,DWT_PLEN_2048,DWT_PLEN_1536,	DWT_PLEN_1024, DWT_PLEN_512,DWT_PLEN_256, DWT_PLEN_128, DWT_PLEN_64, };
	dwt_Config.txPreambLength = preambleCfg[5] ; // preambleCfg[tdoaParameters.preamble];

	switch (dwt_Config.txPreambLength) {
        	case DWT_PLEN_4096:
        	case DWT_PLEN_2048:
        	case DWT_PLEN_1536:
            		dwt_Config.rxPAC = DWT_PAC64;
            		break;
            	//-----------------------------------
        	case DWT_PLEN_1024:
            		dwt_Config.rxPAC = DWT_PAC32;
            		break;
            	//-----------------------------------
        	case DWT_PLEN_512:
        	case DWT_PLEN_256:
            		dwt_Config.rxPAC = DWT_PAC16;
            		break;
            	//-----------------------------------
        	case DWT_PLEN_128:
        	case DWT_PLEN_64:
            		dwt_Config.rxPAC = DWT_PAC8;
            		break;
            	//-----------------------------------
        	default:
           		 //Unknown current mode
            		DebugMsg(RELEASE, "\ne24");
            		ret = 1;
	}

  	static const uint8_t prfCfg[2] = {	DWT_PRF_16M,	DWT_PRF_64M,	};
	dwt_Config.prf = prfCfg[ 1 ] ; //prfCfg[ tdoaParameters.prf];
	dwt_Config.txCode =  12 ; // tdoaParameters.preamCode;
	dwt_Config.rxCode =  12 ; // tdoaParameters.preamCode;
	dwt_Config.nsSFD = 1; // tdoaParameters.nSfd;
	if (dwt_configure(&dwt_Config) != DWT_SUCCESS) ret = 1;

    //tx power
	const uint8_t txPGdelayConfig[8] =
	{
			0x0,    //Channel 0 ----- this is just a place holder so the next array element is channel 1
			0xc9,   //Channel 1 PG_DELAY
			0xc2,   //Channel 2//PG_DELAY
			0xc5,   //Channel 3//PG_DELAY
			0x95,   //Channel 4//PG_DELAY
			0xc0,   //Channel 5//PG_DELAY
			0x0,    //Channel 6 ----- this is just a place holder so the next array element is channel 7//0
			0x93    //Channel 7//PG_DELAY
	};

    if(tdoaParameters.tx_PG_delay == 0UL)
    {
        dwt_Txconfig.PGdly = txPGdelayConfig[ dwt_Config.chan];         //if value from OTP is not correct use predefined value of PG delay
    }
    else
    {
        dwt_Txconfig.PGdly = tdoaParameters.tx_PG_delay;                //else use PG delay that was readed from OTP memory
    }

    dwt_Txconfig.PGdly = txPGdelayConfig[dwt_Config.chan];

	if(dwt_Config.dataRate == DWT_BR_6M8)                               //Smart TX only work with data rate 6M8
	{
		dwt_setsmarttxpower(1);
	}
	else
	{
		dwt_setsmarttxpower(0);
	}

//
// TX POWER
//

	dwt_Txconfig.power = tdoaParameters.tx_pwr_level;

//configure the tx spectrum parameters (power and PG delay)

	dwt_configuretxrf(&dwt_Txconfig);

	return ret;

}
#endif

#if 0
// OTP addresses definitions
#define LDOTUNE_ADDRESS (0x04)
#define PARTID_ADDRESS (0x06)
#define LOTID_ADDRESS  (0x07)
#define VBAT_ADDRESS   (0x08)
#define VTEMP_ADDRESS  (0x09)
#define XTRIM_ADDRESS  (0x1E)
#define OTP_TXPWR_CH1_PRF16_ADDRESS    (0x010)
#define OTP_TXPWR_CH1_PRF64_ADDRESS    (0x011)
#define OTP_TXPWR_CH2_PRF16_ADDRESS    (0x012)
#define OTP_TXPWR_CH2_PRF64_ADDRESS    (0x013)
#define OTP_TXPWR_CH3_PRF16_ADDRESS    (0x014)
#define OTP_TXPWR_CH3_PRF64_ADDRESS    (0x015)
#define OTP_TXPWR_CH4_PRF16_ADDRESS    (0x016)
#define OTP_TXPWR_CH4_PRF64_ADDRESS    (0x017)
#define OTP_TXPWR_CH5_PRF16_ADDRESS    (0x018)
#define OTP_TXPWR_CH5_PRF64_ADDRESS    (0x019)
#define OTP_TXPWR_CH7_PRF16_ADDRESS    (0x01A)
#define OTP_TXPWR_CH7_PRF64_ADDRESS    (0x01B)

#define OTP_PGCNT_ADDRESS    (0x01D)
#define OTP_XTRIM_ADDRESS    (0x01E)

#define TEMP_COMP_FACTOR_CH2 (327) //(INT) (0.0798 * 4096)
#define TEMP_COMP_FACTOR_CH5 (607) //(INT) (0.1482 * 4096)
#define SAR_TEMP_TO_CELCIUS_CONV (1.14)
#define SAR_VBAT_TO_VOLT_CONV (1.0/173)

#define VBAT_COMP_FACTOR      (-2.92f)
/*
typedef struct ref_values {
    uint8   pgdly;
    uint32  power;
    int16   temp; // the DW IC raw register value, which needs conversion if you want a value
    uint16  pgcnt;
} ref_values_t;

ref_values_t ref_local;

int16_t ref_temp_raw;
*/

uint8_t receivedFcode = 0;								/**<Identification code(blink x sync x config) of received message*/
tdoa_uwb_conf_msg confRxBuff;							/**<Received message content*/
uint32_t conf_packet_len = sizeof(tdoa_uwb_conf_msg);	/**<Expected length of config message*/
bool confState = false;									/**<Valid config message was received?*/

// -------------------------------------------------------------------------------------------------------------------
// Structure to hold device data
typedef struct
{
    uint8_t  vBatP;             /**<IC V bat read during production and stored in OTP (Vmeas @ 3V3) */
    uint8_t  tempP;             /**<IC V temp read during production and stored in OTP (Tmeas @ 23C) */
    uint8_t  otprev;            /**<OTP revision number (read during initialisation) */
    uint16_t otp_mask;          /**<Local copy of the OTP mask used in dwt_initialise call */
    uint32_t TXpwr_ch1_prf16;   /**<TX power for channel 1 and PRF16 - read from OTP */
    uint32_t TXpwr_ch1_prf64;   /**<TX power for channel 1 and PRF64 - read from OTP */
    uint32_t TXpwr_ch2_prf16;   /**<TX power for channel 2 and PRF16 - read from OTP */
    uint32_t TXpwr_ch2_prf64;   /**<TX power for channel 2 and PRF64 - read from OTP */
    uint32_t TXpwr_ch3_prf16;   /**<TX power for channel 3 and PRF16 - read from OTP */
    uint32_t TXpwr_ch3_prf64;   /**<TX power for channel 3 and PRF64 - read from OTP */
    uint32_t TXpwr_ch4_prf16;   /**<TX power for channel 4 and PRF16 - read from OTP */
    uint32_t TXpwr_ch4_prf64;   /**<TX power for channel 4 and PRF64 - read from OTP */
    uint32_t TXpwr_ch5_prf16;   /**<TX power for channel 5 and PRF16 - read from OTP */
    uint32_t TXpwr_ch5_prf64;   /**<TX power for channel 5 and PRF64 - read from OTP */
    uint32_t TXpwr_ch7_prf16;   /**<TX power for channel 7 and PRF16 - read from OTP */
    uint32_t TXpwr_ch7_prf64;   /**<TX power for channel 7 and PRF64 - read from OTP */
} otp_data_t;

otp_data_t otp_refs;
#endif
