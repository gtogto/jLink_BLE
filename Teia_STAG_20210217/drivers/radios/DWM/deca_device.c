
#include <stdio.h>
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "../../../platform/timing.h"
#include "../../../main.h"

// Defines for enable_clocks function
#define FORCE_SYS_XTI  0
#define ENABLE_ALL_SEQ 1
#define FORCE_SYS_PLL  2
#define READ_ACC_ON    7
#define READ_ACC_OFF   8
#define FORCE_OTP_ON   11
#define FORCE_OTP_OFF  12
#define FORCE_TX_PLL   13


void 	_dwt_enableclocks(int clocks) ;
void		_dwt_configlde(int prf);
void 	_dwt_loaducodefromrom(void);
uint32 	_dwt_otpread(uint32 address);
uint32 	_dwt_otpprogword32(uint32 data, uint16 address);
void 	_dwt_aonarrayupload(void);

// Structure to hold device data
typedef struct
{
	uint32      deviceID ;
	uint32      partID ;
	uint32      lotID ;
	uint8       chan;
	uint8       longFrames ;
	uint8       otprev ;
	uint32      txFCTRL ;
	uint8       xtrim;
	uint8       dblbuffon;
	uint32      sysCFGreg ;
	uint16      sleep_mode;
	dwt_callback_data_t cdata;
	uint8       wait4resp ;
	int         prfIndex ;
	void (*dwt_txcallback)(const dwt_callback_data_t *txd);
	void (*dwt_rxcallback)(const dwt_callback_data_t *rxd);

} dwt_local_data_t;

static dwt_local_data_t dw1000local ;

//##########################################################################< dwt_initialise
#define LDOTUNE_ADDRESS (0x04)
#define PARTID_ADDRESS (0x06)
#define LOTID_ADDRESS  (0x07)
#define VBAT_ADDRESS   (0x08)
#define VTEMP_ADDRESS  (0x09)
#define XTRIM_ADDRESS  (0x1E)

int dwt_initialise(uint16 config)
{
	uint8 plllockdetect = EC_CTRL_PLLLCK;
	uint16 otp_addr = 0;
	uint32 ldo_tune = 0;
    	dw1000local.dblbuffon = 0;
    	dw1000local.prfIndex = 0;
    	dw1000local.cdata.aatset = 0;
    	dw1000local.wait4resp = 0;
    	dw1000local.sleep_mode = 0;
    	dw1000local.dwt_txcallback = NULL ;
    	dw1000local.dwt_rxcallback = NULL ;
    	dw1000local.deviceID =  dwt_readdevid() ;
    	if (DWT_DEVICE_ID != dw1000local.deviceID) return DWT_ERROR ;
	_dwt_enableclocks(FORCE_SYS_XTI);
	dwt_writetodevice(EXT_SYNC_ID, EC_CTRL_OFFSET, 1, &plllockdetect);
	otp_addr = _dwt_otpread(XTRIM_ADDRESS) & 0xffff;
	dw1000local.otprev = (otp_addr >> 8) & 0xff;
    	ldo_tune = _dwt_otpread(LDOTUNE_ADDRESS);
	if((ldo_tune & 0xFF) != 0)	{
		uint8 ldok = OTP_SF_LDO_KICK;
		dwt_writetodevice(OTP_IF_ID, OTP_SF, 1, &ldok);
		dw1000local.sleep_mode |= AON_WCFG_ONW_LLDO;
	}
    	dw1000local.partID = _dwt_otpread(PARTID_ADDRESS);
    	dw1000local.lotID = _dwt_otpread(LOTID_ADDRESS);
    	dw1000local.xtrim = otp_addr & 0x1F;
    	if (!dw1000local.xtrim) dw1000local.xtrim = FS_XTALT_MIDRANGE ;
    	dwt_xtaltrim(dw1000local.xtrim);
    	if(config & DWT_LOADUCODE) {
        	_dwt_loaducodefromrom();
        	dw1000local.sleep_mode |= AON_WCFG_ONW_LLDE;
	}
	else {
		uint16 rega = dwt_read16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET+1) ;
		rega &= 0xFDFF ;
		dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET+1, rega) ;
	}
     	_dwt_enableclocks(ENABLE_ALL_SEQ);
    	uint8  buf ;
    	buf = 0x00 ;
    	dwt_writetodevice(AON_ID, AON_CFG1_OFFSET, 1, &buf);
    	dw1000local.sysCFGreg = dwt_read32bitreg(SYS_CFG_ID) ;
    	return DWT_SUCCESS ;
}
//===================================================================================
//===================================================================================
//===================================================================================
//===================================================================================
void dwt_isr(void) // Assume interrupt can supply context
{
	uint32  status = 0;
	uint32  clear = 0; // Will clear any events seen
	uint16  len = 0;

	dw1000local.cdata.event = 0;
	dw1000local.cdata.dblbuff = dw1000local.dblbuffon ;
	status = dw1000local.cdata.status = dwt_read32bitreg(SYS_STATUS_ID) ; // Read status register low 32bits

	if((status & SYS_STATUS_LDEDONE) && (dw1000local.dblbuffon == 0)){  // Fix for bug 622 - LDE done flag gets latched on a bad frame
		if((status & (SYS_STATUS_LDEDONE | SYS_STATUS_RXPHD | SYS_STATUS_RXSFDD)) != (SYS_STATUS_LDEDONE | SYS_STATUS_RXPHD | SYS_STATUS_RXSFDD)){
			dwt_forcetrxoff(); //this will clear all events
			dwt_rxreset();
			status &= SYS_STATUS_ALL_TX;
			if(dw1000local.sysCFGreg & SYS_CFG_RXAUTR){
				dwt_write16bitoffsetreg(SYS_CTRL_ID,0,(uint16)SYS_CTRL_RXENAB) ;
			}
			else	{
				dw1000local.cdata.event = DWT_SIG_RX_ERROR  ;
				if(dw1000local.dwt_rxcallback != NULL)	dw1000local.dwt_rxcallback(&dw1000local.cdata);
			}
			// added lms
        		//printf("DW_IRQ_Error\r\n") ;
        		//dwt_forcetrxoff(); //lms
        		//dwt_rxreset(); // lms
        		dwt_rxenable(0);
			// add end
		}
	}
	if(status & SYS_STATUS_RXFCG)  {  // Receiver FCS Good
		if(status & SYS_STATUS_LDEDONE) { // LDE done/finished
			if (status & SYS_STATUS_RXOVRR)  { // NOTE when overrun both HS and RS pointers point to the same buffer
				dwt_forcetrxoff();
				dwt_rxreset();
				if(dw1000local.sysCFGreg & SYS_CFG_RXAUTR) { // Re-enable of RX is ON, then re-enable here (ignore error)
					dwt_write16bitoffsetreg(SYS_CTRL_ID,0,(uint16)SYS_CTRL_RXENAB) ;
				}
				else {// The RX will be re-enabled by the application, report an error
					dw1000local.cdata.event = DWT_SIG_RX_ERROR  ;
					if(dw1000local.dwt_rxcallback != NULL)dw1000local.dwt_rxcallback(&dw1000local.cdata);
				}
				return;
			}
			else  { // No overrun condition - proceed to process the frame
				len = dwt_read16bitoffsetreg(RX_FINFO_ID, 0) & 0x3FF;
				dwt_readfromdevice(RX_BUFFER_ID,0,2,dw1000local.cdata.fctrl) ;
				if (dw1000local.longFrames==0) len &= 0x7F ;
				dwt_readfromdevice(RX_BUFFER_ID,0,len,&dw1000local.cdata.data[0]) ;
				// Standard frame length up to 127, extended frame length up to 1023 bytes
				dw1000local.cdata.datalength = len ;
				dw1000local.cdata.event = DWT_SIG_RX_OKAY ;
				if(dw1000local.dwt_rxcallback != NULL) dw1000local.dwt_rxcallback(&dw1000local.cdata);

				/*
				// Bug 627 workaround - clear the AAT bit if the ACK request bit in the FC is not set
				if((status & SYS_STATUS_AAT) && (((dw1000local.cdata.fctrl[0] & 0x20) == 0) || (dw1000local.cdata.fctrl[0] == 0x02)) ){// But the data frame has it clear or it is an ACK frame
					clear |= SYS_STATUS_AAT ;
					dw1000local.cdata.aatset = 0 ; // ACK request is not set
					dw1000local.wait4resp = 0;
				}
				else dw1000local.cdata.aatset = (status & SYS_STATUS_AAT) ; //check if ACK request is set
				dw1000local.cdata.event = DWT_SIG_RX_OKAY ;
				if(dw1000local.dblbuffon == 0) {// If no double buffering
					clear |= status & SYS_STATUS_ALL_RX_GOOD  ;
					dwt_write32bitreg(SYS_STATUS_ID,clear) ; // Write status register to clear event bits we have seen
					if(dw1000local.dwt_rxcallback != NULL) dw1000local.dwt_rxcallback(&dw1000local.cdata);
				}
				else {// Double buffer
					uint8  buff ;
					uint8 hsrb = 0x01 ;
					dwt_readfromdevice(SYS_STATUS_ID, 3, 1, &buff);
					if((buff & (SYS_STATUS_ICRBP >> 24)) ==  ((buff & (SYS_STATUS_HSRBP >> 24)) << 1)) { // Host Side Receive Buffer Pointer
					    clear |= status & SYS_STATUS_ALL_DBLBUFF;
					    dwt_write32bitreg(SYS_STATUS_ID,clear); // Write status register to clear event bits we have seen
					}
					if((dw1000local.sysCFGreg & SYS_CFG_RXAUTR) == 0)dwt_write16bitoffsetreg(SYS_CTRL_ID,0,(uint16)SYS_CTRL_RXENAB) ;
					if(dw1000local.dwt_rxcallback != NULL)dw1000local.dwt_rxcallback(&dw1000local.cdata);
					if(dwt_checkoverrun() == 0)dwt_writetodevice(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1, &hsrb) ;
					else	{
					    dwt_forcetrxoff();
					    dwt_rxreset();
					    if(dw1000local.sysCFGreg & SYS_CFG_RXAUTR) dwt_write16bitoffsetreg(SYS_CTRL_ID,0,(uint16)SYS_CTRL_RXENAB) ;
					}
				} */

			}// end of no overrun
		} // If LDE_DONE is set (this means we have both SYS_STATUS_RXFCG and SYS_STATUS_LDEDONE)
		else { // No LDE_DONE ?
			if(!(dw1000local.sysCFGreg & SYS_CFG_RXAUTR)) dwt_forcetrxoff();
			dwt_rxreset();	// Reset the RX
			dw1000local.wait4resp = 0;
			dw1000local.cdata.event = DWT_SIG_RX_ERROR  ;
			if(dw1000local.dwt_rxcallback != NULL) dw1000local.dwt_rxcallback(&dw1000local.cdata);
		}
		//printf("DW_IRQ_Rx\r\n") ;
	    	dwt_forcetrxoff();
	    	dwt_rxreset();
	    	dwt_rxenable(0);
	} // end if CRC is good
	else if (status & SYS_STATUS_TXFRS) { // Transmit Frame Sent
		clear |= SYS_STATUS_ALL_TX; //clear TX event bits
		dwt_write32bitreg(SYS_STATUS_ID,clear); // Write status register to clear event bits we have seen
		if(dw1000local.cdata.aatset) {
			dw1000local.cdata.aatset = 0; // The ACK has been sent
			if(dw1000local.dblbuffon == 0)  {// If not double buffered
				if(dw1000local.wait4resp) {// wait4response was set with the last TX start command
					dwt_forcetrxoff();
					//dwt_rxreset();
				}
			}
		}
		dw1000local.cdata.event = DWT_SIG_TX_DONE ; // Signal TX completed
		if(dw1000local.dwt_txcallback != NULL)dw1000local.dwt_txcallback(&dw1000local.cdata);
		//dwt_forcetrxoff();
		//dwt_rxreset();
		// added lms
		//printf("DW_IRQ_TX\r\n") ;
		dwt_forcetrxoff();
		dwt_rxreset();
		dwt_rxenable(0);
		// add end

	}
	else if (status & SYS_STATUS_RXRFTO) { // Receiver Frame Wait timeout
		clear |= status & SYS_STATUS_RXRFTO ;
		dwt_write32bitreg(SYS_STATUS_ID,clear) ; // Write status register to clear event bits we have seen
		dw1000local.cdata.event = DWT_SIG_RX_TIMEOUT  ;
		if(dw1000local.dwt_rxcallback != NULL) dw1000local.dwt_rxcallback(&dw1000local.cdata);
		dw1000local.wait4resp = 0;
		//printf("DW_IRQ_Timeout\r\n") ;
		dwt_forcetrxoff();
		dwt_rxreset();
		dwt_rxenable(0);

    	}
	else if(status & SYS_STATUS_ALL_RX_ERR) {// Catches all other error events
		clear |= status & SYS_STATUS_ALL_RX_ERR;
		dwt_write32bitreg(SYS_STATUS_ID,clear) ; // Write status register to clear event bits we have seen
		dw1000local.wait4resp = 0;
		if(!(dw1000local.sysCFGreg & SYS_CFG_RXAUTR))dwt_forcetrxoff();
		dwt_rxreset();	// Reset the RX
		if(status & SYS_STATUS_RXPHE)dw1000local.cdata.event = DWT_SIG_RX_PHR_ERROR  ;
		else if(status & SYS_STATUS_RXFCE)dw1000local.cdata.event = DWT_SIG_RX_ERROR  ;
		else if(status & SYS_STATUS_RXRFSL)dw1000local.cdata.event = DWT_SIG_RX_SYNCLOSS  ;
		else if(status & SYS_STATUS_RXSFDTO)dw1000local.cdata.event = DWT_SIG_RX_SFDTIMEOUT  ;
		else if(status & SYS_STATUS_RXPTO)dw1000local.cdata.event = DWT_SIG_RX_PTOTIMEOUT  ;
		else dw1000local.cdata.event = DWT_SIG_RX_ERROR  ;
		if(dw1000local.dwt_rxcallback != NULL) dw1000local.dwt_rxcallback(&dw1000local.cdata);
		status &= SYS_STATUS_ALL_TX;
		// added lms
		//printf("DW_IRQ_All Error\r\n") ;
		dwt_forcetrxoff();
		dwt_rxreset();
		dwt_rxenable(0);
		// add end
    	}
}  // end dwt_isr()
//-------------------------------------------------------------------------------------------------------------------
void deca_sleep(uint32 us){
	delay_and_sleep(us,0,false);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_enableRXTXtimeMesure(void) {
	uint8 buf[1];
	dwt_readfromdevice(GPIO_CTRL_ID, 0x02, 1, buf);
	buf[0] = 0b00000101;
	//buf[0] |= 0x00;
	dwt_writetodevice(GPIO_CTRL_ID, 0x02, 1, buf);
}
//-------------------------------------------------------------------------------------------------------------------
uint8 dwt_otprevision(void)
{
	return dw1000local.otprev ;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setGPIOforEXTTRX(void)
{
	uint8 buf[GPIO_MODE_LEN];
	// Set the GPIO to control external PA/LNA
	dwt_readfromdevice(GPIO_CTRL_ID, GPIO_MODE_OFFSET, GPIO_MODE_LEN, buf);
	buf[GPIO_LNA_BYTE_NUM] |= (GPIO_PIN5_EXTTXE_8 + GPIO_PIN6_EXTRXE_8);
	dwt_writetodevice(GPIO_CTRL_ID, GPIO_MODE_OFFSET, GPIO_MODE_LEN, buf);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setGPIOdirection(uint32 gpioNum, uint32 direction)
{
	uint8 buf[GPIO_DIR_LEN];
	uint32 command = direction | gpioNum;
	buf[0] = command & 0xff;
	buf[1] = (command >> 8) & 0xff;
	buf[2] = (command >> 16) & 0xff;
	dwt_writetodevice(GPIO_CTRL_ID, GPIO_DIR_OFFSET, GPIO_DIR_LEN, buf);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setGPIOvalue(uint32 gpioNum, uint32 value)
{
	uint8 buf[GPIO_DOUT_LEN];
	uint32 command = value | gpioNum;
	buf[0] = command & 0xff;
	buf[1] = (command >> 8) & 0xff;
	buf[2] = (command >> 16) & 0xff;
	dwt_writetodevice(GPIO_CTRL_ID, GPIO_DOUT_OFFSET, GPIO_DOUT_LEN, buf);
}
//-------------------------------------------------------------------------------------------------------------------
uint32 dwt_getpartid(void)
{
    	return dw1000local.partID;
}
//-------------------------------------------------------------------------------------------------------------------
uint32 dwt_getlotid(void)
{
    	return dw1000local.lotID;
}
//-------------------------------------------------------------------------------------------------------------------
uint32 dwt_readdevid(void)
{
    	return dwt_read32bitoffsetreg(DEV_ID_ID,0);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_configuretxrf(dwt_txconfig_t *config)
{
    dwt_writetodevice(TX_CAL_ID, TC_PGDELAY_OFFSET, 1, &config->PGdly);
    dwt_write32bitreg(TX_POWER_ID, config->power);
}
//-------------------------------------------------------------------------------------------------------------------
int dwt_configure(dwt_config_t *config)
{
	uint8 nsSfd_result  = 0;
	uint8 useDWnsSFD = 0;
	uint8 chan = config->chan ;
	uint32 regval ;
	uint16 reg16 = lde_replicaCoeff[config->rxCode];
	uint8 prfIndex = dw1000local.prfIndex = config->prf - DWT_PRF_16M;
	uint8 bw = ((chan == 4) || (chan == 7)) ? 1 : 0 ;

	dw1000local.chan = config->chan ;
	if(DWT_BR_110K == config->dataRate)   {
	    dw1000local.sysCFGreg |= SYS_CFG_RXM110K ;
	    reg16 >>= 3; // lde_replicaCoeff must be divided by 8
	}
	else  dw1000local.sysCFGreg &= (~SYS_CFG_RXM110K) ;

	dw1000local.longFrames = config->phrMode ;
	dw1000local.sysCFGreg |= (SYS_CFG_PHR_MODE_11 & (config->phrMode << 16)) ;

	dwt_write32bitreg(SYS_CFG_ID,dw1000local.sysCFGreg) ;
	dwt_write16bitoffsetreg(LDE_IF_ID, LDE_REPC_OFFSET, reg16) ;
	_dwt_configlde(prfIndex);
	dwt_writetodevice(FS_CTRL_ID, FS_PLLCFG_OFFSET, 5, &pll2_config[chan_idx[chan]][0]);
	dwt_writetodevice(RF_CONF_ID, RF_RXCTRLH_OFFSET, 1, &rx_config[bw]);
	dwt_write32bitoffsetreg(RF_CONF_ID, RF_TXCTRL_OFFSET, tx_config[chan_idx[chan]]);
	dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE0b_OFFSET, sftsh[config->dataRate][config->nsSFD]);
	dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1a_OFFSET, dtune1[prfIndex]);

	if(config->dataRate == DWT_BR_110K) dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1b_OFFSET, 0x64);
	else {
	    if(config->txPreambLength == DWT_PLEN_64) {
	        uint8 temp = 0x10;
	        dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1b_OFFSET, 0x10);
	        dwt_writetodevice(DRX_CONF_ID, 0x26, 1, &temp);
	    }
	    else {
	        uint8 temp = 0x28;
	        dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1b_OFFSET, 0x20);
	        dwt_writetodevice(DRX_CONF_ID, 0x26, 1, &temp);
	    }
	}
	dwt_write32bitoffsetreg(DRX_CONF_ID, DRX_TUNE2_OFFSET, digital_bb_config[prfIndex][config->rxPAC]);
	if(config->sfdTO == 0) config->sfdTO = DWT_SFDTOC_DEF;
	dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_SFDTOC_OFFSET, config->sfdTO);
	dwt_write32bitoffsetreg( AGC_CFG_STS_ID, 0xC, agc_config.lo32);
	dwt_write16bitoffsetreg( AGC_CFG_STS_ID, 0x4, agc_config.target[prfIndex]);
	if(config->nsSFD)	{
	     dwt_writetodevice(USR_SFD_ID,0x00,1,&dwnsSFDlen[config->dataRate]);
	     nsSfd_result = 3 ;
	     useDWnsSFD = 1 ;
	}
	regval =  (CHAN_CTRL_TX_CHAN_MASK & (chan << CHAN_CTRL_TX_CHAN_SHIFT)) | // Transmit Channel
	          	(CHAN_CTRL_RX_CHAN_MASK & (chan << CHAN_CTRL_RX_CHAN_SHIFT)) | // Receive Channel
	          	(CHAN_CTRL_RXFPRF_MASK & (config->prf << CHAN_CTRL_RXFPRF_SHIFT)) | // RX PRF
	          	((CHAN_CTRL_TNSSFD|CHAN_CTRL_RNSSFD) & (nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT)) | // nsSFD enable RX&TX
	          	(CHAN_CTRL_DWSFD & (useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT)) | // Use DW nsSFD
	          	(CHAN_CTRL_TX_PCOD_MASK & (config->txCode << CHAN_CTRL_TX_PCOD_SHIFT)) | // TX Preamble Code
	          	(CHAN_CTRL_RX_PCOD_MASK & (config->rxCode << CHAN_CTRL_RX_PCOD_SHIFT)) ; // RX Preamble Code

	dwt_write32bitreg(CHAN_CTRL_ID,regval) ;
	dw1000local.txFCTRL = (config->txPreambLength | config->prf) << 16;
	dw1000local.txFCTRL |= (config->dataRate << TX_FCTRL_TXBR_SHFT) | TX_FCTRL_TR; // Always set ranging bit !!!
	dwt_write32bitoffsetreg(TX_FCTRL_ID,0,dw1000local.txFCTRL) ;
	return DWT_SUCCESS ;

}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setrxantennadelay(uint16 rxDelay)
{
    	// Set the RX antenna delay for auto TX timestamp adjustment
    	dwt_write16bitoffsetreg(LDE_IF_ID, LDE_RXANTD_OFFSET, rxDelay);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_settxantennadelay(uint16 txDelay)
{
	// Set the TX antenna delay for auto TX timestamp adjustment
	dwt_write16bitoffsetreg(TX_ANTD_ID, 0x0, txDelay);
}
//-------------------------------------------------------------------------------------------------------------------
int dwt_writetxdata(uint16 txFrameLength, uint8 *txFrameBytes, uint16 txBufferOffset)
{
	#ifdef DWT_API_ERROR_CHECK
	if (dw1000local.longFrames)  { if (txFrameLength > 1023)  return DWT_ERROR ;}
	else  {if (txFrameLength > 127)      return DWT_ERROR ;    if (txFrameLength < 2)    	return DWT_ERROR ;}
	#endif

	if ((txBufferOffset + txFrameLength) > 1024)    return DWT_ERROR ;
	dwt_writetodevice( TX_BUFFER_ID, txBufferOffset, txFrameLength-2, txFrameBytes) ;
	return DWT_SUCCESS ;
} // end dwt_writetxdata()
//-------------------------------------------------------------------------------------------------------------------
int dwt_writetxfctrl(uint16 txFrameLength, uint16 txBufferOffset)
{
	#ifdef DWT_API_ERROR_CHECK
	if (dw1000local.longFrames)  { if (txFrameLength > 1023)  return DWT_ERROR ;}
	else  {if (txFrameLength > 127)      return DWT_ERROR ;    if (txFrameLength < 2)    	return DWT_ERROR ;}
	#endif

	uint32 reg32 = dw1000local.txFCTRL | txFrameLength | (txBufferOffset << 22);
	dwt_write32bitoffsetreg(TX_FCTRL_ID,0,reg32) ;
	return DWT_SUCCESS ;
} // end dwt_writetxfctrl()
//-------------------------------------------------------------------------------------------------------------------
void dwt_readrxdata(uint8 *buffer, uint16 length, uint16 rxBufferOffset)
{
    	dwt_readfromdevice(RX_BUFFER_ID,rxBufferOffset,length,buffer) ;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_readaccdata(uint8 *buffer, uint16 len, uint16 accOffset)
{
	_dwt_enableclocks(READ_ACC_ON);
	dwt_readfromdevice(ACC_MEM_ID,accOffset,len,buffer) ;
	_dwt_enableclocks(READ_ACC_OFF); // Revert clocks back
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_readdiagnostics(dwt_rxdiag_t *diagnostics)
{
	diagnostics->firstPath = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET);
	diagnostics->maxNoise = dwt_read16bitoffsetreg(LDE_IF_ID, LDE_THRESH_OFFSET);
	dwt_readfromdevice(RX_FQUAL_ID, 0x0, 8, (uint8*)&diagnostics->stdNoise);
	diagnostics->firstPathAmp1 = dwt_read16bitoffsetreg(RX_TIME_ID, 0x7) ;
	diagnostics->rxPreamCount = (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT  ;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_readtxtimestamp(uint8 * timestamp)
{
    	dwt_readfromdevice(TX_TIME_ID, 0, TX_TIME_TX_STAMP_LEN, timestamp) ; // Read bytes directly into buffer
}
//-------------------------------------------------------------------------------------------------------------------
uint32 dwt_readtxtimestamphi32(void)
{
    	return dwt_read32bitoffsetreg(TX_TIME_ID, 1);
}
//-------------------------------------------------------------------------------------------------------------------
uint32 dwt_readtxtimestamplo32(void)
{
    	return dwt_read32bitoffsetreg(TX_TIME_ID, 0);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_readrxtimestamp(uint8 * timestamp)
{
    	dwt_readfromdevice(RX_TIME_ID, 0, RX_TIME_RX_STAMP_LEN, timestamp) ; // Get the adjusted time of arrival
}
//-------------------------------------------------------------------------------------------------------------------
uint32 dwt_readrxtimestamphi32(void)
{
    	return dwt_read32bitoffsetreg(RX_TIME_ID, 1);
}
//-------------------------------------------------------------------------------------------------------------------
uint32 dwt_readrxtimestamplo32(void)
{
    	return dwt_read32bitoffsetreg(RX_TIME_ID, 0);
}
//-------------------------------------------------------------------------------------------------------------------
uint32 dwt_readsystimestamphi32(void)
{
    return dwt_read32bitoffsetreg(SYS_TIME_ID, 1);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_readsystime(uint8 * timestamp)
{
    	dwt_readfromdevice(SYS_TIME_ID, 0, SYS_TIME_LEN, timestamp) ;
}
//-------------------------------------------------------------------------------------------------------------------
int dwt_writetodevice( uint16   recordNumber, uint16   index, uint32  length, uint8 *buffer)
{
	uint8 header[3] ; // Buffer to compose header in
	int   cnt = 0; // Counter for length of header
	#ifdef DWT_API_ERROR_CHECK
	if (recordNumber > 0x3F) return DWT_ERROR ;
	#endif
	if (index == 0)  {// For index of 0, no sub-index is required
	    header[cnt++] = 0x80 | recordNumber ; // Bit-7 is WRITE operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id
	}
	else	{
		#ifdef DWT_API_ERROR_CHECK
		if (index > 0x7FFF)return DWT_ERROR ;
		if ((index + length)> 0x7FFF)return DWT_ERROR ;
		#endif
		header[cnt++] = 0xC0 | recordNumber ; // Bit-7 is WRITE operation, bit-6 one=sub-address follows, bits 5-0 is reg file id

		if (index <= 127) {// For non-zero index < 127, just a single sub-index byte is required
		    	header[cnt++] = (uint8)index ; // Bit-7 zero means no extension, bits 6-0 is index.
		}
		else  {
		    	header[cnt++] = 0x80 | (uint8)(index) ; // Bit-7 one means extended index, bits 6-0 is low seven bits of index.
		    	header[cnt++] =  (uint8) (index >> 7) ; // 8-bit value = high eight bits of index.
		}
	}
	return writetospi(cnt,header,length,buffer);
} // end dwt_writetodevice()

//-------------------------------------------------------------------------------------------------------------------
int dwt_readfromdevice(uint16  recordNumber,uint16  index,uint32  length, uint8   *buffer)
{
	uint8 header[3] ; // Buffer to compose header in
	int   cnt = 0; // Counter for length of header
	#ifdef DWT_API_ERROR_CHECK
	if (recordNumber > 0x3F)return DWT_ERROR ;
	#endif
	if (index == 0) {// For index of 0, no sub-index is required
	    	header[cnt++] = (uint8) recordNumber ; // Bit-7 zero is READ operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id
	}
	else	{
		#ifdef DWT_API_ERROR_CHECK
		if (index > 0x7FFF) return DWT_ERROR ;
		if ((index + length)> 0x7FFF)return DWT_ERROR ;
		#endif
		header[cnt++] = (uint8)(0x40 | recordNumber) ; // Bit-7 zero is READ operation, bit-6 one=sub-address follows, bits 5-0 is reg file id

		if (index <= 127) {// For non-zero index < 127, just a single sub-index byte is required

		    	header[cnt++] = (uint8) index ; // Bit-7 zero means no extension, bits 6-0 is index.
		}
		else   {
		    	header[cnt++] = 0x80 | (uint8)(index) ; // Bit-7 one means extended index, bits 6-0 is low seven bits of index.
		    	header[cnt++] =  (uint8) (index >> 7) ; // 8-bit value = high eight bits of index.
		}
	}
	// Do the read from the SPI
	return readfromspi(cnt, header, length, buffer);  // result is stored in the buffer
} // end dwt_readfromdevice()

//-------------------------------------------------------------------------------------------------------------------
uint32 dwt_read32bitoffsetreg(int regFileID,int regOffset)
{
	uint32  regval = DWT_ERROR ;
	int     j ;
	uint8   buffer[4] ;

	int result = dwt_readfromdevice(regFileID,regOffset,4,buffer); // Read 4 bytes (32-bits) register into buffer
	if(result == DWT_SUCCESS)	{
		for (j = 3 ; j >= 0 ; j --)		{
		    	regval = (regval << 8) + buffer[j] ;
		}
	}
	return regval ;
} // end dwt_read32bitoffsetreg()
//-------------------------------------------------------------------------------------------------------------------
uint16 dwt_read16bitoffsetreg(int regFileID,int regOffset)
{
	uint16  regval = DWT_ERROR ;
	uint8   buffer[2] ;
	int result = dwt_readfromdevice(regFileID,regOffset,2,buffer); // Read 2 bytes (16-bits) register into buffer
	if(result == DWT_SUCCESS)    {
	    	regval = (buffer[1] << 8) + buffer[0] ;
	}
	return regval ;
} // end dwt_read16bitoffsetreg()

//-------------------------------------------------------------------------------------------------------------------
int dwt_write16bitoffsetreg(int regFileID,int regOffset,uint16 regval)
{
	int reg;
	uint8   buffer[2] ;
	buffer[0] = regval & 0xFF;
	buffer[1] = regval >> 8 ;
	reg = dwt_writetodevice(regFileID,regOffset,2,buffer);
	return reg;
} // end dwt_write16bitoffsetreg()
//-------------------------------------------------------------------------------------------------------------------
int dwt_write32bitoffsetreg(int regFileID,int regOffset,uint32 regval)
{
	int     j ;
	int reg;
	uint8   buffer[4] ;
	for ( j = 0 ; j < 4 ; j++ )    {
		buffer[j] = regval & 0xff ;
		regval >>= 8 ;
	}
	reg = dwt_writetodevice(regFileID,regOffset,4,buffer);
	return reg;
} // end dwt_write32bitoffsetreg()
//-------------------------------------------------------------------------------------------------------------------
void dwt_enableframefilter(uint16 enable)
{
	uint32 sysconfig = SYS_CFG_MASK & dwt_read32bitreg(SYS_CFG_ID) ; // Read sysconfig register
	if(enable)    {
	    	// Enable frame filtering and configure frame types
	    	sysconfig &= ~(SYS_CFG_FF_ALL_EN); // Clear all
	    	sysconfig |= (enable & SYS_CFG_FF_ALL_EN) | SYS_CFG_FFE;
	}
	else    {
	    	sysconfig &= ~(SYS_CFG_FFE);
	}
	dw1000local.sysCFGreg = sysconfig ;
	dwt_write32bitreg(SYS_CFG_ID,sysconfig) ;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setpanid(uint16 panID)
{
    	dwt_write16bitoffsetreg(PANADR_ID, 2, panID) ;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setaddress16(uint16 shortAddress)
{
	dwt_write16bitoffsetreg(PANADR_ID, 0, shortAddress) ;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_seteui(uint8 *eui64)
{
    	dwt_writetodevice(EUI_64_ID, 0x0, 8, eui64);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_geteui(uint8 *eui64)
{
    	dwt_readfromdevice(EUI_64_ID, 0x0, 8, eui64);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_otpread(uint32 address, uint32 *array, uint8 length)
{
	int i;
	_dwt_enableclocks(FORCE_SYS_XTI); // NOTE: Set system clock to XTAL - this is necessary to make sure the values read by _dwt_otpread are reliable
	for(i=0; i<length; i++)array[i] = _dwt_otpread(address + i) ;
	_dwt_enableclocks(ENABLE_ALL_SEQ); // Restore system clock to PLL
	return ;
}
//-------------------------------------------------------------------------------------------------------------------
uint32 _dwt_otpread(uint32 address)
{
	uint8 buf[4];
	uint32 ret_data;
	buf[1] = (address>>8) & 0xff;
	buf[0] = address & 0xff;
	dwt_writetodevice(OTP_IF_ID,OTP_ADDR,2,buf);
	buf[0] = 0x03; // 0x03 for manual drive of OTP_READ
	dwt_writetodevice(OTP_IF_ID,OTP_CTRL,1,buf);
	buf[0] = 0x00; // Bit0 is not autoclearing, so clear it (Bit 1 is but we clear it anyway).
	dwt_writetodevice(OTP_IF_ID,OTP_CTRL,1,buf);
	ret_data=dwt_read32bitoffsetreg(OTP_IF_ID,OTP_RDAT);
	return (ret_data);
}
//-------------------------------------------------------------------------------------------------------------------
uint32 _dwt_otpsetmrregs(int mode)
{
	uint8 rd_buf[4];
	uint8 wr_buf[4];
	uint32 mra=0,mrb=0,mr=0;

	wr_buf[0] = 0x03;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL+1,1,wr_buf);
	switch(mode&0x0f) {
		case 0x0 :  	mr =0x0000;   mra=0x0000;	    mrb=0x0000;    	break;
		case 0x1 :    	mr =0x1024;   mra=0x9220;  	    mrb=0x000e;    	break;
		case 0x2 :    	mr =0x1824;   mra=0x9220;	    mrb=0x0003;    	break;
		case 0x3 :    	mr =0x1824;   mra=0x9220;	    mrb=0x004e;    	break;
		case 0x4 :    	mr =0x0000;   mra=0x0000;	    mrb=0x0003;    	break;
		case 0x5 :    	mr =0x0024;   mra=0x0000;	    mrb=0x0003;    	break;
		default :	    	return DWT_ERROR;
	}

	wr_buf[0] = mra & 0x00ff;
	wr_buf[1] = (mra & 0xff00)>>8;
	dwt_writetodevice(OTP_IF_ID, OTP_WDAT,2,wr_buf);
	wr_buf[0] = 0x08;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
	wr_buf[0] = 0x02;
	dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);
	wr_buf[0] = 0x88;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
	wr_buf[0] = 0x80;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
	wr_buf[0] = 0x00;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
	wr_buf[0] = 0x05;
	dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);
	wr_buf[0] = mrb & 0x00ff;
	wr_buf[1] = (mrb & 0xff00)>>8;
	dwt_writetodevice(OTP_IF_ID, OTP_WDAT,2,wr_buf);
	wr_buf[0] = 0x08;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
	wr_buf[0] = 0x04;
	dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);
	wr_buf[0] = 0x88;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
	wr_buf[0] = 0x80;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
	wr_buf[0] = 0x00;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
	wr_buf[0] = 0x01;
	dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);
	wr_buf[0] = mr & 0x00ff;
	wr_buf[1] = (mr & 0xff00)>>8;
	dwt_writetodevice(OTP_IF_ID, OTP_WDAT,2,wr_buf);
	wr_buf[0] = 0x08;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
	deca_sleep(10);
	wr_buf[0] = 0x00;
	dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);
	wr_buf[0] = OTP_CTRL_OTPRDEN;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
	wr_buf[0] = 0x02;
	dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);
	wr_buf[0] = 0x04;
	dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);
	deca_sleep(100);
	wr_buf[0] = 0x00;
	dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);
	wr_buf[0] = 0x00;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
	deca_sleep(10);
	if (((mode&0x0f) == 0x1)||((mode&0x0f) == 0x2))	{
	    	dwt_readfromdevice(OTP_IF_ID, OTP_STAT,1,rd_buf);
	}
	return DWT_SUCCESS;
}
//-------------------------------------------------------------------------------------------------------------------
uint32 _dwt_otpprogword32(uint32 data, uint16 address)
{
	uint8 rd_buf[1];
	uint8 wr_buf[4];
	uint8 otp_done;

	dwt_readfromdevice(OTP_IF_ID, OTP_STAT, 1, rd_buf);
	if((rd_buf[0] & 0x02) != 0x02)return DWT_ERROR;
	wr_buf[3] = (data>>24) & 0xff;
	wr_buf[2] = (data>>16) & 0xff;
	wr_buf[1] = (data>>8) & 0xff;
	wr_buf[0] = data & 0xff;
	dwt_writetodevice(OTP_IF_ID, OTP_WDAT, 4, wr_buf);
	wr_buf[1] = (address>>8) & 0x07;
	wr_buf[0] = address & 0xff;
	dwt_writetodevice(OTP_IF_ID, OTP_ADDR, 2, wr_buf);
	wr_buf[0] = OTP_CTRL_OTPPROG;
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
	wr_buf[0] = 0x00; // And clear
	dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
	otp_done = 0;
	while(otp_done == 0)    {
		deca_sleep(1);
		dwt_readfromdevice(OTP_IF_ID, OTP_STAT, 1, rd_buf);
		if((rd_buf[0] & 0x01) == 0x01)otp_done = 1;
	}
	return DWT_SUCCESS;
}
//-------------------------------------------------------------------------------------------------------------------
uint32 dwt_otpwriteandverify(uint32 value, uint16 address)
{
	int prog_ok = DWT_SUCCESS;
	int retry = 0;
	_dwt_enableclocks(FORCE_SYS_XTI); //set system clock to XTI
	_dwt_otpsetmrregs(1); // Set mode for programming
	while(1)    {
		_dwt_otpprogword32(value, address);
		if(_dwt_otpread(address) == value)break;
		retry++;
		if(retry==5)break;
	}
	_dwt_otpsetmrregs(4); // Set mode for reading
	if(_dwt_otpread(address) != value)prog_ok = DWT_ERROR;// If this does not pass please check voltage supply on VDDIO
	_dwt_otpsetmrregs(0); // Setting OTP mode register for low RM read - resetting the device would be alternativ
	return prog_ok;
}
//-------------------------------------------------------------------------------------------------------------------
void _dwt_aonconfigupload(void)
{
	uint8 buf[1];
	buf[0] = 0x04;
	dwt_writetodevice(AON_ID,AON_CTRL_OFFSET,1,buf);
	buf[0] = 0x00;
	dwt_writetodevice(AON_ID,AON_CTRL_OFFSET,1,buf);
}
//-------------------------------------------------------------------------------------------------------------------
void _dwt_aonarrayupload(void)
{
	uint8 buf[1];
	buf[0] = 0x00;
	dwt_writetodevice(AON_ID,AON_CTRL_OFFSET,1,buf);
	buf[0] = 0x02;
	dwt_writetodevice(AON_ID,AON_CTRL_OFFSET,1,buf);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_entersleep(void)
{
	_dwt_aonarrayupload();
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_configuresleepcnt(uint16 sleepcnt)
{
	uint8 buf[2];
	buf[0] = 0x01;
	dwt_writetodevice(PMSC_ID,PMSC_CTRL0_OFFSET,1,buf);
	buf[0] = 0;
	dwt_writetodevice(AON_ID, AON_CFG0_OFFSET, 1, buf); // To make sure we don't accidentaly go to sleep
	buf[0] = 0;
	dwt_writetodevice(AON_ID, AON_CFG1_OFFSET, 1, buf);
	_dwt_aonconfigupload();
	buf[0] = sleepcnt & 0xFF;
	buf[1] = (sleepcnt >> 8) & 0xFF;
	dwt_writetodevice(AON_ID, (AON_CFG0_OFFSET+2) , 2, buf);
	_dwt_aonconfigupload();
	buf[0] = 1;
	dwt_writetodevice(AON_ID, AON_CFG1_OFFSET, 1, buf);
	_dwt_aonconfigupload();
	buf[0] = 0x00;
	dwt_writetodevice(PMSC_ID,PMSC_CTRL0_OFFSET,1,buf);
}
//-------------------------------------------------------------------------------------------------------------------
uint16 dwt_calibratesleepcnt(void)
{
	uint8 buf[2];
	uint16 result;
	buf[0] = 4;
	dwt_writetodevice(AON_ID, AON_CFG1_OFFSET, 1, buf);
	_dwt_aonconfigupload();
	buf[0] = 0;
	dwt_writetodevice(AON_ID, AON_CFG1_OFFSET, 1, buf);
	_dwt_aonconfigupload();
	buf[0] = 0x01;
	dwt_writetodevice(PMSC_ID,PMSC_CTRL0_OFFSET,1,buf);
	deca_sleep(1);
	buf[0] = 118;
	dwt_writetodevice(AON_ID, AON_ADDR_OFFSET, 1,buf);
	buf[0] = 0x80; // OVR EN
	dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1,buf);
	buf[0] = 0x88; // OVR EN, OVR_RD
	dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1,buf);
	dwt_readfromdevice(AON_ID, AON_RDAT_OFFSET, 1,buf);
	result = buf[0];
	result = result << 8;
	buf[0] = 117;
	dwt_writetodevice(AON_ID, AON_ADDR_OFFSET, 1,buf);
	buf[0] = 0x80; // OVR EN
	dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1,buf);
	buf[0] = 0x88; // OVR EN, OVR_RD
	dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1,buf);
	dwt_readfromdevice(AON_ID, AON_RDAT_OFFSET, 1,buf);
	result |= buf[0];
	buf[0] = 0x00; // Disable OVR EN
	dwt_writetodevice(AON_ID,AON_CTRL_OFFSET,1,buf);
	buf[0] = 0x00;
	dwt_writetodevice(PMSC_ID,PMSC_CTRL0_OFFSET,1,buf);
	return result;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_configuresleep(uint16 mode, uint8 wake)
{
	uint8 buf[1];
	mode |= dw1000local.sleep_mode;
	dwt_write16bitoffsetreg(AON_ID, AON_WCFG_OFFSET, mode);
	buf[0] = wake;
	dwt_writetodevice(AON_ID, AON_CFG0_OFFSET, 1, buf);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_entersleepaftertx(int enable)
{
	uint32 reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET);
	if(enable)reg |= PMSC_CTRL1_ATXSLP;
	else reg &= ~(PMSC_CTRL1_ATXSLP);
	dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, reg);
}
//-------------------------------------------------------------------------------------------------------------------
int dwt_spicswakeup(uint8 *buff, uint16 length)
{
	if(dwt_readdevid() != DWT_DEVICE_ID) {// Device was in deep sleep (the first read fails)
	    dwt_readfromdevice(0x0, 0x0, length, buff); // Do a long read to wake up the chip (hold the chip select low)
	    deca_sleep(5);
	}
	else return DWT_SUCCESS;
	if(dwt_readdevid() != DWT_DEVICE_ID)return DWT_ERROR;
	return DWT_SUCCESS;
}
//-------------------------------------------------------------------------------------------------------------------
void _dwt_configlde(int prfIndex)
{
	uint8 x = LDE_PARAM1;
	dwt_writetodevice( LDE_IF_ID, LDE_CFG1_OFFSET, 1, &x ); // 8-bit configuration register
	if(prfIndex)dwt_write16bitoffsetreg( LDE_IF_ID, LDE_CFG2_OFFSET, (uint16) LDE_PARAM3_64);
	else dwt_write16bitoffsetreg( LDE_IF_ID, LDE_CFG2_OFFSET, (uint16) LDE_PARAM3_16);
}
//-------------------------------------------------------------------------------------------------------------------
void _dwt_loaducodefromrom(void)
{
	uint8 wr_buf[2];
	wr_buf[1] = 0x03;
	wr_buf[0] = 0x01;
	dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 2, wr_buf);
	dwt_write16bitoffsetreg(OTP_IF_ID, OTP_CTRL, OTP_CTRL_LDELOAD); // Set load LDE kick bit
	deca_sleep(1); // Allow time for code to upload (should take up to 120 us)
	_dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_loadopsettabfromotp(uint8 gtab_sel)
{
	uint8 wr_buf[2];
	uint16 reg = (((gtab_sel & 0x3) << 5) | 0x1);
	wr_buf[1] = 0x03;
	wr_buf[0] = 0x01;
	dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 2, wr_buf);
	dwt_write16bitoffsetreg(OTP_IF_ID, OTP_SF, reg); // Set load gtab kick bit (bit0) and gtab selection bit
	_dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setsmarttxpower(int enable)
{
	dw1000local.sysCFGreg = dwt_read32bitreg(SYS_CFG_ID) ; // Read sysconfig register
	if(enable)  dw1000local.sysCFGreg &= ~(SYS_CFG_DIS_STXP) ;
	else dw1000local.sysCFGreg |= SYS_CFG_DIS_STXP ;
	dwt_write32bitreg(SYS_CFG_ID,dw1000local.sysCFGreg) ;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_enableautoack(uint8 responseDelayTime)
{
	dwt_write16bitoffsetreg(ACK_RESP_T_ID, 0x2, (responseDelayTime << 8) ) ; //in symbols
	dw1000local.sysCFGreg |= SYS_CFG_AUTOACK;
	dwt_write32bitreg(SYS_CFG_ID,dw1000local.sysCFGreg) ;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setdblrxbuffmode(int enable)
{
	if(enable)    {
		dw1000local.sysCFGreg &= ~SYS_CFG_DIS_DRXB;
		dw1000local.dblbuffon = 1;
	}
	else   {
	    	dw1000local.sysCFGreg |= SYS_CFG_DIS_DRXB;
	    	dw1000local.dblbuffon = 0;
	}
	dwt_write32bitreg(SYS_CFG_ID,dw1000local.sysCFGreg) ;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setautorxreenable(int enable)
{
	uint8 byte = 0;
	if(enable) dw1000local.sysCFGreg |= SYS_CFG_RXAUTR;
	else dw1000local.sysCFGreg &= ~SYS_CFG_RXAUTR;
	byte = dw1000local.sysCFGreg >> 24;
	dwt_writetodevice(SYS_CFG_ID, 3, 1, &byte) ;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setrxaftertxdelay(uint32 rxDelayTime)
{
	uint32 val = dwt_read32bitreg(ACK_RESP_T_ID) ; // Read ACK_RESP_T_ID register
	val &= ~(ACK_RESP_T_W4R_TIM_MASK) ; // Clear the timer (19:0)
	val |= (rxDelayTime & ACK_RESP_T_W4R_TIM_MASK) ; // In UWB microseconds (e.g. turn the receiver on 20uus after TX)
	dwt_write32bitreg(ACK_RESP_T_ID, val) ;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setcallbacks(void (*txcallback)(const dwt_callback_data_t *), void (*rxcallback)(const dwt_callback_data_t *))
{
	dw1000local.dwt_txcallback = txcallback;
	dw1000local.dwt_rxcallback = rxcallback;
}
//-------------------------------------------------------------------------------------------------------------------
uint8 dwt_checkIRQ(void)
{
	uint8 temp;
	dwt_readfromdevice(SYS_STATUS_ID, 0, 1, &temp);
	return (temp & 0x1) ;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setleds(uint8 test)
{
	uint8 buf[2];
	if(test & 0x1)  {
		dwt_readfromdevice(GPIO_CTRL_ID,0x00,2,buf);
		buf[1] &= ~0x3C; //clear the bits
		buf[1] |= 0x14;
		dwt_writetodevice(GPIO_CTRL_ID,0x01,1,&buf[1]);
		dwt_readfromdevice(PMSC_ID,0x02,1,buf);
		buf[0] |= 0x84; //
		dwt_writetodevice(PMSC_ID,0x02,1,buf);
		buf[0] = 0x10; // Blink period.
		buf[1] = 0x01; // Enable blink counter
		dwt_writetodevice(PMSC_ID,PMSC_LEDC_OFFSET,2,buf);
	}
	else if ((test & 0x1)== 0)  {
		dwt_readfromdevice(GPIO_CTRL_ID,0x00,2,buf);
		buf[1] &= ~(0x14);
		dwt_writetodevice(GPIO_CTRL_ID,0x00,2,buf);
	}
	if(test & 0x2)   {
		buf[0] = 0x0f; // Fire a LED blink trigger
		dwt_writetodevice(PMSC_ID,0x2a,1,buf);
		buf[0] = 0x00; // Clear forced trigger bits
		dwt_writetodevice(PMSC_ID,0x2a,1,buf);
	}
} // end _dwt_enableleds()
 //##########################################################################<  _dwt_enableclocks
void _dwt_enableclocks(int clocks)
{
	uint8 reg[2];
	dwt_readfromdevice(PMSC_ID, PMSC_CTRL0_OFFSET, 2, reg);
	switch(clocks)  {
		case ENABLE_ALL_SEQ:reg[0] = 0x00 ;    reg[1] = reg[1] & 0xfe;	break;
		case FORCE_SYS_XTI: 	reg[0] = 0x01 | (reg[0] & 0xfc);	break;
		case FORCE_SYS_PLL:    reg[0] = 0x02 | (reg[0] & 0xfc);	break;
		case READ_ACC_ON:    reg[0] = 0x48 | (reg[0] & 0xb3);   reg[1] = 0x80 | reg[1];	break;
		case READ_ACC_OFF:    reg[0] = reg[0] & 0xb3;    reg[1] = 0x7f & reg[1];	break;
		case FORCE_OTP_ON:    reg[1] = 0x02 | reg[1];	break;
		case FORCE_OTP_OFF:   reg[1] = reg[1] & 0xfd;	break;
		case FORCE_TX_PLL:    reg[0] = 0x20| (reg[0] & 0xcf);	break;
		default:	break;
	}
	dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 1, &reg[0]);
	dwt_writetodevice(PMSC_ID, 0x1, 1, &reg[1]);
}
//-------------------------------------------------------------------------------------------------------------------
void _dwt_disablesequencing(void) // Disable sequencing and go to state "INIT"
{
	_dwt_enableclocks(FORCE_SYS_XTI); // Set system clock to XTI
	dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE); // Disable PMSC ctrl of RF and RX clk blocks
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setdelayedtrxtime(uint32 starttime)
{
    	dwt_write32bitoffsetreg(DX_TIME_ID, 1, starttime) ;
} // end dwt_setdelayedtrxtime()
//-------------------------------------------------------------------------------------------------------------------
int dwt_starttx(uint8 mode)
{
	int retval = DWT_SUCCESS ;
	uint8 temp  = 0x00;
	uint16 checkTxOK = 0 ;

	if(mode & DWT_RESPONSE_EXPECTED)  {
		temp = (uint8)SYS_CTRL_WAIT4RESP ; // Set wait4response bit
		dwt_writetodevice(SYS_CTRL_ID,0,1,&temp) ;
		dw1000local.wait4resp = 1;
	}
	if (mode & DWT_START_TX_DELAYED){
		temp |= (uint8)(SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT) ;
		dwt_writetodevice(SYS_CTRL_ID,0,1,&temp) ;
		checkTxOK = dwt_read16bitoffsetreg(SYS_STATUS_ID,3) ;
		if ((checkTxOK & SYS_STATUS_TXERR) == 0) {
			retval = DWT_SUCCESS ; // All okay
		}
		else{
			temp = (uint8)SYS_CTRL_TRXOFF; // This assumes the bit is in the lowest byte
			dwt_writetodevice(SYS_CTRL_ID,0,1,&temp) ;
			dwt_entersleepaftertx(0);
			dw1000local.wait4resp = 0;
			retval = DWT_ERROR ; // Failed !
		}
	}
	else {
		temp |= (uint8)SYS_CTRL_TXSTRT ;
		dwt_writetodevice(SYS_CTRL_ID,0,1,&temp) ;
	}
	return retval;
} // end dwt_starttx()

//-------------------------------------------------------------------------------------------------------------------
int dwt_checkoverrun(void)
{
    	return ((dwt_read16bitoffsetreg(SYS_STATUS_ID, 2) & (SYS_STATUS_RXOVRR >> 16)) == (SYS_STATUS_RXOVRR >> 16));
}

//-------------------------------------------------------------------------------------------------------------------
void dwt_forcetrxoff(void)
{
	uint8 temp ;
	uint32 mask;
	temp = (uint8)SYS_CTRL_TRXOFF ; // This assumes the bit is in the lowest byte
	mask = dwt_read32bitreg(SYS_MASK_ID) ; // Read set interrupt mask
	dwt_write32bitreg(SYS_MASK_ID, 0) ; // Clear interrupt mask - so we don't get any unwanted events
	dwt_writetodevice(SYS_CTRL_ID,0,1,&temp) ; // Disable the radio
	dwt_write32bitreg(SYS_STATUS_ID,(SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD)) ;
	dwt_syncrxbufptrs();
	dwt_write32bitreg(SYS_MASK_ID, mask) ; // Set interrupt mask to what it was
	dw1000local.wait4resp = 0;
} // end deviceforcetrxoff()
//-------------------------------------------------------------------------------------------------------------------
void dwt_syncrxbufptrs(void)
{
	uint8  buff ;
	dwt_readfromdevice(SYS_STATUS_ID, 3, 1, &buff);
	if((buff & (SYS_STATUS_ICRBP >> 24)) !=   ((buff & (SYS_STATUS_HSRBP>>24)) << 1) )  {
	    	uint8 hsrb = 0x01;
	    	dwt_writetodevice(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET , 1, &hsrb) ; // We need to swap RX buffer status reg (write one to toggle internally)
	}
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setrxmode(int mode, uint8 rxON, uint8 rxOFF)
{
	uint16 reg16 =  RX_SNIFF_MASK & ((rxOFF << 8) | rxON);
	if(mode & DWT_RX_SNIFF)dwt_write16bitoffsetreg(RX_SNIFF_ID, 0x00, reg16) ; // Enable
	else  dwt_write16bitoffsetreg(RX_SNIFF_ID, 0x00, 0x0000) ; // Disable
}

//-------------------------------------------------------------------------------------------------------------------
int dwt_rxenable(int delayed)
{
	uint16 temp ;
	uint8 temp1 = 0;
	dwt_syncrxbufptrs();
	temp = (uint16)SYS_CTRL_RXENAB ;
	if (delayed) temp |= (uint16)SYS_CTRL_RXDLYE ;
	dwt_write16bitoffsetreg(SYS_CTRL_ID,0,temp) ;
	if (delayed)  {
		dwt_readfromdevice(SYS_STATUS_ID,3,1,&temp1) ;
		if (temp1 & (SYS_STATUS_HPDWARN >> 24)) {
			dwt_forcetrxoff(); // Turn the delayed receive off, and do immediate receive, return warning indication
			temp = (uint16)SYS_CTRL_RXENAB; // Clear the delay bit
			dwt_write16bitoffsetreg(SYS_CTRL_ID,0,temp) ;
			return DWT_ERROR;
		}
	}
	return DWT_SUCCESS;
} // end dwt_rxenable()
//-------------------------------------------------------------------------------------------------------------------
void dwt_setrxtimeout(uint16 time)
{
	uint8 temp ;
	dwt_readfromdevice(SYS_CFG_ID,3,1,&temp) ; // Read register
	if(time > 0)    {
		dwt_write16bitoffsetreg(RX_FWTO_ID, 0x0, time) ;
		temp |= (uint8)(SYS_CFG_RXWTOE>>24);
		dw1000local.sysCFGreg |= SYS_CFG_RXWTOE;
		dwt_writetodevice(SYS_CFG_ID,3,1,&temp) ;
	}
	else    {
		temp &= ~((uint8)(SYS_CFG_RXWTOE>>24));
		dw1000local.sysCFGreg &= ~(SYS_CFG_RXWTOE);
		dwt_writetodevice(SYS_CFG_ID,3,1,&temp) ;
	}
} // end dwt_setrxtimeout()
//-------------------------------------------------------------------------------------------------------------------
void dwt_setpreambledetecttimeout(uint16 timeout)
{
    	dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_PRETOC_OFFSET, timeout);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_setinterrupt(uint32 bitmask, uint8 enable)
{
	uint32 mask = dwt_read32bitreg(SYS_MASK_ID) ; // Read register
	if(enable) mask |= bitmask ;
	else mask &= ~bitmask ; // Clear the bit
	dwt_write32bitreg(SYS_MASK_ID,mask) ; // New value
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_configeventcounters(int enable)
{
	uint8 temp = 0x0;  //disable
	temp = (uint8)(EVC_CLR); // Clear and disable
	dwt_writetodevice(DIG_DIAG_ID, EVC_CTRL_OFFSET, 1, &temp) ;
	if(enable)  {
		temp = (uint8)(EVC_EN); // Enable
		dwt_writetodevice(DIG_DIAG_ID, EVC_CTRL_OFFSET, 1, &temp) ;
	}
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_readeventcounters(dwt_deviceentcnts_t *counters)
{
	uint32 temp;
	temp= dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_PHE_OFFSET); // Read sync loss (31-16), PHE (15-0)
	counters->PHE = temp & 0xFFF;
	counters->RSL = (temp >> 16) & 0xFFF;
	temp = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_FCG_OFFSET); // Read CRC bad (31-16), CRC good (15-0)
	counters->CRCG = temp & 0xFFF;
	counters->CRCB = (temp >> 16) & 0xFFF;
	temp = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_FFR_OFFSET); // Overruns (31-16), address errors (15-0)
	counters->ARFE = temp & 0xFFF;
	counters->OVER = (temp >> 16) & 0xFFF;
	temp = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_STO_OFFSET); // Read PTO (31-16), SFDTO (15-0)
	counters->PTO = (temp >> 16) & 0xFFF;
	counters->SFDTO = temp & 0xFFF;
	temp = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_FWTO_OFFSET); // Read RX TO (31-16), TXFRAME (15-0)
	counters->TXF = (temp >> 16) & 0xFFF;
	counters->RTO = temp & 0xFFF;
	temp = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_HPW_OFFSET); // Read half period warning events
	counters->HPW = temp & 0xFFF;
	counters->TXW = (temp >> 16) & 0xFFF;                       // Power-up warning events
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_rxreset(void)
{
	uint8 resetrx = 0xe0;
	dwt_writetodevice(PMSC_ID, 0x3, 1, &resetrx);
	resetrx = 0xf0; // Clear RX reset
	dwt_writetodevice(PMSC_ID, 0x3, 1, &resetrx);
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_softreset(void)
{
	uint8 temp[1] = {0};
	_dwt_disablesequencing();
	dwt_write16bitoffsetreg(AON_ID, AON_WCFG_OFFSET, 0x0);
	dwt_writetodevice(AON_ID, AON_CFG0_OFFSET, 1, temp);
	_dwt_aonarrayupload();
	dwt_readfromdevice(PMSC_ID, 0x3, 1, temp) ;
	temp[0] &= 0x0F;
	dwt_writetodevice(PMSC_ID, 0x3, 1, &temp[0]) ;
	deca_sleep(10);
	temp[0] |= 0xF0;
	dwt_writetodevice(PMSC_ID, 0x3, 1, &temp[0]) ;
	dw1000local.wait4resp = 0;
}
//-------------------------------------------------------------------------------------------------------------------
void dwt_xtaltrim(uint8 value)
{
	uint8 write_buf;
	dwt_readfromdevice(FS_CTRL_ID,FS_XTALT_OFFSET,1,&write_buf);
	write_buf &= ~FS_XTALT_MASK ;
	write_buf |= (FS_XTALT_MASK & value) ; // We should not change high bits, cause it will cause malfunction
	dwt_writetodevice(FS_CTRL_ID,FS_XTALT_OFFSET,1,&write_buf);
}
//-------------------------------------------------------------------------------------------------------------------
int dwt_configcwmode(uint8 chan)
{
	uint8 write_buf[1];
#ifdef DWT_API_ERROR_CHECK
	if ((chan < 1) || (chan > 7) || (6 == chan))  return DWT_ERROR ;
#endif
	_dwt_disablesequencing();
	dwt_writetodevice(FS_CTRL_ID, FS_PLLCFG_OFFSET, 5, &pll2_config[chan_idx[chan]][0]);
	dwt_write32bitoffsetreg(RF_CONF_ID, RF_TXCTRL_OFFSET, tx_config[chan_idx[chan]]);
	dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXPLLPOWEN_MASK); // Enable LDO and RF PLL blocks
	dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXALLEN_MASK); // Enable the rest of TX blocks
	write_buf[0] = 0x22;
	dwt_writetodevice(PMSC_ID,PMSC_CTRL0_OFFSET,1,write_buf);
	write_buf[0] = 0x07;
	dwt_writetodevice(PMSC_ID,0x1,1,write_buf);
	dwt_write16bitoffsetreg(PMSC_ID, PMSC_TXFINESEQ_OFFSET, PMSC_TXFINESEQ_DIS_MASK);
	write_buf[0] = TC_PGTEST_CW;
	dwt_writetodevice(TX_CAL_ID, TC_PGTEST_OFFSET, TC_PGTEST_LEN, write_buf);
	return DWT_SUCCESS ;
}

//-------------------------------------------------------------------------------------------------------------------
void dwt_configcontinuousframemode(uint32 framerepetitionrate)
{
	uint8 write_buf[4];
	_dwt_disablesequencing();
	dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXPLLPOWEN_MASK); // Enable LDO and RF PLL blocks
	dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXALLEN_MASK); // Enable the rest of TX blocks
	_dwt_enableclocks(FORCE_SYS_PLL);
	_dwt_enableclocks(FORCE_TX_PLL);
	if(framerepetitionrate < 4)  framerepetitionrate = 4;
	dwt_write32bitoffsetreg(DX_TIME_ID, 0, framerepetitionrate) ;
	write_buf[0] = (uint8)(DIAG_TMC_TX_PSTM) ;
	dwt_writetodevice(DIG_DIAG_ID, DIAG_TMC_OFFSET, 1,write_buf); // Turn the tx power spectrum test mode - continuous sending of frames
}

//-------------------------------------------------------------------------------------------------------------------
uint16 dwt_readtempvbat(uint8 fastSPI)
{
	uint8 wr_buf[2];
	uint8 vbat_raw;
	uint8 temp_raw;

	// These writes should be single writes and in sequence
	wr_buf[0] = 0x80; // Enable TLD Bias
	dwt_writetodevice(RF_CONF_ID,0x11,1,wr_buf);
	wr_buf[0] = 0x0A; // Enable TLD Bias and ADC Bias
	dwt_writetodevice(RF_CONF_ID,0x12,1,wr_buf);
	wr_buf[0] = 0x0f; // Enable Outputs (only after Biases are up and running)
	dwt_writetodevice(RF_CONF_ID,0x12,1,wr_buf);    //
	// Reading All SAR inputs
	wr_buf[0] = 0x00;
	dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C,1,wr_buf);
	wr_buf[0] = 0x01; // Set SAR enable
	dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C,1,wr_buf);

	if(fastSPI == 1) {
		deca_sleep(1); // If using PLL clocks(and fast SPI rate) then this sleep is needed
		// Read voltage and temperature.
		dwt_readfromdevice(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET,2,wr_buf);
	}
	else  {
		_dwt_enableclocks(FORCE_SYS_XTI); // NOTE: set system clock to XTI - this is necessary to make sure the values read are reliable
		// Read voltage and temperature.
		dwt_readfromdevice(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET,2,wr_buf);
		// Default clocks (ENABLE_ALL_SEQ)
		_dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
	}

	vbat_raw = wr_buf[0];
	temp_raw = wr_buf[1];
	wr_buf[0] = 0x00; // Clear SAR enable
	dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C,1,wr_buf);
	return ((temp_raw<<8)|(vbat_raw));
}

//-------------------------------------------------------------------------------------------------------------------
uint8 dwt_readwakeuptemp(void)
{
	uint8 temp_raw;
	dwt_readfromdevice(TX_CAL_ID,TC_SARL_SAR_LTEMP_OFFSET,1,&temp_raw);
	return (temp_raw);
}

//-------------------------------------------------------------------------------------------------------------------
uint8 dwt_readwakeupvbat(void)
{
	uint8 vbat_raw;
	dwt_readfromdevice(TX_CAL_ID,TC_SARL_SAR_LVBAT_OFFSET,1,&vbat_raw);
	return (vbat_raw);
}



