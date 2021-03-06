
#ifndef DECA_DEVICE_API_H_
#define DECA_DEVICE_API_H_

//#ifdef __cplusplus
//extern "C" {
//#endif

#include <stdint.h>

#ifndef uint8
#ifndef _DECA_UINT8_
#define _DECA_UINT8_
typedef unsigned char uint8;
#endif
#endif

#ifndef uint16
#ifndef _DECA_UINT16_
#define _DECA_UINT16_
typedef unsigned short uint16;
#endif
#endif

#ifndef uint32
#ifndef _DECA_UINT32_
#define _DECA_UINT32_
typedef unsigned long uint32;
#endif
#endif

#ifndef int8
#ifndef _DECA_INT8_
#define _DECA_INT8_
typedef signed char int8;
#endif
#endif

#ifndef int16
#ifndef _DECA_INT16_
#define _DECA_INT16_
typedef signed short int16;
#endif
#endif

#ifndef int32
#ifndef _DECA_INT32_
#define _DECA_INT32_
typedef signed long int32;
#endif
#endif

#define DWT_SUCCESS (0)
#define DWT_ERROR   (-1)

#define DWT_TIME_UNITS          (1.0/499.2e6/128.0) //!< = 15.65e-12 s

#define DWT_DEVICE_ID   (0xDECA0130) 		//!< DW1000 MP device ID

//! constants for selecting the bit rate for data TX (and RX)
//! These are defined for write (with just a shift) the TX_FCTRL register
#define DWT_BR_110K		0	//!< UWB bit rate 110 kbits/s
#define DWT_BR_850K		1	//!< UWB bit rate 850 kbits/s
#define DWT_BR_6M8		2	//!< UWB bit rate 6.8 Mbits/s

//! constants for specifying the (Nominal) mean Pulse Repetition Frequency
//! These are defined for direct write (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs
#define DWT_PRF_16M		1	//!< UWB PRF 16 MHz
#define DWT_PRF_64M		2	//!< UWB PRF 64 MHz

//! constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols
#define DWT_PAC8		0	//!< PAC  8 (recommended for RX of preamble length  128 and below
#define DWT_PAC16		1	//!< PAC 16 (recommended for RX of preamble length  256
#define DWT_PAC32		2	//!< PAC 32 (recommended for RX of preamble length  512
#define DWT_PAC64		3	//!< PAC 64 (recommended for RX of preamble length 1024 and up

//! constants for specifying TX Preamble length in symbols
//! These are defined to allow them be directly written into byte 2 of the TX_FCTRL register
//! (i.e. a four bit value destined for bits 20..18 but shifted left by 2 for byte alignment)
#define DWT_PLEN_4096	0x0C	//! Standard preamble length 4096 symbols
#define DWT_PLEN_2048	0x28	//! Non-standard preamble length 2048 symbols
#define DWT_PLEN_1536	0x18	//! Non-standard preamble length 1536 symbols
#define DWT_PLEN_1024	0x08	//! Standard preamble length 1024 symbols
#define DWT_PLEN_512	0x34	//! Non-standard preamble length 512 symbols
#define DWT_PLEN_256	0x24	//! Non-standard preamble length 256 symbols
#define DWT_PLEN_128	0x14	//! Non-standard preamble length 128 symbols
#define DWT_PLEN_64		0x04	//! Standard preamble length 64 symbols

//! callback events
#define DWT_SIG_RX_NOERR			0
#define DWT_SIG_TX_DONE             1		// Frame has been sent
#define DWT_SIG_RX_OKAY             2       // Frame Received with Good CRC
#define DWT_SIG_RX_ERROR            3       // Frame Received but CRC is wrong
#define DWT_SIG_RX_TIMEOUT          4       // Timeout on receive has elapsed
#define DWT_SIG_TX_AA_DONE			6		// ACK frame has been sent (as a result of auto-ACK)

#define DWT_SIG_RX_PHR_ERROR        8       // Error found in PHY Header
#define DWT_SIG_RX_SYNCLOSS         9       // Un-recoverable error in Reed Solomon Decoder
#define DWT_SIG_RX_SFDTIMEOUT       10      // Saw preamble but got no SFD within configured time
#define DWT_SIG_RX_PTOTIMEOUT		11		// Got preamble detection timeout (no preamble detected)
#define DWT_SIG_TX_PENDING			12		// Delayed TX is pending
#define DWT_SIG_TX_ERROR			13      // TX failed
#define DWT_SIG_RX_PENDING 			14 		// RX has been re-enabled

#define DWT_SFDTOC_DEF				0x1041	// default SFD timeout value

#define DWT_PHRMODE_STD				0x0		// standard PHR mode
#define DWT_PHRMODE_EXT				0x3		// DW proprietary extended frames PHR mode

// Defined constants for "mode" bitmask parameter passed into dwt_starttx() function.
#define DWT_START_TX_IMMEDIATE      0
#define DWT_START_TX_DELAYED        1
#define DWT_RESPONSE_EXPECTED		2


//frame filtering configuration options
#define DWT_FF_NOTYPE_EN			0x000			// no frame types allowed (FF disabled)
#define DWT_FF_COORD_EN				0x002			// behave as coordinator (can receive frames with no dest address (PAN ID has to match))
#define DWT_FF_BEACON_EN			0x004			// beacon frames allowed
#define DWT_FF_DATA_EN				0x008			// data frames allowed
#define DWT_FF_ACK_EN				0x010			// ack frames allowed
#define DWT_FF_MAC_EN				0x020			// mac control frames allowed
#define DWT_FF_RSVD_EN				0x040			// reserved frame types allowed

//DW1000 interrupt events
#define DWT_INT_TFRS			0x00000080			// frame sent
#define DWT_INT_LDED            0x00000400			// micro-code has finished execution
#define DWT_INT_RFCG			0x00004000			// frame received with good CRC
#define DWT_INT_RPHE			0x00001000			// receiver PHY header error
#define DWT_INT_RFCE			0x00008000			// receiver CRC error
#define DWT_INT_RFSL			0x00010000			// receiver sync loss error
#define DWT_INT_RFTO			0x00020000			// frame wait timeout
#define DWT_INT_RXOVRR			0x00100000			// receiver overrun
#define DWT_INT_RXPTO			0x00200000			// preamble detect timeout
#define DWT_INT_SFDT			0x04000000			// SFD timeout
#define DWT_INT_ARFE			0x20000000			// frame rejected (due to frame filtering configuration)


//DW1000 SLEEP and WAKEUP configuration parameters
#define DWT_PRESRV_SLEEP 0x0100                      // PRES_SLEEP - on wakeup preserve sleep bit
#define DWT_LOADOPSET    0x0080						 // ONW_L64P - on wakeup load operating parameter set for 64 PSR
#define DWT_CONFIG       0x0040						 // ONW_LDC - on wakeup restore (load) the saved configurations (from AON array into HIF)
#define DWT_TANDV        0x0001                      // ONW_RADC - on wakeup run ADC to sample temperature and voltage sensor values

#define DWT_XTAL_EN		 0x10						// keep XTAL running during sleep
#define DWT_WAKE_SLPCNT  0x8						// wake up after sleep count
#define DWT_WAKE_CS      0x4						// wake up on chip select
#define DWT_WAKE_WK      0x2						// wake up on WAKEUP PIN
#define DWT_SLP_EN       0x1						// enable sleep/deep sleep functionality

//DW1000 INIT configuration parameters
#define DWT_LOADUCODE     0x1
#define DWT_LOADNONE	  0x0

//DW1000 OTP operating parameter set selection
#define DWT_OPSET_64LEN   0x0
#define DWT_OPSET_TIGHT   0x1
#define DWT_OPSET_DEFLT   0x2

typedef struct{

	uint32 status;      //initial value of register as ISR is entered
	uint8  event;		//event type
	uint8  aatset;		//auto ACK TX bit is set
	uint16 datalength;	//length of frame
	uint8  fctrl[2];	//frame control bytes
	uint8  dblbuff ;	//set if double buffer is enabled
	uint8  data[1024] ;

}dwt_callback_data_t;


typedef enum {
	 CHAN_CTRL_TXCHAN_1 = 0x01,	/* Selects the transmit channel 1 */
	 CHAN_CTRL_TXCHAN_2 = 0x02,	/* Selects the transmit channel 2 */
	 CHAN_CTRL_TXCHAN_3 = 0x03,	/* Selects the transmit channel 3 */
	 CHAN_CTRL_TXCHAN_4 = 0x04,	/* Selects the transmit channel 4 */
	 CHAN_CTRL_TXCHAN_5 = 0x05,	/* Selects the transmit channel 5 */
	 CHAN_CTRL_TXCHAN_7 = 0x07	/* Selects the transmit channel 7 */
}eCHAN;

#pragma pack(1)
typedef struct
{
    uint8 chan ;           //!< channel number {1, 2, 3, 4, 5, 7 }
    uint8 prf ;            //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
    uint8 txPreambLength ; //!< DWT_PLEN_64..DWT_PLEN_4096
    uint8 rxPAC ;          //!< Acquisition Chunk Size (Relates to RX preamble length)
    uint8 txCode ;         //!< TX preamble code
    uint8 rxCode ;         //!< RX preamble code
    uint8 nsSFD ;          //!< Boolean should we use non-standard SFD for better performance
    uint8 dataRate ;       //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
    uint8 phrMode ;        //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint16 sfdTO ;         //!< SFD timeout value (in symbols)
}__attribute__ ((packed))  dwt_config_t ;
#pragma pack()


typedef struct
{
	uint8	PGdly;
	//TX POWER
	//31:24		BOOST_0.125ms_PWR
	//23:16		BOOST_0.25ms_PWR-TX_SHR_PWR
	//15:8		BOOST_0.5ms_PWR-TX_PHR_PWR
	//7:0		DEFAULT_PWR-TX_DATA_PWR
	uint32	power;
}
dwt_txconfig_t ;


typedef struct
{

	uint16		maxNoise ;			// LDE max value of noise
	uint16		firstPathAmp1 ;	    // Amplitude at floor(index FP) + 1
	uint16      stdNoise ;			// Standard deviation of noise
	uint16		firstPathAmp2 ;		// Amplitude at floor(index FP) + 2
	uint16		firstPathAmp3 ;		// Amplitude at floor(index FP) + 3
	uint16		maxGrowthCIR ;		// Channel Impulse Response max growth CIR
    uint16      rxPreamCount ;      // Count of preamble symbols accumulated
    //uint32        debug1;
    //uint32        debug2;
    uint16      firstPath ;         // First path index (10.6 bits fixed point integer)
}dwt_rxdiag_t ;


typedef struct
{
	//all of the below are mapped to a 12-bit register in DW1000
    uint16 PHE ;					//number of received header errors
	uint16 RSL ;					//number of received frame sync loss events
    uint16 CRCG ;					//number of good CRC received frames
    uint16 CRCB ;					//number of bad CRC (CRC error) received frames
	uint16 ARFE ;					//number of address filter errors
	uint16 OVER ;					//number of receiver overflows (used in double buffer mode)
    uint16 SFDTO ;					//SFD timeouts
    uint16 PTO ;					//Preamble timeouts
    uint16 RTO ;					//RX frame wait timeouts
    uint16 TXF ;					//number of transmitted frames
	uint16 HPW ;					//half period warn
    uint16 TXW ;					//power up warn

} dwt_deviceentcnts_t ;

void dwt_enableRXTXtimeMesure(void);
uint32 dwt_getpartid(void);
uint32 dwt_getlotid(void);
uint32 dwt_readdevid(void);
uint8 dwt_otprevision(void);
void dwt_setGPIOforEXTTRX(void);
void dwt_setGPIOdirection(uint32 gpioNum, uint32 direction);
void dwt_setGPIOvalue(uint32 gpioNum, uint32 value);
int dwt_initialise(uint16 config) ;
int dwt_configure(dwt_config_t *config) ;
void dwt_configuretxrf(dwt_txconfig_t *config) ;
void dwt_setrxantennadelay(uint16 antennaDly);
void dwt_settxantennadelay(uint16 antennaDly);
void dwt_setsmarttxpower(int enable);
int dwt_writetxdata(uint16 txFrameLength, uint8 *txFrameBytes, uint16 txBufferOffset) ;
int dwt_writetxfctrl(uint16 txFrameLength, uint16 txBufferOffset) ;
int dwt_starttx(uint8 mode) ;
void dwt_setdelayedtrxtime(uint32 starttime) ;
void dwt_readtxtimestamp(uint8 * timestamp);
uint32 dwt_readtxtimestamphi32(void);
uint32 dwt_readtxtimestamplo32(void);
void dwt_readrxtimestamp(uint8 * timestamp);
uint32 dwt_readrxtimestamphi32(void);
uint32 dwt_readrxtimestamplo32(void);
uint32 dwt_readsystimestamphi32(void);
void dwt_readsystime(uint8 * timestamp);
int dwt_checkoverrun(void);
void dwt_forcetrxoff(void);
void dwt_syncrxbufptrs(void);
int dwt_rxenable(int delayed) ;

#define DWT_RX_NORMAL (0x0)
#define DWT_RX_SNIFF  (0x1)

void dwt_setrxmode(int mode, uint8 rxON, uint8 rxOFF);
void dwt_setautorxreenable(int enable);
void dwt_setdblrxbuffmode(int enable);
void dwt_setrxtimeout(uint16 time);
void dwt_setpreambledetecttimeout(uint16 timeout);
uint16 dwt_calibratesleepcnt(void);
 void dwt_configuresleepcnt(uint16 sleepcnt);
void dwt_configuresleep(uint16 mode, uint8 wake);
void dwt_entersleep(void);
void dwt_entersleepaftertx(int enable);
int dwt_spicswakeup(uint8 *buff, uint16 length);
void dwt_setcallbacks(void (*txcallback)(const dwt_callback_data_t *), void (*rxcallback)(const dwt_callback_data_t *));
uint8 dwt_checkIRQ(void);
void dwt_isr(void);
void dwt_setinterrupt( uint32 bitmask, uint8 enable);
void dwt_setpanid(uint16 panID);
void dwt_setaddress16(uint16 shortAddress);
void dwt_seteui(uint8 *eui64);
void dwt_geteui(uint8 *eui64);
void dwt_otpread(uint32 address, uint32 *array, uint8 length);
void dwt_enableframefilter(uint16 bitmask);
void dwt_enableautoack(uint8 responseDelayTime);
void dwt_setrxaftertxdelay(uint32 rxDelayTime);
void dwt_rxreset(void);
void dwt_softreset(void) ;
void dwt_readrxdata(uint8 *buffer, uint16 length, uint16 rxBufferOffset);
void dwt_readaccdata(uint8 *buffer, uint16 length, uint16 rxBufferOffset);
void dwt_readdiagnostics(dwt_rxdiag_t * diagnostics);
void  dwt_loadopsettabfromotp(uint8 gtab_sel);
void dwt_configeventcounters(int enable);
void dwt_readeventcounters(dwt_deviceentcnts_t *counters);
uint32 dwt_otpwriteandverify(uint32 value, uint16 address);
void dwt_setleds(uint8 test);
void dwt_xtaltrim(uint8 value);
int dwt_configcwmode(uint8 chan);
void dwt_configcontinuousframemode(uint32 framerepetitionrate);
uint16 dwt_readtempvbat(uint8 fastSPI);
uint8 dwt_readwakeuptemp(void) ;
uint8 dwt_readwakeupvbat(void) ;
int dwt_writetodevice             // returns 0 for success, or, -1 for error.
(
    uint16      recordNumber,   // input parameter - ID of register file or buffer being accessed
    uint16      index,          // input parameter - byte index into register file or buffer being accessed
    uint32      length,         // input parameter - number of bytes being written
    uint8 *buffer         // input parameter - pointer to buffer containing the 'length' bytes to be written
) ;
int dwt_readfromdevice            // returns offset where requested data begins in supplied buffer, or, -1 for error.
(
    uint16  recordNumber,       // input parameter - ID of register file or buffer being accessed
    uint16  index,              // input parameter - byte index into register file or buffer being accessed
    uint32  length,             // input parameter - number of bytes being read
    uint8   *buffer             // input parameter - pointer to buffer in which to return the read data.
) ;
uint32 dwt_read32bitoffsetreg(int regFileID, int regOffset) ;
int dwt_write32bitoffsetreg(int regFileID, int regOffset, uint32 regval);
uint16 dwt_read16bitoffsetreg(int regFileID, int regOffset);
int dwt_write16bitoffsetreg(int regFileID, int regOffset, uint16 regval) ;

#define dwt_write32bitreg(x,y)	dwt_write32bitoffsetreg(x,0,y)
#define dwt_read32bitreg(x)		dwt_read32bitoffsetreg(x,0)
extern int writetospi                          // returns 0 for success, or, -1 for error.
(
    uint16       headerLength,          // input parameter - number of bytes header being written
    uint8       *headerBuffer,          // input parameter - pointer to buffer containing the 'headerLength' bytes of header to be written
    uint32       bodylength,            // input parameter - number of bytes data being written
    uint8       *bodyBuffer             // input parameter - pointer to buffer containing the 'bodylength' bytes od data to be written
) ;

int writetospi1
(
    uint16       headerLength,
    uint8       *headerBuffer,
    uint32       bodylength,
    uint8       *bodyBuffer
);
extern int readfromspi                         // returns offset where requested data begins in supplied buffer, or, -1 for error.
(
    uint16       headerLength,          // input parameter - number of bytes header to write
    uint8        *headerBuffer,          // input parameter - pointer to buffer containing the 'headerLength' bytes of header to write
    uint32       readlength,            // input parameter - number of bytes data being read
    uint8       *readBuffer             // input parameter - pointer to buffer containing to return the data (NB: size required = headerLength + readlength)
) ;
double dwt_getrangebias(uint8 chan, float range, uint8 prf);
typedef int decaIrqStatus_t ; // Type for remembering IRQ status
decaIrqStatus_t decamutexon(void) ;
void decamutexoff(decaIrqStatus_t s) ;


//#ifdef __cplusplus
//}
//#endif

#endif /* _DECA_DEVICE_API_H_ */


