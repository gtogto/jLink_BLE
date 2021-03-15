#ifndef SETTING_H_INCLUDED
#define SETTING_H_INCLUDED

#define TAG_LIION   (0)
#define TAG_PICO    (1)
#define TAG_2_IMU   (2)
#define TAG_2_LITE  (3)

//#define PLATFORM    TAG_2_IMU
//#define PLATFORM    TAG_2_LITE

// Software version
//
#define SW_VER									(3)		//TDOA 1st stage
#define SW_SUBVER								(124) 	//this need to be incremented if changes has been done
#define SW_REV									(6)		// thats are prerelease revisions

#define HW_VERSION								(1) //
#define HW_REVISION								(3) //
#define HW_VER									HW_VERSION
#define HW_REV								    HW_REVISION

//#define USER_PAYLOAD_EN
//#define USER_PAYLOAD_SIZE   4


#if PLATFORM == TAG_2_LITE
// currently the RX configuration message is sent every 50 ms //
#define WAIT_FOR_CONF_RX_ON_TIME				(100)//ms
#define WAIT_FOR_CONF_RX_SLEEP_TIME				(200)//ms
#define WAIT_FOR_CONF_RX_COUNT					(3)
#define WAIT_TIME_TO_RX_AFTER_ON                (2000)//ms
#define ENABLE_WIRELESS_CONFIG_RX				(1)
#define ACCELEROMETER_IN_USE

#define ACCELEROMETER_IN_USE
#define BAROMETER_IN_USE






#elif PLATFORM == TAG_2_IMU

#define WAIT_FOR_CONFIG_TIMEOUT                 (1000)

#define ACCELEROMETER_IN_USE
#define BAROMETER_IN_USE
#define GYROSCOPE_IN_USE
#define MAGNETOMETER_IN_USE


#endif





#define DEVELOP     (0)
#define RELEASE     (1)

#define PRINT_ROLE  DEVELOP

#ifndef NULL
#define NULL 0
#endif

#define WDOG_IN_USE

//
// Pushbuttons
//
#define USER_PB_SHORTPRESS_LIMIT				(500UL) //ms
#define USER_PB_LONGPRESS_LIMIT				    (USER_PB_SHORTPRESS_LIMIT*4UL) //ms
#define USER_PB_VLONGPRESS_LIMIT				(10000-USER_PB_LONGPRESS_LIMIT) //ms


//
//Watch dog
//
#define WD_GUARD_INTERVAL_US                    100000UL

//
// Info blink settings
//
#define DELAY_BETWEEN_INFOBLINKS				(1000)//ms
#define BEFORE1STBLINKDELAY						(152) //cca 5ms
#define NUM_OF_IBLINK_SENDIG_AT_THE_BEGIN		(3)//n

// Frequency of EBLINK Batt send
//
#define EXTENDED_BLINK_BATT_FREQ				(15)//RR x n

// Frequency of IBLINK - complete info blink to be send every xth BATT BLINK
#define IBLINK_FREQ                             (225)// EXTENDED_BLINK_BATT_FREQ * 15

// Uncomment for enable USER_PAYLOAD
// #define USER_PAYLOAD_EN

//
// Battery management
//
#if (PLATFORM == TAG_2_IMU)
#define MEAS_BATT_AFTER_RR						(15) //cycle of RR
#define VCC_BATT_MINLEVEL						(3200) //mV for periodical ADC check
#define VCC_BATT_MINLEVEL_FOR_DFU               (3300) //mV minimal voltage for reliable reception of new FW over BLE
#define MIN_ALLOWED_BATT_VOLTAGE				163//((VCC_BATT_MINLEVEL * 128)/2500) // 163.84 => 163 TODO
#define MAX_ALLOWED_BATT_VOLTAGE          		212 // 4.18 V
#define WAKE_UP_VOLTAGE_DURING_CHARGING         (3300*128/2500)
#define MIN_ALLOWED_BATT_VOLTAGE_FOR_DFU        ((VCC_BATT_MINLEVEL_FOR_DFU  * 128)/2500)
#define SLEEP_AFTER_LOW_BATT_MEASURE			(3) //number of allowed low battery measure. after this the tag goes to sleep due to low battery
#define GO_TO_SLEEP_IF_LOW_STARTUP_VOLTAGE
#define MEAS_BATT_DURING_CHARGE_PERIOD_SEC      (120)
#define MEAS_BATT_DURING_CHARGE_PERIOD_TCKS     (MEAS_BATT_DURING_CHARGE_PERIOD_SEC * 32768)
#elif (PLATFORM == TAG_2_LITE)
#define MEAS_BATT_AFTER_RR						(15) //cycle of RR
#define VCC_MCU_USB_MINLEVEL					(2100)//mV for startup
#define VCC_BATT_MINLEVEL						(2100) //mV for periodical ADC check
#define SLEEP_AFTER_LOW_BATT_MEASURE			(3) //number of allowed low battery measure. after this the tag goes to sleep due to low battery
#define VCC_BATT_MINLEVEL_FOR_RX                (2900) //mV minimal voltage for reliable reception of RX config
#define VCC_BATT_MINLEVEL_FOR_DFU               (2500) //mV minimal voltage for reliable reception of new FW over BLE
#define MIN_ALLOWED_BATT_VOLTAGE				142//((VCC_BATT_MINLEVEL        * 255)/3750)//((VCC_BATT_MINLEVEL * 128)/2500) TODO
#define MAX_ALLOWED_BATT_VOLTAGE				214 // 3.14 V
#define MIN_ALLOWED_BATT_VOLTAGE_CR_FOR_RX      ((VCC_BATT_MINLEVEL_FOR_RX * 256)/3750) // ((Voltage * range)/ reference(1.25V) * 3 - due to used divider)
#define MIN_ALLOWED_BATT_VOLTAGE_FOR_DFU        ((VCC_BATT_MINLEVEL_FOR_DFU * 256)/3750) // ((Voltage * range)/ reference(1.25V) * 3 - due to used divider)
#endif

#define RR_DURING_CHARGE_MS                     (30000)


//======================================================//
//----------------- TDOA SYSTEM PARAMETERS -------------//
//======================================================//


#define ADDR_BYTE_SIZE    							(6)
#define ADDR_BYTE_SIZE_ANCH							(2)
#define TAG_BROADCAST_ADDR							(0xFFFFFFFFFFFF)
#define MAC_ADDR_48b_MASK                           (0xFFFFFFFFFFFFULL)

#define UWB_FCODE_BLINK                             (0xBB)
#define UWB_FCODE_CONF								(0xCC)


////////////////BLINK LIMIT REFRESHRATES///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MAX_USER_REFRESH_RATE_MS 			(60000)//(10000) //ms /////1 min
#define MAX_NO_MOT_REFRESH_RATE_MS 			(129600000) //ms /////1 hod
#if (PLATFORM == TAG_2_IMU)
#define MIN_USER_REFRESH_RATE_MS			(10)//ms
#define MIN_NO_MOT_REFRESH_RATE_MS          (MIN_USER_REFRESH_RATE_MS)
#elif (PLATFORM == TAG_2_LITE)
#define MIN_USER_REFRESH_RATE_MS			(50)//ms - CR TAG MINIMAL RR
#define MIN_NO_MOT_REFRESH_RATE_MS          (MIN_USER_REFRESH_RATE_MS)
#endif

////////////////PERIODICAL UWB RX///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if (PLATFORM == LIION_TAG) || (PLATFORM == TAG_2_IMU)
#define PERIODICAL_RX_EN
#define MIN_RX_PERIOD_MS 							(120)//ms
#define MAX_RX_PERIOD_MS							(64800000)//ms == 18 hours
#define MIN_RX_DURATION_MS							(50)
#define MAX_RX_DURATION_MS							(10000)
#define CONSTANT									(0)
#define VARIABLE									(1)
#define RX_PERIOD									VARIABLE
#endif

//
// Timeout to run with Accelerometer without motion detected
//
#define	ACC_TIME_TO_WAIT_INMOTION				(15000) //ms
#define	RR_IN_IDLE_MODE							(5000) //ms


//
// Acceleration thresholds for wakeup from sleep                                                //TODO insert correct value of LIS2DH12 thresholds
//
#define LIS2DH12_THRESHOLD_STEP		(16)	//it must be set in multiplies of 16
#define LIS2DH12_LOW_THRESHOLD		(48)
#define LIS2DH12_MIN_THRESHOLD		(LIS2DH12_LOW_THRESHOLD)
#define LIS2DH12_MEDIUM_THRESHOLD	(256)
#define LIS2DH12_HIGH_THRESHOLD		(1024)
#define LIS2DH12_MAX_THRESHOLD		(8001)


#define LOW_THRESHOLD_FLAG			(1 << 14)
#define MEDIUM_THRESHOLD_FLAG		(2 << 14)
#define HIGH_THRESHOLD_FLAG			(3 << 14)
#define PREDEFINED_THRESHOLD_MASK	(3 << 14)



#define EEPROM_NUM_OF_RW_ATTEMPTS  (3)
//
//non volatile memory layout
//
#define USER_DATA_BASE          (1)
#define USER_DATA_TOP           (16)
#define RESERVED_BASE           (17)
#define RESERVED_TOP            (30)
#define NEW_SETTING_BASE_ADDR   (31)
#define NEW_SETTING_TOP_ADDR    (38)
#define USER_SET_BASE_ADDR      (39)
#define USER_SET_TOP_ADDR       (46)
#define DEF_SET_BASE_ADDR       (47)
#define DEF_SET_TOP_ADDR        (54)

#define TAG_INFO_ADDR           (55)



#define SETTING_INIT_CHECK  ((SW_VER << 16)| (SW_SUBVER << 8) | (SW_REV))

#endif /* SETTING_H_INCLUDED */
