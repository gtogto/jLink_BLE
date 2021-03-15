#ifndef  LMS_VAR_H_INCLUDED
#define  LMS_VAR_H_INCLUDED

typedef struct {
	uint32_t 	init_state ;
	uint64_t 	MACaddress;
	uint8_t 	channel;
	uint8_t 	RF_profile;
	uint8_t 	data_rate;
	uint8_t 	preamble;
	uint8_t 	prf;
	uint8_t 	preamCode;
	uint8_t 	nSfd;
	uint32_t 	use_random_deviation;
	uint32_t 	frequency_of_Eblink;
	uint8_t 	motion_control_mode;
	uint32_t 	refresh_rate_ms;
	uint32_t 	no_motion_refresh_rate;
	uint32_t 	RX_period_ms;
	uint32_t 	RX_duration_ms;
	uint32_t 	tx_pwr_level;
	uint8_t 	tx_PG_delay;
} LMS_Parameters_t;
LMS_Parameters_t  LMSParameters;
LMS_Parameters_t  INITParameters;


volatile static uint8_t prepared_blink[128];


#include <stdint.h>
#include <stdbool.h>
#include "platform/setting.h"


typedef struct {
	int32_t temperature_MCU;            /**<mesured temperature of MCU. */
	uint32_t need_to_measure_battery;   /**<Counter for periodical battery measurement. */
	uint8_t lowbattcount;               /**<Counter of battery measurement with lower result than defined value. */
	uint8_t actual_batt_voltage_raw;    /**<Battery measurement - ADC output. */
	uint8_t actual_batt_voltage_percent;/**<Battery voltage converted to relative value.*/
	uint8_t platform;                   /**<Tag Leonardo edition on which this FW running. */
	uint16_t hw_ver;                    /**<Hardware version for which this HW is intended. */
	uint16_t fw_ver;                    /**<Version of firmware. */
	volatile bool is_shutdown;          /**<Flag which signalize, that the tag is in shutdown mod. */
} systemValues_t;

systemValues_t systemValues;            /**<Structure that contain general information about HW and FW. */

typedef struct
{
	int16_t x_axis;                     /**<Offset for X axis of some sensor. */
	int16_t y_axis;                     /**<Offset for Y axis of some sensor. */
	int16_t z_axis;                     /**<Offset for Z axis of some sensor. */
} sens_offsets_t;                       /**<Structure that contain offsets for  3-axis sensors.*/

typedef struct __attribute__((packed, aligned(2))){
	uint32_t init;                   /**<Check value - if 0 dta is not stored in EEPROM*/
	uint8_t channel;				 /**<Ultra wide band radio channel setting*/
	uint8_t RF_profile;				 /**<RF profile - Data rate, Preamble length, PRF, Preamble code, nSfd*/
	uint8_t data_rate;				 /**<Data rate (UWB radio) setting*/
	uint8_t preamble;				 /**<Preamble length setting*/
	uint8_t prf;				     /**<Pulse repetition frequency setting*/
	uint8_t preamCode;				 /**<Preamble code setting*/
	uint8_t nSfd;					 /**<Start frame delimiter setting*/
	uint8_t use_random_deviation;	 /**<Setting the random deviation for refresh rate*/
	uint8_t acc_mode;				 /**<Setting the mode of acceleration sleep*/
	uint16_t acc_sens;				 /**<Setting the threshold of acceleration for wakeup the tag from sleep*/
	uint32_t user_refresh_rate;		 /**<Setting the refresh rate*/
    uint32_t no_motion_refresh_rate; /**<Setting the refresh rate during no motion state*/
	uint32_t rx_period;			     /**<Setting the period of RX state (UWB radio) */
	uint32_t rx_duration;			 /**<Setting the duration of RX state (UWB radio) */
	uint32_t tx_pwr_level;			 /**<Transmitter power setting (UWB radio)*/
	uint8_t Eblink_cont;			 /**<Each bit in this byte determines that a particular sensor is active and its data is sent in eblink*/
	uint8_t sens_corr_data;			 /**<Each bit in this byte determines whether data from a particular sensor is being sent in a corrected form*/
	uint8_t GYRO_FS;				 /**<Gyroscope full scale range*/
	uint8_t ACC_FS;					 /**<Accelerometer full scale range*/
	uint8_t BARO_setting;			 /**<Barometer precision setting*/
	uint8_t AHRS_enable;			 /**<Cyclic redundancy check*/
	uint8_t mag_calib_mode;			 /**<Mode of magnetometer calibration*/
	uint16_t geo_mag_flux;			 /**<magnetic flux density of geo-magnetic field*/
	sens_offsets_t acc_offset;		 /**<Calculated offset of accelerometer data in all 3 axis*/
	sens_offsets_t mag_offset;		 /**<Calculated offset of magnetometer data in all 3 axis*/
	sens_offsets_t gyro_offset;		 /**<Calculated offset of gyroscope data in all 3 axis*/
	uint16_t crc;                    /**<Cyclic redundancy check*/
} userData_t;                        /**<Definition of structure of user defined settings.*/

userData_t userData;                 /**<Structure of user defined settings.*/


typedef struct {
	uint32_t RefreshRate_us; 				/**<TDOA refresh rate(blink period) in microseconds*/
	uint32_t RefreshRate_ms; 				/**<TDOA refresh rate(blink period) in milliseconds*/
	uint32_t RefreshRate_tck;  				/**<TDOA refresh rate(blink period) in RTC ticks*/
	uint32_t RRwaitTime_tck;				/**<Time to wait before next blink = RR - time need to sent blink - overhead time*/
	uint32_t RRwaitTime_tck_actual;			/**<Actual time for wait before next blink - used if random deviation is enabled*/
	uint32_t random_rr_value_actual;        /**<Wait time that will be used in current period*/
	uint32_t random_rr_value_old;           /**<Wait time that was used in last period*/
	uint32_t random_rr_deviation_tck;       /**<Deviation calculated for this blink period*/
	uint32_t random_rr_deviation_half_tck;  /**<Deviation calculated for this blink period divided by 2*/
	int32_t  diff_from_RR;                  /**<Difference between defined refresh rate and deviated RR*/
	uint32_t RR_calculated;                 /**<save the calculated refresh rate here*/


	uint32_t RefreshRate_no_mot_ms;		/**<TDOA refresh rate(blink period) in microseconds, during the no motion sleep*/
	uint32_t RefreshRate_no_mot_tck;	/**<TDOA refresh rate(blink period) in RTC ticks, during the no motion sleep*/


	uint32_t RR_in_sleep_calculated; 		/**<Calculated refresh rate in sleep*/


	uint32_t last_Blink_timestamp;			/**<Timestamp of last UWB blink - state of RTC*/

	uint32_t no_motion_delay;				/**<Delay between no motion detection and sleep state of tag*/

	uint32_t  tcks_to_RX;					/**<Ticks of RTC remaining to next RX state*/
	uint32_t RX_period_ms;					/**<Period of RX state (UWB radio) in milliseconds*/
	uint32_t RX_period_tck;					/**<Period of RX state (UWB radio) in RTC ticks*/
	uint32_t RX_duration_ms;				/**<Required duration of RX state (UWB radio) in milliseconds*/
	uint32_t last_RX_timestamp;			    /**<Timestamp of last RX state - RTC count*/
	bool 	 RX_required;					/**<Is RX state of UWB radio now required?*/
	bool 	 random_dev_en;					/**<Random deviation of TDOA refresh rate*/
} tdoaTiming_t;
tdoaTiming_t tdoaTiming;

typedef struct {
	uint32_t no_motion_cycle; 				/**<if motion ranging control mode is set to 1, MCU must wait several cycles (blink periods) before go to sleep*/
	uint32_t send_Eblink_after_RR;			/**<period of sending messages with battery level*/
	uint32_t RR_counter;					/**<counter of blink period*/
	uint32_t inf_b_counter;					/**<counter of battery blink period*/
	uint8_t seq_num;						/**<sequence number of current blink*/
	bool is_first_blink;					/**<determine if current blink is first after start of after reconfigure of tag*/
	bool is_info_blink_requested;		    /**<determine if next blink will be extended info blink*/
} tdoaValues_t;
tdoaValues_t tdoaValues;

typedef struct
{
	bool motion_action_detected;			/**<motion was detected by accelerometer*/
	bool pushbutton_pressed;				/**<user pushbutton was pressed*/
	bool blink_sent;						/**<sending of blink was completed*/
	bool UWB_msg_received;					/**<new config message was received via UWB*/
	bool wake_up;                           /**<generall wake_up flag */
	bool RTC_CMP0_achieved;					/**<RTC counter achieved to value of compare register 0 - RTC delay | RX required | TX reuired during EM2 sleep*/
	bool RTC_CMP1_achieved;					/**<RTC counter achieved to value of compare register 1 - used for measure of time of button pressed*/
	bool UART_rst_required;
}int_flag_t;

int_flag_t int_flag;						/**< interrupt requests flags*/


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

//
// Wireless Config RX message
//
// +-------+----------+--------+---------+----------+----------+-----+-----------+------+---------+----------+-------------+-------------+----+------------+----------+-----+
// | fcode | destAddr | seqNum | channel | datarate | preamble | prf | preamCode | nSfd | randDev | acc_mode |  EblinkCont | AHRS_enable | RR | no_mot_RR  | TX power | CRC |
// |  1B   |    6B    |   1B   |   1B    |    1B    |    1B    | 1B  |    1B     |  1B  |   1B    |   1B     |      1B     |      1B     | 4B |     1B     |    4B    | 2B  |
// +-------+----------+--------+---------+----------+----------+-----+-----------+------+---------+----------+-------------+-------------+----+------------+----------+-----+

typedef struct {
	uint8_t fcode;					  /**<Type of message - OxCC for wireless config message*/
	uint8_t destAddr[ADDR_BYTE_SIZE]; /**<Address of tag that should be set*/
	uint8_t seqNum;				       /**<Sequence number*/
	uint8_t channel;			/**<UWB radio channel*/
	uint8_t datarate;			/**<Data rate - UWB radio*/
	uint8_t preamble;			/**<Preamle length - UWB radio*/
	uint8_t prf;				/**<Pulse repetition frequency*/
	uint8_t preamCode;			/**<UWB radio preamble code*/
	uint8_t nSfd;				/**<Start frame delimiter - standard/non-standard*/
	uint8_t randomDev;			/**<Random deviation of TDOA refresh rate*/
	uint8_t acc_mode;			/**<Mode of sleep controled by acceleration of tag - no motion detection*/
	uint8_t EblinkCont;		 	/**<Setting of sensors whose data should be put into extended blink*/
	uint8_t AHRS_enable;		/**< Rotation determination setting*/
	uint8_t RR[4];				/**<TDOA refreshrate - period of UWB blink*/
    uint8_t no_mot_RR[4];		/**<TDOA refreshrate during no motion*/
	uint8_t tx_power[4];		/**<Transmiter power - UWB radio*/
	uint8_t	MCR_sens[2];		/**<Threshold of acceleration for wakeup tag from deep sleep - motion controled ranging*/
	uint8_t IMU_FS_range;		/**<Full scae range of gyroscope and accelerometer*/
	uint8_t RX_period[4];		/**<Period of RX state - UWB radio*/
	uint8_t RX_duration[2];		/**<Duration of RX state - UWB radio*/
	uint8_t BARO_setting;		/**<Barometer precision*/
	uint8_t CALIB_START;		/**<Sensors calbration start*/
	uint8_t crc[2];				/**<placeholder for CRC - calculated in UWB radio*/
} tdoa_uwb_conf_msg;
tdoa_uwb_conf_msg tdoa_uwb_confmsg;

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
/**
 * A structure to represent TDOA parameters which are USER/DEVELOPER definable
 */
typedef struct {
	uint64_t this_tag_MACaddress;		/**< MAC address of the tag*/
	uint8_t channel;					/**<UWB radio channel*/
	uint8_t RF_profile;					/**<UWB radio - RF profile - Data rate, preamble len, preamble code, PRF, nSfd*/
	uint8_t data_rate;					/**<UWB radio data rate*/
	uint8_t preamble;					/**<Preamble length - UWB radio*/
	uint8_t prf;						/**<Pulse repetition frequency - UWB radio*/
	uint8_t preamCode;					/**<Preamble code -UWB radio*/
	uint8_t nSfd;						/**<Start frame delimiter - UWB radio*/
	uint32_t use_random_deviation;		/**<Random deviation of TDOA refresh rate*/
	uint32_t frequency_of_Eblink;		/**<Frequency of battery blink*/
	uint8_t motion_control_mode;		/**<No motion detection mode - tag is in slee while tag is in no motion state*/
	uint32_t refresh_rate_ms;			/**<TDOA refreshrate - period of blink in miliseconds*/
	uint32_t no_motion_refresh_rate;    /**<TDOA refreshrate - during no motion state*/
	uint32_t RX_period_ms;				/**<Period of UWB RX state*/
	uint32_t RX_duration_ms;			/**<Duration of UWB RX state*/
	uint32_t tx_pwr_level;				/**<Power of UWB transmitter*/
	uint8_t tx_PG_delay;               /**<Propagation_delay of UWB transmitter*/
} tdoaParameters_t;
tdoaParameters_t tdoaParameters;		/**<Parameters of tdoa*/



typedef struct
{
	int16_t x_axis;                     /**<Data from sensors x-axis*/
	int16_t y_axis;                     /**<Data from sensors y-axis*/
	int16_t z_axis;                     /**<Data from sensors z-axis*/
} sens3D_data_t;						/**<Data from 3D sensor*/

typedef struct {
	int8_t q0;
	int8_t q1;
	int8_t q2;
	int8_t q3;
}quatQ8_t;							   /**<Quaternion in fixed point Q8 representation*/

typedef struct {
	float q0;
	float q1;
	float q2;
	float q3;
}quat_float_t;							/**<Quaternion in floating point representation*/

typedef struct {
	uint8_t yaw;                        /**<Yaw angle that describe rotation of tag around Z axis. */
	uint8_t pitch;                      /**<Pitch angle that describe rotation of tag around X axis. */
	uint8_t roll;                       /**<Roll angle that describe rotation of tag around Y axis. */
}TB_angles_t;                           /**<Tait-Bryan angles structure*/

typedef struct {
	uint8_t 	  ACC_mounted;			/**<if 0 accelerometer is not mounted, if 1 mounted accelerometer is MMA8453, if 2 mounted accelerometer is MPU9250*/
	bool    	  MAG_mounted;			/**<if true magnetometer is mounted on tag*/
	bool    	  GYRO_mounted;			/**<if true gyro is mounted on tag*/
	bool 		  BARO_mounted;
	uint8_t		  EBlink_cont;			/**<this byte determines which sensor data will be sent in extended blink*/
	uint8_t		  sens_corr_data;		/**<this byte determines if the data from sensors will be corrected or true RAW*/
	bool		  IMU_just_calib;		/**<true if IMU was just calibrated*/
	bool		  MAG_just_calib;		/**<true if magnetometer was just calibrated*/
	uint8_t		  AHRS_enable;          /**<determine if AHRS should be calculate*/
	uint8_t		  motion_control_mode;  /**<*/
	uint16_t	  wakeup_threshold ;	/**<threshold of acceleration to wake up MCU form acc sleep*/
	uint8_t		  gyro_FS;				/**<dynamic range of gyroscope*/
	uint8_t		  acc_FS;				/**<dynamic range of accelerometer*/
	uint8_t 	  BARO_setting;			/**<setting of BAROMETER*/
	bool          BARO_cont_mode;       /**<if true BAROMETER measure continuously in background mode, else Command mode*/
    bool          MAG_cont_mode;        /**<if true MAGNETOMETER measure continuously in background mode, else Command mode*/
    uint8_t 	  mag_calib_mode;		/**<mode of magnetometer calibration*/
    uint8_t	  	  ACC_LP_period;		/**<accelerometer Low power period in ms>*/
} sensors_params_t;
sensors_params_t sensors_params;





typedef struct {
	sens3D_data_t acc_offsets;			/**<offsets for each axes*/
	sens3D_data_t mag_offsets;			/**<offsets for each axes*/
	uint16_t	  geo_mag_flux;			/**<magnetic flux density of geo-magnetic field*/
	sens3D_data_t gyro_offsets;			/**<offsets for each axes*/
	sens3D_data_t acc_offsets_SC;		/**<offsets for each axes - scaled for chosen dynamic range*/
	sens3D_data_t gyro_offsets_SC;		/**<offsets for each axes - scaled for chosen dynamic range*/
	sens3D_data_t acc_data;				/**<accelerometer raw data*/
	sens3D_data_t mag_data;				/**<magnetometer raw data*/
	sens3D_data_t gyro_data;			/**<gyroscope raw data*/
	uint32_t	  baro_data;			/**<pressure*/
	int32_t		  baro_offset;			/**<pressure offset*/
	uint8_t		  temp_data;			/**<temperature from baro sensor*/
	TB_angles_t   angles;				/**<Tilt-Brian angles which determines orientation of tag*/
	quatQ8_t	  quaternion;			/**<quaternion which determines orientation of tag - stored in fixed point format Q8*/
	quat_float_t  f_quaternion;		    /**<quaternion which determines orientation of tag - stored in floating point format*/
} sensors_data_t;
sensors_data_t sensors_data;

//for validity check
#define RANDDEV_CNT								(2)
#define MCR_CNT										(4)
#define CONF_OK										(0)
#define ERR_CHANNEL								(0x00000001)      /**<Wrong channel -error code*/
#define ERR_DR										(0x00000002)
#define ERR_PREAMBLE								(0x00000003)
#define ERR_RANDDEV								(0x00000004)
#define ERR_MCR										(0x00000005)
#define ERR_RR										(0x00000006)
#define ERR_TXPOX									(0x00000007)
#define ERR_PRF										(0x00000008)
#define ERR_PREAMCODE								(0x00000009)
#define ERR_NSFD     									(0x0000000A)
#define ERR_ACC_FS									(0x0000000B)
#define ERR_AHRS									(0x0000000C)
#define ERR_BARO									(0x0000000D)
#define ERR_EBLINK									(0x0000000E)
#define ERR_GYRO_FS									(0x0000000F)
#define ERR_RX_PERIOD								(0x00000010)
#define ERR_RX_DURATION							(0x00000011)

//
// Data rates
//
#define DATARATE_COUNT 							(3)  /**<Number of usable data rates*/
#define DR0_INDEX 									(0)  /**<Data rate index for data_rate 110k*/
#define DR1_INDEX 									(1)  /**<Data rate index for data_rate 850k*/
#define DR2_INDEX 									(2)  /**<Data rate index for data_rate 6M8*/
#define PREAMBLE_COUNT								(8)  /**<Number of usable preamble lengths*/


#endif /* LMS_VAR_H_INCLUDED */

/**< Message Type IDs that is put into each extended blink into msgType byte  */
//#define MSGTYPE_EBLINK_INFO					    (0) /**<Message type with informations about tag HW,FW etc.. Since fw v3.116 it is changed to (7)*/
#define MSGTYPE_EBLINK_BATT							(1)  /**<Blink message with information about battery state.*/
#define MSGTYPE_EBLINK_ORIENT						(2)  /**<Blink message with information about spatial rotation of tag - that is represented as Tait-Bryan angles.*/
#define MSGTYPE_EBLINK_BATT_ORIENT					(3)  /**<Blink message with informations about battery state and spatial rotation of tag - that is represented as Tait-Bryan angles.*/
#define MSGTYPE_EBLINK_INFO_ORIENT					(4)  /**<Message type with information about tag HW,FW etc. and spatial rotation of tag - that is represented as Tait-Bryan angles.*/
#define MSGTYPE_EBLINK_SENS_RAW						(5)  /**<Message type with RAW data from sensors mounted on tag*/
#define MSGTYPE_EBLINK_BATT_SENS_RAW				(6)  /**<Message type with information about battery and with RAW data from sensors mounted on tag*/

//#define MSGTYPE_EBLINK_INFO							(7)//since fw v3.116 it is changed to (8)
#ifdef SEND_OLD_BLINK
#define MSGTYPE_EBLINK_INFO							(8)   /**<Message type with informations about tag HW,FW,user settings etc..*/
#endif
#define MSGTYPE_EBLINK_QUATERNION                   (9)   /**<Blink message with information about spatial rotation of tag - that is represented as quaternion in fixed point format*/
#define MSGTYPE_EBLINK_BATT_QUATERNION				(10)  /**<Blink message with information about battery level and spatial rotation of tag - that is represented as quaternion in fixed point format*/
#define MSGTYPE_EBLINK_QUAT_PRS                     (11)  /**<Blink message with information about spatial rotation of tag - that is represented as quaternion in fixed point format, furthermore that contain atmospheric pressure and temperature.*/
#define MSGTYPE_EBLINK_BATT_QUAT_PRS				(12)  /**<Blink message with information about battery level and spatial rotation of tag - that is represented as quaternion in fixed point format, furthermore that contain atmospheric pressure and temperature.*/
#define MSGTYPE_EBLINK_INFO							(13)  /**<Message type with informations about tag HW,FW,user settings etc..*/
#define MSGTYPE_EBLINK_CHARGING                     (252) /**<If the tag charged msg type is 252 after msg type the paylod defined by tag setting is appended*/
#define MSGTYPE_EBLINK_NO_MOTION                    (253) /**<If the tag is not in motion and still send blinks the msg type is 253 after msg type the paylod - defined by tag setting is appended*/
#define MSGTYPE_EBLINK_USER_PAYLOAD					(255) /**<Blink message with user defined payload*/
#define MSGTYPE_EBLINK_BATT_USER_PAYLOAD			(254) /**<Blink message with information about battery level with user defined payload*/


typedef struct {
	uint8_t fcode;						/**<Mark on the start of UWB message, which indicate that the message is blink -always 0xBB*/
	uint8_t srcAddr[ADDR_BYTE_SIZE]; 	/**<MAC address of tag*/
	uint8_t seqNum;						/**<Sequence number of blink*/
	uint8_t msgType;					/**<Message type [0x08]*/
	uint8_t battVoltage;				/**<Battery level in RAW form.*/
	uint8_t platform;					/**<Type of tag.*/
	uint8_t hw_ver[2];					/**<Hardware version and revision.*/
	uint8_t fw_ver[2];					/**<Firmware version and subversion.*/
	uint8_t channel;					/**<Used UWB radio channel*/
	uint8_t data_rate;					/**<UWB radio data rate*/
	uint8_t preamble;					/**<UWB radio - preamble code length*/
	uint8_t prf;						/**<UWB radio - pulse repetition frequency*/
	uint8_t preamCode;					/**<UWB radio preamble code*/
	uint8_t nSfd;						/**<Start frame delimiter*/
	uint8_t mcr;						/**<No motion sleep mode*/
	uint8_t random_dev;					/**<Random deviation of TDOA blink refreshrate*/
	uint8_t refresh_rate[4];			/**<TDOA Blink refresh rate*/
	uint8_t TXpower[4];					/**<UWB radio transmitter power*/
	uint8_t mounted_sensors;			/**<Each bite represent one sensor, if set : concrete sensor is mounted on tag*/
	uint8_t active_sensors;				/**<Each bite represent one sensor, if set : concrete sensor is active*/
	uint8_t mcr_threshold[2];			/**<No motion - acceleration threshold*/
	uint8_t IMU_FS_range;				/**<Inertial measurement unit (accelerometer and gyroscope full scale ranges*/
	uint8_t BARO_setting;				/**<Barometer precision setting*/
	uint8_t RX_period[4];				/**<RX period in milliseconds*/
	uint8_t RX_duration[2];				/**<RX duration in milliseconds*/
	uint8_t AHRS_representation;		/**<Spatial rotation representation*/
	uint8_t crc[2];						/**<Placeholder for UWB radio which calculate CRC - always set to [0x00 0x00]*/
} tdoa_uwb_eblink_info_msg;
tdoa_uwb_eblink_info_msg tx_uwb_message_tag_eblink_info;
