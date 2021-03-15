#ifndef DEFAULT_SET_H_INCLUDED
#define DEFAULT_SET_H_INCLUDED

#include "setting.h"

/** Default UWB channel. */
#define CHANNEL_DEFAULT								(2)

/** Default RF_profile */
#define RF_PROFILE_DEFAULT							(4)

/** Default data rate [1 -> 850k]  */
#define DR_MODE_DEFAULT								(1)

/** Default preamble length [5 -> 256 b]  */
#define PREAMBLE_DEFAULT							(5)

/** Default Pulse Repetition Frequency [1 -> 64 MHz]  */
#define PRF_DEFAULT									(1)

/** Default Preamble code [12]*/
#define PREAM_CODE_DEFAULT							(12)

/** Default (Non) Standard Frame Delimiter ([1 -> non-standard]*/
#define NSFD										(1)

/** Default setting for random deviation of refresh rate [1 -> enable] */
#define RANDOM_DEVIATION_DEFAULT					(1)

/** Default setting for sleep modes - motion controlled ranging [enable -immediate]*/
#define MCR_LEVEL_DEFAULT							(2)

/** Default threshold of acceleration for LIION IMU tag[12 mg] */
#define MCR_SENS_IMU_DEFAULT						(12UL | LOW_THRESHOLD_FLAG)

/** Default threshold of acceleration for CR & LIION tag[63 mg] */
#define MCR_SENS_DEFAULT							(378UL)//(63UL | LOW_THRESHOLD_FLAG)

/** Default setting for sensors data sending  [0 -> all disabled] */
#define EBLINK_DEFAULT                              (0)

/** Default setting for Spatial Rotation determination [0 -> disable] */
#define AHRS_DEFAULT                                (0)

/** Default setting for RX period [0 -> disable] */
#define RX_PERIOD_DEFAULT							(0)

/** Default setting for RX state duration [1 s] */
#define RX_DURATION_DEFAULT							(1000) // [ms]

#if PLATFORM == TAG_2_IMU
/** Default setting refresh rate [1 s] */
#define REFRESH_RATE_DEFAULT						(1000) //[ms]

#elif PLATFORM == TAG_2_LITE
#define REFRESH_RATE_DEFAULT						(10000) //[ms]
#endif

/** Default setting no motion refresh rate [0 -> disable] */
#define NO_MOT_REFRESH_RATE_DEFAULT                 (0)  //[ms]

/** Default setting for barometer precision [0 ->disable] */
#define BARO_DEFAULT     							(0x00)

/** Default setting for gyroscope full scale range [0 -> +-250 dps] */
#define GYRO_FS_DEFAULT								(0)

/** Default setting for accelerometer full scale range [0 -> +-2 g] */
#define ACC_FS_DEFAULT								(0)


/** Default setting for magnetometer calibration mode [0 -> Standard] */
#define MAG_CALIB_MODE_DEFAULT						(0)

/** Default setting for UWB radio transmitter power [0x75757575 -> nominal power] */
#define TX_LEVEL_DEFAULT							(0x75757575)


/** @} (end group Settings) */
/** @} (end group default_setting) */


#endif /* DEFAULT_SET_H_INCLUDED */
