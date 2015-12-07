/* CWMCU.h - header file for CyWee digital 3-axis gyroscope
 *
 * Copyright (C) 2010 CyWee Group Ltd.
 * Author: Joe Wei <joewei@cywee.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __CWMCUSENSOR_H__
#define __CWMCUSENSOR_H__
#include <linux/ioctl.h>

#define CWMCU_I2C_NAME "CwMcuSensor"

enum ABS_status {
	CW_ABS_X = 0x01,
	CW_ABS_Y,
	CW_ABS_Z,
	CW_ABS_X1,
	CW_ABS_Y1,
	CW_ABS_Z1,
	CW_ABS_TIMEDIFF,
	CW_ABS_ACCURACY
};

enum MCU_mode {
	CW_NORMAL = 0x00,
	CW_SLEEP,
	CW_NO_SLEEP,
	CW_BOOT
};

/* power manager status */
typedef enum {
	SWITCH_POWER_ENABLE     = 0,
	SWITCH_POWER_DELAY,
	SWITCH_POWER_BATCH,
	SWITCH_POWER_NORMAL,
	SWITCH_POWER_CALIB,
	SWITCH_POWER_INTERRUPT
} SWITCH_POWER_ID;

/* interrupt status */
typedef enum {
	INTERRUPT_NON   = 0,
	INTERRUPT_INIT  = 1,
	INTERRUPT_GESTURE = 2,
	INTERRUPT_BATCHTIMEOUT   = 3,
	INTERRUPT_BATCHFULL   = 4,
	INTERRUPT_ERROR = 5,
	INTERRUPT_DATAREADY   = 6
} INTERRUPT_STATUS_LIST;

/* calibrator command */
typedef enum {
	CWMCU_ACCELERATION_CALIBRATOR = 0,
	CWMCU_MAGNETIC_CALIBRATOR,
	CWMCU_GYRO_CALIBRATOR,
	CWMCU_LIGHT_CALIBRATOR,
	CWMCU_PROXIMITY_CALIBRATOR,
	CWMCU_PRESSURE_CALIBRATOR,
	CWMCU_CALIBRATOR_STATUS = 6,
	CWMCU_CALIBRATOR_INFO = 7
} CALIBRATOR_CMD;

/* firmware update command */
typedef enum {
	CHANGE_TO_BOOTLOADER_MODE	= 1,
	ERASE_MCU_MEMORY,
	WRITE_MCU_MEMORY,
	MCU_GO,
	CHANGE_TO_NORMAL_MODE		= 5,
	CHECK_FIRMWAVE_VERSION,
	CHECK_ACC_DATA,
	CHECK_MAG_DATA,
	CHECK_GYRO_DATA,
	CHECK_GAME_R_DATA			= 10,
	CHECK_GEMO_R_DATA,
	CHECK_UNCALMAG_DATA,
	CHECK_UNCALGYRO_DATA,
	CHECK_HWID,
	CHECK_INFO					= 15,
	CHECK_MAG1_INFO,
	CHECK_MAG2_INFO,
	TEST,
	TEST2
} FIRMWARE_CMD;

/* sensor id */
typedef enum {
	CW_ACCELERATION					= 0,
	CW_MAGNETIC					= 1,
	CW_GYRO						= 2,
	CW_LIGHT					= 3,
	CW_PROXIMITY					= 4,
	CW_PRESSURE					= 5,
	CW_ORIENTATION					= 6,
	CW_ROTATIONVECTOR				= 7,
	CW_LINEARACCELERATION				= 8,
	CW_GRAVITY					= 9,
	CW_PEDOMETER					= 10,
	CW_AIRMOUSE					= 11,
	CW_SNAP						= 12,
	CW_SHAKE					= 13,
	CW_TAP						= 14,
	CW_PEDOMETER_DETECTOR				= 15,
	CW_MAGNETIC_UNCALIBRATED			= 16,
	CW_GYROSCOPE_UNCALIBRATED			= 17,
	CW_GAME_ROTATION_VECTOR				= 18,
	CW_GEOMAGNETIC_ROTATION_VECTOR			= 19,
	CW_SIGNIFICANT_MOTION				= 20,
	CW_FLIP								= 21,
	CW_TWIST							= 22,
	CW_TILT								= 23,
	CW_PDR								= 24,
	CW_FALLING							= 25,
	CW_RGB								= 26,
	CW_PROXIMITY_GESTURE				= 27,
	CW_CONTEXT_AWARE					= 28,
	CW_SCREEN_ON						= 29,
	CW_SENSORS_ID_END,
	CW_META_DATA						= 99,
	CW_MAGNETIC_UNCALIBRATED_BIAS		= 100,
	CW_GYROSCOPE_UNCALIBRATED_BIAS		= 101
} CW_SENSORS_ID;

#define CW_ENABLE_REG										0x01
#define CW_BATCH_ENABLE_REG									0x06

#define Cw_BATCHENABLE_STATUS								0x0A
#define CW_BATCHTIMEOUT										0x0B	/* 4Byte ms */
#define CW_BATCHFLUSH										0x0C
#define CW_INTERRUPT_STATUS									0x0F	/* 4Byte */

#define CW_DELAY_ACC										0x10	/* 1byte ms */
#define CW_DELAY_MAG										0x11
#define CW_DELAY_GRYO										0x12
#define CW_DELAY_LIGHT										0x13
#define CW_DELAY_PROXIMITY									0x14
#define CW_DELAY_PRESSURE									0x15
#define CW_DELAY_ORIENTATION								0x16
#define CW_DELAY_ROTATIONVECTOR								0x17
#define CW_DELAY_LINEEARACCELERATION						0x18
#define CW_DELAY_GRAVITY									0x19
#define CW_DELAY_PEDOMETER									0x1A
#define CW_DELAY_AIRMOUSE									0x1B
#define CW_DELAY_SNAP										0x1C
#define CW_DELAY_SHAKE										0x1D
#define CW_DELAY_TAP										0x1E
#define CW_DELAY_PEDOMETERDETECTOR							0x1F
#define CW_DELAY_MAGNETICUNCALIBRATED						0x20
#define CW_DELAY_GYROUNCALIBRATED							0x21
#define CW_DELAY_GAMEROTATIONVECTOR							0x22
#define CW_DELAY_GEOMANGETICROTATIONVECTOR					0x23
#define CW_DELAY_SIGNIFICANTMOTION							0x24
#define CW_DELAY_FLIP										0x25
#define CW_DELAY_TWIST										0x26
#define CW_DELAY_TILT										0x27
#define CW_DELAY_PDR										0x28
#define CW_DELAY_FALLING									0x29
#define CW_DELAY_RGB										0x2A
#define CW_DELAY_PROXIMITY_GESTURE							0x2B

#define CW_I2C_REG_SENSORS_GET_MAGINFO						0x2f

#define CW_READ_GESTURE_EVENT_COUNT							0x50
#define CW_READ_GESTURE_EVENT_DATA							0x51	/* read 2byte id: 1byte, data: 1byte */

#define CW_I2C_REG_SET_SYSTEM_EVENT							0x42
#define CW_I2C_REG_GET_SYSTEM_EVENT_DATA					0x52

/* batch mode register */
#define CW_BATCHCOUNT										0x55
#define CW_BATCHEVENTDATA									0x56

#define	CWMCU_I2C_SENSORS_REG_START							(0x60)

#define CW_READ_ACCELERATION					(CWMCU_I2C_SENSORS_REG_START + CW_ACCELERATION)
#define CW_READ_MAGNETIC						(CWMCU_I2C_SENSORS_REG_START + CW_MAGNETIC)
#define CW_READ_GYRO							(CWMCU_I2C_SENSORS_REG_START + CW_GYRO)
#define CW_READ_LIGHT   						(CWMCU_I2C_SENSORS_REG_START + CW_LIGHT)
#define CW_READ_PROXIMITY    					(CWMCU_I2C_SENSORS_REG_START + CW_PROXIMITY)
#define CW_READ_PRESSURE	    				(CWMCU_I2C_SENSORS_REG_START + CW_PRESSURE)
#define CW_READ_ORIENTATION    					(CWMCU_I2C_SENSORS_REG_START + CW_ORIENTATION)
#define CW_READ_ROTATIONVECTOR    				(CWMCU_I2C_SENSORS_REG_START + CW_ROTATIONVECTOR)
#define CW_READ_LINEARACCELERATION				(CWMCU_I2C_SENSORS_REG_START + CW_LINEARACCELERATION)
#define CW_READ_GRAVITY   						(CWMCU_I2C_SENSORS_REG_START + CW_GRAVITY)
#define CW_READ_PEDOMETER						(CWMCU_I2C_SENSORS_REG_START + CW_PEDOMETER)
#define CW_READ_AIRMOUSE						(CWMCU_I2C_SENSORS_REG_START + CW_AIRMOUSE)
#define CW_READ_MAGNETIC_UNCALIBRATED			(CWMCU_I2C_SENSORS_REG_START + CW_MAGNETIC_UNCALIBRATED)
#define CW_READ_GYROSCOPE_UNCALIBRATED			(CWMCU_I2C_SENSORS_REG_START + CW_GYROSCOPE_UNCALIBRATED)
#define CW_READ_GAME_ROTATION_VECTOR			(CWMCU_I2C_SENSORS_REG_START + CW_GAME_ROTATION_VECTOR)
#define CW_READ_GEOMAGNETIC_ROTATION_VECTOR		(CWMCU_I2C_SENSORS_REG_START + CW_GEOMAGNETIC_ROTATION_VECTOR)
#define CW_READ_TILT							(CWMCU_I2C_SENSORS_REG_START + CW_TILT)
#define CW_READ_PDR								(CWMCU_I2C_SENSORS_REG_START + CW_PDR)
#define CW_READ_RGB								(CWMCU_I2C_SENSORS_REG_START + CW_RGB)

#define CW_HWID_ACCELERATION								0X30
#define CW_HW_SLAVEADDRESS_ACCELERATION						0X31
#define CW_HWID_POSITION_ACCELERATION						0X32

#define CW_HWID_MAGNETIC									0X33
#define CW_HW_SLAVEADDRESS_MAGNETIC							0X34
#define CW_HWID_POSITION_MAGNETIC							0X35

#define CW_HWID_GYRO										0X36
#define CW_HW_SLAVEADDRESS_GYRO								0X37
#define CW_HWID_POSITION_GYRO								0X38

#define CW_HWID_LIGHT										0X39
#define CW_HW_SLAVEADDRESS_LIGHT							0X3A

#define CW_HWID_PROXIMITY									0X3B
#define CW_HW_SLAVEADDRESS_PROXIMITY						0X3C

#define CW_HWID_PRESSURE									0X3D
#define CW_HW_SLAVEADDRESS_PRESSURE							0X3E
#define CW_HWID_POSITION_PRESSURE							0X3F

#define CW_HWSTATUS											0X40

#define CW_ACCURACY											0X4F

#define CW_ERRORCOUNT										0X57
#define CW_ERRORLOG											0X58

#define CW_WATCHDOG											0x5A

#define CW_CALIBRATOR_TYPE									0x40
#define CW_CALIBRATOR_SENSORLIST							0x41
#define CW_CALIBRATOR_STATUS								0x42
#define CW_CALIBRATOR_BIAS_ACC								0x43
#define CW_CALIBRATOR_BIAS_MAG								0x44
#define CW_CALIBRATOR_BIAS_GYRO								0x45
#define CW_CALIBRATOR_BIAS_LIGHT							0x46
#define CW_CALIBRATOR_BIAS_PROXIMITY						0x47
#define CW_CALIBRATOR_BIAS_PRESSURE							0x48

#define CW_FWVERSION										0x80

#define CW_MAGINFO											0x91

/* check data of queue if queue is empty */
#define CWMCU_NODATA	0xff

#define DPS_MAX			(1 << (16 - 1))
#ifdef __KERNEL__

struct CWMCU_platform_data {
/*
	unsigned char Acceleration_hwid;
	unsigned char Acceleration_deviceaddr;
	unsigned char Acceleration_axes;
	unsigned char Magnetic_hwid;
	unsigned char Magnetic_deviceaddr;
	unsigned char Magnetic_axes;
	unsigned char Gyro_hwid;
	unsigned char Gyro_deviceaddr;
	unsigned char Gyro_axes;
*/
	int irq_gpio;
	int reset_gpio;
	int boot_gpio;
	int wake_up_gpio;
	int sleep_mcu_gpio;
	int mcu_busy_gpio;
	unsigned long irq_flags;
	const char *pwr_reg_name;
	const char *pwr18_reg_name;

};
#endif /* __KERNEL */

#endif /* __CWMCUSENSOR_H__ */
