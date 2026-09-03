/** @file
 *  @brief MAVLink comm protocol generated from gemini.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_GEMINI_H
#define MAVLINK_GEMINI_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_GEMINI.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_HASH
#define MAVLINK_THIS_XML_HASH 1902963619411886953

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {0, 8, 1, 1, 1, 1, 5, 1, 1, 4, 4, 12, 8, 2, 4, 8, 1, 5, 5, 7, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 74, 64, 32, 60, 32, 32, 32, 32, 56, 21, 5, 19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 14, 14, 46, 28, 27, 53, 77, 29, 176, 163, 92, 76, 42, 35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {0, 136, 198, 165, 248, 184, 215, 160, 226, 113, 38, 71, 67, 218, 44, 81, 181, 110, 22, 65, 79, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 50, 146, 57, 72, 87, 229, 245, 212, 140, 148, 6, 155, 87, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 117, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 183, 242, 142, 108, 133, 234, 66, 11, 214, 84, 245, 115, 63, 79, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_GEMINI

// ENUM DEFINITIONS


/** @brief Enum that lists all the system IDs of the various devices */
#ifndef HAVE_ENUM_SysIDs
#define HAVE_ENUM_SysIDs
typedef enum SysIDs
{
   MAV_SYSID_MAIN=1, /*  | */
   MAV_SYSID_PAYLOAD=2, /*  | */
   MAV_SYSID_RIG=3, /*  | */
   MAV_SYSID_GS=4, /*  | */
   SysIDs_ENUM_END=5, /*  | */
} SysIDs;
#endif

/** @brief Enum list for all the telemetries that can be requested */
#ifndef HAVE_ENUM_SystemTMList
#define HAVE_ENUM_SystemTMList
typedef enum SystemTMList
{
   MAV_SYS_ID=1, /* State of init results about system hardware/software components | */
   MAV_FSM_ID=2, /* States of all On-Board FSMs | */
   MAV_PIN_OBS_ID=3, /* Pin observer data | */
   MAV_LOGGER_ID=4, /* SD Logger stats | */
   MAV_MAVLINK_STATS=5, /* Mavlink driver stats | */
   MAV_TASK_STATS_ID=6, /* Task scheduler statistics answer: n mavlink messages where n is the number of tasks | */
   MAV_ADA_ID=7, /* ADA Status | */
   MAV_NAS_ID=8, /* NavigationSystem data | */
   MAV_MEA_ID=9, /* MEA Status | */
   MAV_CAN_ID=10, /* Canbus stats | */
   MAV_FLIGHT_ID=11, /* Flight telemetry | */
   MAV_STATS_ID=12, /* Satistics telemetry | */
   MAV_SENSORS_STATE_ID=13, /* Sensors init state telemetry | */
   MAV_GSE_ID=14, /* Ground Segnement Equipment | */
   MAV_MOTOR_ID=15, /* Rocket Motor data | */
   SystemTMList_ENUM_END=16, /*  | */
} SystemTMList;
#endif

/** @brief Enum list of all sensors telemetries that can be requested */
#ifndef HAVE_ENUM_SensorsTMList
#define HAVE_ENUM_SensorsTMList
typedef enum SensorsTMList
{
   MAV_GPS_ID=1, /* GPS data | */
   MAV_BMX160_ID=2, /* BMX160 IMU data | */
   MAV_VN100_ID=3, /* VN100 IMU data | */
   MAV_MPU9250_ID=4, /* MPU9250 IMU data | */
   MAV_ADS_ID=5, /* ADS 8 channel ADC data | */
   MAV_MS5803_ID=6, /* MS5803 barometer data | */
   MAV_BME280_ID=7, /* BME280 barometer data | */
   MAV_CURRENT_SENSE_ID=8, /* Electrical current sensors data | */
   MAV_LIS3MDL_ID=9, /* LIS3MDL compass data | */
   MAV_DPL_PRESS_ID=10, /* Deployment pressure data | */
   MAV_STATIC_PRESS_ID=11, /* Static pressure data | */
   MAV_PITOT_PRESS_ID=12, /* Pitot pressure data | */
   MAV_BATTERY_VOLTAGE_ID=13, /* Battery voltage data | */
   MAV_LOAD_CELL_ID=14, /* Load cell data | */
   MAV_FILLING_PRESS_ID=15, /* Filling line pressure | */
   MAV_TANK_TOP_PRESS_ID=16, /* Top tank pressure | */
   MAV_TANK_BOTTOM_PRESS_ID=17, /* Bottom tank pressure | */
   MAV_TANK_TEMP_ID=18, /* Tank temperature | */
   MAV_COMBUSTION_PRESS_ID=19, /* Combustion chamber pressure | */
   MAV_VESSEL_PRESS_ID=20, /* Vessel pressure | */
   MAV_LOAD_CELL_VESSEL_ID=21, /* Vessel tank weight | */
   MAV_LOAD_CELL_TANK_ID=22, /* Tank weight | */
   MAV_LIS2MDL_ID=23, /* Magnetometer data | */
   MAV_LPS28DFW_ID=24, /* Pressure sensor data | */
   MAV_LSM6DSRX_ID=25, /* IMU data | */
   MAV_H3LIS331DL_ID=26, /* 400G accelerometer | */
   MAV_LPS22DF_ID=27, /* Pressure sensor data | */
   SensorsTMList_ENUM_END=28, /*  | */
} SensorsTMList;
#endif

/** @brief Enum of the commands */
#ifndef HAVE_ENUM_MavCommandList
#define HAVE_ENUM_MavCommandList
typedef enum MavCommandList
{
   MAV_CMD_ARM=1, /* Command to arm the rocket | */
   MAV_CMD_DISARM=2, /* Command to disarm the rocket | */
   MAV_CMD_CALIBRATE=3, /* Command to trigger the calibration | */
   MAV_CMD_SAVE_CALIBRATION=4, /* Command to save the current calibration into a file | */
   MAV_CMD_FORCE_INIT=5, /* Command to init the rocket | */
   MAV_CMD_FORCE_LAUNCH=6, /* Command to force the launch state on the rocket | */
   MAV_CMD_FORCE_LANDING=7, /* Command to communicate the end of the mission and close the file descriptors in the SD card | */
   MAV_CMD_FORCE_APOGEE=8, /* Command to trigger the apogee event | */
   MAV_CMD_FORCE_EXPULSION=9, /* Command to open the nosecone | */
   MAV_CMD_FORCE_DEPLOYMENT=10, /* Command to activate the thermal cutters and cut the drogue, activating both thermal cutters sequentially | */
   MAV_CMD_START_LOGGING=11, /* Command to enable sensor logging | */
   MAV_CMD_STOP_LOGGING=12, /* Command to permanently close the log file | */
   MAV_CMD_FORCE_REBOOT=13, /* Command to reset the board from test status | */
   MAV_CMD_ENTER_TEST_MODE=14, /* Command to enter the test mode | */
   MAV_CMD_EXIT_TEST_MODE=15, /* Command to exit the test mode | */
   MAV_CMD_START_RECORDING=16, /* Command to start the internal cameras recordings | */
   MAV_CMD_STOP_RECORDING=17, /* Command to stop the internal cameras recordings | */
   MavCommandList_ENUM_END=18, /*  | */
} MavCommandList;
#endif

/** @brief Enum of all the servos used on Gemini */
#ifndef HAVE_ENUM_ServosList
#define HAVE_ENUM_ServosList
typedef enum ServosList
{
   AIR_BRAKES_SERVO=1, /*  | */
   EXPULSION_SERVO=2, /*  | */
   PARAFOIL_LEFT_SERVO=3, /*  | */
   PARAFOIL_RIGHT_SERVO=4, /*  | */
   MAIN_VALVE=5, /*  | */
   VENTING_VALVE=6, /*  | */
   RELEASE_VALVE=7, /*  | */
   FILLING_VALVE=8, /*  | */
   DISCONNECT_SERVO=9, /*  | */
   ServosList_ENUM_END=10, /*  | */
} ServosList;
#endif

/** @brief Enum of all the pins used on Gemini */
#ifndef HAVE_ENUM_PinsList
#define HAVE_ENUM_PinsList
typedef enum PinsList
{
   LAUNCH_PIN=1, /*  | */
   NOSECONE_PIN=2, /*  | */
   DEPLOYMENT_PIN=3, /*  | */
   QUICK_CONNECTOR_PIN=4, /*  | */
   PinsList_ENUM_END=5, /*  | */
} PinsList;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_ping_tc.h"
#include "./mavlink_msg_command_tc.h"
#include "./mavlink_msg_system_tm_request_tc.h"
#include "./mavlink_msg_sensor_tm_request_tc.h"
#include "./mavlink_msg_servo_tm_request_tc.h"
#include "./mavlink_msg_set_servo_angle_tc.h"
#include "./mavlink_msg_wiggle_servo_tc.h"
#include "./mavlink_msg_reset_servo_tc.h"
#include "./mavlink_msg_set_reference_altitude_tc.h"
#include "./mavlink_msg_set_reference_temperature_tc.h"
#include "./mavlink_msg_set_orientation_tc.h"
#include "./mavlink_msg_set_coordinates_tc.h"
#include "./mavlink_msg_raw_event_tc.h"
#include "./mavlink_msg_set_deployment_altitude_tc.h"
#include "./mavlink_msg_set_target_coordinates_tc.h"
#include "./mavlink_msg_set_algorithm_tc.h"
#include "./mavlink_msg_set_atomic_valve_timing_tc.h"
#include "./mavlink_msg_set_valve_maximum_aperture_tc.h"
#include "./mavlink_msg_conrig_state_tc.h"
#include "./mavlink_msg_set_ignition_time_tc.h"
#include "./mavlink_msg_ack_tm.h"
#include "./mavlink_msg_nack_tm.h"
#include "./mavlink_msg_gps_tm.h"
#include "./mavlink_msg_imu_tm.h"
#include "./mavlink_msg_pressure_tm.h"
#include "./mavlink_msg_adc_tm.h"
#include "./mavlink_msg_voltage_tm.h"
#include "./mavlink_msg_current_tm.h"
#include "./mavlink_msg_temp_tm.h"
#include "./mavlink_msg_load_tm.h"
#include "./mavlink_msg_attitude_tm.h"
#include "./mavlink_msg_sensor_state_tm.h"
#include "./mavlink_msg_servo_tm.h"
#include "./mavlink_msg_pin_tm.h"
#include "./mavlink_msg_receiver_tm.h"
#include "./mavlink_msg_sys_tm.h"
#include "./mavlink_msg_fsm_tm.h"
#include "./mavlink_msg_logger_tm.h"
#include "./mavlink_msg_mavlink_stats_tm.h"
#include "./mavlink_msg_task_stats_tm.h"
#include "./mavlink_msg_ada_tm.h"
#include "./mavlink_msg_nas_tm.h"
#include "./mavlink_msg_mea_tm.h"
#include "./mavlink_msg_rocket_flight_tm.h"
#include "./mavlink_msg_payload_flight_tm.h"
#include "./mavlink_msg_rocket_stats_tm.h"
#include "./mavlink_msg_payload_stats_tm.h"
#include "./mavlink_msg_gse_tm.h"
#include "./mavlink_msg_motor_tm.h"

// base include


#undef MAVLINK_THIS_XML_HASH
#define MAVLINK_THIS_XML_HASH 1902963619411886953

#if MAVLINK_THIS_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {{"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_PING_TC, MAVLINK_MESSAGE_INFO_COMMAND_TC, MAVLINK_MESSAGE_INFO_SYSTEM_TM_REQUEST_TC, MAVLINK_MESSAGE_INFO_SENSOR_TM_REQUEST_TC, MAVLINK_MESSAGE_INFO_SERVO_TM_REQUEST_TC, MAVLINK_MESSAGE_INFO_SET_SERVO_ANGLE_TC, MAVLINK_MESSAGE_INFO_WIGGLE_SERVO_TC, MAVLINK_MESSAGE_INFO_RESET_SERVO_TC, MAVLINK_MESSAGE_INFO_SET_REFERENCE_ALTITUDE_TC, MAVLINK_MESSAGE_INFO_SET_REFERENCE_TEMPERATURE_TC, MAVLINK_MESSAGE_INFO_SET_ORIENTATION_TC, MAVLINK_MESSAGE_INFO_SET_COORDINATES_TC, MAVLINK_MESSAGE_INFO_RAW_EVENT_TC, MAVLINK_MESSAGE_INFO_SET_DEPLOYMENT_ALTITUDE_TC, MAVLINK_MESSAGE_INFO_SET_TARGET_COORDINATES_TC, MAVLINK_MESSAGE_INFO_SET_ALGORITHM_TC, MAVLINK_MESSAGE_INFO_SET_ATOMIC_VALVE_TIMING_TC, MAVLINK_MESSAGE_INFO_SET_VALVE_MAXIMUM_APERTURE_TC, MAVLINK_MESSAGE_INFO_CONRIG_STATE_TC, MAVLINK_MESSAGE_INFO_SET_IGNITION_TIME_TC, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_ACK_TM, MAVLINK_MESSAGE_INFO_NACK_TM, MAVLINK_MESSAGE_INFO_GPS_TM, MAVLINK_MESSAGE_INFO_IMU_TM, MAVLINK_MESSAGE_INFO_PRESSURE_TM, MAVLINK_MESSAGE_INFO_ADC_TM, MAVLINK_MESSAGE_INFO_VOLTAGE_TM, MAVLINK_MESSAGE_INFO_CURRENT_TM, MAVLINK_MESSAGE_INFO_TEMP_TM, MAVLINK_MESSAGE_INFO_LOAD_TM, MAVLINK_MESSAGE_INFO_ATTITUDE_TM, MAVLINK_MESSAGE_INFO_SENSOR_STATE_TM, MAVLINK_MESSAGE_INFO_SERVO_TM, MAVLINK_MESSAGE_INFO_PIN_TM, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_RECEIVER_TM, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_SYS_TM, MAVLINK_MESSAGE_INFO_FSM_TM, MAVLINK_MESSAGE_INFO_LOGGER_TM, MAVLINK_MESSAGE_INFO_MAVLINK_STATS_TM, MAVLINK_MESSAGE_INFO_TASK_STATS_TM, MAVLINK_MESSAGE_INFO_ADA_TM, MAVLINK_MESSAGE_INFO_NAS_TM, MAVLINK_MESSAGE_INFO_MEA_TM, MAVLINK_MESSAGE_INFO_ROCKET_FLIGHT_TM, MAVLINK_MESSAGE_INFO_PAYLOAD_FLIGHT_TM, MAVLINK_MESSAGE_INFO_ROCKET_STATS_TM, MAVLINK_MESSAGE_INFO_PAYLOAD_STATS_TM, MAVLINK_MESSAGE_INFO_GSE_TM, MAVLINK_MESSAGE_INFO_MOTOR_TM, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}}
# define MAVLINK_MESSAGE_NAMES {{ "ACK_TM", 100 }, { "ADA_TM", 205 }, { "ADC_TM", 105 }, { "ATTITUDE_TM", 110 }, { "COMMAND_TC", 2 }, { "CONRIG_STATE_TC", 19 }, { "CURRENT_TM", 107 }, { "FSM_TM", 201 }, { "GPS_TM", 102 }, { "GSE_TM", 212 }, { "IMU_TM", 103 }, { "LOAD_TM", 109 }, { "LOGGER_TM", 202 }, { "MAVLINK_STATS_TM", 203 }, { "MEA_TM", 207 }, { "MOTOR_TM", 213 }, { "NACK_TM", 101 }, { "NAS_TM", 206 }, { "PAYLOAD_FLIGHT_TM", 209 }, { "PAYLOAD_STATS_TM", 211 }, { "PING_TC", 1 }, { "PIN_TM", 113 }, { "PRESSURE_TM", 104 }, { "RAW_EVENT_TC", 13 }, { "RECEIVER_TM", 150 }, { "RESET_SERVO_TC", 8 }, { "ROCKET_FLIGHT_TM", 208 }, { "ROCKET_STATS_TM", 210 }, { "SENSOR_STATE_TM", 111 }, { "SENSOR_TM_REQUEST_TC", 4 }, { "SERVO_TM", 112 }, { "SERVO_TM_REQUEST_TC", 5 }, { "SET_ALGORITHM_TC", 16 }, { "SET_ATOMIC_VALVE_TIMING_TC", 17 }, { "SET_COORDINATES_TC", 12 }, { "SET_DEPLOYMENT_ALTITUDE_TC", 14 }, { "SET_IGNITION_TIME_TC", 20 }, { "SET_ORIENTATION_TC", 11 }, { "SET_REFERENCE_ALTITUDE_TC", 9 }, { "SET_REFERENCE_TEMPERATURE_TC", 10 }, { "SET_SERVO_ANGLE_TC", 6 }, { "SET_TARGET_COORDINATES_TC", 15 }, { "SET_VALVE_MAXIMUM_APERTURE_TC", 18 }, { "SYSTEM_TM_REQUEST_TC", 3 }, { "SYS_TM", 200 }, { "TASK_STATS_TM", 204 }, { "TEMP_TM", 108 }, { "VOLTAGE_TM", 106 }, { "WIGGLE_SERVO_TC", 7 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_GEMINI_H
