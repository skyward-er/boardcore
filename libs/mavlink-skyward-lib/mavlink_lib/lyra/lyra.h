/** @file
 *  @brief MAVLink comm protocol generated from lyra.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_LYRA_H
#define MAVLINK_LYRA_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_LYRA.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_HASH
#define MAVLINK_THIS_XML_HASH -5660799532835485979

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {0, 8, 1, 1, 1, 1, 5, 1, 1, 4, 4, 12, 16, 8, 2, 4, 8, 1, 4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 7, 4, 4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 5, 12, 12, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 4, 4, 74, 64, 32, 60, 32, 32, 32, 32, 56, 22, 5, 19, 36, 36, 36, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 104, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 46, 28, 20, 44, 53, 77, 29, 178, 158, 180, 170, 64, 37, 76, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {0, 136, 198, 165, 248, 184, 215, 226, 160, 113, 38, 71, 168, 67, 218, 44, 81, 181, 199, 221, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 110, 22, 65, 79, 167, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 180, 246, 173, 183, 220, 181, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 50, 251, 51, 57, 72, 87, 229, 245, 212, 140, 148, 6, 165, 87, 255, 103, 9, 68, 234, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 239, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 48, 142, 108, 39, 19, 234, 66, 11, 235, 188, 195, 99, 87, 67, 210, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_LYRA

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
   MAV_SYSID_ARP=5, /*  | */
   MAV_SYSID_GS_BACKUP=6, /*  | */
   MAV_SYSID_ARP_BACKUP=7, /*  | */
   SysIDs_ENUM_END=8, /*  | */
} SysIDs;
#endif

/** @brief Enum list for all the telemetries that can be requested */
#ifndef HAVE_ENUM_SystemTMList
#define HAVE_ENUM_SystemTMList
typedef enum SystemTMList
{
   MAV_SYS_ID=1, /* State of init results about system hardware/software components | */
   MAV_PIN_OBS_ID=2, /* Pin observer data | */
   MAV_LOGGER_ID=3, /* SD Logger stats | */
   MAV_MAVLINK_STATS_ID=4, /* Mavlink driver stats | */
   MAV_TASK_STATS_ID=5, /* Task scheduler statistics answer: n mavlink messages where n is the number of tasks | */
   MAV_ADA_ID=6, /* ADA Status | */
   MAV_NAS_ID=7, /* NavigationSystem data | */
   MAV_MEA_ID=8, /* MEA Status | */
   MAV_CAN_STATS_ID=9, /* Canbus stats | */
   MAV_FLIGHT_ID=10, /* Flight telemetry | */
   MAV_STATS_ID=11, /* Satistics telemetry | */
   MAV_SENSORS_STATE_ID=12, /* Sensors init state telemetry | */
   MAV_GSE_ID=13, /* Ground Segnement Equipment | */
   MAV_MOTOR_ID=14, /* Rocket Motor data | */
   MAV_REGISTRY_ID=15, /* Command to fetch all registry keys | */
   MAV_REFERENCE_ID=16, /* Command to fetch reference values | */
   MAV_CALIBRATION_ID=17, /* Command to fetch calibration values | */
   SystemTMList_ENUM_END=18, /*  | */
} SystemTMList;
#endif

/** @brief Enum list of all sensors telemetries that can be requested */
#ifndef HAVE_ENUM_SensorsTMList
#define HAVE_ENUM_SensorsTMList
typedef enum SensorsTMList
{
   MAV_BMX160_ID=1, /* BMX160 IMU data | */
   MAV_VN100_ID=2, /* VN100 IMU data | */
   MAV_MPU9250_ID=3, /* MPU9250 IMU data | */
   MAV_ADS131M08_ID=4, /* ADS 8 channel ADC data | */
   MAV_MS5803_ID=5, /* MS5803 barometer data | */
   MAV_BME280_ID=6, /* BME280 barometer data | */
   MAV_LIS3MDL_ID=7, /* LIS3MDL compass data | */
   MAV_LIS2MDL_ID=8, /* Magnetometer data | */
   MAV_LPS28DFW_ID=9, /* Pressure sensor data | */
   MAV_LSM6DSRX_ID=10, /* IMU data | */
   MAV_H3LIS331DL_ID=11, /* 400G accelerometer | */
   MAV_LPS22DF_ID=12, /* Pressure sensor data | */
   MAV_GPS_ID=13, /* GPS data | */
   MAV_CURRENT_SENSE_ID=14, /* Electrical current sensors data | */
   MAV_BATTERY_VOLTAGE_ID=15, /* Battery voltage data | */
   MAV_ROTATED_IMU_ID=16, /* Load cell data | */
   MAV_DPL_PRESS_ID=17, /* Deployment pressure data | */
   MAV_STATIC_PRESS_ID=18, /* Static pressure data | */
   MAV_BACKUP_STATIC_PRESS_ID=19, /* Static pressure data | */
   MAV_STATIC_PITOT_PRESS_ID=20, /* Pitot pressure data | */
   MAV_TOTAL_PITOT_PRESS_ID=21, /* Pitot pressure data | */
   MAV_DYNAMIC_PITOT_PRESS_ID=22, /* Pitot pressure data | */
   MAV_LOAD_CELL_ID=23, /* Load cell data | */
   MAV_TANK_TOP_PRESS_ID=24, /* Top tank pressure | */
   MAV_TANK_BOTTOM_PRESS_ID=25, /* Bottom tank pressure | */
   MAV_TANK_TEMP_ID=26, /* Tank temperature | */
   MAV_COMBUSTION_PRESS_ID=27, /* Combustion chamber pressure | */
   MAV_FILLING_PRESS_ID=28, /* Filling line pressure | */
   MAV_VESSEL_PRESS_ID=29, /* Vessel pressure | */
   MAV_LOAD_CELL_VESSEL_ID=30, /* Vessel tank weight | */
   MAV_LOAD_CELL_TANK_ID=31, /* Tank weight | */
   SensorsTMList_ENUM_END=32, /*  | */
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
   MAV_CMD_FORCE_ENGINE_SHUTDOWN=7, /* Command to trigger engine shutdown | */
   MAV_CMD_FORCE_EXPULSION=8, /* Command to trigger nosecone expulsion | */
   MAV_CMD_FORCE_DEPLOYMENT=9, /* Command to activate the thermal cutters and cut the drogue, activating both thermal cutters sequentially | */
   MAV_CMD_FORCE_LANDING=10, /* Command to communicate the end of the mission and close the file descriptors in the SD card | */
   MAV_CMD_START_LOGGING=11, /* Command to enable sensor logging | */
   MAV_CMD_STOP_LOGGING=12, /* Command to permanently close the log file | */
   MAV_CMD_FORCE_REBOOT=13, /* Command to reset the board from test status | */
   MAV_CMD_ENTER_TEST_MODE=14, /* Command to enter the test mode | */
   MAV_CMD_EXIT_TEST_MODE=15, /* Command to exit the test mode | */
   MAV_CMD_START_RECORDING=16, /* Command to start the internal cameras recordings | */
   MAV_CMD_STOP_RECORDING=17, /* Command to stop the internal cameras recordings | */
   MAV_CMD_OPEN_NITROGEN=18, /* Command to open the nitrogen valve | */
   MAV_CMD_REGISTRY_LOAD=19, /* Command to reload the registry from memory | */
   MAV_CMD_REGISTRY_SAVE=20, /* Command to commit the registry to memory | */
   MAV_CMD_REGISTRY_CLEAR=21, /* Command to clear the registry | */
   MAV_CMD_ENTER_HIL=22, /* Command to enter HIL mode at next reboot | */
   MAV_CMD_EXIT_HIL=23, /* Command to exit HIL mode at next reboot | */
   MAV_CMD_RESET_ALGORITHM=24, /* Command to reset the set algorithm. Used for now in ARP to reset the follow algorithm and return to armed state. | */
   MAV_CMD_ARP_FORCE_NO_FEEDBACK=25, /* Command to force ARP system to entern the no feedback from VN300 state | */
   MAV_CMD_ARP_FOLLOW=26, /* Command to enter in the ARP's follow state and procedure to follow the rocket | */
   MavCommandList_ENUM_END=27, /*  | */
} MavCommandList;
#endif

/** @brief Enum of all the servos */
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

/** @brief Enum of all the steppers */
#ifndef HAVE_ENUM_StepperList
#define HAVE_ENUM_StepperList
typedef enum StepperList
{
   STEPPER_X=1, /*  | */
   STEPPER_Y=2, /*  | */
   StepperList_ENUM_END=3, /*  | */
} StepperList;
#endif

/** @brief Enum of all the pins */
#ifndef HAVE_ENUM_PinsList
#define HAVE_ENUM_PinsList
typedef enum PinsList
{
   RAMP_PIN=1, /*  | */
   NOSECONE_PIN=2, /*  | */
   PinsList_ENUM_END=3, /*  | */
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
#include "./mavlink_msg_reset_servo_tc.h"
#include "./mavlink_msg_wiggle_servo_tc.h"
#include "./mavlink_msg_set_reference_altitude_tc.h"
#include "./mavlink_msg_set_reference_temperature_tc.h"
#include "./mavlink_msg_set_orientation_tc.h"
#include "./mavlink_msg_set_orientation_quat_tc.h"
#include "./mavlink_msg_set_coordinates_tc.h"
#include "./mavlink_msg_raw_event_tc.h"
#include "./mavlink_msg_set_deployment_altitude_tc.h"
#include "./mavlink_msg_set_target_coordinates_tc.h"
#include "./mavlink_msg_set_algorithm_tc.h"
#include "./mavlink_msg_set_calibration_pressure_tc.h"
#include "./mavlink_msg_set_initial_mea_mass_tc.h"
#include "./mavlink_msg_set_atomic_valve_timing_tc.h"
#include "./mavlink_msg_set_valve_maximum_aperture_tc.h"
#include "./mavlink_msg_conrig_state_tc.h"
#include "./mavlink_msg_set_ignition_time_tc.h"
#include "./mavlink_msg_set_nitrogen_time_tc.h"
#include "./mavlink_msg_set_cooling_time_tc.h"
#include "./mavlink_msg_set_stepper_angle_tc.h"
#include "./mavlink_msg_set_stepper_steps_tc.h"
#include "./mavlink_msg_set_stepper_multiplier_tc.h"
#include "./mavlink_msg_set_antenna_coordinates_arp_tc.h"
#include "./mavlink_msg_set_rocket_coordinates_arp_tc.h"
#include "./mavlink_msg_arp_command_tc.h"
#include "./mavlink_msg_ack_tm.h"
#include "./mavlink_msg_nack_tm.h"
#include "./mavlink_msg_wack_tm.h"
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
#include "./mavlink_msg_reference_tm.h"
#include "./mavlink_msg_registry_float_tm.h"
#include "./mavlink_msg_registry_int_tm.h"
#include "./mavlink_msg_registry_coord_tm.h"
#include "./mavlink_msg_arp_tm.h"
#include "./mavlink_msg_sys_tm.h"
#include "./mavlink_msg_logger_tm.h"
#include "./mavlink_msg_mavlink_stats_tm.h"
#include "./mavlink_msg_can_stats_tm.h"
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
#include "./mavlink_msg_calibration_tm.h"

// base include


#undef MAVLINK_THIS_XML_HASH
#define MAVLINK_THIS_XML_HASH -5660799532835485979

#if MAVLINK_THIS_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {{"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_PING_TC, MAVLINK_MESSAGE_INFO_COMMAND_TC, MAVLINK_MESSAGE_INFO_SYSTEM_TM_REQUEST_TC, MAVLINK_MESSAGE_INFO_SENSOR_TM_REQUEST_TC, MAVLINK_MESSAGE_INFO_SERVO_TM_REQUEST_TC, MAVLINK_MESSAGE_INFO_SET_SERVO_ANGLE_TC, MAVLINK_MESSAGE_INFO_RESET_SERVO_TC, MAVLINK_MESSAGE_INFO_WIGGLE_SERVO_TC, MAVLINK_MESSAGE_INFO_SET_REFERENCE_ALTITUDE_TC, MAVLINK_MESSAGE_INFO_SET_REFERENCE_TEMPERATURE_TC, MAVLINK_MESSAGE_INFO_SET_ORIENTATION_TC, MAVLINK_MESSAGE_INFO_SET_ORIENTATION_QUAT_TC, MAVLINK_MESSAGE_INFO_SET_COORDINATES_TC, MAVLINK_MESSAGE_INFO_RAW_EVENT_TC, MAVLINK_MESSAGE_INFO_SET_DEPLOYMENT_ALTITUDE_TC, MAVLINK_MESSAGE_INFO_SET_TARGET_COORDINATES_TC, MAVLINK_MESSAGE_INFO_SET_ALGORITHM_TC, MAVLINK_MESSAGE_INFO_SET_CALIBRATION_PRESSURE_TC, MAVLINK_MESSAGE_INFO_SET_INITIAL_MEA_MASS_TC, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_SET_ATOMIC_VALVE_TIMING_TC, MAVLINK_MESSAGE_INFO_SET_VALVE_MAXIMUM_APERTURE_TC, MAVLINK_MESSAGE_INFO_CONRIG_STATE_TC, MAVLINK_MESSAGE_INFO_SET_IGNITION_TIME_TC, MAVLINK_MESSAGE_INFO_SET_NITROGEN_TIME_TC, MAVLINK_MESSAGE_INFO_SET_COOLING_TIME_TC, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_SET_STEPPER_ANGLE_TC, MAVLINK_MESSAGE_INFO_SET_STEPPER_STEPS_TC, MAVLINK_MESSAGE_INFO_SET_STEPPER_MULTIPLIER_TC, MAVLINK_MESSAGE_INFO_SET_ANTENNA_COORDINATES_ARP_TC, MAVLINK_MESSAGE_INFO_SET_ROCKET_COORDINATES_ARP_TC, MAVLINK_MESSAGE_INFO_ARP_COMMAND_TC, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_ACK_TM, MAVLINK_MESSAGE_INFO_NACK_TM, MAVLINK_MESSAGE_INFO_WACK_TM, MAVLINK_MESSAGE_INFO_GPS_TM, MAVLINK_MESSAGE_INFO_IMU_TM, MAVLINK_MESSAGE_INFO_PRESSURE_TM, MAVLINK_MESSAGE_INFO_ADC_TM, MAVLINK_MESSAGE_INFO_VOLTAGE_TM, MAVLINK_MESSAGE_INFO_CURRENT_TM, MAVLINK_MESSAGE_INFO_TEMP_TM, MAVLINK_MESSAGE_INFO_LOAD_TM, MAVLINK_MESSAGE_INFO_ATTITUDE_TM, MAVLINK_MESSAGE_INFO_SENSOR_STATE_TM, MAVLINK_MESSAGE_INFO_SERVO_TM, MAVLINK_MESSAGE_INFO_PIN_TM, MAVLINK_MESSAGE_INFO_REFERENCE_TM, MAVLINK_MESSAGE_INFO_REGISTRY_FLOAT_TM, MAVLINK_MESSAGE_INFO_REGISTRY_INT_TM, MAVLINK_MESSAGE_INFO_REGISTRY_COORD_TM, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_ARP_TM, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_SYS_TM, MAVLINK_MESSAGE_INFO_LOGGER_TM, MAVLINK_MESSAGE_INFO_MAVLINK_STATS_TM, MAVLINK_MESSAGE_INFO_CAN_STATS_TM, MAVLINK_MESSAGE_INFO_TASK_STATS_TM, MAVLINK_MESSAGE_INFO_ADA_TM, MAVLINK_MESSAGE_INFO_NAS_TM, MAVLINK_MESSAGE_INFO_MEA_TM, MAVLINK_MESSAGE_INFO_ROCKET_FLIGHT_TM, MAVLINK_MESSAGE_INFO_PAYLOAD_FLIGHT_TM, MAVLINK_MESSAGE_INFO_ROCKET_STATS_TM, MAVLINK_MESSAGE_INFO_PAYLOAD_STATS_TM, MAVLINK_MESSAGE_INFO_GSE_TM, MAVLINK_MESSAGE_INFO_MOTOR_TM, MAVLINK_MESSAGE_INFO_CALIBRATION_TM, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}}
# define MAVLINK_MESSAGE_NAMES {{ "ACK_TM", 100 }, { "ADA_TM", 205 }, { "ADC_TM", 106 }, { "ARP_COMMAND_TC", 65 }, { "ARP_TM", 150 }, { "ATTITUDE_TM", 111 }, { "CALIBRATION_TM", 214 }, { "CAN_STATS_TM", 203 }, { "COMMAND_TC", 2 }, { "CONRIG_STATE_TC", 32 }, { "CURRENT_TM", 108 }, { "GPS_TM", 103 }, { "GSE_TM", 212 }, { "IMU_TM", 104 }, { "LOAD_TM", 110 }, { "LOGGER_TM", 201 }, { "MAVLINK_STATS_TM", 202 }, { "MEA_TM", 207 }, { "MOTOR_TM", 213 }, { "NACK_TM", 101 }, { "NAS_TM", 206 }, { "PAYLOAD_FLIGHT_TM", 209 }, { "PAYLOAD_STATS_TM", 211 }, { "PING_TC", 1 }, { "PIN_TM", 114 }, { "PRESSURE_TM", 105 }, { "RAW_EVENT_TC", 14 }, { "REFERENCE_TM", 115 }, { "REGISTRY_COORD_TM", 118 }, { "REGISTRY_FLOAT_TM", 116 }, { "REGISTRY_INT_TM", 117 }, { "RESET_SERVO_TC", 7 }, { "ROCKET_FLIGHT_TM", 208 }, { "ROCKET_STATS_TM", 210 }, { "SENSOR_STATE_TM", 112 }, { "SENSOR_TM_REQUEST_TC", 4 }, { "SERVO_TM", 113 }, { "SERVO_TM_REQUEST_TC", 5 }, { "SET_ALGORITHM_TC", 17 }, { "SET_ANTENNA_COORDINATES_ARP_TC", 63 }, { "SET_ATOMIC_VALVE_TIMING_TC", 30 }, { "SET_CALIBRATION_PRESSURE_TC", 18 }, { "SET_COOLING_TIME_TC", 35 }, { "SET_COORDINATES_TC", 13 }, { "SET_DEPLOYMENT_ALTITUDE_TC", 15 }, { "SET_IGNITION_TIME_TC", 33 }, { "SET_INITIAL_MEA_MASS_TC", 19 }, { "SET_NITROGEN_TIME_TC", 34 }, { "SET_ORIENTATION_QUAT_TC", 12 }, { "SET_ORIENTATION_TC", 11 }, { "SET_REFERENCE_ALTITUDE_TC", 9 }, { "SET_REFERENCE_TEMPERATURE_TC", 10 }, { "SET_ROCKET_COORDINATES_ARP_TC", 64 }, { "SET_SERVO_ANGLE_TC", 6 }, { "SET_STEPPER_ANGLE_TC", 60 }, { "SET_STEPPER_MULTIPLIER_TC", 62 }, { "SET_STEPPER_STEPS_TC", 61 }, { "SET_TARGET_COORDINATES_TC", 16 }, { "SET_VALVE_MAXIMUM_APERTURE_TC", 31 }, { "SYSTEM_TM_REQUEST_TC", 3 }, { "SYS_TM", 200 }, { "TASK_STATS_TM", 204 }, { "TEMP_TM", 109 }, { "VOLTAGE_TM", 107 }, { "WACK_TM", 102 }, { "WIGGLE_SERVO_TC", 8 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_LYRA_H
