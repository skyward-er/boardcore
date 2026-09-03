#pragma once
// MESSAGE ROCKET_STATS_TM PACKING

#define MAVLINK_MSG_ID_ROCKET_STATS_TM 210


typedef struct __mavlink_rocket_stats_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 uint64_t liftoff_ts; /*< [us] System clock at liftoff*/
 uint64_t liftoff_max_acc_ts; /*< [us] System clock at the maximum liftoff acceleration*/
 uint64_t max_speed_ts; /*< [us] System clock at max speed*/
 uint64_t max_mach_ts; /*<  System clock at maximum measured mach*/
 uint64_t shutdown_ts; /*<  System clock at shutdown*/
 uint64_t apogee_ts; /*< [us] System clock at apogee*/
 uint64_t apogee_max_acc_ts; /*< [us] System clock at max acceleration after apogee*/
 uint64_t dpl_ts; /*< [us] System clock at main deployment*/
 uint64_t dpl_max_acc_ts; /*< [us] System clock at max acceleration during main deployment*/
 uint64_t dpl_bay_max_pressure_ts; /*< [us] System clock at max pressure in the deployment bay during drogue deployment*/
 float liftoff_max_acc; /*< [m/s2] Maximum liftoff acceleration*/
 float max_speed; /*< [m/s] Max measured speed*/
 float max_speed_altitude; /*< [m] Altitude at max speed*/
 float max_mach; /*<  Maximum measured mach*/
 float shutdown_alt; /*<  Altitude at shutdown*/
 float apogee_lat; /*< [deg] Apogee latitude*/
 float apogee_lon; /*< [deg] Apogee longitude*/
 float apogee_alt; /*< [m] Apogee altitude*/
 float apogee_max_acc; /*< [m/s2] Max acceleration after apogee*/
 float dpl_alt; /*< [m] Deployment altitude*/
 float dpl_max_acc; /*< [m/s2] Max acceleration during main deployment*/
 float dpl_bay_max_pressure; /*< [Pa] Max pressure in the deployment bay during drogue deployment*/
 float ref_lat; /*< [deg] Reference altitude*/
 float ref_lon; /*< [deg] Reference longitude*/
 float ref_alt; /*< [m] Reference altitude*/
 float cpu_load; /*<  CPU load in percentage*/
 uint32_t free_heap; /*<  Amount of available heap in memory*/
 float exp_angle; /*<  Expulsion servo angle*/
 int32_t log_good; /*<  0 if log failed, 1 otherwise*/
 int16_t log_number; /*<  Currently active log file, -1 if the logger is inactive*/
 uint8_t ada_state; /*<  ADA Controller State*/
 uint8_t abk_state; /*<  Airbrake FSM state*/
 uint8_t nas_state; /*<  NAS FSM State*/
 uint8_t mea_state; /*<  MEA Controller State*/
 uint8_t pin_launch; /*<  Launch pin status (1 = connected, 0 = disconnected)*/
 uint8_t pin_nosecone; /*<  Nosecone pin status (1 = connected, 0 = disconnected)*/
 uint8_t pin_expulsion; /*<  Servo sensor status (1 = actuated, 0 = idle)*/
 uint8_t cutter_presence; /*<  Cutter presence status (1 = present, 0 = missing)*/
 uint8_t payload_board_state; /*<  Main board fmm state*/
 uint8_t motor_board_state; /*<  Motor board fmm state*/
 uint8_t payload_can_status; /*<  Main CAN status*/
 uint8_t motor_can_status; /*<  Motor CAN status*/
 uint8_t rig_can_status; /*<  RIG CAN status*/
 uint8_t hil_state; /*<  1 if the board is currently in HIL mode*/
} mavlink_rocket_stats_tm_t;

#define MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN 180
#define MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN 180
#define MAVLINK_MSG_ID_210_LEN 180
#define MAVLINK_MSG_ID_210_MIN_LEN 180

#define MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC 195
#define MAVLINK_MSG_ID_210_CRC 195



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROCKET_STATS_TM { \
    210, \
    "ROCKET_STATS_TM", \
    45, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rocket_stats_tm_t, timestamp) }, \
         { "liftoff_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_rocket_stats_tm_t, liftoff_ts) }, \
         { "liftoff_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_rocket_stats_tm_t, liftoff_max_acc_ts) }, \
         { "liftoff_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_rocket_stats_tm_t, liftoff_max_acc) }, \
         { "max_speed_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 24, offsetof(mavlink_rocket_stats_tm_t, max_speed_ts) }, \
         { "max_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_rocket_stats_tm_t, max_speed) }, \
         { "max_speed_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_rocket_stats_tm_t, max_speed_altitude) }, \
         { "max_mach_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 32, offsetof(mavlink_rocket_stats_tm_t, max_mach_ts) }, \
         { "max_mach", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_rocket_stats_tm_t, max_mach) }, \
         { "shutdown_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 40, offsetof(mavlink_rocket_stats_tm_t, shutdown_ts) }, \
         { "shutdown_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_rocket_stats_tm_t, shutdown_alt) }, \
         { "apogee_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 48, offsetof(mavlink_rocket_stats_tm_t, apogee_ts) }, \
         { "apogee_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_rocket_stats_tm_t, apogee_lat) }, \
         { "apogee_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_rocket_stats_tm_t, apogee_lon) }, \
         { "apogee_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_rocket_stats_tm_t, apogee_alt) }, \
         { "apogee_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 56, offsetof(mavlink_rocket_stats_tm_t, apogee_max_acc_ts) }, \
         { "apogee_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_rocket_stats_tm_t, apogee_max_acc) }, \
         { "dpl_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 64, offsetof(mavlink_rocket_stats_tm_t, dpl_ts) }, \
         { "dpl_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_rocket_stats_tm_t, dpl_alt) }, \
         { "dpl_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 72, offsetof(mavlink_rocket_stats_tm_t, dpl_max_acc_ts) }, \
         { "dpl_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_rocket_stats_tm_t, dpl_max_acc) }, \
         { "dpl_bay_max_pressure_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 80, offsetof(mavlink_rocket_stats_tm_t, dpl_bay_max_pressure_ts) }, \
         { "dpl_bay_max_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_rocket_stats_tm_t, dpl_bay_max_pressure) }, \
         { "ref_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_rocket_stats_tm_t, ref_lat) }, \
         { "ref_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_rocket_stats_tm_t, ref_lon) }, \
         { "ref_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_rocket_stats_tm_t, ref_alt) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_rocket_stats_tm_t, cpu_load) }, \
         { "free_heap", NULL, MAVLINK_TYPE_UINT32_T, 0, 152, offsetof(mavlink_rocket_stats_tm_t, free_heap) }, \
         { "ada_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 166, offsetof(mavlink_rocket_stats_tm_t, ada_state) }, \
         { "abk_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 167, offsetof(mavlink_rocket_stats_tm_t, abk_state) }, \
         { "nas_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 168, offsetof(mavlink_rocket_stats_tm_t, nas_state) }, \
         { "mea_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 169, offsetof(mavlink_rocket_stats_tm_t, mea_state) }, \
         { "exp_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 156, offsetof(mavlink_rocket_stats_tm_t, exp_angle) }, \
         { "pin_launch", NULL, MAVLINK_TYPE_UINT8_T, 0, 170, offsetof(mavlink_rocket_stats_tm_t, pin_launch) }, \
         { "pin_nosecone", NULL, MAVLINK_TYPE_UINT8_T, 0, 171, offsetof(mavlink_rocket_stats_tm_t, pin_nosecone) }, \
         { "pin_expulsion", NULL, MAVLINK_TYPE_UINT8_T, 0, 172, offsetof(mavlink_rocket_stats_tm_t, pin_expulsion) }, \
         { "cutter_presence", NULL, MAVLINK_TYPE_UINT8_T, 0, 173, offsetof(mavlink_rocket_stats_tm_t, cutter_presence) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 164, offsetof(mavlink_rocket_stats_tm_t, log_number) }, \
         { "log_good", NULL, MAVLINK_TYPE_INT32_T, 0, 160, offsetof(mavlink_rocket_stats_tm_t, log_good) }, \
         { "payload_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 174, offsetof(mavlink_rocket_stats_tm_t, payload_board_state) }, \
         { "motor_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 175, offsetof(mavlink_rocket_stats_tm_t, motor_board_state) }, \
         { "payload_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 176, offsetof(mavlink_rocket_stats_tm_t, payload_can_status) }, \
         { "motor_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 177, offsetof(mavlink_rocket_stats_tm_t, motor_can_status) }, \
         { "rig_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 178, offsetof(mavlink_rocket_stats_tm_t, rig_can_status) }, \
         { "hil_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 179, offsetof(mavlink_rocket_stats_tm_t, hil_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROCKET_STATS_TM { \
    "ROCKET_STATS_TM", \
    45, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rocket_stats_tm_t, timestamp) }, \
         { "liftoff_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_rocket_stats_tm_t, liftoff_ts) }, \
         { "liftoff_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_rocket_stats_tm_t, liftoff_max_acc_ts) }, \
         { "liftoff_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_rocket_stats_tm_t, liftoff_max_acc) }, \
         { "max_speed_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 24, offsetof(mavlink_rocket_stats_tm_t, max_speed_ts) }, \
         { "max_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_rocket_stats_tm_t, max_speed) }, \
         { "max_speed_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_rocket_stats_tm_t, max_speed_altitude) }, \
         { "max_mach_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 32, offsetof(mavlink_rocket_stats_tm_t, max_mach_ts) }, \
         { "max_mach", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_rocket_stats_tm_t, max_mach) }, \
         { "shutdown_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 40, offsetof(mavlink_rocket_stats_tm_t, shutdown_ts) }, \
         { "shutdown_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_rocket_stats_tm_t, shutdown_alt) }, \
         { "apogee_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 48, offsetof(mavlink_rocket_stats_tm_t, apogee_ts) }, \
         { "apogee_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_rocket_stats_tm_t, apogee_lat) }, \
         { "apogee_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_rocket_stats_tm_t, apogee_lon) }, \
         { "apogee_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_rocket_stats_tm_t, apogee_alt) }, \
         { "apogee_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 56, offsetof(mavlink_rocket_stats_tm_t, apogee_max_acc_ts) }, \
         { "apogee_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_rocket_stats_tm_t, apogee_max_acc) }, \
         { "dpl_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 64, offsetof(mavlink_rocket_stats_tm_t, dpl_ts) }, \
         { "dpl_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_rocket_stats_tm_t, dpl_alt) }, \
         { "dpl_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 72, offsetof(mavlink_rocket_stats_tm_t, dpl_max_acc_ts) }, \
         { "dpl_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_rocket_stats_tm_t, dpl_max_acc) }, \
         { "dpl_bay_max_pressure_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 80, offsetof(mavlink_rocket_stats_tm_t, dpl_bay_max_pressure_ts) }, \
         { "dpl_bay_max_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_rocket_stats_tm_t, dpl_bay_max_pressure) }, \
         { "ref_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_rocket_stats_tm_t, ref_lat) }, \
         { "ref_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_rocket_stats_tm_t, ref_lon) }, \
         { "ref_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_rocket_stats_tm_t, ref_alt) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_rocket_stats_tm_t, cpu_load) }, \
         { "free_heap", NULL, MAVLINK_TYPE_UINT32_T, 0, 152, offsetof(mavlink_rocket_stats_tm_t, free_heap) }, \
         { "ada_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 166, offsetof(mavlink_rocket_stats_tm_t, ada_state) }, \
         { "abk_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 167, offsetof(mavlink_rocket_stats_tm_t, abk_state) }, \
         { "nas_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 168, offsetof(mavlink_rocket_stats_tm_t, nas_state) }, \
         { "mea_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 169, offsetof(mavlink_rocket_stats_tm_t, mea_state) }, \
         { "exp_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 156, offsetof(mavlink_rocket_stats_tm_t, exp_angle) }, \
         { "pin_launch", NULL, MAVLINK_TYPE_UINT8_T, 0, 170, offsetof(mavlink_rocket_stats_tm_t, pin_launch) }, \
         { "pin_nosecone", NULL, MAVLINK_TYPE_UINT8_T, 0, 171, offsetof(mavlink_rocket_stats_tm_t, pin_nosecone) }, \
         { "pin_expulsion", NULL, MAVLINK_TYPE_UINT8_T, 0, 172, offsetof(mavlink_rocket_stats_tm_t, pin_expulsion) }, \
         { "cutter_presence", NULL, MAVLINK_TYPE_UINT8_T, 0, 173, offsetof(mavlink_rocket_stats_tm_t, cutter_presence) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 164, offsetof(mavlink_rocket_stats_tm_t, log_number) }, \
         { "log_good", NULL, MAVLINK_TYPE_INT32_T, 0, 160, offsetof(mavlink_rocket_stats_tm_t, log_good) }, \
         { "payload_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 174, offsetof(mavlink_rocket_stats_tm_t, payload_board_state) }, \
         { "motor_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 175, offsetof(mavlink_rocket_stats_tm_t, motor_board_state) }, \
         { "payload_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 176, offsetof(mavlink_rocket_stats_tm_t, payload_can_status) }, \
         { "motor_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 177, offsetof(mavlink_rocket_stats_tm_t, motor_can_status) }, \
         { "rig_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 178, offsetof(mavlink_rocket_stats_tm_t, rig_can_status) }, \
         { "hil_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 179, offsetof(mavlink_rocket_stats_tm_t, hil_state) }, \
         } \
}
#endif

/**
 * @brief Pack a rocket_stats_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param liftoff_ts [us] System clock at liftoff
 * @param liftoff_max_acc_ts [us] System clock at the maximum liftoff acceleration
 * @param liftoff_max_acc [m/s2] Maximum liftoff acceleration
 * @param max_speed_ts [us] System clock at max speed
 * @param max_speed [m/s] Max measured speed
 * @param max_speed_altitude [m] Altitude at max speed
 * @param max_mach_ts  System clock at maximum measured mach
 * @param max_mach  Maximum measured mach
 * @param shutdown_ts  System clock at shutdown
 * @param shutdown_alt  Altitude at shutdown
 * @param apogee_ts [us] System clock at apogee
 * @param apogee_lat [deg] Apogee latitude
 * @param apogee_lon [deg] Apogee longitude
 * @param apogee_alt [m] Apogee altitude
 * @param apogee_max_acc_ts [us] System clock at max acceleration after apogee
 * @param apogee_max_acc [m/s2] Max acceleration after apogee
 * @param dpl_ts [us] System clock at main deployment
 * @param dpl_alt [m] Deployment altitude
 * @param dpl_max_acc_ts [us] System clock at max acceleration during main deployment
 * @param dpl_max_acc [m/s2] Max acceleration during main deployment
 * @param dpl_bay_max_pressure_ts [us] System clock at max pressure in the deployment bay during drogue deployment
 * @param dpl_bay_max_pressure [Pa] Max pressure in the deployment bay during drogue deployment
 * @param ref_lat [deg] Reference altitude
 * @param ref_lon [deg] Reference longitude
 * @param ref_alt [m] Reference altitude
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @param ada_state  ADA Controller State
 * @param abk_state  Airbrake FSM state
 * @param nas_state  NAS FSM State
 * @param mea_state  MEA Controller State
 * @param exp_angle  Expulsion servo angle
 * @param pin_launch  Launch pin status (1 = connected, 0 = disconnected)
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param pin_expulsion  Servo sensor status (1 = actuated, 0 = idle)
 * @param cutter_presence  Cutter presence status (1 = present, 0 = missing)
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @param payload_board_state  Main board fmm state
 * @param motor_board_state  Motor board fmm state
 * @param payload_can_status  Main CAN status
 * @param motor_can_status  Motor CAN status
 * @param rig_can_status  RIG CAN status
 * @param hil_state  1 if the board is currently in HIL mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rocket_stats_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint64_t liftoff_ts, uint64_t liftoff_max_acc_ts, float liftoff_max_acc, uint64_t max_speed_ts, float max_speed, float max_speed_altitude, uint64_t max_mach_ts, float max_mach, uint64_t shutdown_ts, float shutdown_alt, uint64_t apogee_ts, float apogee_lat, float apogee_lon, float apogee_alt, uint64_t apogee_max_acc_ts, float apogee_max_acc, uint64_t dpl_ts, float dpl_alt, uint64_t dpl_max_acc_ts, float dpl_max_acc, uint64_t dpl_bay_max_pressure_ts, float dpl_bay_max_pressure, float ref_lat, float ref_lon, float ref_alt, float cpu_load, uint32_t free_heap, uint8_t ada_state, uint8_t abk_state, uint8_t nas_state, uint8_t mea_state, float exp_angle, uint8_t pin_launch, uint8_t pin_nosecone, uint8_t pin_expulsion, uint8_t cutter_presence, int16_t log_number, int32_t log_good, uint8_t payload_board_state, uint8_t motor_board_state, uint8_t payload_can_status, uint8_t motor_can_status, uint8_t rig_can_status, uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, liftoff_ts);
    _mav_put_uint64_t(buf, 16, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 24, max_speed_ts);
    _mav_put_uint64_t(buf, 32, max_mach_ts);
    _mav_put_uint64_t(buf, 40, shutdown_ts);
    _mav_put_uint64_t(buf, 48, apogee_ts);
    _mav_put_uint64_t(buf, 56, apogee_max_acc_ts);
    _mav_put_uint64_t(buf, 64, dpl_ts);
    _mav_put_uint64_t(buf, 72, dpl_max_acc_ts);
    _mav_put_uint64_t(buf, 80, dpl_bay_max_pressure_ts);
    _mav_put_float(buf, 88, liftoff_max_acc);
    _mav_put_float(buf, 92, max_speed);
    _mav_put_float(buf, 96, max_speed_altitude);
    _mav_put_float(buf, 100, max_mach);
    _mav_put_float(buf, 104, shutdown_alt);
    _mav_put_float(buf, 108, apogee_lat);
    _mav_put_float(buf, 112, apogee_lon);
    _mav_put_float(buf, 116, apogee_alt);
    _mav_put_float(buf, 120, apogee_max_acc);
    _mav_put_float(buf, 124, dpl_alt);
    _mav_put_float(buf, 128, dpl_max_acc);
    _mav_put_float(buf, 132, dpl_bay_max_pressure);
    _mav_put_float(buf, 136, ref_lat);
    _mav_put_float(buf, 140, ref_lon);
    _mav_put_float(buf, 144, ref_alt);
    _mav_put_float(buf, 148, cpu_load);
    _mav_put_uint32_t(buf, 152, free_heap);
    _mav_put_float(buf, 156, exp_angle);
    _mav_put_int32_t(buf, 160, log_good);
    _mav_put_int16_t(buf, 164, log_number);
    _mav_put_uint8_t(buf, 166, ada_state);
    _mav_put_uint8_t(buf, 167, abk_state);
    _mav_put_uint8_t(buf, 168, nas_state);
    _mav_put_uint8_t(buf, 169, mea_state);
    _mav_put_uint8_t(buf, 170, pin_launch);
    _mav_put_uint8_t(buf, 171, pin_nosecone);
    _mav_put_uint8_t(buf, 172, pin_expulsion);
    _mav_put_uint8_t(buf, 173, cutter_presence);
    _mav_put_uint8_t(buf, 174, payload_board_state);
    _mav_put_uint8_t(buf, 175, motor_board_state);
    _mav_put_uint8_t(buf, 176, payload_can_status);
    _mav_put_uint8_t(buf, 177, motor_can_status);
    _mav_put_uint8_t(buf, 178, rig_can_status);
    _mav_put_uint8_t(buf, 179, hil_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN);
#else
    mavlink_rocket_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.liftoff_ts = liftoff_ts;
    packet.liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet.max_speed_ts = max_speed_ts;
    packet.max_mach_ts = max_mach_ts;
    packet.shutdown_ts = shutdown_ts;
    packet.apogee_ts = apogee_ts;
    packet.apogee_max_acc_ts = apogee_max_acc_ts;
    packet.dpl_ts = dpl_ts;
    packet.dpl_max_acc_ts = dpl_max_acc_ts;
    packet.dpl_bay_max_pressure_ts = dpl_bay_max_pressure_ts;
    packet.liftoff_max_acc = liftoff_max_acc;
    packet.max_speed = max_speed;
    packet.max_speed_altitude = max_speed_altitude;
    packet.max_mach = max_mach;
    packet.shutdown_alt = shutdown_alt;
    packet.apogee_lat = apogee_lat;
    packet.apogee_lon = apogee_lon;
    packet.apogee_alt = apogee_alt;
    packet.apogee_max_acc = apogee_max_acc;
    packet.dpl_alt = dpl_alt;
    packet.dpl_max_acc = dpl_max_acc;
    packet.dpl_bay_max_pressure = dpl_bay_max_pressure;
    packet.ref_lat = ref_lat;
    packet.ref_lon = ref_lon;
    packet.ref_alt = ref_alt;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;
    packet.exp_angle = exp_angle;
    packet.log_good = log_good;
    packet.log_number = log_number;
    packet.ada_state = ada_state;
    packet.abk_state = abk_state;
    packet.nas_state = nas_state;
    packet.mea_state = mea_state;
    packet.pin_launch = pin_launch;
    packet.pin_nosecone = pin_nosecone;
    packet.pin_expulsion = pin_expulsion;
    packet.cutter_presence = cutter_presence;
    packet.payload_board_state = payload_board_state;
    packet.motor_board_state = motor_board_state;
    packet.payload_can_status = payload_can_status;
    packet.motor_can_status = motor_can_status;
    packet.rig_can_status = rig_can_status;
    packet.hil_state = hil_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROCKET_STATS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
}

/**
 * @brief Pack a rocket_stats_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp in microseconds
 * @param liftoff_ts [us] System clock at liftoff
 * @param liftoff_max_acc_ts [us] System clock at the maximum liftoff acceleration
 * @param liftoff_max_acc [m/s2] Maximum liftoff acceleration
 * @param max_speed_ts [us] System clock at max speed
 * @param max_speed [m/s] Max measured speed
 * @param max_speed_altitude [m] Altitude at max speed
 * @param max_mach_ts  System clock at maximum measured mach
 * @param max_mach  Maximum measured mach
 * @param shutdown_ts  System clock at shutdown
 * @param shutdown_alt  Altitude at shutdown
 * @param apogee_ts [us] System clock at apogee
 * @param apogee_lat [deg] Apogee latitude
 * @param apogee_lon [deg] Apogee longitude
 * @param apogee_alt [m] Apogee altitude
 * @param apogee_max_acc_ts [us] System clock at max acceleration after apogee
 * @param apogee_max_acc [m/s2] Max acceleration after apogee
 * @param dpl_ts [us] System clock at main deployment
 * @param dpl_alt [m] Deployment altitude
 * @param dpl_max_acc_ts [us] System clock at max acceleration during main deployment
 * @param dpl_max_acc [m/s2] Max acceleration during main deployment
 * @param dpl_bay_max_pressure_ts [us] System clock at max pressure in the deployment bay during drogue deployment
 * @param dpl_bay_max_pressure [Pa] Max pressure in the deployment bay during drogue deployment
 * @param ref_lat [deg] Reference altitude
 * @param ref_lon [deg] Reference longitude
 * @param ref_alt [m] Reference altitude
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @param ada_state  ADA Controller State
 * @param abk_state  Airbrake FSM state
 * @param nas_state  NAS FSM State
 * @param mea_state  MEA Controller State
 * @param exp_angle  Expulsion servo angle
 * @param pin_launch  Launch pin status (1 = connected, 0 = disconnected)
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param pin_expulsion  Servo sensor status (1 = actuated, 0 = idle)
 * @param cutter_presence  Cutter presence status (1 = present, 0 = missing)
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @param payload_board_state  Main board fmm state
 * @param motor_board_state  Motor board fmm state
 * @param payload_can_status  Main CAN status
 * @param motor_can_status  Motor CAN status
 * @param rig_can_status  RIG CAN status
 * @param hil_state  1 if the board is currently in HIL mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rocket_stats_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint64_t liftoff_ts,uint64_t liftoff_max_acc_ts,float liftoff_max_acc,uint64_t max_speed_ts,float max_speed,float max_speed_altitude,uint64_t max_mach_ts,float max_mach,uint64_t shutdown_ts,float shutdown_alt,uint64_t apogee_ts,float apogee_lat,float apogee_lon,float apogee_alt,uint64_t apogee_max_acc_ts,float apogee_max_acc,uint64_t dpl_ts,float dpl_alt,uint64_t dpl_max_acc_ts,float dpl_max_acc,uint64_t dpl_bay_max_pressure_ts,float dpl_bay_max_pressure,float ref_lat,float ref_lon,float ref_alt,float cpu_load,uint32_t free_heap,uint8_t ada_state,uint8_t abk_state,uint8_t nas_state,uint8_t mea_state,float exp_angle,uint8_t pin_launch,uint8_t pin_nosecone,uint8_t pin_expulsion,uint8_t cutter_presence,int16_t log_number,int32_t log_good,uint8_t payload_board_state,uint8_t motor_board_state,uint8_t payload_can_status,uint8_t motor_can_status,uint8_t rig_can_status,uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, liftoff_ts);
    _mav_put_uint64_t(buf, 16, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 24, max_speed_ts);
    _mav_put_uint64_t(buf, 32, max_mach_ts);
    _mav_put_uint64_t(buf, 40, shutdown_ts);
    _mav_put_uint64_t(buf, 48, apogee_ts);
    _mav_put_uint64_t(buf, 56, apogee_max_acc_ts);
    _mav_put_uint64_t(buf, 64, dpl_ts);
    _mav_put_uint64_t(buf, 72, dpl_max_acc_ts);
    _mav_put_uint64_t(buf, 80, dpl_bay_max_pressure_ts);
    _mav_put_float(buf, 88, liftoff_max_acc);
    _mav_put_float(buf, 92, max_speed);
    _mav_put_float(buf, 96, max_speed_altitude);
    _mav_put_float(buf, 100, max_mach);
    _mav_put_float(buf, 104, shutdown_alt);
    _mav_put_float(buf, 108, apogee_lat);
    _mav_put_float(buf, 112, apogee_lon);
    _mav_put_float(buf, 116, apogee_alt);
    _mav_put_float(buf, 120, apogee_max_acc);
    _mav_put_float(buf, 124, dpl_alt);
    _mav_put_float(buf, 128, dpl_max_acc);
    _mav_put_float(buf, 132, dpl_bay_max_pressure);
    _mav_put_float(buf, 136, ref_lat);
    _mav_put_float(buf, 140, ref_lon);
    _mav_put_float(buf, 144, ref_alt);
    _mav_put_float(buf, 148, cpu_load);
    _mav_put_uint32_t(buf, 152, free_heap);
    _mav_put_float(buf, 156, exp_angle);
    _mav_put_int32_t(buf, 160, log_good);
    _mav_put_int16_t(buf, 164, log_number);
    _mav_put_uint8_t(buf, 166, ada_state);
    _mav_put_uint8_t(buf, 167, abk_state);
    _mav_put_uint8_t(buf, 168, nas_state);
    _mav_put_uint8_t(buf, 169, mea_state);
    _mav_put_uint8_t(buf, 170, pin_launch);
    _mav_put_uint8_t(buf, 171, pin_nosecone);
    _mav_put_uint8_t(buf, 172, pin_expulsion);
    _mav_put_uint8_t(buf, 173, cutter_presence);
    _mav_put_uint8_t(buf, 174, payload_board_state);
    _mav_put_uint8_t(buf, 175, motor_board_state);
    _mav_put_uint8_t(buf, 176, payload_can_status);
    _mav_put_uint8_t(buf, 177, motor_can_status);
    _mav_put_uint8_t(buf, 178, rig_can_status);
    _mav_put_uint8_t(buf, 179, hil_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN);
#else
    mavlink_rocket_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.liftoff_ts = liftoff_ts;
    packet.liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet.max_speed_ts = max_speed_ts;
    packet.max_mach_ts = max_mach_ts;
    packet.shutdown_ts = shutdown_ts;
    packet.apogee_ts = apogee_ts;
    packet.apogee_max_acc_ts = apogee_max_acc_ts;
    packet.dpl_ts = dpl_ts;
    packet.dpl_max_acc_ts = dpl_max_acc_ts;
    packet.dpl_bay_max_pressure_ts = dpl_bay_max_pressure_ts;
    packet.liftoff_max_acc = liftoff_max_acc;
    packet.max_speed = max_speed;
    packet.max_speed_altitude = max_speed_altitude;
    packet.max_mach = max_mach;
    packet.shutdown_alt = shutdown_alt;
    packet.apogee_lat = apogee_lat;
    packet.apogee_lon = apogee_lon;
    packet.apogee_alt = apogee_alt;
    packet.apogee_max_acc = apogee_max_acc;
    packet.dpl_alt = dpl_alt;
    packet.dpl_max_acc = dpl_max_acc;
    packet.dpl_bay_max_pressure = dpl_bay_max_pressure;
    packet.ref_lat = ref_lat;
    packet.ref_lon = ref_lon;
    packet.ref_alt = ref_alt;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;
    packet.exp_angle = exp_angle;
    packet.log_good = log_good;
    packet.log_number = log_number;
    packet.ada_state = ada_state;
    packet.abk_state = abk_state;
    packet.nas_state = nas_state;
    packet.mea_state = mea_state;
    packet.pin_launch = pin_launch;
    packet.pin_nosecone = pin_nosecone;
    packet.pin_expulsion = pin_expulsion;
    packet.cutter_presence = cutter_presence;
    packet.payload_board_state = payload_board_state;
    packet.motor_board_state = motor_board_state;
    packet.payload_can_status = payload_can_status;
    packet.motor_can_status = motor_can_status;
    packet.rig_can_status = rig_can_status;
    packet.hil_state = hil_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROCKET_STATS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
}

/**
 * @brief Encode a rocket_stats_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rocket_stats_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rocket_stats_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rocket_stats_tm_t* rocket_stats_tm)
{
    return mavlink_msg_rocket_stats_tm_pack(system_id, component_id, msg, rocket_stats_tm->timestamp, rocket_stats_tm->liftoff_ts, rocket_stats_tm->liftoff_max_acc_ts, rocket_stats_tm->liftoff_max_acc, rocket_stats_tm->max_speed_ts, rocket_stats_tm->max_speed, rocket_stats_tm->max_speed_altitude, rocket_stats_tm->max_mach_ts, rocket_stats_tm->max_mach, rocket_stats_tm->shutdown_ts, rocket_stats_tm->shutdown_alt, rocket_stats_tm->apogee_ts, rocket_stats_tm->apogee_lat, rocket_stats_tm->apogee_lon, rocket_stats_tm->apogee_alt, rocket_stats_tm->apogee_max_acc_ts, rocket_stats_tm->apogee_max_acc, rocket_stats_tm->dpl_ts, rocket_stats_tm->dpl_alt, rocket_stats_tm->dpl_max_acc_ts, rocket_stats_tm->dpl_max_acc, rocket_stats_tm->dpl_bay_max_pressure_ts, rocket_stats_tm->dpl_bay_max_pressure, rocket_stats_tm->ref_lat, rocket_stats_tm->ref_lon, rocket_stats_tm->ref_alt, rocket_stats_tm->cpu_load, rocket_stats_tm->free_heap, rocket_stats_tm->ada_state, rocket_stats_tm->abk_state, rocket_stats_tm->nas_state, rocket_stats_tm->mea_state, rocket_stats_tm->exp_angle, rocket_stats_tm->pin_launch, rocket_stats_tm->pin_nosecone, rocket_stats_tm->pin_expulsion, rocket_stats_tm->cutter_presence, rocket_stats_tm->log_number, rocket_stats_tm->log_good, rocket_stats_tm->payload_board_state, rocket_stats_tm->motor_board_state, rocket_stats_tm->payload_can_status, rocket_stats_tm->motor_can_status, rocket_stats_tm->rig_can_status, rocket_stats_tm->hil_state);
}

/**
 * @brief Encode a rocket_stats_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rocket_stats_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rocket_stats_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rocket_stats_tm_t* rocket_stats_tm)
{
    return mavlink_msg_rocket_stats_tm_pack_chan(system_id, component_id, chan, msg, rocket_stats_tm->timestamp, rocket_stats_tm->liftoff_ts, rocket_stats_tm->liftoff_max_acc_ts, rocket_stats_tm->liftoff_max_acc, rocket_stats_tm->max_speed_ts, rocket_stats_tm->max_speed, rocket_stats_tm->max_speed_altitude, rocket_stats_tm->max_mach_ts, rocket_stats_tm->max_mach, rocket_stats_tm->shutdown_ts, rocket_stats_tm->shutdown_alt, rocket_stats_tm->apogee_ts, rocket_stats_tm->apogee_lat, rocket_stats_tm->apogee_lon, rocket_stats_tm->apogee_alt, rocket_stats_tm->apogee_max_acc_ts, rocket_stats_tm->apogee_max_acc, rocket_stats_tm->dpl_ts, rocket_stats_tm->dpl_alt, rocket_stats_tm->dpl_max_acc_ts, rocket_stats_tm->dpl_max_acc, rocket_stats_tm->dpl_bay_max_pressure_ts, rocket_stats_tm->dpl_bay_max_pressure, rocket_stats_tm->ref_lat, rocket_stats_tm->ref_lon, rocket_stats_tm->ref_alt, rocket_stats_tm->cpu_load, rocket_stats_tm->free_heap, rocket_stats_tm->ada_state, rocket_stats_tm->abk_state, rocket_stats_tm->nas_state, rocket_stats_tm->mea_state, rocket_stats_tm->exp_angle, rocket_stats_tm->pin_launch, rocket_stats_tm->pin_nosecone, rocket_stats_tm->pin_expulsion, rocket_stats_tm->cutter_presence, rocket_stats_tm->log_number, rocket_stats_tm->log_good, rocket_stats_tm->payload_board_state, rocket_stats_tm->motor_board_state, rocket_stats_tm->payload_can_status, rocket_stats_tm->motor_can_status, rocket_stats_tm->rig_can_status, rocket_stats_tm->hil_state);
}

/**
 * @brief Send a rocket_stats_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param liftoff_ts [us] System clock at liftoff
 * @param liftoff_max_acc_ts [us] System clock at the maximum liftoff acceleration
 * @param liftoff_max_acc [m/s2] Maximum liftoff acceleration
 * @param max_speed_ts [us] System clock at max speed
 * @param max_speed [m/s] Max measured speed
 * @param max_speed_altitude [m] Altitude at max speed
 * @param max_mach_ts  System clock at maximum measured mach
 * @param max_mach  Maximum measured mach
 * @param shutdown_ts  System clock at shutdown
 * @param shutdown_alt  Altitude at shutdown
 * @param apogee_ts [us] System clock at apogee
 * @param apogee_lat [deg] Apogee latitude
 * @param apogee_lon [deg] Apogee longitude
 * @param apogee_alt [m] Apogee altitude
 * @param apogee_max_acc_ts [us] System clock at max acceleration after apogee
 * @param apogee_max_acc [m/s2] Max acceleration after apogee
 * @param dpl_ts [us] System clock at main deployment
 * @param dpl_alt [m] Deployment altitude
 * @param dpl_max_acc_ts [us] System clock at max acceleration during main deployment
 * @param dpl_max_acc [m/s2] Max acceleration during main deployment
 * @param dpl_bay_max_pressure_ts [us] System clock at max pressure in the deployment bay during drogue deployment
 * @param dpl_bay_max_pressure [Pa] Max pressure in the deployment bay during drogue deployment
 * @param ref_lat [deg] Reference altitude
 * @param ref_lon [deg] Reference longitude
 * @param ref_alt [m] Reference altitude
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @param ada_state  ADA Controller State
 * @param abk_state  Airbrake FSM state
 * @param nas_state  NAS FSM State
 * @param mea_state  MEA Controller State
 * @param exp_angle  Expulsion servo angle
 * @param pin_launch  Launch pin status (1 = connected, 0 = disconnected)
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param pin_expulsion  Servo sensor status (1 = actuated, 0 = idle)
 * @param cutter_presence  Cutter presence status (1 = present, 0 = missing)
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @param payload_board_state  Main board fmm state
 * @param motor_board_state  Motor board fmm state
 * @param payload_can_status  Main CAN status
 * @param motor_can_status  Motor CAN status
 * @param rig_can_status  RIG CAN status
 * @param hil_state  1 if the board is currently in HIL mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rocket_stats_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint64_t liftoff_ts, uint64_t liftoff_max_acc_ts, float liftoff_max_acc, uint64_t max_speed_ts, float max_speed, float max_speed_altitude, uint64_t max_mach_ts, float max_mach, uint64_t shutdown_ts, float shutdown_alt, uint64_t apogee_ts, float apogee_lat, float apogee_lon, float apogee_alt, uint64_t apogee_max_acc_ts, float apogee_max_acc, uint64_t dpl_ts, float dpl_alt, uint64_t dpl_max_acc_ts, float dpl_max_acc, uint64_t dpl_bay_max_pressure_ts, float dpl_bay_max_pressure, float ref_lat, float ref_lon, float ref_alt, float cpu_load, uint32_t free_heap, uint8_t ada_state, uint8_t abk_state, uint8_t nas_state, uint8_t mea_state, float exp_angle, uint8_t pin_launch, uint8_t pin_nosecone, uint8_t pin_expulsion, uint8_t cutter_presence, int16_t log_number, int32_t log_good, uint8_t payload_board_state, uint8_t motor_board_state, uint8_t payload_can_status, uint8_t motor_can_status, uint8_t rig_can_status, uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, liftoff_ts);
    _mav_put_uint64_t(buf, 16, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 24, max_speed_ts);
    _mav_put_uint64_t(buf, 32, max_mach_ts);
    _mav_put_uint64_t(buf, 40, shutdown_ts);
    _mav_put_uint64_t(buf, 48, apogee_ts);
    _mav_put_uint64_t(buf, 56, apogee_max_acc_ts);
    _mav_put_uint64_t(buf, 64, dpl_ts);
    _mav_put_uint64_t(buf, 72, dpl_max_acc_ts);
    _mav_put_uint64_t(buf, 80, dpl_bay_max_pressure_ts);
    _mav_put_float(buf, 88, liftoff_max_acc);
    _mav_put_float(buf, 92, max_speed);
    _mav_put_float(buf, 96, max_speed_altitude);
    _mav_put_float(buf, 100, max_mach);
    _mav_put_float(buf, 104, shutdown_alt);
    _mav_put_float(buf, 108, apogee_lat);
    _mav_put_float(buf, 112, apogee_lon);
    _mav_put_float(buf, 116, apogee_alt);
    _mav_put_float(buf, 120, apogee_max_acc);
    _mav_put_float(buf, 124, dpl_alt);
    _mav_put_float(buf, 128, dpl_max_acc);
    _mav_put_float(buf, 132, dpl_bay_max_pressure);
    _mav_put_float(buf, 136, ref_lat);
    _mav_put_float(buf, 140, ref_lon);
    _mav_put_float(buf, 144, ref_alt);
    _mav_put_float(buf, 148, cpu_load);
    _mav_put_uint32_t(buf, 152, free_heap);
    _mav_put_float(buf, 156, exp_angle);
    _mav_put_int32_t(buf, 160, log_good);
    _mav_put_int16_t(buf, 164, log_number);
    _mav_put_uint8_t(buf, 166, ada_state);
    _mav_put_uint8_t(buf, 167, abk_state);
    _mav_put_uint8_t(buf, 168, nas_state);
    _mav_put_uint8_t(buf, 169, mea_state);
    _mav_put_uint8_t(buf, 170, pin_launch);
    _mav_put_uint8_t(buf, 171, pin_nosecone);
    _mav_put_uint8_t(buf, 172, pin_expulsion);
    _mav_put_uint8_t(buf, 173, cutter_presence);
    _mav_put_uint8_t(buf, 174, payload_board_state);
    _mav_put_uint8_t(buf, 175, motor_board_state);
    _mav_put_uint8_t(buf, 176, payload_can_status);
    _mav_put_uint8_t(buf, 177, motor_can_status);
    _mav_put_uint8_t(buf, 178, rig_can_status);
    _mav_put_uint8_t(buf, 179, hil_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_STATS_TM, buf, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
#else
    mavlink_rocket_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.liftoff_ts = liftoff_ts;
    packet.liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet.max_speed_ts = max_speed_ts;
    packet.max_mach_ts = max_mach_ts;
    packet.shutdown_ts = shutdown_ts;
    packet.apogee_ts = apogee_ts;
    packet.apogee_max_acc_ts = apogee_max_acc_ts;
    packet.dpl_ts = dpl_ts;
    packet.dpl_max_acc_ts = dpl_max_acc_ts;
    packet.dpl_bay_max_pressure_ts = dpl_bay_max_pressure_ts;
    packet.liftoff_max_acc = liftoff_max_acc;
    packet.max_speed = max_speed;
    packet.max_speed_altitude = max_speed_altitude;
    packet.max_mach = max_mach;
    packet.shutdown_alt = shutdown_alt;
    packet.apogee_lat = apogee_lat;
    packet.apogee_lon = apogee_lon;
    packet.apogee_alt = apogee_alt;
    packet.apogee_max_acc = apogee_max_acc;
    packet.dpl_alt = dpl_alt;
    packet.dpl_max_acc = dpl_max_acc;
    packet.dpl_bay_max_pressure = dpl_bay_max_pressure;
    packet.ref_lat = ref_lat;
    packet.ref_lon = ref_lon;
    packet.ref_alt = ref_alt;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;
    packet.exp_angle = exp_angle;
    packet.log_good = log_good;
    packet.log_number = log_number;
    packet.ada_state = ada_state;
    packet.abk_state = abk_state;
    packet.nas_state = nas_state;
    packet.mea_state = mea_state;
    packet.pin_launch = pin_launch;
    packet.pin_nosecone = pin_nosecone;
    packet.pin_expulsion = pin_expulsion;
    packet.cutter_presence = cutter_presence;
    packet.payload_board_state = payload_board_state;
    packet.motor_board_state = motor_board_state;
    packet.payload_can_status = payload_can_status;
    packet.motor_can_status = motor_can_status;
    packet.rig_can_status = rig_can_status;
    packet.hil_state = hil_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_STATS_TM, (const char *)&packet, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
#endif
}

/**
 * @brief Send a rocket_stats_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rocket_stats_tm_send_struct(mavlink_channel_t chan, const mavlink_rocket_stats_tm_t* rocket_stats_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rocket_stats_tm_send(chan, rocket_stats_tm->timestamp, rocket_stats_tm->liftoff_ts, rocket_stats_tm->liftoff_max_acc_ts, rocket_stats_tm->liftoff_max_acc, rocket_stats_tm->max_speed_ts, rocket_stats_tm->max_speed, rocket_stats_tm->max_speed_altitude, rocket_stats_tm->max_mach_ts, rocket_stats_tm->max_mach, rocket_stats_tm->shutdown_ts, rocket_stats_tm->shutdown_alt, rocket_stats_tm->apogee_ts, rocket_stats_tm->apogee_lat, rocket_stats_tm->apogee_lon, rocket_stats_tm->apogee_alt, rocket_stats_tm->apogee_max_acc_ts, rocket_stats_tm->apogee_max_acc, rocket_stats_tm->dpl_ts, rocket_stats_tm->dpl_alt, rocket_stats_tm->dpl_max_acc_ts, rocket_stats_tm->dpl_max_acc, rocket_stats_tm->dpl_bay_max_pressure_ts, rocket_stats_tm->dpl_bay_max_pressure, rocket_stats_tm->ref_lat, rocket_stats_tm->ref_lon, rocket_stats_tm->ref_alt, rocket_stats_tm->cpu_load, rocket_stats_tm->free_heap, rocket_stats_tm->ada_state, rocket_stats_tm->abk_state, rocket_stats_tm->nas_state, rocket_stats_tm->mea_state, rocket_stats_tm->exp_angle, rocket_stats_tm->pin_launch, rocket_stats_tm->pin_nosecone, rocket_stats_tm->pin_expulsion, rocket_stats_tm->cutter_presence, rocket_stats_tm->log_number, rocket_stats_tm->log_good, rocket_stats_tm->payload_board_state, rocket_stats_tm->motor_board_state, rocket_stats_tm->payload_can_status, rocket_stats_tm->motor_can_status, rocket_stats_tm->rig_can_status, rocket_stats_tm->hil_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_STATS_TM, (const char *)rocket_stats_tm, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rocket_stats_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint64_t liftoff_ts, uint64_t liftoff_max_acc_ts, float liftoff_max_acc, uint64_t max_speed_ts, float max_speed, float max_speed_altitude, uint64_t max_mach_ts, float max_mach, uint64_t shutdown_ts, float shutdown_alt, uint64_t apogee_ts, float apogee_lat, float apogee_lon, float apogee_alt, uint64_t apogee_max_acc_ts, float apogee_max_acc, uint64_t dpl_ts, float dpl_alt, uint64_t dpl_max_acc_ts, float dpl_max_acc, uint64_t dpl_bay_max_pressure_ts, float dpl_bay_max_pressure, float ref_lat, float ref_lon, float ref_alt, float cpu_load, uint32_t free_heap, uint8_t ada_state, uint8_t abk_state, uint8_t nas_state, uint8_t mea_state, float exp_angle, uint8_t pin_launch, uint8_t pin_nosecone, uint8_t pin_expulsion, uint8_t cutter_presence, int16_t log_number, int32_t log_good, uint8_t payload_board_state, uint8_t motor_board_state, uint8_t payload_can_status, uint8_t motor_can_status, uint8_t rig_can_status, uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, liftoff_ts);
    _mav_put_uint64_t(buf, 16, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 24, max_speed_ts);
    _mav_put_uint64_t(buf, 32, max_mach_ts);
    _mav_put_uint64_t(buf, 40, shutdown_ts);
    _mav_put_uint64_t(buf, 48, apogee_ts);
    _mav_put_uint64_t(buf, 56, apogee_max_acc_ts);
    _mav_put_uint64_t(buf, 64, dpl_ts);
    _mav_put_uint64_t(buf, 72, dpl_max_acc_ts);
    _mav_put_uint64_t(buf, 80, dpl_bay_max_pressure_ts);
    _mav_put_float(buf, 88, liftoff_max_acc);
    _mav_put_float(buf, 92, max_speed);
    _mav_put_float(buf, 96, max_speed_altitude);
    _mav_put_float(buf, 100, max_mach);
    _mav_put_float(buf, 104, shutdown_alt);
    _mav_put_float(buf, 108, apogee_lat);
    _mav_put_float(buf, 112, apogee_lon);
    _mav_put_float(buf, 116, apogee_alt);
    _mav_put_float(buf, 120, apogee_max_acc);
    _mav_put_float(buf, 124, dpl_alt);
    _mav_put_float(buf, 128, dpl_max_acc);
    _mav_put_float(buf, 132, dpl_bay_max_pressure);
    _mav_put_float(buf, 136, ref_lat);
    _mav_put_float(buf, 140, ref_lon);
    _mav_put_float(buf, 144, ref_alt);
    _mav_put_float(buf, 148, cpu_load);
    _mav_put_uint32_t(buf, 152, free_heap);
    _mav_put_float(buf, 156, exp_angle);
    _mav_put_int32_t(buf, 160, log_good);
    _mav_put_int16_t(buf, 164, log_number);
    _mav_put_uint8_t(buf, 166, ada_state);
    _mav_put_uint8_t(buf, 167, abk_state);
    _mav_put_uint8_t(buf, 168, nas_state);
    _mav_put_uint8_t(buf, 169, mea_state);
    _mav_put_uint8_t(buf, 170, pin_launch);
    _mav_put_uint8_t(buf, 171, pin_nosecone);
    _mav_put_uint8_t(buf, 172, pin_expulsion);
    _mav_put_uint8_t(buf, 173, cutter_presence);
    _mav_put_uint8_t(buf, 174, payload_board_state);
    _mav_put_uint8_t(buf, 175, motor_board_state);
    _mav_put_uint8_t(buf, 176, payload_can_status);
    _mav_put_uint8_t(buf, 177, motor_can_status);
    _mav_put_uint8_t(buf, 178, rig_can_status);
    _mav_put_uint8_t(buf, 179, hil_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_STATS_TM, buf, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
#else
    mavlink_rocket_stats_tm_t *packet = (mavlink_rocket_stats_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->liftoff_ts = liftoff_ts;
    packet->liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet->max_speed_ts = max_speed_ts;
    packet->max_mach_ts = max_mach_ts;
    packet->shutdown_ts = shutdown_ts;
    packet->apogee_ts = apogee_ts;
    packet->apogee_max_acc_ts = apogee_max_acc_ts;
    packet->dpl_ts = dpl_ts;
    packet->dpl_max_acc_ts = dpl_max_acc_ts;
    packet->dpl_bay_max_pressure_ts = dpl_bay_max_pressure_ts;
    packet->liftoff_max_acc = liftoff_max_acc;
    packet->max_speed = max_speed;
    packet->max_speed_altitude = max_speed_altitude;
    packet->max_mach = max_mach;
    packet->shutdown_alt = shutdown_alt;
    packet->apogee_lat = apogee_lat;
    packet->apogee_lon = apogee_lon;
    packet->apogee_alt = apogee_alt;
    packet->apogee_max_acc = apogee_max_acc;
    packet->dpl_alt = dpl_alt;
    packet->dpl_max_acc = dpl_max_acc;
    packet->dpl_bay_max_pressure = dpl_bay_max_pressure;
    packet->ref_lat = ref_lat;
    packet->ref_lon = ref_lon;
    packet->ref_alt = ref_alt;
    packet->cpu_load = cpu_load;
    packet->free_heap = free_heap;
    packet->exp_angle = exp_angle;
    packet->log_good = log_good;
    packet->log_number = log_number;
    packet->ada_state = ada_state;
    packet->abk_state = abk_state;
    packet->nas_state = nas_state;
    packet->mea_state = mea_state;
    packet->pin_launch = pin_launch;
    packet->pin_nosecone = pin_nosecone;
    packet->pin_expulsion = pin_expulsion;
    packet->cutter_presence = cutter_presence;
    packet->payload_board_state = payload_board_state;
    packet->motor_board_state = motor_board_state;
    packet->payload_can_status = payload_can_status;
    packet->motor_can_status = motor_can_status;
    packet->rig_can_status = rig_can_status;
    packet->hil_state = hil_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_STATS_TM, (const char *)packet, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ROCKET_STATS_TM UNPACKING


/**
 * @brief Get field timestamp from rocket_stats_tm message
 *
 * @return [us] Timestamp in microseconds
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field liftoff_ts from rocket_stats_tm message
 *
 * @return [us] System clock at liftoff
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_liftoff_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field liftoff_max_acc_ts from rocket_stats_tm message
 *
 * @return [us] System clock at the maximum liftoff acceleration
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_liftoff_max_acc_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Get field liftoff_max_acc from rocket_stats_tm message
 *
 * @return [m/s2] Maximum liftoff acceleration
 */
static inline float mavlink_msg_rocket_stats_tm_get_liftoff_max_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field max_speed_ts from rocket_stats_tm message
 *
 * @return [us] System clock at max speed
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_max_speed_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  24);
}

/**
 * @brief Get field max_speed from rocket_stats_tm message
 *
 * @return [m/s] Max measured speed
 */
static inline float mavlink_msg_rocket_stats_tm_get_max_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field max_speed_altitude from rocket_stats_tm message
 *
 * @return [m] Altitude at max speed
 */
static inline float mavlink_msg_rocket_stats_tm_get_max_speed_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field max_mach_ts from rocket_stats_tm message
 *
 * @return  System clock at maximum measured mach
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_max_mach_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  32);
}

/**
 * @brief Get field max_mach from rocket_stats_tm message
 *
 * @return  Maximum measured mach
 */
static inline float mavlink_msg_rocket_stats_tm_get_max_mach(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Get field shutdown_ts from rocket_stats_tm message
 *
 * @return  System clock at shutdown
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_shutdown_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  40);
}

/**
 * @brief Get field shutdown_alt from rocket_stats_tm message
 *
 * @return  Altitude at shutdown
 */
static inline float mavlink_msg_rocket_stats_tm_get_shutdown_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  104);
}

/**
 * @brief Get field apogee_ts from rocket_stats_tm message
 *
 * @return [us] System clock at apogee
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_apogee_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  48);
}

/**
 * @brief Get field apogee_lat from rocket_stats_tm message
 *
 * @return [deg] Apogee latitude
 */
static inline float mavlink_msg_rocket_stats_tm_get_apogee_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Get field apogee_lon from rocket_stats_tm message
 *
 * @return [deg] Apogee longitude
 */
static inline float mavlink_msg_rocket_stats_tm_get_apogee_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Get field apogee_alt from rocket_stats_tm message
 *
 * @return [m] Apogee altitude
 */
static inline float mavlink_msg_rocket_stats_tm_get_apogee_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  116);
}

/**
 * @brief Get field apogee_max_acc_ts from rocket_stats_tm message
 *
 * @return [us] System clock at max acceleration after apogee
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_apogee_max_acc_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  56);
}

/**
 * @brief Get field apogee_max_acc from rocket_stats_tm message
 *
 * @return [m/s2] Max acceleration after apogee
 */
static inline float mavlink_msg_rocket_stats_tm_get_apogee_max_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  120);
}

/**
 * @brief Get field dpl_ts from rocket_stats_tm message
 *
 * @return [us] System clock at main deployment
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_dpl_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  64);
}

/**
 * @brief Get field dpl_alt from rocket_stats_tm message
 *
 * @return [m] Deployment altitude
 */
static inline float mavlink_msg_rocket_stats_tm_get_dpl_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  124);
}

/**
 * @brief Get field dpl_max_acc_ts from rocket_stats_tm message
 *
 * @return [us] System clock at max acceleration during main deployment
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_dpl_max_acc_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  72);
}

/**
 * @brief Get field dpl_max_acc from rocket_stats_tm message
 *
 * @return [m/s2] Max acceleration during main deployment
 */
static inline float mavlink_msg_rocket_stats_tm_get_dpl_max_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  128);
}

/**
 * @brief Get field dpl_bay_max_pressure_ts from rocket_stats_tm message
 *
 * @return [us] System clock at max pressure in the deployment bay during drogue deployment
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_dpl_bay_max_pressure_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  80);
}

/**
 * @brief Get field dpl_bay_max_pressure from rocket_stats_tm message
 *
 * @return [Pa] Max pressure in the deployment bay during drogue deployment
 */
static inline float mavlink_msg_rocket_stats_tm_get_dpl_bay_max_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  132);
}

/**
 * @brief Get field ref_lat from rocket_stats_tm message
 *
 * @return [deg] Reference altitude
 */
static inline float mavlink_msg_rocket_stats_tm_get_ref_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  136);
}

/**
 * @brief Get field ref_lon from rocket_stats_tm message
 *
 * @return [deg] Reference longitude
 */
static inline float mavlink_msg_rocket_stats_tm_get_ref_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  140);
}

/**
 * @brief Get field ref_alt from rocket_stats_tm message
 *
 * @return [m] Reference altitude
 */
static inline float mavlink_msg_rocket_stats_tm_get_ref_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  144);
}

/**
 * @brief Get field cpu_load from rocket_stats_tm message
 *
 * @return  CPU load in percentage
 */
static inline float mavlink_msg_rocket_stats_tm_get_cpu_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  148);
}

/**
 * @brief Get field free_heap from rocket_stats_tm message
 *
 * @return  Amount of available heap in memory
 */
static inline uint32_t mavlink_msg_rocket_stats_tm_get_free_heap(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  152);
}

/**
 * @brief Get field ada_state from rocket_stats_tm message
 *
 * @return  ADA Controller State
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_ada_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  166);
}

/**
 * @brief Get field abk_state from rocket_stats_tm message
 *
 * @return  Airbrake FSM state
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_abk_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  167);
}

/**
 * @brief Get field nas_state from rocket_stats_tm message
 *
 * @return  NAS FSM State
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_nas_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  168);
}

/**
 * @brief Get field mea_state from rocket_stats_tm message
 *
 * @return  MEA Controller State
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_mea_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  169);
}

/**
 * @brief Get field exp_angle from rocket_stats_tm message
 *
 * @return  Expulsion servo angle
 */
static inline float mavlink_msg_rocket_stats_tm_get_exp_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  156);
}

/**
 * @brief Get field pin_launch from rocket_stats_tm message
 *
 * @return  Launch pin status (1 = connected, 0 = disconnected)
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_pin_launch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  170);
}

/**
 * @brief Get field pin_nosecone from rocket_stats_tm message
 *
 * @return  Nosecone pin status (1 = connected, 0 = disconnected)
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_pin_nosecone(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  171);
}

/**
 * @brief Get field pin_expulsion from rocket_stats_tm message
 *
 * @return  Servo sensor status (1 = actuated, 0 = idle)
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_pin_expulsion(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  172);
}

/**
 * @brief Get field cutter_presence from rocket_stats_tm message
 *
 * @return  Cutter presence status (1 = present, 0 = missing)
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_cutter_presence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  173);
}

/**
 * @brief Get field log_number from rocket_stats_tm message
 *
 * @return  Currently active log file, -1 if the logger is inactive
 */
static inline int16_t mavlink_msg_rocket_stats_tm_get_log_number(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  164);
}

/**
 * @brief Get field log_good from rocket_stats_tm message
 *
 * @return  0 if log failed, 1 otherwise
 */
static inline int32_t mavlink_msg_rocket_stats_tm_get_log_good(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  160);
}

/**
 * @brief Get field payload_board_state from rocket_stats_tm message
 *
 * @return  Main board fmm state
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_payload_board_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  174);
}

/**
 * @brief Get field motor_board_state from rocket_stats_tm message
 *
 * @return  Motor board fmm state
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_motor_board_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  175);
}

/**
 * @brief Get field payload_can_status from rocket_stats_tm message
 *
 * @return  Main CAN status
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_payload_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  176);
}

/**
 * @brief Get field motor_can_status from rocket_stats_tm message
 *
 * @return  Motor CAN status
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_motor_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  177);
}

/**
 * @brief Get field rig_can_status from rocket_stats_tm message
 *
 * @return  RIG CAN status
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_rig_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  178);
}

/**
 * @brief Get field hil_state from rocket_stats_tm message
 *
 * @return  1 if the board is currently in HIL mode
 */
static inline uint8_t mavlink_msg_rocket_stats_tm_get_hil_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  179);
}

/**
 * @brief Decode a rocket_stats_tm message into a struct
 *
 * @param msg The message to decode
 * @param rocket_stats_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_rocket_stats_tm_decode(const mavlink_message_t* msg, mavlink_rocket_stats_tm_t* rocket_stats_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rocket_stats_tm->timestamp = mavlink_msg_rocket_stats_tm_get_timestamp(msg);
    rocket_stats_tm->liftoff_ts = mavlink_msg_rocket_stats_tm_get_liftoff_ts(msg);
    rocket_stats_tm->liftoff_max_acc_ts = mavlink_msg_rocket_stats_tm_get_liftoff_max_acc_ts(msg);
    rocket_stats_tm->max_speed_ts = mavlink_msg_rocket_stats_tm_get_max_speed_ts(msg);
    rocket_stats_tm->max_mach_ts = mavlink_msg_rocket_stats_tm_get_max_mach_ts(msg);
    rocket_stats_tm->shutdown_ts = mavlink_msg_rocket_stats_tm_get_shutdown_ts(msg);
    rocket_stats_tm->apogee_ts = mavlink_msg_rocket_stats_tm_get_apogee_ts(msg);
    rocket_stats_tm->apogee_max_acc_ts = mavlink_msg_rocket_stats_tm_get_apogee_max_acc_ts(msg);
    rocket_stats_tm->dpl_ts = mavlink_msg_rocket_stats_tm_get_dpl_ts(msg);
    rocket_stats_tm->dpl_max_acc_ts = mavlink_msg_rocket_stats_tm_get_dpl_max_acc_ts(msg);
    rocket_stats_tm->dpl_bay_max_pressure_ts = mavlink_msg_rocket_stats_tm_get_dpl_bay_max_pressure_ts(msg);
    rocket_stats_tm->liftoff_max_acc = mavlink_msg_rocket_stats_tm_get_liftoff_max_acc(msg);
    rocket_stats_tm->max_speed = mavlink_msg_rocket_stats_tm_get_max_speed(msg);
    rocket_stats_tm->max_speed_altitude = mavlink_msg_rocket_stats_tm_get_max_speed_altitude(msg);
    rocket_stats_tm->max_mach = mavlink_msg_rocket_stats_tm_get_max_mach(msg);
    rocket_stats_tm->shutdown_alt = mavlink_msg_rocket_stats_tm_get_shutdown_alt(msg);
    rocket_stats_tm->apogee_lat = mavlink_msg_rocket_stats_tm_get_apogee_lat(msg);
    rocket_stats_tm->apogee_lon = mavlink_msg_rocket_stats_tm_get_apogee_lon(msg);
    rocket_stats_tm->apogee_alt = mavlink_msg_rocket_stats_tm_get_apogee_alt(msg);
    rocket_stats_tm->apogee_max_acc = mavlink_msg_rocket_stats_tm_get_apogee_max_acc(msg);
    rocket_stats_tm->dpl_alt = mavlink_msg_rocket_stats_tm_get_dpl_alt(msg);
    rocket_stats_tm->dpl_max_acc = mavlink_msg_rocket_stats_tm_get_dpl_max_acc(msg);
    rocket_stats_tm->dpl_bay_max_pressure = mavlink_msg_rocket_stats_tm_get_dpl_bay_max_pressure(msg);
    rocket_stats_tm->ref_lat = mavlink_msg_rocket_stats_tm_get_ref_lat(msg);
    rocket_stats_tm->ref_lon = mavlink_msg_rocket_stats_tm_get_ref_lon(msg);
    rocket_stats_tm->ref_alt = mavlink_msg_rocket_stats_tm_get_ref_alt(msg);
    rocket_stats_tm->cpu_load = mavlink_msg_rocket_stats_tm_get_cpu_load(msg);
    rocket_stats_tm->free_heap = mavlink_msg_rocket_stats_tm_get_free_heap(msg);
    rocket_stats_tm->exp_angle = mavlink_msg_rocket_stats_tm_get_exp_angle(msg);
    rocket_stats_tm->log_good = mavlink_msg_rocket_stats_tm_get_log_good(msg);
    rocket_stats_tm->log_number = mavlink_msg_rocket_stats_tm_get_log_number(msg);
    rocket_stats_tm->ada_state = mavlink_msg_rocket_stats_tm_get_ada_state(msg);
    rocket_stats_tm->abk_state = mavlink_msg_rocket_stats_tm_get_abk_state(msg);
    rocket_stats_tm->nas_state = mavlink_msg_rocket_stats_tm_get_nas_state(msg);
    rocket_stats_tm->mea_state = mavlink_msg_rocket_stats_tm_get_mea_state(msg);
    rocket_stats_tm->pin_launch = mavlink_msg_rocket_stats_tm_get_pin_launch(msg);
    rocket_stats_tm->pin_nosecone = mavlink_msg_rocket_stats_tm_get_pin_nosecone(msg);
    rocket_stats_tm->pin_expulsion = mavlink_msg_rocket_stats_tm_get_pin_expulsion(msg);
    rocket_stats_tm->cutter_presence = mavlink_msg_rocket_stats_tm_get_cutter_presence(msg);
    rocket_stats_tm->payload_board_state = mavlink_msg_rocket_stats_tm_get_payload_board_state(msg);
    rocket_stats_tm->motor_board_state = mavlink_msg_rocket_stats_tm_get_motor_board_state(msg);
    rocket_stats_tm->payload_can_status = mavlink_msg_rocket_stats_tm_get_payload_can_status(msg);
    rocket_stats_tm->motor_can_status = mavlink_msg_rocket_stats_tm_get_motor_can_status(msg);
    rocket_stats_tm->rig_can_status = mavlink_msg_rocket_stats_tm_get_rig_can_status(msg);
    rocket_stats_tm->hil_state = mavlink_msg_rocket_stats_tm_get_hil_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN? msg->len : MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN;
        memset(rocket_stats_tm, 0, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN);
    memcpy(rocket_stats_tm, _MAV_PAYLOAD(msg), len);
#endif
}
