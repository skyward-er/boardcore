#pragma once
// MESSAGE PAYLOAD_STATS_TM PACKING

#define MAVLINK_MSG_ID_PAYLOAD_STATS_TM 211


typedef struct __mavlink_payload_stats_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 uint64_t liftoff_ts; /*< [us] System clock at liftoff*/
 uint64_t liftoff_max_acc_ts; /*< [us] System clock at the maximum liftoff acceleration*/
 uint64_t max_speed_ts; /*< [us] System clock at max speed*/
 uint64_t max_mach_ts; /*<  System clock at maximum measured mach*/
 uint64_t apogee_ts; /*< [us] System clock at apogee*/
 uint64_t apogee_max_acc_ts; /*< [us] System clock at max acceleration after apogee*/
 uint64_t dpl_ts; /*< [us] System clock at main deployment*/
 uint64_t dpl_max_acc_ts; /*< [us] System clock at max acceleration during main deployment*/
 float liftoff_max_acc; /*< [m/s2] Maximum liftoff acceleration*/
 float max_speed; /*< [m/s] Max measured speed*/
 float max_speed_altitude; /*< [m] Altitude at max speed*/
 float max_mach; /*<  Maximum measured mach*/
 float apogee_lat; /*< [deg] Apogee latitude*/
 float apogee_lon; /*< [deg] Apogee longitude*/
 float apogee_alt; /*< [m] Apogee altitude*/
 float apogee_max_acc; /*< [m/s2] Max acceleration after apogee*/
 float wing_target_lat; /*< [deg] Wing target latitude*/
 float wing_target_lon; /*< [deg] Wing target longitude*/
 float wing_active_target_n; /*< [m] Wing active target N*/
 float wing_active_target_e; /*< [m] Wing active target E*/
 float dpl_alt; /*< [m] Deployment altitude*/
 float dpl_max_acc; /*< [m/s2] Max acceleration during main deployment*/
 float ref_lat; /*< [deg] Reference altitude*/
 float ref_lon; /*< [deg] Reference longitude*/
 float ref_alt; /*< [m] Reference altitude*/
 float min_pressure; /*< [Pa] Apogee pressure - Digital barometer*/
 float cpu_load; /*<  CPU load in percentage*/
 uint32_t free_heap; /*<  Amount of available heap in memory*/
 int32_t log_good; /*<  0 if log failed, 1 otherwise*/
 int16_t log_number; /*<  Currently active log file, -1 if the logger is inactive*/
 uint8_t wing_algorithm; /*<  Wing selected algorithm*/
 uint8_t nas_state; /*<  NAS FSM State*/
 uint8_t wnc_state; /*<  Wing Controller State*/
 uint8_t pin_launch; /*<  Launch pin status (1 = connected, 0 = disconnected)*/
 uint8_t pin_nosecone; /*<  Nosecone pin status (1 = connected, 0 = disconnected)*/
 uint8_t cutter_presence; /*<  Cutter presence status (1 = present, 0 = missing)*/
 uint8_t main_board_state; /*<  Main board fmm state*/
 uint8_t motor_board_state; /*<  Motor board fmm state*/
 uint8_t main_can_status; /*<  Main CAN status*/
 uint8_t motor_can_status; /*<  Motor CAN status*/
 uint8_t rig_can_status; /*<  RIG CAN status*/
 uint8_t hil_state; /*<  1 if the board is currently in HIL mode*/
} mavlink_payload_stats_tm_t;

#define MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN 170
#define MAVLINK_MSG_ID_PAYLOAD_STATS_TM_MIN_LEN 170
#define MAVLINK_MSG_ID_211_LEN 170
#define MAVLINK_MSG_ID_211_MIN_LEN 170

#define MAVLINK_MSG_ID_PAYLOAD_STATS_TM_CRC 99
#define MAVLINK_MSG_ID_211_CRC 99



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PAYLOAD_STATS_TM { \
    211, \
    "PAYLOAD_STATS_TM", \
    43, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_payload_stats_tm_t, timestamp) }, \
         { "liftoff_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_payload_stats_tm_t, liftoff_ts) }, \
         { "liftoff_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_payload_stats_tm_t, liftoff_max_acc_ts) }, \
         { "liftoff_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_payload_stats_tm_t, liftoff_max_acc) }, \
         { "max_speed_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 24, offsetof(mavlink_payload_stats_tm_t, max_speed_ts) }, \
         { "max_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_payload_stats_tm_t, max_speed) }, \
         { "max_speed_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_payload_stats_tm_t, max_speed_altitude) }, \
         { "max_mach_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 32, offsetof(mavlink_payload_stats_tm_t, max_mach_ts) }, \
         { "max_mach", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_payload_stats_tm_t, max_mach) }, \
         { "apogee_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 40, offsetof(mavlink_payload_stats_tm_t, apogee_ts) }, \
         { "apogee_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_payload_stats_tm_t, apogee_lat) }, \
         { "apogee_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_payload_stats_tm_t, apogee_lon) }, \
         { "apogee_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_payload_stats_tm_t, apogee_alt) }, \
         { "apogee_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 48, offsetof(mavlink_payload_stats_tm_t, apogee_max_acc_ts) }, \
         { "apogee_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_payload_stats_tm_t, apogee_max_acc) }, \
         { "wing_target_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_payload_stats_tm_t, wing_target_lat) }, \
         { "wing_target_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_payload_stats_tm_t, wing_target_lon) }, \
         { "wing_active_target_n", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_payload_stats_tm_t, wing_active_target_n) }, \
         { "wing_active_target_e", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_payload_stats_tm_t, wing_active_target_e) }, \
         { "wing_algorithm", NULL, MAVLINK_TYPE_UINT8_T, 0, 158, offsetof(mavlink_payload_stats_tm_t, wing_algorithm) }, \
         { "dpl_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 56, offsetof(mavlink_payload_stats_tm_t, dpl_ts) }, \
         { "dpl_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_payload_stats_tm_t, dpl_alt) }, \
         { "dpl_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 64, offsetof(mavlink_payload_stats_tm_t, dpl_max_acc_ts) }, \
         { "dpl_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_payload_stats_tm_t, dpl_max_acc) }, \
         { "ref_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_payload_stats_tm_t, ref_lat) }, \
         { "ref_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_payload_stats_tm_t, ref_lon) }, \
         { "ref_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_payload_stats_tm_t, ref_alt) }, \
         { "min_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_payload_stats_tm_t, min_pressure) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_payload_stats_tm_t, cpu_load) }, \
         { "free_heap", NULL, MAVLINK_TYPE_UINT32_T, 0, 148, offsetof(mavlink_payload_stats_tm_t, free_heap) }, \
         { "nas_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 159, offsetof(mavlink_payload_stats_tm_t, nas_state) }, \
         { "wnc_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 160, offsetof(mavlink_payload_stats_tm_t, wnc_state) }, \
         { "pin_launch", NULL, MAVLINK_TYPE_UINT8_T, 0, 161, offsetof(mavlink_payload_stats_tm_t, pin_launch) }, \
         { "pin_nosecone", NULL, MAVLINK_TYPE_UINT8_T, 0, 162, offsetof(mavlink_payload_stats_tm_t, pin_nosecone) }, \
         { "cutter_presence", NULL, MAVLINK_TYPE_UINT8_T, 0, 163, offsetof(mavlink_payload_stats_tm_t, cutter_presence) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 156, offsetof(mavlink_payload_stats_tm_t, log_number) }, \
         { "log_good", NULL, MAVLINK_TYPE_INT32_T, 0, 152, offsetof(mavlink_payload_stats_tm_t, log_good) }, \
         { "main_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 164, offsetof(mavlink_payload_stats_tm_t, main_board_state) }, \
         { "motor_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 165, offsetof(mavlink_payload_stats_tm_t, motor_board_state) }, \
         { "main_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 166, offsetof(mavlink_payload_stats_tm_t, main_can_status) }, \
         { "motor_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 167, offsetof(mavlink_payload_stats_tm_t, motor_can_status) }, \
         { "rig_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 168, offsetof(mavlink_payload_stats_tm_t, rig_can_status) }, \
         { "hil_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 169, offsetof(mavlink_payload_stats_tm_t, hil_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PAYLOAD_STATS_TM { \
    "PAYLOAD_STATS_TM", \
    43, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_payload_stats_tm_t, timestamp) }, \
         { "liftoff_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_payload_stats_tm_t, liftoff_ts) }, \
         { "liftoff_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_payload_stats_tm_t, liftoff_max_acc_ts) }, \
         { "liftoff_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_payload_stats_tm_t, liftoff_max_acc) }, \
         { "max_speed_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 24, offsetof(mavlink_payload_stats_tm_t, max_speed_ts) }, \
         { "max_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_payload_stats_tm_t, max_speed) }, \
         { "max_speed_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_payload_stats_tm_t, max_speed_altitude) }, \
         { "max_mach_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 32, offsetof(mavlink_payload_stats_tm_t, max_mach_ts) }, \
         { "max_mach", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_payload_stats_tm_t, max_mach) }, \
         { "apogee_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 40, offsetof(mavlink_payload_stats_tm_t, apogee_ts) }, \
         { "apogee_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_payload_stats_tm_t, apogee_lat) }, \
         { "apogee_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_payload_stats_tm_t, apogee_lon) }, \
         { "apogee_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_payload_stats_tm_t, apogee_alt) }, \
         { "apogee_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 48, offsetof(mavlink_payload_stats_tm_t, apogee_max_acc_ts) }, \
         { "apogee_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_payload_stats_tm_t, apogee_max_acc) }, \
         { "wing_target_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_payload_stats_tm_t, wing_target_lat) }, \
         { "wing_target_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_payload_stats_tm_t, wing_target_lon) }, \
         { "wing_active_target_n", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_payload_stats_tm_t, wing_active_target_n) }, \
         { "wing_active_target_e", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_payload_stats_tm_t, wing_active_target_e) }, \
         { "wing_algorithm", NULL, MAVLINK_TYPE_UINT8_T, 0, 158, offsetof(mavlink_payload_stats_tm_t, wing_algorithm) }, \
         { "dpl_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 56, offsetof(mavlink_payload_stats_tm_t, dpl_ts) }, \
         { "dpl_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_payload_stats_tm_t, dpl_alt) }, \
         { "dpl_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 64, offsetof(mavlink_payload_stats_tm_t, dpl_max_acc_ts) }, \
         { "dpl_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_payload_stats_tm_t, dpl_max_acc) }, \
         { "ref_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_payload_stats_tm_t, ref_lat) }, \
         { "ref_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_payload_stats_tm_t, ref_lon) }, \
         { "ref_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_payload_stats_tm_t, ref_alt) }, \
         { "min_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_payload_stats_tm_t, min_pressure) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_payload_stats_tm_t, cpu_load) }, \
         { "free_heap", NULL, MAVLINK_TYPE_UINT32_T, 0, 148, offsetof(mavlink_payload_stats_tm_t, free_heap) }, \
         { "nas_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 159, offsetof(mavlink_payload_stats_tm_t, nas_state) }, \
         { "wnc_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 160, offsetof(mavlink_payload_stats_tm_t, wnc_state) }, \
         { "pin_launch", NULL, MAVLINK_TYPE_UINT8_T, 0, 161, offsetof(mavlink_payload_stats_tm_t, pin_launch) }, \
         { "pin_nosecone", NULL, MAVLINK_TYPE_UINT8_T, 0, 162, offsetof(mavlink_payload_stats_tm_t, pin_nosecone) }, \
         { "cutter_presence", NULL, MAVLINK_TYPE_UINT8_T, 0, 163, offsetof(mavlink_payload_stats_tm_t, cutter_presence) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 156, offsetof(mavlink_payload_stats_tm_t, log_number) }, \
         { "log_good", NULL, MAVLINK_TYPE_INT32_T, 0, 152, offsetof(mavlink_payload_stats_tm_t, log_good) }, \
         { "main_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 164, offsetof(mavlink_payload_stats_tm_t, main_board_state) }, \
         { "motor_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 165, offsetof(mavlink_payload_stats_tm_t, motor_board_state) }, \
         { "main_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 166, offsetof(mavlink_payload_stats_tm_t, main_can_status) }, \
         { "motor_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 167, offsetof(mavlink_payload_stats_tm_t, motor_can_status) }, \
         { "rig_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 168, offsetof(mavlink_payload_stats_tm_t, rig_can_status) }, \
         { "hil_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 169, offsetof(mavlink_payload_stats_tm_t, hil_state) }, \
         } \
}
#endif

/**
 * @brief Pack a payload_stats_tm message
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
 * @param apogee_ts [us] System clock at apogee
 * @param apogee_lat [deg] Apogee latitude
 * @param apogee_lon [deg] Apogee longitude
 * @param apogee_alt [m] Apogee altitude
 * @param apogee_max_acc_ts [us] System clock at max acceleration after apogee
 * @param apogee_max_acc [m/s2] Max acceleration after apogee
 * @param wing_target_lat [deg] Wing target latitude
 * @param wing_target_lon [deg] Wing target longitude
 * @param wing_active_target_n [m] Wing active target N
 * @param wing_active_target_e [m] Wing active target E
 * @param wing_algorithm  Wing selected algorithm
 * @param dpl_ts [us] System clock at main deployment
 * @param dpl_alt [m] Deployment altitude
 * @param dpl_max_acc_ts [us] System clock at max acceleration during main deployment
 * @param dpl_max_acc [m/s2] Max acceleration during main deployment
 * @param ref_lat [deg] Reference altitude
 * @param ref_lon [deg] Reference longitude
 * @param ref_alt [m] Reference altitude
 * @param min_pressure [Pa] Apogee pressure - Digital barometer
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @param nas_state  NAS FSM State
 * @param wnc_state  Wing Controller State
 * @param pin_launch  Launch pin status (1 = connected, 0 = disconnected)
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param cutter_presence  Cutter presence status (1 = present, 0 = missing)
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @param main_board_state  Main board fmm state
 * @param motor_board_state  Motor board fmm state
 * @param main_can_status  Main CAN status
 * @param motor_can_status  Motor CAN status
 * @param rig_can_status  RIG CAN status
 * @param hil_state  1 if the board is currently in HIL mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_payload_stats_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint64_t liftoff_ts, uint64_t liftoff_max_acc_ts, float liftoff_max_acc, uint64_t max_speed_ts, float max_speed, float max_speed_altitude, uint64_t max_mach_ts, float max_mach, uint64_t apogee_ts, float apogee_lat, float apogee_lon, float apogee_alt, uint64_t apogee_max_acc_ts, float apogee_max_acc, float wing_target_lat, float wing_target_lon, float wing_active_target_n, float wing_active_target_e, uint8_t wing_algorithm, uint64_t dpl_ts, float dpl_alt, uint64_t dpl_max_acc_ts, float dpl_max_acc, float ref_lat, float ref_lon, float ref_alt, float min_pressure, float cpu_load, uint32_t free_heap, uint8_t nas_state, uint8_t wnc_state, uint8_t pin_launch, uint8_t pin_nosecone, uint8_t cutter_presence, int16_t log_number, int32_t log_good, uint8_t main_board_state, uint8_t motor_board_state, uint8_t main_can_status, uint8_t motor_can_status, uint8_t rig_can_status, uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, liftoff_ts);
    _mav_put_uint64_t(buf, 16, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 24, max_speed_ts);
    _mav_put_uint64_t(buf, 32, max_mach_ts);
    _mav_put_uint64_t(buf, 40, apogee_ts);
    _mav_put_uint64_t(buf, 48, apogee_max_acc_ts);
    _mav_put_uint64_t(buf, 56, dpl_ts);
    _mav_put_uint64_t(buf, 64, dpl_max_acc_ts);
    _mav_put_float(buf, 72, liftoff_max_acc);
    _mav_put_float(buf, 76, max_speed);
    _mav_put_float(buf, 80, max_speed_altitude);
    _mav_put_float(buf, 84, max_mach);
    _mav_put_float(buf, 88, apogee_lat);
    _mav_put_float(buf, 92, apogee_lon);
    _mav_put_float(buf, 96, apogee_alt);
    _mav_put_float(buf, 100, apogee_max_acc);
    _mav_put_float(buf, 104, wing_target_lat);
    _mav_put_float(buf, 108, wing_target_lon);
    _mav_put_float(buf, 112, wing_active_target_n);
    _mav_put_float(buf, 116, wing_active_target_e);
    _mav_put_float(buf, 120, dpl_alt);
    _mav_put_float(buf, 124, dpl_max_acc);
    _mav_put_float(buf, 128, ref_lat);
    _mav_put_float(buf, 132, ref_lon);
    _mav_put_float(buf, 136, ref_alt);
    _mav_put_float(buf, 140, min_pressure);
    _mav_put_float(buf, 144, cpu_load);
    _mav_put_uint32_t(buf, 148, free_heap);
    _mav_put_int32_t(buf, 152, log_good);
    _mav_put_int16_t(buf, 156, log_number);
    _mav_put_uint8_t(buf, 158, wing_algorithm);
    _mav_put_uint8_t(buf, 159, nas_state);
    _mav_put_uint8_t(buf, 160, wnc_state);
    _mav_put_uint8_t(buf, 161, pin_launch);
    _mav_put_uint8_t(buf, 162, pin_nosecone);
    _mav_put_uint8_t(buf, 163, cutter_presence);
    _mav_put_uint8_t(buf, 164, main_board_state);
    _mav_put_uint8_t(buf, 165, motor_board_state);
    _mav_put_uint8_t(buf, 166, main_can_status);
    _mav_put_uint8_t(buf, 167, motor_can_status);
    _mav_put_uint8_t(buf, 168, rig_can_status);
    _mav_put_uint8_t(buf, 169, hil_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN);
#else
    mavlink_payload_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.liftoff_ts = liftoff_ts;
    packet.liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet.max_speed_ts = max_speed_ts;
    packet.max_mach_ts = max_mach_ts;
    packet.apogee_ts = apogee_ts;
    packet.apogee_max_acc_ts = apogee_max_acc_ts;
    packet.dpl_ts = dpl_ts;
    packet.dpl_max_acc_ts = dpl_max_acc_ts;
    packet.liftoff_max_acc = liftoff_max_acc;
    packet.max_speed = max_speed;
    packet.max_speed_altitude = max_speed_altitude;
    packet.max_mach = max_mach;
    packet.apogee_lat = apogee_lat;
    packet.apogee_lon = apogee_lon;
    packet.apogee_alt = apogee_alt;
    packet.apogee_max_acc = apogee_max_acc;
    packet.wing_target_lat = wing_target_lat;
    packet.wing_target_lon = wing_target_lon;
    packet.wing_active_target_n = wing_active_target_n;
    packet.wing_active_target_e = wing_active_target_e;
    packet.dpl_alt = dpl_alt;
    packet.dpl_max_acc = dpl_max_acc;
    packet.ref_lat = ref_lat;
    packet.ref_lon = ref_lon;
    packet.ref_alt = ref_alt;
    packet.min_pressure = min_pressure;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;
    packet.log_good = log_good;
    packet.log_number = log_number;
    packet.wing_algorithm = wing_algorithm;
    packet.nas_state = nas_state;
    packet.wnc_state = wnc_state;
    packet.pin_launch = pin_launch;
    packet.pin_nosecone = pin_nosecone;
    packet.cutter_presence = cutter_presence;
    packet.main_board_state = main_board_state;
    packet.motor_board_state = motor_board_state;
    packet.main_can_status = main_can_status;
    packet.motor_can_status = motor_can_status;
    packet.rig_can_status = rig_can_status;
    packet.hil_state = hil_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PAYLOAD_STATS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_CRC);
}

/**
 * @brief Pack a payload_stats_tm message on a channel
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
 * @param apogee_ts [us] System clock at apogee
 * @param apogee_lat [deg] Apogee latitude
 * @param apogee_lon [deg] Apogee longitude
 * @param apogee_alt [m] Apogee altitude
 * @param apogee_max_acc_ts [us] System clock at max acceleration after apogee
 * @param apogee_max_acc [m/s2] Max acceleration after apogee
 * @param wing_target_lat [deg] Wing target latitude
 * @param wing_target_lon [deg] Wing target longitude
 * @param wing_active_target_n [m] Wing active target N
 * @param wing_active_target_e [m] Wing active target E
 * @param wing_algorithm  Wing selected algorithm
 * @param dpl_ts [us] System clock at main deployment
 * @param dpl_alt [m] Deployment altitude
 * @param dpl_max_acc_ts [us] System clock at max acceleration during main deployment
 * @param dpl_max_acc [m/s2] Max acceleration during main deployment
 * @param ref_lat [deg] Reference altitude
 * @param ref_lon [deg] Reference longitude
 * @param ref_alt [m] Reference altitude
 * @param min_pressure [Pa] Apogee pressure - Digital barometer
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @param nas_state  NAS FSM State
 * @param wnc_state  Wing Controller State
 * @param pin_launch  Launch pin status (1 = connected, 0 = disconnected)
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param cutter_presence  Cutter presence status (1 = present, 0 = missing)
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @param main_board_state  Main board fmm state
 * @param motor_board_state  Motor board fmm state
 * @param main_can_status  Main CAN status
 * @param motor_can_status  Motor CAN status
 * @param rig_can_status  RIG CAN status
 * @param hil_state  1 if the board is currently in HIL mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_payload_stats_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint64_t liftoff_ts,uint64_t liftoff_max_acc_ts,float liftoff_max_acc,uint64_t max_speed_ts,float max_speed,float max_speed_altitude,uint64_t max_mach_ts,float max_mach,uint64_t apogee_ts,float apogee_lat,float apogee_lon,float apogee_alt,uint64_t apogee_max_acc_ts,float apogee_max_acc,float wing_target_lat,float wing_target_lon,float wing_active_target_n,float wing_active_target_e,uint8_t wing_algorithm,uint64_t dpl_ts,float dpl_alt,uint64_t dpl_max_acc_ts,float dpl_max_acc,float ref_lat,float ref_lon,float ref_alt,float min_pressure,float cpu_load,uint32_t free_heap,uint8_t nas_state,uint8_t wnc_state,uint8_t pin_launch,uint8_t pin_nosecone,uint8_t cutter_presence,int16_t log_number,int32_t log_good,uint8_t main_board_state,uint8_t motor_board_state,uint8_t main_can_status,uint8_t motor_can_status,uint8_t rig_can_status,uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, liftoff_ts);
    _mav_put_uint64_t(buf, 16, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 24, max_speed_ts);
    _mav_put_uint64_t(buf, 32, max_mach_ts);
    _mav_put_uint64_t(buf, 40, apogee_ts);
    _mav_put_uint64_t(buf, 48, apogee_max_acc_ts);
    _mav_put_uint64_t(buf, 56, dpl_ts);
    _mav_put_uint64_t(buf, 64, dpl_max_acc_ts);
    _mav_put_float(buf, 72, liftoff_max_acc);
    _mav_put_float(buf, 76, max_speed);
    _mav_put_float(buf, 80, max_speed_altitude);
    _mav_put_float(buf, 84, max_mach);
    _mav_put_float(buf, 88, apogee_lat);
    _mav_put_float(buf, 92, apogee_lon);
    _mav_put_float(buf, 96, apogee_alt);
    _mav_put_float(buf, 100, apogee_max_acc);
    _mav_put_float(buf, 104, wing_target_lat);
    _mav_put_float(buf, 108, wing_target_lon);
    _mav_put_float(buf, 112, wing_active_target_n);
    _mav_put_float(buf, 116, wing_active_target_e);
    _mav_put_float(buf, 120, dpl_alt);
    _mav_put_float(buf, 124, dpl_max_acc);
    _mav_put_float(buf, 128, ref_lat);
    _mav_put_float(buf, 132, ref_lon);
    _mav_put_float(buf, 136, ref_alt);
    _mav_put_float(buf, 140, min_pressure);
    _mav_put_float(buf, 144, cpu_load);
    _mav_put_uint32_t(buf, 148, free_heap);
    _mav_put_int32_t(buf, 152, log_good);
    _mav_put_int16_t(buf, 156, log_number);
    _mav_put_uint8_t(buf, 158, wing_algorithm);
    _mav_put_uint8_t(buf, 159, nas_state);
    _mav_put_uint8_t(buf, 160, wnc_state);
    _mav_put_uint8_t(buf, 161, pin_launch);
    _mav_put_uint8_t(buf, 162, pin_nosecone);
    _mav_put_uint8_t(buf, 163, cutter_presence);
    _mav_put_uint8_t(buf, 164, main_board_state);
    _mav_put_uint8_t(buf, 165, motor_board_state);
    _mav_put_uint8_t(buf, 166, main_can_status);
    _mav_put_uint8_t(buf, 167, motor_can_status);
    _mav_put_uint8_t(buf, 168, rig_can_status);
    _mav_put_uint8_t(buf, 169, hil_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN);
#else
    mavlink_payload_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.liftoff_ts = liftoff_ts;
    packet.liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet.max_speed_ts = max_speed_ts;
    packet.max_mach_ts = max_mach_ts;
    packet.apogee_ts = apogee_ts;
    packet.apogee_max_acc_ts = apogee_max_acc_ts;
    packet.dpl_ts = dpl_ts;
    packet.dpl_max_acc_ts = dpl_max_acc_ts;
    packet.liftoff_max_acc = liftoff_max_acc;
    packet.max_speed = max_speed;
    packet.max_speed_altitude = max_speed_altitude;
    packet.max_mach = max_mach;
    packet.apogee_lat = apogee_lat;
    packet.apogee_lon = apogee_lon;
    packet.apogee_alt = apogee_alt;
    packet.apogee_max_acc = apogee_max_acc;
    packet.wing_target_lat = wing_target_lat;
    packet.wing_target_lon = wing_target_lon;
    packet.wing_active_target_n = wing_active_target_n;
    packet.wing_active_target_e = wing_active_target_e;
    packet.dpl_alt = dpl_alt;
    packet.dpl_max_acc = dpl_max_acc;
    packet.ref_lat = ref_lat;
    packet.ref_lon = ref_lon;
    packet.ref_alt = ref_alt;
    packet.min_pressure = min_pressure;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;
    packet.log_good = log_good;
    packet.log_number = log_number;
    packet.wing_algorithm = wing_algorithm;
    packet.nas_state = nas_state;
    packet.wnc_state = wnc_state;
    packet.pin_launch = pin_launch;
    packet.pin_nosecone = pin_nosecone;
    packet.cutter_presence = cutter_presence;
    packet.main_board_state = main_board_state;
    packet.motor_board_state = motor_board_state;
    packet.main_can_status = main_can_status;
    packet.motor_can_status = motor_can_status;
    packet.rig_can_status = rig_can_status;
    packet.hil_state = hil_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PAYLOAD_STATS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_CRC);
}

/**
 * @brief Encode a payload_stats_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param payload_stats_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_payload_stats_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_payload_stats_tm_t* payload_stats_tm)
{
    return mavlink_msg_payload_stats_tm_pack(system_id, component_id, msg, payload_stats_tm->timestamp, payload_stats_tm->liftoff_ts, payload_stats_tm->liftoff_max_acc_ts, payload_stats_tm->liftoff_max_acc, payload_stats_tm->max_speed_ts, payload_stats_tm->max_speed, payload_stats_tm->max_speed_altitude, payload_stats_tm->max_mach_ts, payload_stats_tm->max_mach, payload_stats_tm->apogee_ts, payload_stats_tm->apogee_lat, payload_stats_tm->apogee_lon, payload_stats_tm->apogee_alt, payload_stats_tm->apogee_max_acc_ts, payload_stats_tm->apogee_max_acc, payload_stats_tm->wing_target_lat, payload_stats_tm->wing_target_lon, payload_stats_tm->wing_active_target_n, payload_stats_tm->wing_active_target_e, payload_stats_tm->wing_algorithm, payload_stats_tm->dpl_ts, payload_stats_tm->dpl_alt, payload_stats_tm->dpl_max_acc_ts, payload_stats_tm->dpl_max_acc, payload_stats_tm->ref_lat, payload_stats_tm->ref_lon, payload_stats_tm->ref_alt, payload_stats_tm->min_pressure, payload_stats_tm->cpu_load, payload_stats_tm->free_heap, payload_stats_tm->nas_state, payload_stats_tm->wnc_state, payload_stats_tm->pin_launch, payload_stats_tm->pin_nosecone, payload_stats_tm->cutter_presence, payload_stats_tm->log_number, payload_stats_tm->log_good, payload_stats_tm->main_board_state, payload_stats_tm->motor_board_state, payload_stats_tm->main_can_status, payload_stats_tm->motor_can_status, payload_stats_tm->rig_can_status, payload_stats_tm->hil_state);
}

/**
 * @brief Encode a payload_stats_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param payload_stats_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_payload_stats_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_payload_stats_tm_t* payload_stats_tm)
{
    return mavlink_msg_payload_stats_tm_pack_chan(system_id, component_id, chan, msg, payload_stats_tm->timestamp, payload_stats_tm->liftoff_ts, payload_stats_tm->liftoff_max_acc_ts, payload_stats_tm->liftoff_max_acc, payload_stats_tm->max_speed_ts, payload_stats_tm->max_speed, payload_stats_tm->max_speed_altitude, payload_stats_tm->max_mach_ts, payload_stats_tm->max_mach, payload_stats_tm->apogee_ts, payload_stats_tm->apogee_lat, payload_stats_tm->apogee_lon, payload_stats_tm->apogee_alt, payload_stats_tm->apogee_max_acc_ts, payload_stats_tm->apogee_max_acc, payload_stats_tm->wing_target_lat, payload_stats_tm->wing_target_lon, payload_stats_tm->wing_active_target_n, payload_stats_tm->wing_active_target_e, payload_stats_tm->wing_algorithm, payload_stats_tm->dpl_ts, payload_stats_tm->dpl_alt, payload_stats_tm->dpl_max_acc_ts, payload_stats_tm->dpl_max_acc, payload_stats_tm->ref_lat, payload_stats_tm->ref_lon, payload_stats_tm->ref_alt, payload_stats_tm->min_pressure, payload_stats_tm->cpu_load, payload_stats_tm->free_heap, payload_stats_tm->nas_state, payload_stats_tm->wnc_state, payload_stats_tm->pin_launch, payload_stats_tm->pin_nosecone, payload_stats_tm->cutter_presence, payload_stats_tm->log_number, payload_stats_tm->log_good, payload_stats_tm->main_board_state, payload_stats_tm->motor_board_state, payload_stats_tm->main_can_status, payload_stats_tm->motor_can_status, payload_stats_tm->rig_can_status, payload_stats_tm->hil_state);
}

/**
 * @brief Send a payload_stats_tm message
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
 * @param apogee_ts [us] System clock at apogee
 * @param apogee_lat [deg] Apogee latitude
 * @param apogee_lon [deg] Apogee longitude
 * @param apogee_alt [m] Apogee altitude
 * @param apogee_max_acc_ts [us] System clock at max acceleration after apogee
 * @param apogee_max_acc [m/s2] Max acceleration after apogee
 * @param wing_target_lat [deg] Wing target latitude
 * @param wing_target_lon [deg] Wing target longitude
 * @param wing_active_target_n [m] Wing active target N
 * @param wing_active_target_e [m] Wing active target E
 * @param wing_algorithm  Wing selected algorithm
 * @param dpl_ts [us] System clock at main deployment
 * @param dpl_alt [m] Deployment altitude
 * @param dpl_max_acc_ts [us] System clock at max acceleration during main deployment
 * @param dpl_max_acc [m/s2] Max acceleration during main deployment
 * @param ref_lat [deg] Reference altitude
 * @param ref_lon [deg] Reference longitude
 * @param ref_alt [m] Reference altitude
 * @param min_pressure [Pa] Apogee pressure - Digital barometer
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @param nas_state  NAS FSM State
 * @param wnc_state  Wing Controller State
 * @param pin_launch  Launch pin status (1 = connected, 0 = disconnected)
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param cutter_presence  Cutter presence status (1 = present, 0 = missing)
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @param main_board_state  Main board fmm state
 * @param motor_board_state  Motor board fmm state
 * @param main_can_status  Main CAN status
 * @param motor_can_status  Motor CAN status
 * @param rig_can_status  RIG CAN status
 * @param hil_state  1 if the board is currently in HIL mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_payload_stats_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint64_t liftoff_ts, uint64_t liftoff_max_acc_ts, float liftoff_max_acc, uint64_t max_speed_ts, float max_speed, float max_speed_altitude, uint64_t max_mach_ts, float max_mach, uint64_t apogee_ts, float apogee_lat, float apogee_lon, float apogee_alt, uint64_t apogee_max_acc_ts, float apogee_max_acc, float wing_target_lat, float wing_target_lon, float wing_active_target_n, float wing_active_target_e, uint8_t wing_algorithm, uint64_t dpl_ts, float dpl_alt, uint64_t dpl_max_acc_ts, float dpl_max_acc, float ref_lat, float ref_lon, float ref_alt, float min_pressure, float cpu_load, uint32_t free_heap, uint8_t nas_state, uint8_t wnc_state, uint8_t pin_launch, uint8_t pin_nosecone, uint8_t cutter_presence, int16_t log_number, int32_t log_good, uint8_t main_board_state, uint8_t motor_board_state, uint8_t main_can_status, uint8_t motor_can_status, uint8_t rig_can_status, uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, liftoff_ts);
    _mav_put_uint64_t(buf, 16, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 24, max_speed_ts);
    _mav_put_uint64_t(buf, 32, max_mach_ts);
    _mav_put_uint64_t(buf, 40, apogee_ts);
    _mav_put_uint64_t(buf, 48, apogee_max_acc_ts);
    _mav_put_uint64_t(buf, 56, dpl_ts);
    _mav_put_uint64_t(buf, 64, dpl_max_acc_ts);
    _mav_put_float(buf, 72, liftoff_max_acc);
    _mav_put_float(buf, 76, max_speed);
    _mav_put_float(buf, 80, max_speed_altitude);
    _mav_put_float(buf, 84, max_mach);
    _mav_put_float(buf, 88, apogee_lat);
    _mav_put_float(buf, 92, apogee_lon);
    _mav_put_float(buf, 96, apogee_alt);
    _mav_put_float(buf, 100, apogee_max_acc);
    _mav_put_float(buf, 104, wing_target_lat);
    _mav_put_float(buf, 108, wing_target_lon);
    _mav_put_float(buf, 112, wing_active_target_n);
    _mav_put_float(buf, 116, wing_active_target_e);
    _mav_put_float(buf, 120, dpl_alt);
    _mav_put_float(buf, 124, dpl_max_acc);
    _mav_put_float(buf, 128, ref_lat);
    _mav_put_float(buf, 132, ref_lon);
    _mav_put_float(buf, 136, ref_alt);
    _mav_put_float(buf, 140, min_pressure);
    _mav_put_float(buf, 144, cpu_load);
    _mav_put_uint32_t(buf, 148, free_heap);
    _mav_put_int32_t(buf, 152, log_good);
    _mav_put_int16_t(buf, 156, log_number);
    _mav_put_uint8_t(buf, 158, wing_algorithm);
    _mav_put_uint8_t(buf, 159, nas_state);
    _mav_put_uint8_t(buf, 160, wnc_state);
    _mav_put_uint8_t(buf, 161, pin_launch);
    _mav_put_uint8_t(buf, 162, pin_nosecone);
    _mav_put_uint8_t(buf, 163, cutter_presence);
    _mav_put_uint8_t(buf, 164, main_board_state);
    _mav_put_uint8_t(buf, 165, motor_board_state);
    _mav_put_uint8_t(buf, 166, main_can_status);
    _mav_put_uint8_t(buf, 167, motor_can_status);
    _mav_put_uint8_t(buf, 168, rig_can_status);
    _mav_put_uint8_t(buf, 169, hil_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_STATS_TM, buf, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_CRC);
#else
    mavlink_payload_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.liftoff_ts = liftoff_ts;
    packet.liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet.max_speed_ts = max_speed_ts;
    packet.max_mach_ts = max_mach_ts;
    packet.apogee_ts = apogee_ts;
    packet.apogee_max_acc_ts = apogee_max_acc_ts;
    packet.dpl_ts = dpl_ts;
    packet.dpl_max_acc_ts = dpl_max_acc_ts;
    packet.liftoff_max_acc = liftoff_max_acc;
    packet.max_speed = max_speed;
    packet.max_speed_altitude = max_speed_altitude;
    packet.max_mach = max_mach;
    packet.apogee_lat = apogee_lat;
    packet.apogee_lon = apogee_lon;
    packet.apogee_alt = apogee_alt;
    packet.apogee_max_acc = apogee_max_acc;
    packet.wing_target_lat = wing_target_lat;
    packet.wing_target_lon = wing_target_lon;
    packet.wing_active_target_n = wing_active_target_n;
    packet.wing_active_target_e = wing_active_target_e;
    packet.dpl_alt = dpl_alt;
    packet.dpl_max_acc = dpl_max_acc;
    packet.ref_lat = ref_lat;
    packet.ref_lon = ref_lon;
    packet.ref_alt = ref_alt;
    packet.min_pressure = min_pressure;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;
    packet.log_good = log_good;
    packet.log_number = log_number;
    packet.wing_algorithm = wing_algorithm;
    packet.nas_state = nas_state;
    packet.wnc_state = wnc_state;
    packet.pin_launch = pin_launch;
    packet.pin_nosecone = pin_nosecone;
    packet.cutter_presence = cutter_presence;
    packet.main_board_state = main_board_state;
    packet.motor_board_state = motor_board_state;
    packet.main_can_status = main_can_status;
    packet.motor_can_status = motor_can_status;
    packet.rig_can_status = rig_can_status;
    packet.hil_state = hil_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_STATS_TM, (const char *)&packet, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_CRC);
#endif
}

/**
 * @brief Send a payload_stats_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_payload_stats_tm_send_struct(mavlink_channel_t chan, const mavlink_payload_stats_tm_t* payload_stats_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_payload_stats_tm_send(chan, payload_stats_tm->timestamp, payload_stats_tm->liftoff_ts, payload_stats_tm->liftoff_max_acc_ts, payload_stats_tm->liftoff_max_acc, payload_stats_tm->max_speed_ts, payload_stats_tm->max_speed, payload_stats_tm->max_speed_altitude, payload_stats_tm->max_mach_ts, payload_stats_tm->max_mach, payload_stats_tm->apogee_ts, payload_stats_tm->apogee_lat, payload_stats_tm->apogee_lon, payload_stats_tm->apogee_alt, payload_stats_tm->apogee_max_acc_ts, payload_stats_tm->apogee_max_acc, payload_stats_tm->wing_target_lat, payload_stats_tm->wing_target_lon, payload_stats_tm->wing_active_target_n, payload_stats_tm->wing_active_target_e, payload_stats_tm->wing_algorithm, payload_stats_tm->dpl_ts, payload_stats_tm->dpl_alt, payload_stats_tm->dpl_max_acc_ts, payload_stats_tm->dpl_max_acc, payload_stats_tm->ref_lat, payload_stats_tm->ref_lon, payload_stats_tm->ref_alt, payload_stats_tm->min_pressure, payload_stats_tm->cpu_load, payload_stats_tm->free_heap, payload_stats_tm->nas_state, payload_stats_tm->wnc_state, payload_stats_tm->pin_launch, payload_stats_tm->pin_nosecone, payload_stats_tm->cutter_presence, payload_stats_tm->log_number, payload_stats_tm->log_good, payload_stats_tm->main_board_state, payload_stats_tm->motor_board_state, payload_stats_tm->main_can_status, payload_stats_tm->motor_can_status, payload_stats_tm->rig_can_status, payload_stats_tm->hil_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_STATS_TM, (const char *)payload_stats_tm, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_payload_stats_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint64_t liftoff_ts, uint64_t liftoff_max_acc_ts, float liftoff_max_acc, uint64_t max_speed_ts, float max_speed, float max_speed_altitude, uint64_t max_mach_ts, float max_mach, uint64_t apogee_ts, float apogee_lat, float apogee_lon, float apogee_alt, uint64_t apogee_max_acc_ts, float apogee_max_acc, float wing_target_lat, float wing_target_lon, float wing_active_target_n, float wing_active_target_e, uint8_t wing_algorithm, uint64_t dpl_ts, float dpl_alt, uint64_t dpl_max_acc_ts, float dpl_max_acc, float ref_lat, float ref_lon, float ref_alt, float min_pressure, float cpu_load, uint32_t free_heap, uint8_t nas_state, uint8_t wnc_state, uint8_t pin_launch, uint8_t pin_nosecone, uint8_t cutter_presence, int16_t log_number, int32_t log_good, uint8_t main_board_state, uint8_t motor_board_state, uint8_t main_can_status, uint8_t motor_can_status, uint8_t rig_can_status, uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, liftoff_ts);
    _mav_put_uint64_t(buf, 16, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 24, max_speed_ts);
    _mav_put_uint64_t(buf, 32, max_mach_ts);
    _mav_put_uint64_t(buf, 40, apogee_ts);
    _mav_put_uint64_t(buf, 48, apogee_max_acc_ts);
    _mav_put_uint64_t(buf, 56, dpl_ts);
    _mav_put_uint64_t(buf, 64, dpl_max_acc_ts);
    _mav_put_float(buf, 72, liftoff_max_acc);
    _mav_put_float(buf, 76, max_speed);
    _mav_put_float(buf, 80, max_speed_altitude);
    _mav_put_float(buf, 84, max_mach);
    _mav_put_float(buf, 88, apogee_lat);
    _mav_put_float(buf, 92, apogee_lon);
    _mav_put_float(buf, 96, apogee_alt);
    _mav_put_float(buf, 100, apogee_max_acc);
    _mav_put_float(buf, 104, wing_target_lat);
    _mav_put_float(buf, 108, wing_target_lon);
    _mav_put_float(buf, 112, wing_active_target_n);
    _mav_put_float(buf, 116, wing_active_target_e);
    _mav_put_float(buf, 120, dpl_alt);
    _mav_put_float(buf, 124, dpl_max_acc);
    _mav_put_float(buf, 128, ref_lat);
    _mav_put_float(buf, 132, ref_lon);
    _mav_put_float(buf, 136, ref_alt);
    _mav_put_float(buf, 140, min_pressure);
    _mav_put_float(buf, 144, cpu_load);
    _mav_put_uint32_t(buf, 148, free_heap);
    _mav_put_int32_t(buf, 152, log_good);
    _mav_put_int16_t(buf, 156, log_number);
    _mav_put_uint8_t(buf, 158, wing_algorithm);
    _mav_put_uint8_t(buf, 159, nas_state);
    _mav_put_uint8_t(buf, 160, wnc_state);
    _mav_put_uint8_t(buf, 161, pin_launch);
    _mav_put_uint8_t(buf, 162, pin_nosecone);
    _mav_put_uint8_t(buf, 163, cutter_presence);
    _mav_put_uint8_t(buf, 164, main_board_state);
    _mav_put_uint8_t(buf, 165, motor_board_state);
    _mav_put_uint8_t(buf, 166, main_can_status);
    _mav_put_uint8_t(buf, 167, motor_can_status);
    _mav_put_uint8_t(buf, 168, rig_can_status);
    _mav_put_uint8_t(buf, 169, hil_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_STATS_TM, buf, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_CRC);
#else
    mavlink_payload_stats_tm_t *packet = (mavlink_payload_stats_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->liftoff_ts = liftoff_ts;
    packet->liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet->max_speed_ts = max_speed_ts;
    packet->max_mach_ts = max_mach_ts;
    packet->apogee_ts = apogee_ts;
    packet->apogee_max_acc_ts = apogee_max_acc_ts;
    packet->dpl_ts = dpl_ts;
    packet->dpl_max_acc_ts = dpl_max_acc_ts;
    packet->liftoff_max_acc = liftoff_max_acc;
    packet->max_speed = max_speed;
    packet->max_speed_altitude = max_speed_altitude;
    packet->max_mach = max_mach;
    packet->apogee_lat = apogee_lat;
    packet->apogee_lon = apogee_lon;
    packet->apogee_alt = apogee_alt;
    packet->apogee_max_acc = apogee_max_acc;
    packet->wing_target_lat = wing_target_lat;
    packet->wing_target_lon = wing_target_lon;
    packet->wing_active_target_n = wing_active_target_n;
    packet->wing_active_target_e = wing_active_target_e;
    packet->dpl_alt = dpl_alt;
    packet->dpl_max_acc = dpl_max_acc;
    packet->ref_lat = ref_lat;
    packet->ref_lon = ref_lon;
    packet->ref_alt = ref_alt;
    packet->min_pressure = min_pressure;
    packet->cpu_load = cpu_load;
    packet->free_heap = free_heap;
    packet->log_good = log_good;
    packet->log_number = log_number;
    packet->wing_algorithm = wing_algorithm;
    packet->nas_state = nas_state;
    packet->wnc_state = wnc_state;
    packet->pin_launch = pin_launch;
    packet->pin_nosecone = pin_nosecone;
    packet->cutter_presence = cutter_presence;
    packet->main_board_state = main_board_state;
    packet->motor_board_state = motor_board_state;
    packet->main_can_status = main_can_status;
    packet->motor_can_status = motor_can_status;
    packet->rig_can_status = rig_can_status;
    packet->hil_state = hil_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_STATS_TM, (const char *)packet, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE PAYLOAD_STATS_TM UNPACKING


/**
 * @brief Get field timestamp from payload_stats_tm message
 *
 * @return [us] Timestamp in microseconds
 */
static inline uint64_t mavlink_msg_payload_stats_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field liftoff_ts from payload_stats_tm message
 *
 * @return [us] System clock at liftoff
 */
static inline uint64_t mavlink_msg_payload_stats_tm_get_liftoff_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field liftoff_max_acc_ts from payload_stats_tm message
 *
 * @return [us] System clock at the maximum liftoff acceleration
 */
static inline uint64_t mavlink_msg_payload_stats_tm_get_liftoff_max_acc_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Get field liftoff_max_acc from payload_stats_tm message
 *
 * @return [m/s2] Maximum liftoff acceleration
 */
static inline float mavlink_msg_payload_stats_tm_get_liftoff_max_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field max_speed_ts from payload_stats_tm message
 *
 * @return [us] System clock at max speed
 */
static inline uint64_t mavlink_msg_payload_stats_tm_get_max_speed_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  24);
}

/**
 * @brief Get field max_speed from payload_stats_tm message
 *
 * @return [m/s] Max measured speed
 */
static inline float mavlink_msg_payload_stats_tm_get_max_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field max_speed_altitude from payload_stats_tm message
 *
 * @return [m] Altitude at max speed
 */
static inline float mavlink_msg_payload_stats_tm_get_max_speed_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field max_mach_ts from payload_stats_tm message
 *
 * @return  System clock at maximum measured mach
 */
static inline uint64_t mavlink_msg_payload_stats_tm_get_max_mach_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  32);
}

/**
 * @brief Get field max_mach from payload_stats_tm message
 *
 * @return  Maximum measured mach
 */
static inline float mavlink_msg_payload_stats_tm_get_max_mach(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field apogee_ts from payload_stats_tm message
 *
 * @return [us] System clock at apogee
 */
static inline uint64_t mavlink_msg_payload_stats_tm_get_apogee_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  40);
}

/**
 * @brief Get field apogee_lat from payload_stats_tm message
 *
 * @return [deg] Apogee latitude
 */
static inline float mavlink_msg_payload_stats_tm_get_apogee_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field apogee_lon from payload_stats_tm message
 *
 * @return [deg] Apogee longitude
 */
static inline float mavlink_msg_payload_stats_tm_get_apogee_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field apogee_alt from payload_stats_tm message
 *
 * @return [m] Apogee altitude
 */
static inline float mavlink_msg_payload_stats_tm_get_apogee_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field apogee_max_acc_ts from payload_stats_tm message
 *
 * @return [us] System clock at max acceleration after apogee
 */
static inline uint64_t mavlink_msg_payload_stats_tm_get_apogee_max_acc_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  48);
}

/**
 * @brief Get field apogee_max_acc from payload_stats_tm message
 *
 * @return [m/s2] Max acceleration after apogee
 */
static inline float mavlink_msg_payload_stats_tm_get_apogee_max_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Get field wing_target_lat from payload_stats_tm message
 *
 * @return [deg] Wing target latitude
 */
static inline float mavlink_msg_payload_stats_tm_get_wing_target_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  104);
}

/**
 * @brief Get field wing_target_lon from payload_stats_tm message
 *
 * @return [deg] Wing target longitude
 */
static inline float mavlink_msg_payload_stats_tm_get_wing_target_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Get field wing_active_target_n from payload_stats_tm message
 *
 * @return [m] Wing active target N
 */
static inline float mavlink_msg_payload_stats_tm_get_wing_active_target_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Get field wing_active_target_e from payload_stats_tm message
 *
 * @return [m] Wing active target E
 */
static inline float mavlink_msg_payload_stats_tm_get_wing_active_target_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  116);
}

/**
 * @brief Get field wing_algorithm from payload_stats_tm message
 *
 * @return  Wing selected algorithm
 */
static inline uint8_t mavlink_msg_payload_stats_tm_get_wing_algorithm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  158);
}

/**
 * @brief Get field dpl_ts from payload_stats_tm message
 *
 * @return [us] System clock at main deployment
 */
static inline uint64_t mavlink_msg_payload_stats_tm_get_dpl_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  56);
}

/**
 * @brief Get field dpl_alt from payload_stats_tm message
 *
 * @return [m] Deployment altitude
 */
static inline float mavlink_msg_payload_stats_tm_get_dpl_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  120);
}

/**
 * @brief Get field dpl_max_acc_ts from payload_stats_tm message
 *
 * @return [us] System clock at max acceleration during main deployment
 */
static inline uint64_t mavlink_msg_payload_stats_tm_get_dpl_max_acc_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  64);
}

/**
 * @brief Get field dpl_max_acc from payload_stats_tm message
 *
 * @return [m/s2] Max acceleration during main deployment
 */
static inline float mavlink_msg_payload_stats_tm_get_dpl_max_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  124);
}

/**
 * @brief Get field ref_lat from payload_stats_tm message
 *
 * @return [deg] Reference altitude
 */
static inline float mavlink_msg_payload_stats_tm_get_ref_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  128);
}

/**
 * @brief Get field ref_lon from payload_stats_tm message
 *
 * @return [deg] Reference longitude
 */
static inline float mavlink_msg_payload_stats_tm_get_ref_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  132);
}

/**
 * @brief Get field ref_alt from payload_stats_tm message
 *
 * @return [m] Reference altitude
 */
static inline float mavlink_msg_payload_stats_tm_get_ref_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  136);
}

/**
 * @brief Get field min_pressure from payload_stats_tm message
 *
 * @return [Pa] Apogee pressure - Digital barometer
 */
static inline float mavlink_msg_payload_stats_tm_get_min_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  140);
}

/**
 * @brief Get field cpu_load from payload_stats_tm message
 *
 * @return  CPU load in percentage
 */
static inline float mavlink_msg_payload_stats_tm_get_cpu_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  144);
}

/**
 * @brief Get field free_heap from payload_stats_tm message
 *
 * @return  Amount of available heap in memory
 */
static inline uint32_t mavlink_msg_payload_stats_tm_get_free_heap(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  148);
}

/**
 * @brief Get field nas_state from payload_stats_tm message
 *
 * @return  NAS FSM State
 */
static inline uint8_t mavlink_msg_payload_stats_tm_get_nas_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  159);
}

/**
 * @brief Get field wnc_state from payload_stats_tm message
 *
 * @return  Wing Controller State
 */
static inline uint8_t mavlink_msg_payload_stats_tm_get_wnc_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  160);
}

/**
 * @brief Get field pin_launch from payload_stats_tm message
 *
 * @return  Launch pin status (1 = connected, 0 = disconnected)
 */
static inline uint8_t mavlink_msg_payload_stats_tm_get_pin_launch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  161);
}

/**
 * @brief Get field pin_nosecone from payload_stats_tm message
 *
 * @return  Nosecone pin status (1 = connected, 0 = disconnected)
 */
static inline uint8_t mavlink_msg_payload_stats_tm_get_pin_nosecone(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  162);
}

/**
 * @brief Get field cutter_presence from payload_stats_tm message
 *
 * @return  Cutter presence status (1 = present, 0 = missing)
 */
static inline uint8_t mavlink_msg_payload_stats_tm_get_cutter_presence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  163);
}

/**
 * @brief Get field log_number from payload_stats_tm message
 *
 * @return  Currently active log file, -1 if the logger is inactive
 */
static inline int16_t mavlink_msg_payload_stats_tm_get_log_number(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  156);
}

/**
 * @brief Get field log_good from payload_stats_tm message
 *
 * @return  0 if log failed, 1 otherwise
 */
static inline int32_t mavlink_msg_payload_stats_tm_get_log_good(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  152);
}

/**
 * @brief Get field main_board_state from payload_stats_tm message
 *
 * @return  Main board fmm state
 */
static inline uint8_t mavlink_msg_payload_stats_tm_get_main_board_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  164);
}

/**
 * @brief Get field motor_board_state from payload_stats_tm message
 *
 * @return  Motor board fmm state
 */
static inline uint8_t mavlink_msg_payload_stats_tm_get_motor_board_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  165);
}

/**
 * @brief Get field main_can_status from payload_stats_tm message
 *
 * @return  Main CAN status
 */
static inline uint8_t mavlink_msg_payload_stats_tm_get_main_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  166);
}

/**
 * @brief Get field motor_can_status from payload_stats_tm message
 *
 * @return  Motor CAN status
 */
static inline uint8_t mavlink_msg_payload_stats_tm_get_motor_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  167);
}

/**
 * @brief Get field rig_can_status from payload_stats_tm message
 *
 * @return  RIG CAN status
 */
static inline uint8_t mavlink_msg_payload_stats_tm_get_rig_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  168);
}

/**
 * @brief Get field hil_state from payload_stats_tm message
 *
 * @return  1 if the board is currently in HIL mode
 */
static inline uint8_t mavlink_msg_payload_stats_tm_get_hil_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  169);
}

/**
 * @brief Decode a payload_stats_tm message into a struct
 *
 * @param msg The message to decode
 * @param payload_stats_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_payload_stats_tm_decode(const mavlink_message_t* msg, mavlink_payload_stats_tm_t* payload_stats_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    payload_stats_tm->timestamp = mavlink_msg_payload_stats_tm_get_timestamp(msg);
    payload_stats_tm->liftoff_ts = mavlink_msg_payload_stats_tm_get_liftoff_ts(msg);
    payload_stats_tm->liftoff_max_acc_ts = mavlink_msg_payload_stats_tm_get_liftoff_max_acc_ts(msg);
    payload_stats_tm->max_speed_ts = mavlink_msg_payload_stats_tm_get_max_speed_ts(msg);
    payload_stats_tm->max_mach_ts = mavlink_msg_payload_stats_tm_get_max_mach_ts(msg);
    payload_stats_tm->apogee_ts = mavlink_msg_payload_stats_tm_get_apogee_ts(msg);
    payload_stats_tm->apogee_max_acc_ts = mavlink_msg_payload_stats_tm_get_apogee_max_acc_ts(msg);
    payload_stats_tm->dpl_ts = mavlink_msg_payload_stats_tm_get_dpl_ts(msg);
    payload_stats_tm->dpl_max_acc_ts = mavlink_msg_payload_stats_tm_get_dpl_max_acc_ts(msg);
    payload_stats_tm->liftoff_max_acc = mavlink_msg_payload_stats_tm_get_liftoff_max_acc(msg);
    payload_stats_tm->max_speed = mavlink_msg_payload_stats_tm_get_max_speed(msg);
    payload_stats_tm->max_speed_altitude = mavlink_msg_payload_stats_tm_get_max_speed_altitude(msg);
    payload_stats_tm->max_mach = mavlink_msg_payload_stats_tm_get_max_mach(msg);
    payload_stats_tm->apogee_lat = mavlink_msg_payload_stats_tm_get_apogee_lat(msg);
    payload_stats_tm->apogee_lon = mavlink_msg_payload_stats_tm_get_apogee_lon(msg);
    payload_stats_tm->apogee_alt = mavlink_msg_payload_stats_tm_get_apogee_alt(msg);
    payload_stats_tm->apogee_max_acc = mavlink_msg_payload_stats_tm_get_apogee_max_acc(msg);
    payload_stats_tm->wing_target_lat = mavlink_msg_payload_stats_tm_get_wing_target_lat(msg);
    payload_stats_tm->wing_target_lon = mavlink_msg_payload_stats_tm_get_wing_target_lon(msg);
    payload_stats_tm->wing_active_target_n = mavlink_msg_payload_stats_tm_get_wing_active_target_n(msg);
    payload_stats_tm->wing_active_target_e = mavlink_msg_payload_stats_tm_get_wing_active_target_e(msg);
    payload_stats_tm->dpl_alt = mavlink_msg_payload_stats_tm_get_dpl_alt(msg);
    payload_stats_tm->dpl_max_acc = mavlink_msg_payload_stats_tm_get_dpl_max_acc(msg);
    payload_stats_tm->ref_lat = mavlink_msg_payload_stats_tm_get_ref_lat(msg);
    payload_stats_tm->ref_lon = mavlink_msg_payload_stats_tm_get_ref_lon(msg);
    payload_stats_tm->ref_alt = mavlink_msg_payload_stats_tm_get_ref_alt(msg);
    payload_stats_tm->min_pressure = mavlink_msg_payload_stats_tm_get_min_pressure(msg);
    payload_stats_tm->cpu_load = mavlink_msg_payload_stats_tm_get_cpu_load(msg);
    payload_stats_tm->free_heap = mavlink_msg_payload_stats_tm_get_free_heap(msg);
    payload_stats_tm->log_good = mavlink_msg_payload_stats_tm_get_log_good(msg);
    payload_stats_tm->log_number = mavlink_msg_payload_stats_tm_get_log_number(msg);
    payload_stats_tm->wing_algorithm = mavlink_msg_payload_stats_tm_get_wing_algorithm(msg);
    payload_stats_tm->nas_state = mavlink_msg_payload_stats_tm_get_nas_state(msg);
    payload_stats_tm->wnc_state = mavlink_msg_payload_stats_tm_get_wnc_state(msg);
    payload_stats_tm->pin_launch = mavlink_msg_payload_stats_tm_get_pin_launch(msg);
    payload_stats_tm->pin_nosecone = mavlink_msg_payload_stats_tm_get_pin_nosecone(msg);
    payload_stats_tm->cutter_presence = mavlink_msg_payload_stats_tm_get_cutter_presence(msg);
    payload_stats_tm->main_board_state = mavlink_msg_payload_stats_tm_get_main_board_state(msg);
    payload_stats_tm->motor_board_state = mavlink_msg_payload_stats_tm_get_motor_board_state(msg);
    payload_stats_tm->main_can_status = mavlink_msg_payload_stats_tm_get_main_can_status(msg);
    payload_stats_tm->motor_can_status = mavlink_msg_payload_stats_tm_get_motor_can_status(msg);
    payload_stats_tm->rig_can_status = mavlink_msg_payload_stats_tm_get_rig_can_status(msg);
    payload_stats_tm->hil_state = mavlink_msg_payload_stats_tm_get_hil_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN? msg->len : MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN;
        memset(payload_stats_tm, 0, MAVLINK_MSG_ID_PAYLOAD_STATS_TM_LEN);
    memcpy(payload_stats_tm, _MAV_PAYLOAD(msg), len);
#endif
}
