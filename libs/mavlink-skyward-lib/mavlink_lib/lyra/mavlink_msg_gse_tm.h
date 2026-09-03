#pragma once
// MESSAGE GSE_TM PACKING

#define MAVLINK_MSG_ID_GSE_TM 212


typedef struct __mavlink_gse_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 float loadcell_rocket; /*< [kg] Rocket weight*/
 float loadcell_vessel; /*< [kg] External tank weight*/
 float filling_pressure; /*< [Bar] Refueling line pressure*/
 float vessel_pressure; /*< [Bar] Vessel tank pressure*/
 float cpu_load; /*<  CPU load in percentage*/
 uint32_t free_heap; /*<  Amount of available heap in memory*/
 float battery_voltage; /*<  Battery voltage*/
 float current_consumption; /*< [A] RIG current*/
 float umbilical_current_consumption; /*< [A] Umbilical current*/
 int32_t log_good; /*<  0 if log failed, 1 otherwise*/
 int16_t log_number; /*<  Currently active log file, -1 if the logger is inactive*/
 uint8_t filling_valve_state; /*<  1 If the filling valve is open*/
 uint8_t venting_valve_state; /*<  1 If the venting valve is open*/
 uint8_t release_valve_state; /*<  1 If the release valve is open*/
 uint8_t main_valve_state; /*<  1 If the main valve is open*/
 uint8_t nitrogen_valve_state; /*<   1 If the nitrogen valve is open*/
 uint8_t gmm_state; /*<  State of the GroundModeManager*/
 uint8_t tars_state; /*<  State of Tars*/
 uint8_t arming_state; /*<  Arming state (1 if armed, 0 otherwise)*/
 uint8_t main_board_state; /*<  Main board fmm state*/
 uint8_t payload_board_state; /*<  Payload board fmm state*/
 uint8_t motor_board_state; /*<  Motor board fmm state*/
 uint8_t main_can_status; /*<  Main CAN status*/
 uint8_t payload_can_status; /*<  Payload CAN status*/
 uint8_t motor_can_status; /*<  Motor CAN status*/
} mavlink_gse_tm_t;

#define MAVLINK_MSG_ID_GSE_TM_LEN 64
#define MAVLINK_MSG_ID_GSE_TM_MIN_LEN 64
#define MAVLINK_MSG_ID_212_LEN 64
#define MAVLINK_MSG_ID_212_MIN_LEN 64

#define MAVLINK_MSG_ID_GSE_TM_CRC 87
#define MAVLINK_MSG_ID_212_CRC 87



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GSE_TM { \
    212, \
    "GSE_TM", \
    26, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gse_tm_t, timestamp) }, \
         { "loadcell_rocket", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gse_tm_t, loadcell_rocket) }, \
         { "loadcell_vessel", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gse_tm_t, loadcell_vessel) }, \
         { "filling_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gse_tm_t, filling_pressure) }, \
         { "vessel_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gse_tm_t, vessel_pressure) }, \
         { "filling_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_gse_tm_t, filling_valve_state) }, \
         { "venting_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_gse_tm_t, venting_valve_state) }, \
         { "release_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_gse_tm_t, release_valve_state) }, \
         { "main_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_gse_tm_t, main_valve_state) }, \
         { "nitrogen_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_gse_tm_t, nitrogen_valve_state) }, \
         { "gmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_gse_tm_t, gmm_state) }, \
         { "tars_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_gse_tm_t, tars_state) }, \
         { "arming_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 57, offsetof(mavlink_gse_tm_t, arming_state) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gse_tm_t, cpu_load) }, \
         { "free_heap", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_gse_tm_t, free_heap) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gse_tm_t, battery_voltage) }, \
         { "current_consumption", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gse_tm_t, current_consumption) }, \
         { "umbilical_current_consumption", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gse_tm_t, umbilical_current_consumption) }, \
         { "main_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 58, offsetof(mavlink_gse_tm_t, main_board_state) }, \
         { "payload_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 59, offsetof(mavlink_gse_tm_t, payload_board_state) }, \
         { "motor_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_gse_tm_t, motor_board_state) }, \
         { "main_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_gse_tm_t, main_can_status) }, \
         { "payload_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_gse_tm_t, payload_can_status) }, \
         { "motor_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 63, offsetof(mavlink_gse_tm_t, motor_can_status) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_gse_tm_t, log_number) }, \
         { "log_good", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_gse_tm_t, log_good) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GSE_TM { \
    "GSE_TM", \
    26, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gse_tm_t, timestamp) }, \
         { "loadcell_rocket", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gse_tm_t, loadcell_rocket) }, \
         { "loadcell_vessel", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gse_tm_t, loadcell_vessel) }, \
         { "filling_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gse_tm_t, filling_pressure) }, \
         { "vessel_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gse_tm_t, vessel_pressure) }, \
         { "filling_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_gse_tm_t, filling_valve_state) }, \
         { "venting_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_gse_tm_t, venting_valve_state) }, \
         { "release_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_gse_tm_t, release_valve_state) }, \
         { "main_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_gse_tm_t, main_valve_state) }, \
         { "nitrogen_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_gse_tm_t, nitrogen_valve_state) }, \
         { "gmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_gse_tm_t, gmm_state) }, \
         { "tars_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_gse_tm_t, tars_state) }, \
         { "arming_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 57, offsetof(mavlink_gse_tm_t, arming_state) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gse_tm_t, cpu_load) }, \
         { "free_heap", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_gse_tm_t, free_heap) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gse_tm_t, battery_voltage) }, \
         { "current_consumption", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gse_tm_t, current_consumption) }, \
         { "umbilical_current_consumption", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gse_tm_t, umbilical_current_consumption) }, \
         { "main_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 58, offsetof(mavlink_gse_tm_t, main_board_state) }, \
         { "payload_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 59, offsetof(mavlink_gse_tm_t, payload_board_state) }, \
         { "motor_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_gse_tm_t, motor_board_state) }, \
         { "main_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_gse_tm_t, main_can_status) }, \
         { "payload_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_gse_tm_t, payload_can_status) }, \
         { "motor_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 63, offsetof(mavlink_gse_tm_t, motor_can_status) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_gse_tm_t, log_number) }, \
         { "log_good", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_gse_tm_t, log_good) }, \
         } \
}
#endif

/**
 * @brief Pack a gse_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param loadcell_rocket [kg] Rocket weight
 * @param loadcell_vessel [kg] External tank weight
 * @param filling_pressure [Bar] Refueling line pressure
 * @param vessel_pressure [Bar] Vessel tank pressure
 * @param filling_valve_state  1 If the filling valve is open
 * @param venting_valve_state  1 If the venting valve is open
 * @param release_valve_state  1 If the release valve is open
 * @param main_valve_state  1 If the main valve is open
 * @param nitrogen_valve_state   1 If the nitrogen valve is open
 * @param gmm_state  State of the GroundModeManager
 * @param tars_state  State of Tars
 * @param arming_state  Arming state (1 if armed, 0 otherwise)
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @param battery_voltage  Battery voltage
 * @param current_consumption [A] RIG current
 * @param umbilical_current_consumption [A] Umbilical current
 * @param main_board_state  Main board fmm state
 * @param payload_board_state  Payload board fmm state
 * @param motor_board_state  Motor board fmm state
 * @param main_can_status  Main CAN status
 * @param payload_can_status  Payload CAN status
 * @param motor_can_status  Motor CAN status
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gse_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float loadcell_rocket, float loadcell_vessel, float filling_pressure, float vessel_pressure, uint8_t filling_valve_state, uint8_t venting_valve_state, uint8_t release_valve_state, uint8_t main_valve_state, uint8_t nitrogen_valve_state, uint8_t gmm_state, uint8_t tars_state, uint8_t arming_state, float cpu_load, uint32_t free_heap, float battery_voltage, float current_consumption, float umbilical_current_consumption, uint8_t main_board_state, uint8_t payload_board_state, uint8_t motor_board_state, uint8_t main_can_status, uint8_t payload_can_status, uint8_t motor_can_status, int16_t log_number, int32_t log_good)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, loadcell_rocket);
    _mav_put_float(buf, 12, loadcell_vessel);
    _mav_put_float(buf, 16, filling_pressure);
    _mav_put_float(buf, 20, vessel_pressure);
    _mav_put_float(buf, 24, cpu_load);
    _mav_put_uint32_t(buf, 28, free_heap);
    _mav_put_float(buf, 32, battery_voltage);
    _mav_put_float(buf, 36, current_consumption);
    _mav_put_float(buf, 40, umbilical_current_consumption);
    _mav_put_int32_t(buf, 44, log_good);
    _mav_put_int16_t(buf, 48, log_number);
    _mav_put_uint8_t(buf, 50, filling_valve_state);
    _mav_put_uint8_t(buf, 51, venting_valve_state);
    _mav_put_uint8_t(buf, 52, release_valve_state);
    _mav_put_uint8_t(buf, 53, main_valve_state);
    _mav_put_uint8_t(buf, 54, nitrogen_valve_state);
    _mav_put_uint8_t(buf, 55, gmm_state);
    _mav_put_uint8_t(buf, 56, tars_state);
    _mav_put_uint8_t(buf, 57, arming_state);
    _mav_put_uint8_t(buf, 58, main_board_state);
    _mav_put_uint8_t(buf, 59, payload_board_state);
    _mav_put_uint8_t(buf, 60, motor_board_state);
    _mav_put_uint8_t(buf, 61, main_can_status);
    _mav_put_uint8_t(buf, 62, payload_can_status);
    _mav_put_uint8_t(buf, 63, motor_can_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GSE_TM_LEN);
#else
    mavlink_gse_tm_t packet;
    packet.timestamp = timestamp;
    packet.loadcell_rocket = loadcell_rocket;
    packet.loadcell_vessel = loadcell_vessel;
    packet.filling_pressure = filling_pressure;
    packet.vessel_pressure = vessel_pressure;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;
    packet.battery_voltage = battery_voltage;
    packet.current_consumption = current_consumption;
    packet.umbilical_current_consumption = umbilical_current_consumption;
    packet.log_good = log_good;
    packet.log_number = log_number;
    packet.filling_valve_state = filling_valve_state;
    packet.venting_valve_state = venting_valve_state;
    packet.release_valve_state = release_valve_state;
    packet.main_valve_state = main_valve_state;
    packet.nitrogen_valve_state = nitrogen_valve_state;
    packet.gmm_state = gmm_state;
    packet.tars_state = tars_state;
    packet.arming_state = arming_state;
    packet.main_board_state = main_board_state;
    packet.payload_board_state = payload_board_state;
    packet.motor_board_state = motor_board_state;
    packet.main_can_status = main_can_status;
    packet.payload_can_status = payload_can_status;
    packet.motor_can_status = motor_can_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GSE_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GSE_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GSE_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_TM_LEN, MAVLINK_MSG_ID_GSE_TM_CRC);
}

/**
 * @brief Pack a gse_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp in microseconds
 * @param loadcell_rocket [kg] Rocket weight
 * @param loadcell_vessel [kg] External tank weight
 * @param filling_pressure [Bar] Refueling line pressure
 * @param vessel_pressure [Bar] Vessel tank pressure
 * @param filling_valve_state  1 If the filling valve is open
 * @param venting_valve_state  1 If the venting valve is open
 * @param release_valve_state  1 If the release valve is open
 * @param main_valve_state  1 If the main valve is open
 * @param nitrogen_valve_state   1 If the nitrogen valve is open
 * @param gmm_state  State of the GroundModeManager
 * @param tars_state  State of Tars
 * @param arming_state  Arming state (1 if armed, 0 otherwise)
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @param battery_voltage  Battery voltage
 * @param current_consumption [A] RIG current
 * @param umbilical_current_consumption [A] Umbilical current
 * @param main_board_state  Main board fmm state
 * @param payload_board_state  Payload board fmm state
 * @param motor_board_state  Motor board fmm state
 * @param main_can_status  Main CAN status
 * @param payload_can_status  Payload CAN status
 * @param motor_can_status  Motor CAN status
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gse_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float loadcell_rocket,float loadcell_vessel,float filling_pressure,float vessel_pressure,uint8_t filling_valve_state,uint8_t venting_valve_state,uint8_t release_valve_state,uint8_t main_valve_state,uint8_t nitrogen_valve_state,uint8_t gmm_state,uint8_t tars_state,uint8_t arming_state,float cpu_load,uint32_t free_heap,float battery_voltage,float current_consumption,float umbilical_current_consumption,uint8_t main_board_state,uint8_t payload_board_state,uint8_t motor_board_state,uint8_t main_can_status,uint8_t payload_can_status,uint8_t motor_can_status,int16_t log_number,int32_t log_good)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, loadcell_rocket);
    _mav_put_float(buf, 12, loadcell_vessel);
    _mav_put_float(buf, 16, filling_pressure);
    _mav_put_float(buf, 20, vessel_pressure);
    _mav_put_float(buf, 24, cpu_load);
    _mav_put_uint32_t(buf, 28, free_heap);
    _mav_put_float(buf, 32, battery_voltage);
    _mav_put_float(buf, 36, current_consumption);
    _mav_put_float(buf, 40, umbilical_current_consumption);
    _mav_put_int32_t(buf, 44, log_good);
    _mav_put_int16_t(buf, 48, log_number);
    _mav_put_uint8_t(buf, 50, filling_valve_state);
    _mav_put_uint8_t(buf, 51, venting_valve_state);
    _mav_put_uint8_t(buf, 52, release_valve_state);
    _mav_put_uint8_t(buf, 53, main_valve_state);
    _mav_put_uint8_t(buf, 54, nitrogen_valve_state);
    _mav_put_uint8_t(buf, 55, gmm_state);
    _mav_put_uint8_t(buf, 56, tars_state);
    _mav_put_uint8_t(buf, 57, arming_state);
    _mav_put_uint8_t(buf, 58, main_board_state);
    _mav_put_uint8_t(buf, 59, payload_board_state);
    _mav_put_uint8_t(buf, 60, motor_board_state);
    _mav_put_uint8_t(buf, 61, main_can_status);
    _mav_put_uint8_t(buf, 62, payload_can_status);
    _mav_put_uint8_t(buf, 63, motor_can_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GSE_TM_LEN);
#else
    mavlink_gse_tm_t packet;
    packet.timestamp = timestamp;
    packet.loadcell_rocket = loadcell_rocket;
    packet.loadcell_vessel = loadcell_vessel;
    packet.filling_pressure = filling_pressure;
    packet.vessel_pressure = vessel_pressure;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;
    packet.battery_voltage = battery_voltage;
    packet.current_consumption = current_consumption;
    packet.umbilical_current_consumption = umbilical_current_consumption;
    packet.log_good = log_good;
    packet.log_number = log_number;
    packet.filling_valve_state = filling_valve_state;
    packet.venting_valve_state = venting_valve_state;
    packet.release_valve_state = release_valve_state;
    packet.main_valve_state = main_valve_state;
    packet.nitrogen_valve_state = nitrogen_valve_state;
    packet.gmm_state = gmm_state;
    packet.tars_state = tars_state;
    packet.arming_state = arming_state;
    packet.main_board_state = main_board_state;
    packet.payload_board_state = payload_board_state;
    packet.motor_board_state = motor_board_state;
    packet.main_can_status = main_can_status;
    packet.payload_can_status = payload_can_status;
    packet.motor_can_status = motor_can_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GSE_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GSE_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GSE_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_TM_LEN, MAVLINK_MSG_ID_GSE_TM_CRC);
}

/**
 * @brief Encode a gse_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gse_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gse_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gse_tm_t* gse_tm)
{
    return mavlink_msg_gse_tm_pack(system_id, component_id, msg, gse_tm->timestamp, gse_tm->loadcell_rocket, gse_tm->loadcell_vessel, gse_tm->filling_pressure, gse_tm->vessel_pressure, gse_tm->filling_valve_state, gse_tm->venting_valve_state, gse_tm->release_valve_state, gse_tm->main_valve_state, gse_tm->nitrogen_valve_state, gse_tm->gmm_state, gse_tm->tars_state, gse_tm->arming_state, gse_tm->cpu_load, gse_tm->free_heap, gse_tm->battery_voltage, gse_tm->current_consumption, gse_tm->umbilical_current_consumption, gse_tm->main_board_state, gse_tm->payload_board_state, gse_tm->motor_board_state, gse_tm->main_can_status, gse_tm->payload_can_status, gse_tm->motor_can_status, gse_tm->log_number, gse_tm->log_good);
}

/**
 * @brief Encode a gse_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gse_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gse_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gse_tm_t* gse_tm)
{
    return mavlink_msg_gse_tm_pack_chan(system_id, component_id, chan, msg, gse_tm->timestamp, gse_tm->loadcell_rocket, gse_tm->loadcell_vessel, gse_tm->filling_pressure, gse_tm->vessel_pressure, gse_tm->filling_valve_state, gse_tm->venting_valve_state, gse_tm->release_valve_state, gse_tm->main_valve_state, gse_tm->nitrogen_valve_state, gse_tm->gmm_state, gse_tm->tars_state, gse_tm->arming_state, gse_tm->cpu_load, gse_tm->free_heap, gse_tm->battery_voltage, gse_tm->current_consumption, gse_tm->umbilical_current_consumption, gse_tm->main_board_state, gse_tm->payload_board_state, gse_tm->motor_board_state, gse_tm->main_can_status, gse_tm->payload_can_status, gse_tm->motor_can_status, gse_tm->log_number, gse_tm->log_good);
}

/**
 * @brief Send a gse_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param loadcell_rocket [kg] Rocket weight
 * @param loadcell_vessel [kg] External tank weight
 * @param filling_pressure [Bar] Refueling line pressure
 * @param vessel_pressure [Bar] Vessel tank pressure
 * @param filling_valve_state  1 If the filling valve is open
 * @param venting_valve_state  1 If the venting valve is open
 * @param release_valve_state  1 If the release valve is open
 * @param main_valve_state  1 If the main valve is open
 * @param nitrogen_valve_state   1 If the nitrogen valve is open
 * @param gmm_state  State of the GroundModeManager
 * @param tars_state  State of Tars
 * @param arming_state  Arming state (1 if armed, 0 otherwise)
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @param battery_voltage  Battery voltage
 * @param current_consumption [A] RIG current
 * @param umbilical_current_consumption [A] Umbilical current
 * @param main_board_state  Main board fmm state
 * @param payload_board_state  Payload board fmm state
 * @param motor_board_state  Motor board fmm state
 * @param main_can_status  Main CAN status
 * @param payload_can_status  Payload CAN status
 * @param motor_can_status  Motor CAN status
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gse_tm_send(mavlink_channel_t chan, uint64_t timestamp, float loadcell_rocket, float loadcell_vessel, float filling_pressure, float vessel_pressure, uint8_t filling_valve_state, uint8_t venting_valve_state, uint8_t release_valve_state, uint8_t main_valve_state, uint8_t nitrogen_valve_state, uint8_t gmm_state, uint8_t tars_state, uint8_t arming_state, float cpu_load, uint32_t free_heap, float battery_voltage, float current_consumption, float umbilical_current_consumption, uint8_t main_board_state, uint8_t payload_board_state, uint8_t motor_board_state, uint8_t main_can_status, uint8_t payload_can_status, uint8_t motor_can_status, int16_t log_number, int32_t log_good)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, loadcell_rocket);
    _mav_put_float(buf, 12, loadcell_vessel);
    _mav_put_float(buf, 16, filling_pressure);
    _mav_put_float(buf, 20, vessel_pressure);
    _mav_put_float(buf, 24, cpu_load);
    _mav_put_uint32_t(buf, 28, free_heap);
    _mav_put_float(buf, 32, battery_voltage);
    _mav_put_float(buf, 36, current_consumption);
    _mav_put_float(buf, 40, umbilical_current_consumption);
    _mav_put_int32_t(buf, 44, log_good);
    _mav_put_int16_t(buf, 48, log_number);
    _mav_put_uint8_t(buf, 50, filling_valve_state);
    _mav_put_uint8_t(buf, 51, venting_valve_state);
    _mav_put_uint8_t(buf, 52, release_valve_state);
    _mav_put_uint8_t(buf, 53, main_valve_state);
    _mav_put_uint8_t(buf, 54, nitrogen_valve_state);
    _mav_put_uint8_t(buf, 55, gmm_state);
    _mav_put_uint8_t(buf, 56, tars_state);
    _mav_put_uint8_t(buf, 57, arming_state);
    _mav_put_uint8_t(buf, 58, main_board_state);
    _mav_put_uint8_t(buf, 59, payload_board_state);
    _mav_put_uint8_t(buf, 60, motor_board_state);
    _mav_put_uint8_t(buf, 61, main_can_status);
    _mav_put_uint8_t(buf, 62, payload_can_status);
    _mav_put_uint8_t(buf, 63, motor_can_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSE_TM, buf, MAVLINK_MSG_ID_GSE_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_TM_LEN, MAVLINK_MSG_ID_GSE_TM_CRC);
#else
    mavlink_gse_tm_t packet;
    packet.timestamp = timestamp;
    packet.loadcell_rocket = loadcell_rocket;
    packet.loadcell_vessel = loadcell_vessel;
    packet.filling_pressure = filling_pressure;
    packet.vessel_pressure = vessel_pressure;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;
    packet.battery_voltage = battery_voltage;
    packet.current_consumption = current_consumption;
    packet.umbilical_current_consumption = umbilical_current_consumption;
    packet.log_good = log_good;
    packet.log_number = log_number;
    packet.filling_valve_state = filling_valve_state;
    packet.venting_valve_state = venting_valve_state;
    packet.release_valve_state = release_valve_state;
    packet.main_valve_state = main_valve_state;
    packet.nitrogen_valve_state = nitrogen_valve_state;
    packet.gmm_state = gmm_state;
    packet.tars_state = tars_state;
    packet.arming_state = arming_state;
    packet.main_board_state = main_board_state;
    packet.payload_board_state = payload_board_state;
    packet.motor_board_state = motor_board_state;
    packet.main_can_status = main_can_status;
    packet.payload_can_status = payload_can_status;
    packet.motor_can_status = motor_can_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSE_TM, (const char *)&packet, MAVLINK_MSG_ID_GSE_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_TM_LEN, MAVLINK_MSG_ID_GSE_TM_CRC);
#endif
}

/**
 * @brief Send a gse_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gse_tm_send_struct(mavlink_channel_t chan, const mavlink_gse_tm_t* gse_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gse_tm_send(chan, gse_tm->timestamp, gse_tm->loadcell_rocket, gse_tm->loadcell_vessel, gse_tm->filling_pressure, gse_tm->vessel_pressure, gse_tm->filling_valve_state, gse_tm->venting_valve_state, gse_tm->release_valve_state, gse_tm->main_valve_state, gse_tm->nitrogen_valve_state, gse_tm->gmm_state, gse_tm->tars_state, gse_tm->arming_state, gse_tm->cpu_load, gse_tm->free_heap, gse_tm->battery_voltage, gse_tm->current_consumption, gse_tm->umbilical_current_consumption, gse_tm->main_board_state, gse_tm->payload_board_state, gse_tm->motor_board_state, gse_tm->main_can_status, gse_tm->payload_can_status, gse_tm->motor_can_status, gse_tm->log_number, gse_tm->log_good);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSE_TM, (const char *)gse_tm, MAVLINK_MSG_ID_GSE_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_TM_LEN, MAVLINK_MSG_ID_GSE_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_GSE_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gse_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float loadcell_rocket, float loadcell_vessel, float filling_pressure, float vessel_pressure, uint8_t filling_valve_state, uint8_t venting_valve_state, uint8_t release_valve_state, uint8_t main_valve_state, uint8_t nitrogen_valve_state, uint8_t gmm_state, uint8_t tars_state, uint8_t arming_state, float cpu_load, uint32_t free_heap, float battery_voltage, float current_consumption, float umbilical_current_consumption, uint8_t main_board_state, uint8_t payload_board_state, uint8_t motor_board_state, uint8_t main_can_status, uint8_t payload_can_status, uint8_t motor_can_status, int16_t log_number, int32_t log_good)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, loadcell_rocket);
    _mav_put_float(buf, 12, loadcell_vessel);
    _mav_put_float(buf, 16, filling_pressure);
    _mav_put_float(buf, 20, vessel_pressure);
    _mav_put_float(buf, 24, cpu_load);
    _mav_put_uint32_t(buf, 28, free_heap);
    _mav_put_float(buf, 32, battery_voltage);
    _mav_put_float(buf, 36, current_consumption);
    _mav_put_float(buf, 40, umbilical_current_consumption);
    _mav_put_int32_t(buf, 44, log_good);
    _mav_put_int16_t(buf, 48, log_number);
    _mav_put_uint8_t(buf, 50, filling_valve_state);
    _mav_put_uint8_t(buf, 51, venting_valve_state);
    _mav_put_uint8_t(buf, 52, release_valve_state);
    _mav_put_uint8_t(buf, 53, main_valve_state);
    _mav_put_uint8_t(buf, 54, nitrogen_valve_state);
    _mav_put_uint8_t(buf, 55, gmm_state);
    _mav_put_uint8_t(buf, 56, tars_state);
    _mav_put_uint8_t(buf, 57, arming_state);
    _mav_put_uint8_t(buf, 58, main_board_state);
    _mav_put_uint8_t(buf, 59, payload_board_state);
    _mav_put_uint8_t(buf, 60, motor_board_state);
    _mav_put_uint8_t(buf, 61, main_can_status);
    _mav_put_uint8_t(buf, 62, payload_can_status);
    _mav_put_uint8_t(buf, 63, motor_can_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSE_TM, buf, MAVLINK_MSG_ID_GSE_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_TM_LEN, MAVLINK_MSG_ID_GSE_TM_CRC);
#else
    mavlink_gse_tm_t *packet = (mavlink_gse_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->loadcell_rocket = loadcell_rocket;
    packet->loadcell_vessel = loadcell_vessel;
    packet->filling_pressure = filling_pressure;
    packet->vessel_pressure = vessel_pressure;
    packet->cpu_load = cpu_load;
    packet->free_heap = free_heap;
    packet->battery_voltage = battery_voltage;
    packet->current_consumption = current_consumption;
    packet->umbilical_current_consumption = umbilical_current_consumption;
    packet->log_good = log_good;
    packet->log_number = log_number;
    packet->filling_valve_state = filling_valve_state;
    packet->venting_valve_state = venting_valve_state;
    packet->release_valve_state = release_valve_state;
    packet->main_valve_state = main_valve_state;
    packet->nitrogen_valve_state = nitrogen_valve_state;
    packet->gmm_state = gmm_state;
    packet->tars_state = tars_state;
    packet->arming_state = arming_state;
    packet->main_board_state = main_board_state;
    packet->payload_board_state = payload_board_state;
    packet->motor_board_state = motor_board_state;
    packet->main_can_status = main_can_status;
    packet->payload_can_status = payload_can_status;
    packet->motor_can_status = motor_can_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSE_TM, (const char *)packet, MAVLINK_MSG_ID_GSE_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_TM_LEN, MAVLINK_MSG_ID_GSE_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE GSE_TM UNPACKING


/**
 * @brief Get field timestamp from gse_tm message
 *
 * @return [us] Timestamp in microseconds
 */
static inline uint64_t mavlink_msg_gse_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field loadcell_rocket from gse_tm message
 *
 * @return [kg] Rocket weight
 */
static inline float mavlink_msg_gse_tm_get_loadcell_rocket(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field loadcell_vessel from gse_tm message
 *
 * @return [kg] External tank weight
 */
static inline float mavlink_msg_gse_tm_get_loadcell_vessel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field filling_pressure from gse_tm message
 *
 * @return [Bar] Refueling line pressure
 */
static inline float mavlink_msg_gse_tm_get_filling_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vessel_pressure from gse_tm message
 *
 * @return [Bar] Vessel tank pressure
 */
static inline float mavlink_msg_gse_tm_get_vessel_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field filling_valve_state from gse_tm message
 *
 * @return  1 If the filling valve is open
 */
static inline uint8_t mavlink_msg_gse_tm_get_filling_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field venting_valve_state from gse_tm message
 *
 * @return  1 If the venting valve is open
 */
static inline uint8_t mavlink_msg_gse_tm_get_venting_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Get field release_valve_state from gse_tm message
 *
 * @return  1 If the release valve is open
 */
static inline uint8_t mavlink_msg_gse_tm_get_release_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field main_valve_state from gse_tm message
 *
 * @return  1 If the main valve is open
 */
static inline uint8_t mavlink_msg_gse_tm_get_main_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Get field nitrogen_valve_state from gse_tm message
 *
 * @return   1 If the nitrogen valve is open
 */
static inline uint8_t mavlink_msg_gse_tm_get_nitrogen_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  54);
}

/**
 * @brief Get field gmm_state from gse_tm message
 *
 * @return  State of the GroundModeManager
 */
static inline uint8_t mavlink_msg_gse_tm_get_gmm_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  55);
}

/**
 * @brief Get field tars_state from gse_tm message
 *
 * @return  State of Tars
 */
static inline uint8_t mavlink_msg_gse_tm_get_tars_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  56);
}

/**
 * @brief Get field arming_state from gse_tm message
 *
 * @return  Arming state (1 if armed, 0 otherwise)
 */
static inline uint8_t mavlink_msg_gse_tm_get_arming_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  57);
}

/**
 * @brief Get field cpu_load from gse_tm message
 *
 * @return  CPU load in percentage
 */
static inline float mavlink_msg_gse_tm_get_cpu_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field free_heap from gse_tm message
 *
 * @return  Amount of available heap in memory
 */
static inline uint32_t mavlink_msg_gse_tm_get_free_heap(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Get field battery_voltage from gse_tm message
 *
 * @return  Battery voltage
 */
static inline float mavlink_msg_gse_tm_get_battery_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field current_consumption from gse_tm message
 *
 * @return [A] RIG current
 */
static inline float mavlink_msg_gse_tm_get_current_consumption(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field umbilical_current_consumption from gse_tm message
 *
 * @return [A] Umbilical current
 */
static inline float mavlink_msg_gse_tm_get_umbilical_current_consumption(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field main_board_state from gse_tm message
 *
 * @return  Main board fmm state
 */
static inline uint8_t mavlink_msg_gse_tm_get_main_board_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  58);
}

/**
 * @brief Get field payload_board_state from gse_tm message
 *
 * @return  Payload board fmm state
 */
static inline uint8_t mavlink_msg_gse_tm_get_payload_board_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  59);
}

/**
 * @brief Get field motor_board_state from gse_tm message
 *
 * @return  Motor board fmm state
 */
static inline uint8_t mavlink_msg_gse_tm_get_motor_board_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  60);
}

/**
 * @brief Get field main_can_status from gse_tm message
 *
 * @return  Main CAN status
 */
static inline uint8_t mavlink_msg_gse_tm_get_main_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  61);
}

/**
 * @brief Get field payload_can_status from gse_tm message
 *
 * @return  Payload CAN status
 */
static inline uint8_t mavlink_msg_gse_tm_get_payload_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  62);
}

/**
 * @brief Get field motor_can_status from gse_tm message
 *
 * @return  Motor CAN status
 */
static inline uint8_t mavlink_msg_gse_tm_get_motor_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  63);
}

/**
 * @brief Get field log_number from gse_tm message
 *
 * @return  Currently active log file, -1 if the logger is inactive
 */
static inline int16_t mavlink_msg_gse_tm_get_log_number(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  48);
}

/**
 * @brief Get field log_good from gse_tm message
 *
 * @return  0 if log failed, 1 otherwise
 */
static inline int32_t mavlink_msg_gse_tm_get_log_good(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  44);
}

/**
 * @brief Decode a gse_tm message into a struct
 *
 * @param msg The message to decode
 * @param gse_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_gse_tm_decode(const mavlink_message_t* msg, mavlink_gse_tm_t* gse_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gse_tm->timestamp = mavlink_msg_gse_tm_get_timestamp(msg);
    gse_tm->loadcell_rocket = mavlink_msg_gse_tm_get_loadcell_rocket(msg);
    gse_tm->loadcell_vessel = mavlink_msg_gse_tm_get_loadcell_vessel(msg);
    gse_tm->filling_pressure = mavlink_msg_gse_tm_get_filling_pressure(msg);
    gse_tm->vessel_pressure = mavlink_msg_gse_tm_get_vessel_pressure(msg);
    gse_tm->cpu_load = mavlink_msg_gse_tm_get_cpu_load(msg);
    gse_tm->free_heap = mavlink_msg_gse_tm_get_free_heap(msg);
    gse_tm->battery_voltage = mavlink_msg_gse_tm_get_battery_voltage(msg);
    gse_tm->current_consumption = mavlink_msg_gse_tm_get_current_consumption(msg);
    gse_tm->umbilical_current_consumption = mavlink_msg_gse_tm_get_umbilical_current_consumption(msg);
    gse_tm->log_good = mavlink_msg_gse_tm_get_log_good(msg);
    gse_tm->log_number = mavlink_msg_gse_tm_get_log_number(msg);
    gse_tm->filling_valve_state = mavlink_msg_gse_tm_get_filling_valve_state(msg);
    gse_tm->venting_valve_state = mavlink_msg_gse_tm_get_venting_valve_state(msg);
    gse_tm->release_valve_state = mavlink_msg_gse_tm_get_release_valve_state(msg);
    gse_tm->main_valve_state = mavlink_msg_gse_tm_get_main_valve_state(msg);
    gse_tm->nitrogen_valve_state = mavlink_msg_gse_tm_get_nitrogen_valve_state(msg);
    gse_tm->gmm_state = mavlink_msg_gse_tm_get_gmm_state(msg);
    gse_tm->tars_state = mavlink_msg_gse_tm_get_tars_state(msg);
    gse_tm->arming_state = mavlink_msg_gse_tm_get_arming_state(msg);
    gse_tm->main_board_state = mavlink_msg_gse_tm_get_main_board_state(msg);
    gse_tm->payload_board_state = mavlink_msg_gse_tm_get_payload_board_state(msg);
    gse_tm->motor_board_state = mavlink_msg_gse_tm_get_motor_board_state(msg);
    gse_tm->main_can_status = mavlink_msg_gse_tm_get_main_can_status(msg);
    gse_tm->payload_can_status = mavlink_msg_gse_tm_get_payload_can_status(msg);
    gse_tm->motor_can_status = mavlink_msg_gse_tm_get_motor_can_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GSE_TM_LEN? msg->len : MAVLINK_MSG_ID_GSE_TM_LEN;
        memset(gse_tm, 0, MAVLINK_MSG_ID_GSE_TM_LEN);
    memcpy(gse_tm, _MAV_PAYLOAD(msg), len);
#endif
}
