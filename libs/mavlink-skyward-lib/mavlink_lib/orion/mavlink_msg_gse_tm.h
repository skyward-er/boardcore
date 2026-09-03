#pragma once
// MESSAGE GSE_TM PACKING

#define MAVLINK_MSG_ID_GSE_TM 212


typedef struct __mavlink_gse_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 float rocket_mass; /*< [kg] Rocket mass loadcell measurement*/
 float ox_tank_mass; /*< [kg] OX tank mass loadcell measurement*/
 float ox_vessel_mass; /*< [kg] OX vessel tank mass loadcell measurement*/
 float ox_filling_pressure; /*< [Bar] OX refueling line pressure*/
 float ox_vessel_pressure; /*< [Bar] OX vessel tank pressure*/
 float n2_filling_pressure; /*< [Bar] N2 refueling line pressure*/
 float n2_vessel_1_pressure; /*< [Bar] N2 vessel 1 tank pressure*/
 float n2_vessel_2_pressure; /*< [Bar] N2 vessel 2 tank pressure*/
 float cpu_load; /*<  CPU load in percentage*/
 uint32_t free_heap; /*<  Amount of available heap memory*/
 float battery_voltage; /*<  Battery voltage*/
 float current_consumption; /*< [A] RIG current*/
 float umbilical_current_consumption; /*< [A] Umbilical current*/
 int16_t log_number; /*<  Currently active log file, -1 if the logger is inactive*/
 uint8_t ox_filling_valve_state; /*<  OX filling valve state (1: open, 0: close)*/
 uint8_t ox_release_valve_state; /*<  OX release line pressure valve state (1: open, 0: close)*/
 uint8_t ox_detach_state; /*<  OX quick connector detach state (1: open, 0: close)*/
 uint8_t ox_venting_valve_state; /*<  OX venting valve state (1: open, 0: close)*/
 uint8_t n2_filling_valve_state; /*<  N2 filling valve state (1: open, 0: close)*/
 uint8_t n2_release_valve_state; /*<  N2 release line pressure valve state (1: open, 0: close)*/
 uint8_t n2_detach_state; /*<  N2 quick connector detach state (1: open, 0: close)*/
 uint8_t n2_quenching_valve_state; /*<  N2 quenching valve state (1: open, 0: close)*/
 uint8_t n2_3way_valve_state; /*<  N2 3-way valve state (1: open, 0: close)*/
 uint8_t main_valve_state; /*<  Rocket main OX valve state (1: open, 0: close)*/
 uint8_t nitrogen_valve_state; /*<  Rocket main N2 valve state (1: open, 0: close)*/
 uint8_t chamber_valve_state; /*<  Chamber pressurization valve state (1: enabled, 0: disabled)*/
 uint8_t gmm_state; /*<  State of the GroundModeManager*/
 uint8_t tars1_state; /*<  State of TARS 1*/
 uint8_t tars3_state; /*<  State of TARS 3*/
 uint8_t arming_state; /*<  Arming state (1: armed, 0: otherwise)*/
 uint8_t main_board_state; /*<  Main board FMM state*/
 uint8_t payload_board_state; /*<  Payload board FMM state*/
 uint8_t motor_board_state; /*<  Motor board FMM state*/
 uint8_t main_can_status; /*<  Main CAN status*/
 uint8_t payload_can_status; /*<  Payload CAN status*/
 uint8_t motor_can_status; /*<  Motor CAN status*/
 uint8_t log_good; /*<  0 if log failed, 1 otherwise*/
} mavlink_gse_tm_t;

#define MAVLINK_MSG_ID_GSE_TM_LEN 85
#define MAVLINK_MSG_ID_GSE_TM_MIN_LEN 85
#define MAVLINK_MSG_ID_212_LEN 85
#define MAVLINK_MSG_ID_212_MIN_LEN 85

#define MAVLINK_MSG_ID_GSE_TM_CRC 255
#define MAVLINK_MSG_ID_212_CRC 255



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GSE_TM { \
    212, \
    "GSE_TM", \
    38, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gse_tm_t, timestamp) }, \
         { "rocket_mass", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gse_tm_t, rocket_mass) }, \
         { "ox_tank_mass", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gse_tm_t, ox_tank_mass) }, \
         { "ox_vessel_mass", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gse_tm_t, ox_vessel_mass) }, \
         { "ox_filling_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gse_tm_t, ox_filling_pressure) }, \
         { "ox_vessel_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gse_tm_t, ox_vessel_pressure) }, \
         { "n2_filling_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gse_tm_t, n2_filling_pressure) }, \
         { "n2_vessel_1_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gse_tm_t, n2_vessel_1_pressure) }, \
         { "n2_vessel_2_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gse_tm_t, n2_vessel_2_pressure) }, \
         { "ox_filling_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_gse_tm_t, ox_filling_valve_state) }, \
         { "ox_release_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 63, offsetof(mavlink_gse_tm_t, ox_release_valve_state) }, \
         { "ox_detach_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_gse_tm_t, ox_detach_state) }, \
         { "ox_venting_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 65, offsetof(mavlink_gse_tm_t, ox_venting_valve_state) }, \
         { "n2_filling_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 66, offsetof(mavlink_gse_tm_t, n2_filling_valve_state) }, \
         { "n2_release_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 67, offsetof(mavlink_gse_tm_t, n2_release_valve_state) }, \
         { "n2_detach_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 68, offsetof(mavlink_gse_tm_t, n2_detach_state) }, \
         { "n2_quenching_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 69, offsetof(mavlink_gse_tm_t, n2_quenching_valve_state) }, \
         { "n2_3way_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_gse_tm_t, n2_3way_valve_state) }, \
         { "main_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 71, offsetof(mavlink_gse_tm_t, main_valve_state) }, \
         { "nitrogen_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_gse_tm_t, nitrogen_valve_state) }, \
         { "chamber_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 73, offsetof(mavlink_gse_tm_t, chamber_valve_state) }, \
         { "gmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 74, offsetof(mavlink_gse_tm_t, gmm_state) }, \
         { "tars1_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 75, offsetof(mavlink_gse_tm_t, tars1_state) }, \
         { "tars3_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 76, offsetof(mavlink_gse_tm_t, tars3_state) }, \
         { "arming_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 77, offsetof(mavlink_gse_tm_t, arming_state) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gse_tm_t, cpu_load) }, \
         { "free_heap", NULL, MAVLINK_TYPE_UINT32_T, 0, 44, offsetof(mavlink_gse_tm_t, free_heap) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_gse_tm_t, battery_voltage) }, \
         { "current_consumption", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_gse_tm_t, current_consumption) }, \
         { "umbilical_current_consumption", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_gse_tm_t, umbilical_current_consumption) }, \
         { "main_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 78, offsetof(mavlink_gse_tm_t, main_board_state) }, \
         { "payload_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 79, offsetof(mavlink_gse_tm_t, payload_board_state) }, \
         { "motor_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_gse_tm_t, motor_board_state) }, \
         { "main_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 81, offsetof(mavlink_gse_tm_t, main_can_status) }, \
         { "payload_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 82, offsetof(mavlink_gse_tm_t, payload_can_status) }, \
         { "motor_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 83, offsetof(mavlink_gse_tm_t, motor_can_status) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 60, offsetof(mavlink_gse_tm_t, log_number) }, \
         { "log_good", NULL, MAVLINK_TYPE_UINT8_T, 0, 84, offsetof(mavlink_gse_tm_t, log_good) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GSE_TM { \
    "GSE_TM", \
    38, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gse_tm_t, timestamp) }, \
         { "rocket_mass", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gse_tm_t, rocket_mass) }, \
         { "ox_tank_mass", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gse_tm_t, ox_tank_mass) }, \
         { "ox_vessel_mass", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gse_tm_t, ox_vessel_mass) }, \
         { "ox_filling_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gse_tm_t, ox_filling_pressure) }, \
         { "ox_vessel_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gse_tm_t, ox_vessel_pressure) }, \
         { "n2_filling_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gse_tm_t, n2_filling_pressure) }, \
         { "n2_vessel_1_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gse_tm_t, n2_vessel_1_pressure) }, \
         { "n2_vessel_2_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gse_tm_t, n2_vessel_2_pressure) }, \
         { "ox_filling_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_gse_tm_t, ox_filling_valve_state) }, \
         { "ox_release_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 63, offsetof(mavlink_gse_tm_t, ox_release_valve_state) }, \
         { "ox_detach_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_gse_tm_t, ox_detach_state) }, \
         { "ox_venting_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 65, offsetof(mavlink_gse_tm_t, ox_venting_valve_state) }, \
         { "n2_filling_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 66, offsetof(mavlink_gse_tm_t, n2_filling_valve_state) }, \
         { "n2_release_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 67, offsetof(mavlink_gse_tm_t, n2_release_valve_state) }, \
         { "n2_detach_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 68, offsetof(mavlink_gse_tm_t, n2_detach_state) }, \
         { "n2_quenching_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 69, offsetof(mavlink_gse_tm_t, n2_quenching_valve_state) }, \
         { "n2_3way_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_gse_tm_t, n2_3way_valve_state) }, \
         { "main_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 71, offsetof(mavlink_gse_tm_t, main_valve_state) }, \
         { "nitrogen_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_gse_tm_t, nitrogen_valve_state) }, \
         { "chamber_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 73, offsetof(mavlink_gse_tm_t, chamber_valve_state) }, \
         { "gmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 74, offsetof(mavlink_gse_tm_t, gmm_state) }, \
         { "tars1_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 75, offsetof(mavlink_gse_tm_t, tars1_state) }, \
         { "tars3_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 76, offsetof(mavlink_gse_tm_t, tars3_state) }, \
         { "arming_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 77, offsetof(mavlink_gse_tm_t, arming_state) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gse_tm_t, cpu_load) }, \
         { "free_heap", NULL, MAVLINK_TYPE_UINT32_T, 0, 44, offsetof(mavlink_gse_tm_t, free_heap) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_gse_tm_t, battery_voltage) }, \
         { "current_consumption", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_gse_tm_t, current_consumption) }, \
         { "umbilical_current_consumption", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_gse_tm_t, umbilical_current_consumption) }, \
         { "main_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 78, offsetof(mavlink_gse_tm_t, main_board_state) }, \
         { "payload_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 79, offsetof(mavlink_gse_tm_t, payload_board_state) }, \
         { "motor_board_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_gse_tm_t, motor_board_state) }, \
         { "main_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 81, offsetof(mavlink_gse_tm_t, main_can_status) }, \
         { "payload_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 82, offsetof(mavlink_gse_tm_t, payload_can_status) }, \
         { "motor_can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 83, offsetof(mavlink_gse_tm_t, motor_can_status) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 60, offsetof(mavlink_gse_tm_t, log_number) }, \
         { "log_good", NULL, MAVLINK_TYPE_UINT8_T, 0, 84, offsetof(mavlink_gse_tm_t, log_good) }, \
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
 * @param rocket_mass [kg] Rocket mass loadcell measurement
 * @param ox_tank_mass [kg] OX tank mass loadcell measurement
 * @param ox_vessel_mass [kg] OX vessel tank mass loadcell measurement
 * @param ox_filling_pressure [Bar] OX refueling line pressure
 * @param ox_vessel_pressure [Bar] OX vessel tank pressure
 * @param n2_filling_pressure [Bar] N2 refueling line pressure
 * @param n2_vessel_1_pressure [Bar] N2 vessel 1 tank pressure
 * @param n2_vessel_2_pressure [Bar] N2 vessel 2 tank pressure
 * @param ox_filling_valve_state  OX filling valve state (1: open, 0: close)
 * @param ox_release_valve_state  OX release line pressure valve state (1: open, 0: close)
 * @param ox_detach_state  OX quick connector detach state (1: open, 0: close)
 * @param ox_venting_valve_state  OX venting valve state (1: open, 0: close)
 * @param n2_filling_valve_state  N2 filling valve state (1: open, 0: close)
 * @param n2_release_valve_state  N2 release line pressure valve state (1: open, 0: close)
 * @param n2_detach_state  N2 quick connector detach state (1: open, 0: close)
 * @param n2_quenching_valve_state  N2 quenching valve state (1: open, 0: close)
 * @param n2_3way_valve_state  N2 3-way valve state (1: open, 0: close)
 * @param main_valve_state  Rocket main OX valve state (1: open, 0: close)
 * @param nitrogen_valve_state  Rocket main N2 valve state (1: open, 0: close)
 * @param chamber_valve_state  Chamber pressurization valve state (1: enabled, 0: disabled)
 * @param gmm_state  State of the GroundModeManager
 * @param tars1_state  State of TARS 1
 * @param tars3_state  State of TARS 3
 * @param arming_state  Arming state (1: armed, 0: otherwise)
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap memory
 * @param battery_voltage  Battery voltage
 * @param current_consumption [A] RIG current
 * @param umbilical_current_consumption [A] Umbilical current
 * @param main_board_state  Main board FMM state
 * @param payload_board_state  Payload board FMM state
 * @param motor_board_state  Motor board FMM state
 * @param main_can_status  Main CAN status
 * @param payload_can_status  Payload CAN status
 * @param motor_can_status  Motor CAN status
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gse_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float rocket_mass, float ox_tank_mass, float ox_vessel_mass, float ox_filling_pressure, float ox_vessel_pressure, float n2_filling_pressure, float n2_vessel_1_pressure, float n2_vessel_2_pressure, uint8_t ox_filling_valve_state, uint8_t ox_release_valve_state, uint8_t ox_detach_state, uint8_t ox_venting_valve_state, uint8_t n2_filling_valve_state, uint8_t n2_release_valve_state, uint8_t n2_detach_state, uint8_t n2_quenching_valve_state, uint8_t n2_3way_valve_state, uint8_t main_valve_state, uint8_t nitrogen_valve_state, uint8_t chamber_valve_state, uint8_t gmm_state, uint8_t tars1_state, uint8_t tars3_state, uint8_t arming_state, float cpu_load, uint32_t free_heap, float battery_voltage, float current_consumption, float umbilical_current_consumption, uint8_t main_board_state, uint8_t payload_board_state, uint8_t motor_board_state, uint8_t main_can_status, uint8_t payload_can_status, uint8_t motor_can_status, int16_t log_number, uint8_t log_good)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, rocket_mass);
    _mav_put_float(buf, 12, ox_tank_mass);
    _mav_put_float(buf, 16, ox_vessel_mass);
    _mav_put_float(buf, 20, ox_filling_pressure);
    _mav_put_float(buf, 24, ox_vessel_pressure);
    _mav_put_float(buf, 28, n2_filling_pressure);
    _mav_put_float(buf, 32, n2_vessel_1_pressure);
    _mav_put_float(buf, 36, n2_vessel_2_pressure);
    _mav_put_float(buf, 40, cpu_load);
    _mav_put_uint32_t(buf, 44, free_heap);
    _mav_put_float(buf, 48, battery_voltage);
    _mav_put_float(buf, 52, current_consumption);
    _mav_put_float(buf, 56, umbilical_current_consumption);
    _mav_put_int16_t(buf, 60, log_number);
    _mav_put_uint8_t(buf, 62, ox_filling_valve_state);
    _mav_put_uint8_t(buf, 63, ox_release_valve_state);
    _mav_put_uint8_t(buf, 64, ox_detach_state);
    _mav_put_uint8_t(buf, 65, ox_venting_valve_state);
    _mav_put_uint8_t(buf, 66, n2_filling_valve_state);
    _mav_put_uint8_t(buf, 67, n2_release_valve_state);
    _mav_put_uint8_t(buf, 68, n2_detach_state);
    _mav_put_uint8_t(buf, 69, n2_quenching_valve_state);
    _mav_put_uint8_t(buf, 70, n2_3way_valve_state);
    _mav_put_uint8_t(buf, 71, main_valve_state);
    _mav_put_uint8_t(buf, 72, nitrogen_valve_state);
    _mav_put_uint8_t(buf, 73, chamber_valve_state);
    _mav_put_uint8_t(buf, 74, gmm_state);
    _mav_put_uint8_t(buf, 75, tars1_state);
    _mav_put_uint8_t(buf, 76, tars3_state);
    _mav_put_uint8_t(buf, 77, arming_state);
    _mav_put_uint8_t(buf, 78, main_board_state);
    _mav_put_uint8_t(buf, 79, payload_board_state);
    _mav_put_uint8_t(buf, 80, motor_board_state);
    _mav_put_uint8_t(buf, 81, main_can_status);
    _mav_put_uint8_t(buf, 82, payload_can_status);
    _mav_put_uint8_t(buf, 83, motor_can_status);
    _mav_put_uint8_t(buf, 84, log_good);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GSE_TM_LEN);
#else
    mavlink_gse_tm_t packet;
    packet.timestamp = timestamp;
    packet.rocket_mass = rocket_mass;
    packet.ox_tank_mass = ox_tank_mass;
    packet.ox_vessel_mass = ox_vessel_mass;
    packet.ox_filling_pressure = ox_filling_pressure;
    packet.ox_vessel_pressure = ox_vessel_pressure;
    packet.n2_filling_pressure = n2_filling_pressure;
    packet.n2_vessel_1_pressure = n2_vessel_1_pressure;
    packet.n2_vessel_2_pressure = n2_vessel_2_pressure;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;
    packet.battery_voltage = battery_voltage;
    packet.current_consumption = current_consumption;
    packet.umbilical_current_consumption = umbilical_current_consumption;
    packet.log_number = log_number;
    packet.ox_filling_valve_state = ox_filling_valve_state;
    packet.ox_release_valve_state = ox_release_valve_state;
    packet.ox_detach_state = ox_detach_state;
    packet.ox_venting_valve_state = ox_venting_valve_state;
    packet.n2_filling_valve_state = n2_filling_valve_state;
    packet.n2_release_valve_state = n2_release_valve_state;
    packet.n2_detach_state = n2_detach_state;
    packet.n2_quenching_valve_state = n2_quenching_valve_state;
    packet.n2_3way_valve_state = n2_3way_valve_state;
    packet.main_valve_state = main_valve_state;
    packet.nitrogen_valve_state = nitrogen_valve_state;
    packet.chamber_valve_state = chamber_valve_state;
    packet.gmm_state = gmm_state;
    packet.tars1_state = tars1_state;
    packet.tars3_state = tars3_state;
    packet.arming_state = arming_state;
    packet.main_board_state = main_board_state;
    packet.payload_board_state = payload_board_state;
    packet.motor_board_state = motor_board_state;
    packet.main_can_status = main_can_status;
    packet.payload_can_status = payload_can_status;
    packet.motor_can_status = motor_can_status;
    packet.log_good = log_good;

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
 * @param rocket_mass [kg] Rocket mass loadcell measurement
 * @param ox_tank_mass [kg] OX tank mass loadcell measurement
 * @param ox_vessel_mass [kg] OX vessel tank mass loadcell measurement
 * @param ox_filling_pressure [Bar] OX refueling line pressure
 * @param ox_vessel_pressure [Bar] OX vessel tank pressure
 * @param n2_filling_pressure [Bar] N2 refueling line pressure
 * @param n2_vessel_1_pressure [Bar] N2 vessel 1 tank pressure
 * @param n2_vessel_2_pressure [Bar] N2 vessel 2 tank pressure
 * @param ox_filling_valve_state  OX filling valve state (1: open, 0: close)
 * @param ox_release_valve_state  OX release line pressure valve state (1: open, 0: close)
 * @param ox_detach_state  OX quick connector detach state (1: open, 0: close)
 * @param ox_venting_valve_state  OX venting valve state (1: open, 0: close)
 * @param n2_filling_valve_state  N2 filling valve state (1: open, 0: close)
 * @param n2_release_valve_state  N2 release line pressure valve state (1: open, 0: close)
 * @param n2_detach_state  N2 quick connector detach state (1: open, 0: close)
 * @param n2_quenching_valve_state  N2 quenching valve state (1: open, 0: close)
 * @param n2_3way_valve_state  N2 3-way valve state (1: open, 0: close)
 * @param main_valve_state  Rocket main OX valve state (1: open, 0: close)
 * @param nitrogen_valve_state  Rocket main N2 valve state (1: open, 0: close)
 * @param chamber_valve_state  Chamber pressurization valve state (1: enabled, 0: disabled)
 * @param gmm_state  State of the GroundModeManager
 * @param tars1_state  State of TARS 1
 * @param tars3_state  State of TARS 3
 * @param arming_state  Arming state (1: armed, 0: otherwise)
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap memory
 * @param battery_voltage  Battery voltage
 * @param current_consumption [A] RIG current
 * @param umbilical_current_consumption [A] Umbilical current
 * @param main_board_state  Main board FMM state
 * @param payload_board_state  Payload board FMM state
 * @param motor_board_state  Motor board FMM state
 * @param main_can_status  Main CAN status
 * @param payload_can_status  Payload CAN status
 * @param motor_can_status  Motor CAN status
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gse_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float rocket_mass,float ox_tank_mass,float ox_vessel_mass,float ox_filling_pressure,float ox_vessel_pressure,float n2_filling_pressure,float n2_vessel_1_pressure,float n2_vessel_2_pressure,uint8_t ox_filling_valve_state,uint8_t ox_release_valve_state,uint8_t ox_detach_state,uint8_t ox_venting_valve_state,uint8_t n2_filling_valve_state,uint8_t n2_release_valve_state,uint8_t n2_detach_state,uint8_t n2_quenching_valve_state,uint8_t n2_3way_valve_state,uint8_t main_valve_state,uint8_t nitrogen_valve_state,uint8_t chamber_valve_state,uint8_t gmm_state,uint8_t tars1_state,uint8_t tars3_state,uint8_t arming_state,float cpu_load,uint32_t free_heap,float battery_voltage,float current_consumption,float umbilical_current_consumption,uint8_t main_board_state,uint8_t payload_board_state,uint8_t motor_board_state,uint8_t main_can_status,uint8_t payload_can_status,uint8_t motor_can_status,int16_t log_number,uint8_t log_good)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, rocket_mass);
    _mav_put_float(buf, 12, ox_tank_mass);
    _mav_put_float(buf, 16, ox_vessel_mass);
    _mav_put_float(buf, 20, ox_filling_pressure);
    _mav_put_float(buf, 24, ox_vessel_pressure);
    _mav_put_float(buf, 28, n2_filling_pressure);
    _mav_put_float(buf, 32, n2_vessel_1_pressure);
    _mav_put_float(buf, 36, n2_vessel_2_pressure);
    _mav_put_float(buf, 40, cpu_load);
    _mav_put_uint32_t(buf, 44, free_heap);
    _mav_put_float(buf, 48, battery_voltage);
    _mav_put_float(buf, 52, current_consumption);
    _mav_put_float(buf, 56, umbilical_current_consumption);
    _mav_put_int16_t(buf, 60, log_number);
    _mav_put_uint8_t(buf, 62, ox_filling_valve_state);
    _mav_put_uint8_t(buf, 63, ox_release_valve_state);
    _mav_put_uint8_t(buf, 64, ox_detach_state);
    _mav_put_uint8_t(buf, 65, ox_venting_valve_state);
    _mav_put_uint8_t(buf, 66, n2_filling_valve_state);
    _mav_put_uint8_t(buf, 67, n2_release_valve_state);
    _mav_put_uint8_t(buf, 68, n2_detach_state);
    _mav_put_uint8_t(buf, 69, n2_quenching_valve_state);
    _mav_put_uint8_t(buf, 70, n2_3way_valve_state);
    _mav_put_uint8_t(buf, 71, main_valve_state);
    _mav_put_uint8_t(buf, 72, nitrogen_valve_state);
    _mav_put_uint8_t(buf, 73, chamber_valve_state);
    _mav_put_uint8_t(buf, 74, gmm_state);
    _mav_put_uint8_t(buf, 75, tars1_state);
    _mav_put_uint8_t(buf, 76, tars3_state);
    _mav_put_uint8_t(buf, 77, arming_state);
    _mav_put_uint8_t(buf, 78, main_board_state);
    _mav_put_uint8_t(buf, 79, payload_board_state);
    _mav_put_uint8_t(buf, 80, motor_board_state);
    _mav_put_uint8_t(buf, 81, main_can_status);
    _mav_put_uint8_t(buf, 82, payload_can_status);
    _mav_put_uint8_t(buf, 83, motor_can_status);
    _mav_put_uint8_t(buf, 84, log_good);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GSE_TM_LEN);
#else
    mavlink_gse_tm_t packet;
    packet.timestamp = timestamp;
    packet.rocket_mass = rocket_mass;
    packet.ox_tank_mass = ox_tank_mass;
    packet.ox_vessel_mass = ox_vessel_mass;
    packet.ox_filling_pressure = ox_filling_pressure;
    packet.ox_vessel_pressure = ox_vessel_pressure;
    packet.n2_filling_pressure = n2_filling_pressure;
    packet.n2_vessel_1_pressure = n2_vessel_1_pressure;
    packet.n2_vessel_2_pressure = n2_vessel_2_pressure;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;
    packet.battery_voltage = battery_voltage;
    packet.current_consumption = current_consumption;
    packet.umbilical_current_consumption = umbilical_current_consumption;
    packet.log_number = log_number;
    packet.ox_filling_valve_state = ox_filling_valve_state;
    packet.ox_release_valve_state = ox_release_valve_state;
    packet.ox_detach_state = ox_detach_state;
    packet.ox_venting_valve_state = ox_venting_valve_state;
    packet.n2_filling_valve_state = n2_filling_valve_state;
    packet.n2_release_valve_state = n2_release_valve_state;
    packet.n2_detach_state = n2_detach_state;
    packet.n2_quenching_valve_state = n2_quenching_valve_state;
    packet.n2_3way_valve_state = n2_3way_valve_state;
    packet.main_valve_state = main_valve_state;
    packet.nitrogen_valve_state = nitrogen_valve_state;
    packet.chamber_valve_state = chamber_valve_state;
    packet.gmm_state = gmm_state;
    packet.tars1_state = tars1_state;
    packet.tars3_state = tars3_state;
    packet.arming_state = arming_state;
    packet.main_board_state = main_board_state;
    packet.payload_board_state = payload_board_state;
    packet.motor_board_state = motor_board_state;
    packet.main_can_status = main_can_status;
    packet.payload_can_status = payload_can_status;
    packet.motor_can_status = motor_can_status;
    packet.log_good = log_good;

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
    return mavlink_msg_gse_tm_pack(system_id, component_id, msg, gse_tm->timestamp, gse_tm->rocket_mass, gse_tm->ox_tank_mass, gse_tm->ox_vessel_mass, gse_tm->ox_filling_pressure, gse_tm->ox_vessel_pressure, gse_tm->n2_filling_pressure, gse_tm->n2_vessel_1_pressure, gse_tm->n2_vessel_2_pressure, gse_tm->ox_filling_valve_state, gse_tm->ox_release_valve_state, gse_tm->ox_detach_state, gse_tm->ox_venting_valve_state, gse_tm->n2_filling_valve_state, gse_tm->n2_release_valve_state, gse_tm->n2_detach_state, gse_tm->n2_quenching_valve_state, gse_tm->n2_3way_valve_state, gse_tm->main_valve_state, gse_tm->nitrogen_valve_state, gse_tm->chamber_valve_state, gse_tm->gmm_state, gse_tm->tars1_state, gse_tm->tars3_state, gse_tm->arming_state, gse_tm->cpu_load, gse_tm->free_heap, gse_tm->battery_voltage, gse_tm->current_consumption, gse_tm->umbilical_current_consumption, gse_tm->main_board_state, gse_tm->payload_board_state, gse_tm->motor_board_state, gse_tm->main_can_status, gse_tm->payload_can_status, gse_tm->motor_can_status, gse_tm->log_number, gse_tm->log_good);
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
    return mavlink_msg_gse_tm_pack_chan(system_id, component_id, chan, msg, gse_tm->timestamp, gse_tm->rocket_mass, gse_tm->ox_tank_mass, gse_tm->ox_vessel_mass, gse_tm->ox_filling_pressure, gse_tm->ox_vessel_pressure, gse_tm->n2_filling_pressure, gse_tm->n2_vessel_1_pressure, gse_tm->n2_vessel_2_pressure, gse_tm->ox_filling_valve_state, gse_tm->ox_release_valve_state, gse_tm->ox_detach_state, gse_tm->ox_venting_valve_state, gse_tm->n2_filling_valve_state, gse_tm->n2_release_valve_state, gse_tm->n2_detach_state, gse_tm->n2_quenching_valve_state, gse_tm->n2_3way_valve_state, gse_tm->main_valve_state, gse_tm->nitrogen_valve_state, gse_tm->chamber_valve_state, gse_tm->gmm_state, gse_tm->tars1_state, gse_tm->tars3_state, gse_tm->arming_state, gse_tm->cpu_load, gse_tm->free_heap, gse_tm->battery_voltage, gse_tm->current_consumption, gse_tm->umbilical_current_consumption, gse_tm->main_board_state, gse_tm->payload_board_state, gse_tm->motor_board_state, gse_tm->main_can_status, gse_tm->payload_can_status, gse_tm->motor_can_status, gse_tm->log_number, gse_tm->log_good);
}

/**
 * @brief Send a gse_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param rocket_mass [kg] Rocket mass loadcell measurement
 * @param ox_tank_mass [kg] OX tank mass loadcell measurement
 * @param ox_vessel_mass [kg] OX vessel tank mass loadcell measurement
 * @param ox_filling_pressure [Bar] OX refueling line pressure
 * @param ox_vessel_pressure [Bar] OX vessel tank pressure
 * @param n2_filling_pressure [Bar] N2 refueling line pressure
 * @param n2_vessel_1_pressure [Bar] N2 vessel 1 tank pressure
 * @param n2_vessel_2_pressure [Bar] N2 vessel 2 tank pressure
 * @param ox_filling_valve_state  OX filling valve state (1: open, 0: close)
 * @param ox_release_valve_state  OX release line pressure valve state (1: open, 0: close)
 * @param ox_detach_state  OX quick connector detach state (1: open, 0: close)
 * @param ox_venting_valve_state  OX venting valve state (1: open, 0: close)
 * @param n2_filling_valve_state  N2 filling valve state (1: open, 0: close)
 * @param n2_release_valve_state  N2 release line pressure valve state (1: open, 0: close)
 * @param n2_detach_state  N2 quick connector detach state (1: open, 0: close)
 * @param n2_quenching_valve_state  N2 quenching valve state (1: open, 0: close)
 * @param n2_3way_valve_state  N2 3-way valve state (1: open, 0: close)
 * @param main_valve_state  Rocket main OX valve state (1: open, 0: close)
 * @param nitrogen_valve_state  Rocket main N2 valve state (1: open, 0: close)
 * @param chamber_valve_state  Chamber pressurization valve state (1: enabled, 0: disabled)
 * @param gmm_state  State of the GroundModeManager
 * @param tars1_state  State of TARS 1
 * @param tars3_state  State of TARS 3
 * @param arming_state  Arming state (1: armed, 0: otherwise)
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap memory
 * @param battery_voltage  Battery voltage
 * @param current_consumption [A] RIG current
 * @param umbilical_current_consumption [A] Umbilical current
 * @param main_board_state  Main board FMM state
 * @param payload_board_state  Payload board FMM state
 * @param motor_board_state  Motor board FMM state
 * @param main_can_status  Main CAN status
 * @param payload_can_status  Payload CAN status
 * @param motor_can_status  Motor CAN status
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gse_tm_send(mavlink_channel_t chan, uint64_t timestamp, float rocket_mass, float ox_tank_mass, float ox_vessel_mass, float ox_filling_pressure, float ox_vessel_pressure, float n2_filling_pressure, float n2_vessel_1_pressure, float n2_vessel_2_pressure, uint8_t ox_filling_valve_state, uint8_t ox_release_valve_state, uint8_t ox_detach_state, uint8_t ox_venting_valve_state, uint8_t n2_filling_valve_state, uint8_t n2_release_valve_state, uint8_t n2_detach_state, uint8_t n2_quenching_valve_state, uint8_t n2_3way_valve_state, uint8_t main_valve_state, uint8_t nitrogen_valve_state, uint8_t chamber_valve_state, uint8_t gmm_state, uint8_t tars1_state, uint8_t tars3_state, uint8_t arming_state, float cpu_load, uint32_t free_heap, float battery_voltage, float current_consumption, float umbilical_current_consumption, uint8_t main_board_state, uint8_t payload_board_state, uint8_t motor_board_state, uint8_t main_can_status, uint8_t payload_can_status, uint8_t motor_can_status, int16_t log_number, uint8_t log_good)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, rocket_mass);
    _mav_put_float(buf, 12, ox_tank_mass);
    _mav_put_float(buf, 16, ox_vessel_mass);
    _mav_put_float(buf, 20, ox_filling_pressure);
    _mav_put_float(buf, 24, ox_vessel_pressure);
    _mav_put_float(buf, 28, n2_filling_pressure);
    _mav_put_float(buf, 32, n2_vessel_1_pressure);
    _mav_put_float(buf, 36, n2_vessel_2_pressure);
    _mav_put_float(buf, 40, cpu_load);
    _mav_put_uint32_t(buf, 44, free_heap);
    _mav_put_float(buf, 48, battery_voltage);
    _mav_put_float(buf, 52, current_consumption);
    _mav_put_float(buf, 56, umbilical_current_consumption);
    _mav_put_int16_t(buf, 60, log_number);
    _mav_put_uint8_t(buf, 62, ox_filling_valve_state);
    _mav_put_uint8_t(buf, 63, ox_release_valve_state);
    _mav_put_uint8_t(buf, 64, ox_detach_state);
    _mav_put_uint8_t(buf, 65, ox_venting_valve_state);
    _mav_put_uint8_t(buf, 66, n2_filling_valve_state);
    _mav_put_uint8_t(buf, 67, n2_release_valve_state);
    _mav_put_uint8_t(buf, 68, n2_detach_state);
    _mav_put_uint8_t(buf, 69, n2_quenching_valve_state);
    _mav_put_uint8_t(buf, 70, n2_3way_valve_state);
    _mav_put_uint8_t(buf, 71, main_valve_state);
    _mav_put_uint8_t(buf, 72, nitrogen_valve_state);
    _mav_put_uint8_t(buf, 73, chamber_valve_state);
    _mav_put_uint8_t(buf, 74, gmm_state);
    _mav_put_uint8_t(buf, 75, tars1_state);
    _mav_put_uint8_t(buf, 76, tars3_state);
    _mav_put_uint8_t(buf, 77, arming_state);
    _mav_put_uint8_t(buf, 78, main_board_state);
    _mav_put_uint8_t(buf, 79, payload_board_state);
    _mav_put_uint8_t(buf, 80, motor_board_state);
    _mav_put_uint8_t(buf, 81, main_can_status);
    _mav_put_uint8_t(buf, 82, payload_can_status);
    _mav_put_uint8_t(buf, 83, motor_can_status);
    _mav_put_uint8_t(buf, 84, log_good);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSE_TM, buf, MAVLINK_MSG_ID_GSE_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_TM_LEN, MAVLINK_MSG_ID_GSE_TM_CRC);
#else
    mavlink_gse_tm_t packet;
    packet.timestamp = timestamp;
    packet.rocket_mass = rocket_mass;
    packet.ox_tank_mass = ox_tank_mass;
    packet.ox_vessel_mass = ox_vessel_mass;
    packet.ox_filling_pressure = ox_filling_pressure;
    packet.ox_vessel_pressure = ox_vessel_pressure;
    packet.n2_filling_pressure = n2_filling_pressure;
    packet.n2_vessel_1_pressure = n2_vessel_1_pressure;
    packet.n2_vessel_2_pressure = n2_vessel_2_pressure;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;
    packet.battery_voltage = battery_voltage;
    packet.current_consumption = current_consumption;
    packet.umbilical_current_consumption = umbilical_current_consumption;
    packet.log_number = log_number;
    packet.ox_filling_valve_state = ox_filling_valve_state;
    packet.ox_release_valve_state = ox_release_valve_state;
    packet.ox_detach_state = ox_detach_state;
    packet.ox_venting_valve_state = ox_venting_valve_state;
    packet.n2_filling_valve_state = n2_filling_valve_state;
    packet.n2_release_valve_state = n2_release_valve_state;
    packet.n2_detach_state = n2_detach_state;
    packet.n2_quenching_valve_state = n2_quenching_valve_state;
    packet.n2_3way_valve_state = n2_3way_valve_state;
    packet.main_valve_state = main_valve_state;
    packet.nitrogen_valve_state = nitrogen_valve_state;
    packet.chamber_valve_state = chamber_valve_state;
    packet.gmm_state = gmm_state;
    packet.tars1_state = tars1_state;
    packet.tars3_state = tars3_state;
    packet.arming_state = arming_state;
    packet.main_board_state = main_board_state;
    packet.payload_board_state = payload_board_state;
    packet.motor_board_state = motor_board_state;
    packet.main_can_status = main_can_status;
    packet.payload_can_status = payload_can_status;
    packet.motor_can_status = motor_can_status;
    packet.log_good = log_good;

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
    mavlink_msg_gse_tm_send(chan, gse_tm->timestamp, gse_tm->rocket_mass, gse_tm->ox_tank_mass, gse_tm->ox_vessel_mass, gse_tm->ox_filling_pressure, gse_tm->ox_vessel_pressure, gse_tm->n2_filling_pressure, gse_tm->n2_vessel_1_pressure, gse_tm->n2_vessel_2_pressure, gse_tm->ox_filling_valve_state, gse_tm->ox_release_valve_state, gse_tm->ox_detach_state, gse_tm->ox_venting_valve_state, gse_tm->n2_filling_valve_state, gse_tm->n2_release_valve_state, gse_tm->n2_detach_state, gse_tm->n2_quenching_valve_state, gse_tm->n2_3way_valve_state, gse_tm->main_valve_state, gse_tm->nitrogen_valve_state, gse_tm->chamber_valve_state, gse_tm->gmm_state, gse_tm->tars1_state, gse_tm->tars3_state, gse_tm->arming_state, gse_tm->cpu_load, gse_tm->free_heap, gse_tm->battery_voltage, gse_tm->current_consumption, gse_tm->umbilical_current_consumption, gse_tm->main_board_state, gse_tm->payload_board_state, gse_tm->motor_board_state, gse_tm->main_can_status, gse_tm->payload_can_status, gse_tm->motor_can_status, gse_tm->log_number, gse_tm->log_good);
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
static inline void mavlink_msg_gse_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float rocket_mass, float ox_tank_mass, float ox_vessel_mass, float ox_filling_pressure, float ox_vessel_pressure, float n2_filling_pressure, float n2_vessel_1_pressure, float n2_vessel_2_pressure, uint8_t ox_filling_valve_state, uint8_t ox_release_valve_state, uint8_t ox_detach_state, uint8_t ox_venting_valve_state, uint8_t n2_filling_valve_state, uint8_t n2_release_valve_state, uint8_t n2_detach_state, uint8_t n2_quenching_valve_state, uint8_t n2_3way_valve_state, uint8_t main_valve_state, uint8_t nitrogen_valve_state, uint8_t chamber_valve_state, uint8_t gmm_state, uint8_t tars1_state, uint8_t tars3_state, uint8_t arming_state, float cpu_load, uint32_t free_heap, float battery_voltage, float current_consumption, float umbilical_current_consumption, uint8_t main_board_state, uint8_t payload_board_state, uint8_t motor_board_state, uint8_t main_can_status, uint8_t payload_can_status, uint8_t motor_can_status, int16_t log_number, uint8_t log_good)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, rocket_mass);
    _mav_put_float(buf, 12, ox_tank_mass);
    _mav_put_float(buf, 16, ox_vessel_mass);
    _mav_put_float(buf, 20, ox_filling_pressure);
    _mav_put_float(buf, 24, ox_vessel_pressure);
    _mav_put_float(buf, 28, n2_filling_pressure);
    _mav_put_float(buf, 32, n2_vessel_1_pressure);
    _mav_put_float(buf, 36, n2_vessel_2_pressure);
    _mav_put_float(buf, 40, cpu_load);
    _mav_put_uint32_t(buf, 44, free_heap);
    _mav_put_float(buf, 48, battery_voltage);
    _mav_put_float(buf, 52, current_consumption);
    _mav_put_float(buf, 56, umbilical_current_consumption);
    _mav_put_int16_t(buf, 60, log_number);
    _mav_put_uint8_t(buf, 62, ox_filling_valve_state);
    _mav_put_uint8_t(buf, 63, ox_release_valve_state);
    _mav_put_uint8_t(buf, 64, ox_detach_state);
    _mav_put_uint8_t(buf, 65, ox_venting_valve_state);
    _mav_put_uint8_t(buf, 66, n2_filling_valve_state);
    _mav_put_uint8_t(buf, 67, n2_release_valve_state);
    _mav_put_uint8_t(buf, 68, n2_detach_state);
    _mav_put_uint8_t(buf, 69, n2_quenching_valve_state);
    _mav_put_uint8_t(buf, 70, n2_3way_valve_state);
    _mav_put_uint8_t(buf, 71, main_valve_state);
    _mav_put_uint8_t(buf, 72, nitrogen_valve_state);
    _mav_put_uint8_t(buf, 73, chamber_valve_state);
    _mav_put_uint8_t(buf, 74, gmm_state);
    _mav_put_uint8_t(buf, 75, tars1_state);
    _mav_put_uint8_t(buf, 76, tars3_state);
    _mav_put_uint8_t(buf, 77, arming_state);
    _mav_put_uint8_t(buf, 78, main_board_state);
    _mav_put_uint8_t(buf, 79, payload_board_state);
    _mav_put_uint8_t(buf, 80, motor_board_state);
    _mav_put_uint8_t(buf, 81, main_can_status);
    _mav_put_uint8_t(buf, 82, payload_can_status);
    _mav_put_uint8_t(buf, 83, motor_can_status);
    _mav_put_uint8_t(buf, 84, log_good);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSE_TM, buf, MAVLINK_MSG_ID_GSE_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_TM_LEN, MAVLINK_MSG_ID_GSE_TM_CRC);
#else
    mavlink_gse_tm_t *packet = (mavlink_gse_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->rocket_mass = rocket_mass;
    packet->ox_tank_mass = ox_tank_mass;
    packet->ox_vessel_mass = ox_vessel_mass;
    packet->ox_filling_pressure = ox_filling_pressure;
    packet->ox_vessel_pressure = ox_vessel_pressure;
    packet->n2_filling_pressure = n2_filling_pressure;
    packet->n2_vessel_1_pressure = n2_vessel_1_pressure;
    packet->n2_vessel_2_pressure = n2_vessel_2_pressure;
    packet->cpu_load = cpu_load;
    packet->free_heap = free_heap;
    packet->battery_voltage = battery_voltage;
    packet->current_consumption = current_consumption;
    packet->umbilical_current_consumption = umbilical_current_consumption;
    packet->log_number = log_number;
    packet->ox_filling_valve_state = ox_filling_valve_state;
    packet->ox_release_valve_state = ox_release_valve_state;
    packet->ox_detach_state = ox_detach_state;
    packet->ox_venting_valve_state = ox_venting_valve_state;
    packet->n2_filling_valve_state = n2_filling_valve_state;
    packet->n2_release_valve_state = n2_release_valve_state;
    packet->n2_detach_state = n2_detach_state;
    packet->n2_quenching_valve_state = n2_quenching_valve_state;
    packet->n2_3way_valve_state = n2_3way_valve_state;
    packet->main_valve_state = main_valve_state;
    packet->nitrogen_valve_state = nitrogen_valve_state;
    packet->chamber_valve_state = chamber_valve_state;
    packet->gmm_state = gmm_state;
    packet->tars1_state = tars1_state;
    packet->tars3_state = tars3_state;
    packet->arming_state = arming_state;
    packet->main_board_state = main_board_state;
    packet->payload_board_state = payload_board_state;
    packet->motor_board_state = motor_board_state;
    packet->main_can_status = main_can_status;
    packet->payload_can_status = payload_can_status;
    packet->motor_can_status = motor_can_status;
    packet->log_good = log_good;

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
 * @brief Get field rocket_mass from gse_tm message
 *
 * @return [kg] Rocket mass loadcell measurement
 */
static inline float mavlink_msg_gse_tm_get_rocket_mass(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field ox_tank_mass from gse_tm message
 *
 * @return [kg] OX tank mass loadcell measurement
 */
static inline float mavlink_msg_gse_tm_get_ox_tank_mass(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field ox_vessel_mass from gse_tm message
 *
 * @return [kg] OX vessel tank mass loadcell measurement
 */
static inline float mavlink_msg_gse_tm_get_ox_vessel_mass(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field ox_filling_pressure from gse_tm message
 *
 * @return [Bar] OX refueling line pressure
 */
static inline float mavlink_msg_gse_tm_get_ox_filling_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ox_vessel_pressure from gse_tm message
 *
 * @return [Bar] OX vessel tank pressure
 */
static inline float mavlink_msg_gse_tm_get_ox_vessel_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field n2_filling_pressure from gse_tm message
 *
 * @return [Bar] N2 refueling line pressure
 */
static inline float mavlink_msg_gse_tm_get_n2_filling_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field n2_vessel_1_pressure from gse_tm message
 *
 * @return [Bar] N2 vessel 1 tank pressure
 */
static inline float mavlink_msg_gse_tm_get_n2_vessel_1_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field n2_vessel_2_pressure from gse_tm message
 *
 * @return [Bar] N2 vessel 2 tank pressure
 */
static inline float mavlink_msg_gse_tm_get_n2_vessel_2_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field ox_filling_valve_state from gse_tm message
 *
 * @return  OX filling valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_gse_tm_get_ox_filling_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  62);
}

/**
 * @brief Get field ox_release_valve_state from gse_tm message
 *
 * @return  OX release line pressure valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_gse_tm_get_ox_release_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  63);
}

/**
 * @brief Get field ox_detach_state from gse_tm message
 *
 * @return  OX quick connector detach state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_gse_tm_get_ox_detach_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  64);
}

/**
 * @brief Get field ox_venting_valve_state from gse_tm message
 *
 * @return  OX venting valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_gse_tm_get_ox_venting_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  65);
}

/**
 * @brief Get field n2_filling_valve_state from gse_tm message
 *
 * @return  N2 filling valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_gse_tm_get_n2_filling_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  66);
}

/**
 * @brief Get field n2_release_valve_state from gse_tm message
 *
 * @return  N2 release line pressure valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_gse_tm_get_n2_release_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  67);
}

/**
 * @brief Get field n2_detach_state from gse_tm message
 *
 * @return  N2 quick connector detach state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_gse_tm_get_n2_detach_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  68);
}

/**
 * @brief Get field n2_quenching_valve_state from gse_tm message
 *
 * @return  N2 quenching valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_gse_tm_get_n2_quenching_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  69);
}

/**
 * @brief Get field n2_3way_valve_state from gse_tm message
 *
 * @return  N2 3-way valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_gse_tm_get_n2_3way_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  70);
}

/**
 * @brief Get field main_valve_state from gse_tm message
 *
 * @return  Rocket main OX valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_gse_tm_get_main_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  71);
}

/**
 * @brief Get field nitrogen_valve_state from gse_tm message
 *
 * @return  Rocket main N2 valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_gse_tm_get_nitrogen_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  72);
}

/**
 * @brief Get field chamber_valve_state from gse_tm message
 *
 * @return  Chamber pressurization valve state (1: enabled, 0: disabled)
 */
static inline uint8_t mavlink_msg_gse_tm_get_chamber_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  73);
}

/**
 * @brief Get field gmm_state from gse_tm message
 *
 * @return  State of the GroundModeManager
 */
static inline uint8_t mavlink_msg_gse_tm_get_gmm_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  74);
}

/**
 * @brief Get field tars1_state from gse_tm message
 *
 * @return  State of TARS 1
 */
static inline uint8_t mavlink_msg_gse_tm_get_tars1_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  75);
}

/**
 * @brief Get field tars3_state from gse_tm message
 *
 * @return  State of TARS 3
 */
static inline uint8_t mavlink_msg_gse_tm_get_tars3_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  76);
}

/**
 * @brief Get field arming_state from gse_tm message
 *
 * @return  Arming state (1: armed, 0: otherwise)
 */
static inline uint8_t mavlink_msg_gse_tm_get_arming_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  77);
}

/**
 * @brief Get field cpu_load from gse_tm message
 *
 * @return  CPU load in percentage
 */
static inline float mavlink_msg_gse_tm_get_cpu_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field free_heap from gse_tm message
 *
 * @return  Amount of available heap memory
 */
static inline uint32_t mavlink_msg_gse_tm_get_free_heap(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  44);
}

/**
 * @brief Get field battery_voltage from gse_tm message
 *
 * @return  Battery voltage
 */
static inline float mavlink_msg_gse_tm_get_battery_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field current_consumption from gse_tm message
 *
 * @return [A] RIG current
 */
static inline float mavlink_msg_gse_tm_get_current_consumption(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field umbilical_current_consumption from gse_tm message
 *
 * @return [A] Umbilical current
 */
static inline float mavlink_msg_gse_tm_get_umbilical_current_consumption(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field main_board_state from gse_tm message
 *
 * @return  Main board FMM state
 */
static inline uint8_t mavlink_msg_gse_tm_get_main_board_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  78);
}

/**
 * @brief Get field payload_board_state from gse_tm message
 *
 * @return  Payload board FMM state
 */
static inline uint8_t mavlink_msg_gse_tm_get_payload_board_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  79);
}

/**
 * @brief Get field motor_board_state from gse_tm message
 *
 * @return  Motor board FMM state
 */
static inline uint8_t mavlink_msg_gse_tm_get_motor_board_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  80);
}

/**
 * @brief Get field main_can_status from gse_tm message
 *
 * @return  Main CAN status
 */
static inline uint8_t mavlink_msg_gse_tm_get_main_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  81);
}

/**
 * @brief Get field payload_can_status from gse_tm message
 *
 * @return  Payload CAN status
 */
static inline uint8_t mavlink_msg_gse_tm_get_payload_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  82);
}

/**
 * @brief Get field motor_can_status from gse_tm message
 *
 * @return  Motor CAN status
 */
static inline uint8_t mavlink_msg_gse_tm_get_motor_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  83);
}

/**
 * @brief Get field log_number from gse_tm message
 *
 * @return  Currently active log file, -1 if the logger is inactive
 */
static inline int16_t mavlink_msg_gse_tm_get_log_number(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  60);
}

/**
 * @brief Get field log_good from gse_tm message
 *
 * @return  0 if log failed, 1 otherwise
 */
static inline uint8_t mavlink_msg_gse_tm_get_log_good(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  84);
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
    gse_tm->rocket_mass = mavlink_msg_gse_tm_get_rocket_mass(msg);
    gse_tm->ox_tank_mass = mavlink_msg_gse_tm_get_ox_tank_mass(msg);
    gse_tm->ox_vessel_mass = mavlink_msg_gse_tm_get_ox_vessel_mass(msg);
    gse_tm->ox_filling_pressure = mavlink_msg_gse_tm_get_ox_filling_pressure(msg);
    gse_tm->ox_vessel_pressure = mavlink_msg_gse_tm_get_ox_vessel_pressure(msg);
    gse_tm->n2_filling_pressure = mavlink_msg_gse_tm_get_n2_filling_pressure(msg);
    gse_tm->n2_vessel_1_pressure = mavlink_msg_gse_tm_get_n2_vessel_1_pressure(msg);
    gse_tm->n2_vessel_2_pressure = mavlink_msg_gse_tm_get_n2_vessel_2_pressure(msg);
    gse_tm->cpu_load = mavlink_msg_gse_tm_get_cpu_load(msg);
    gse_tm->free_heap = mavlink_msg_gse_tm_get_free_heap(msg);
    gse_tm->battery_voltage = mavlink_msg_gse_tm_get_battery_voltage(msg);
    gse_tm->current_consumption = mavlink_msg_gse_tm_get_current_consumption(msg);
    gse_tm->umbilical_current_consumption = mavlink_msg_gse_tm_get_umbilical_current_consumption(msg);
    gse_tm->log_number = mavlink_msg_gse_tm_get_log_number(msg);
    gse_tm->ox_filling_valve_state = mavlink_msg_gse_tm_get_ox_filling_valve_state(msg);
    gse_tm->ox_release_valve_state = mavlink_msg_gse_tm_get_ox_release_valve_state(msg);
    gse_tm->ox_detach_state = mavlink_msg_gse_tm_get_ox_detach_state(msg);
    gse_tm->ox_venting_valve_state = mavlink_msg_gse_tm_get_ox_venting_valve_state(msg);
    gse_tm->n2_filling_valve_state = mavlink_msg_gse_tm_get_n2_filling_valve_state(msg);
    gse_tm->n2_release_valve_state = mavlink_msg_gse_tm_get_n2_release_valve_state(msg);
    gse_tm->n2_detach_state = mavlink_msg_gse_tm_get_n2_detach_state(msg);
    gse_tm->n2_quenching_valve_state = mavlink_msg_gse_tm_get_n2_quenching_valve_state(msg);
    gse_tm->n2_3way_valve_state = mavlink_msg_gse_tm_get_n2_3way_valve_state(msg);
    gse_tm->main_valve_state = mavlink_msg_gse_tm_get_main_valve_state(msg);
    gse_tm->nitrogen_valve_state = mavlink_msg_gse_tm_get_nitrogen_valve_state(msg);
    gse_tm->chamber_valve_state = mavlink_msg_gse_tm_get_chamber_valve_state(msg);
    gse_tm->gmm_state = mavlink_msg_gse_tm_get_gmm_state(msg);
    gse_tm->tars1_state = mavlink_msg_gse_tm_get_tars1_state(msg);
    gse_tm->tars3_state = mavlink_msg_gse_tm_get_tars3_state(msg);
    gse_tm->arming_state = mavlink_msg_gse_tm_get_arming_state(msg);
    gse_tm->main_board_state = mavlink_msg_gse_tm_get_main_board_state(msg);
    gse_tm->payload_board_state = mavlink_msg_gse_tm_get_payload_board_state(msg);
    gse_tm->motor_board_state = mavlink_msg_gse_tm_get_motor_board_state(msg);
    gse_tm->main_can_status = mavlink_msg_gse_tm_get_main_can_status(msg);
    gse_tm->payload_can_status = mavlink_msg_gse_tm_get_payload_can_status(msg);
    gse_tm->motor_can_status = mavlink_msg_gse_tm_get_motor_can_status(msg);
    gse_tm->log_good = mavlink_msg_gse_tm_get_log_good(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GSE_TM_LEN? msg->len : MAVLINK_MSG_ID_GSE_TM_LEN;
        memset(gse_tm, 0, MAVLINK_MSG_ID_GSE_TM_LEN);
    memcpy(gse_tm, _MAV_PAYLOAD(msg), len);
#endif
}
