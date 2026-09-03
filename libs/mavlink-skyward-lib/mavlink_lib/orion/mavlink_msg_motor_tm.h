#pragma once
// MESSAGE MOTOR_TM PACKING

#define MAVLINK_MSG_ID_MOTOR_TM 213


typedef struct __mavlink_motor_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 float n2_tank_pressure; /*< [Bar] N2 tank pressure*/
 float reg_out_pressure; /*< [Bar] Regulator out pressure*/
 float ox_tank_top_pressure; /*< [Bar] OX tank top pressure*/
 float ox_tank_bot_0_pressure; /*< [Bar] OX tank bottom 0 pressure*/
 float ox_tank_bot_1_pressure; /*< [Bar] OX tank bottom 1 pressure*/
 float combustion_chamber_pressure; /*< [Bar] Pressure inside the combustion chamber*/
 float thermocouple_temperature; /*<  Thermocouple temperature*/
 float battery_voltage; /*< [V] Battery voltage*/
 float current_consumption; /*< [A] Current consumption*/
 int16_t log_number; /*<  Currently active log file, -1 if the logger is inactive*/
 uint8_t n2_quenching_valve_state; /*<  N2 quenching valve state (1: open, 0: close)*/
 uint8_t ox_venting_valve_state; /*<  OX venting valve state (1: open, 0: close)*/
 uint8_t nitrogen_valve_state; /*<  Rocket main N2 valve state (1: open, 0: close)*/
 uint8_t main_valve_state; /*<  Rocket main OX valve state (1: open, 0: close)*/
 uint8_t log_good; /*<  0 if log failed, 1 otherwise*/
 uint8_t hil_state; /*<  1 if the board is currently in HIL mode*/
} mavlink_motor_tm_t;

#define MAVLINK_MSG_ID_MOTOR_TM_LEN 52
#define MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN 52
#define MAVLINK_MSG_ID_213_LEN 52
#define MAVLINK_MSG_ID_213_MIN_LEN 52

#define MAVLINK_MSG_ID_MOTOR_TM_CRC 34
#define MAVLINK_MSG_ID_213_CRC 34



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOTOR_TM { \
    213, \
    "MOTOR_TM", \
    17, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_motor_tm_t, timestamp) }, \
         { "n2_tank_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_tm_t, n2_tank_pressure) }, \
         { "reg_out_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_motor_tm_t, reg_out_pressure) }, \
         { "ox_tank_top_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_motor_tm_t, ox_tank_top_pressure) }, \
         { "ox_tank_bot_0_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_motor_tm_t, ox_tank_bot_0_pressure) }, \
         { "ox_tank_bot_1_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_motor_tm_t, ox_tank_bot_1_pressure) }, \
         { "combustion_chamber_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_motor_tm_t, combustion_chamber_pressure) }, \
         { "thermocouple_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_motor_tm_t, thermocouple_temperature) }, \
         { "n2_quenching_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_motor_tm_t, n2_quenching_valve_state) }, \
         { "ox_venting_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 47, offsetof(mavlink_motor_tm_t, ox_venting_valve_state) }, \
         { "nitrogen_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_motor_tm_t, nitrogen_valve_state) }, \
         { "main_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_motor_tm_t, main_valve_state) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_motor_tm_t, battery_voltage) }, \
         { "current_consumption", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_motor_tm_t, current_consumption) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_motor_tm_t, log_number) }, \
         { "log_good", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_motor_tm_t, log_good) }, \
         { "hil_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_motor_tm_t, hil_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOTOR_TM { \
    "MOTOR_TM", \
    17, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_motor_tm_t, timestamp) }, \
         { "n2_tank_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_tm_t, n2_tank_pressure) }, \
         { "reg_out_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_motor_tm_t, reg_out_pressure) }, \
         { "ox_tank_top_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_motor_tm_t, ox_tank_top_pressure) }, \
         { "ox_tank_bot_0_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_motor_tm_t, ox_tank_bot_0_pressure) }, \
         { "ox_tank_bot_1_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_motor_tm_t, ox_tank_bot_1_pressure) }, \
         { "combustion_chamber_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_motor_tm_t, combustion_chamber_pressure) }, \
         { "thermocouple_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_motor_tm_t, thermocouple_temperature) }, \
         { "n2_quenching_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_motor_tm_t, n2_quenching_valve_state) }, \
         { "ox_venting_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 47, offsetof(mavlink_motor_tm_t, ox_venting_valve_state) }, \
         { "nitrogen_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_motor_tm_t, nitrogen_valve_state) }, \
         { "main_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_motor_tm_t, main_valve_state) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_motor_tm_t, battery_voltage) }, \
         { "current_consumption", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_motor_tm_t, current_consumption) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_motor_tm_t, log_number) }, \
         { "log_good", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_motor_tm_t, log_good) }, \
         { "hil_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_motor_tm_t, hil_state) }, \
         } \
}
#endif

/**
 * @brief Pack a motor_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param n2_tank_pressure [Bar] N2 tank pressure
 * @param reg_out_pressure [Bar] Regulator out pressure
 * @param ox_tank_top_pressure [Bar] OX tank top pressure
 * @param ox_tank_bot_0_pressure [Bar] OX tank bottom 0 pressure
 * @param ox_tank_bot_1_pressure [Bar] OX tank bottom 1 pressure
 * @param combustion_chamber_pressure [Bar] Pressure inside the combustion chamber
 * @param thermocouple_temperature  Thermocouple temperature
 * @param n2_quenching_valve_state  N2 quenching valve state (1: open, 0: close)
 * @param ox_venting_valve_state  OX venting valve state (1: open, 0: close)
 * @param nitrogen_valve_state  Rocket main N2 valve state (1: open, 0: close)
 * @param main_valve_state  Rocket main OX valve state (1: open, 0: close)
 * @param battery_voltage [V] Battery voltage
 * @param current_consumption [A] Current consumption
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @param hil_state  1 if the board is currently in HIL mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float n2_tank_pressure, float reg_out_pressure, float ox_tank_top_pressure, float ox_tank_bot_0_pressure, float ox_tank_bot_1_pressure, float combustion_chamber_pressure, float thermocouple_temperature, uint8_t n2_quenching_valve_state, uint8_t ox_venting_valve_state, uint8_t nitrogen_valve_state, uint8_t main_valve_state, float battery_voltage, float current_consumption, int16_t log_number, uint8_t log_good, uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, n2_tank_pressure);
    _mav_put_float(buf, 12, reg_out_pressure);
    _mav_put_float(buf, 16, ox_tank_top_pressure);
    _mav_put_float(buf, 20, ox_tank_bot_0_pressure);
    _mav_put_float(buf, 24, ox_tank_bot_1_pressure);
    _mav_put_float(buf, 28, combustion_chamber_pressure);
    _mav_put_float(buf, 32, thermocouple_temperature);
    _mav_put_float(buf, 36, battery_voltage);
    _mav_put_float(buf, 40, current_consumption);
    _mav_put_int16_t(buf, 44, log_number);
    _mav_put_uint8_t(buf, 46, n2_quenching_valve_state);
    _mav_put_uint8_t(buf, 47, ox_venting_valve_state);
    _mav_put_uint8_t(buf, 48, nitrogen_valve_state);
    _mav_put_uint8_t(buf, 49, main_valve_state);
    _mav_put_uint8_t(buf, 50, log_good);
    _mav_put_uint8_t(buf, 51, hil_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_TM_LEN);
#else
    mavlink_motor_tm_t packet;
    packet.timestamp = timestamp;
    packet.n2_tank_pressure = n2_tank_pressure;
    packet.reg_out_pressure = reg_out_pressure;
    packet.ox_tank_top_pressure = ox_tank_top_pressure;
    packet.ox_tank_bot_0_pressure = ox_tank_bot_0_pressure;
    packet.ox_tank_bot_1_pressure = ox_tank_bot_1_pressure;
    packet.combustion_chamber_pressure = combustion_chamber_pressure;
    packet.thermocouple_temperature = thermocouple_temperature;
    packet.battery_voltage = battery_voltage;
    packet.current_consumption = current_consumption;
    packet.log_number = log_number;
    packet.n2_quenching_valve_state = n2_quenching_valve_state;
    packet.ox_venting_valve_state = ox_venting_valve_state;
    packet.nitrogen_valve_state = nitrogen_valve_state;
    packet.main_valve_state = main_valve_state;
    packet.log_good = log_good;
    packet.hil_state = hil_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN, MAVLINK_MSG_ID_MOTOR_TM_LEN, MAVLINK_MSG_ID_MOTOR_TM_CRC);
}

/**
 * @brief Pack a motor_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp in microseconds
 * @param n2_tank_pressure [Bar] N2 tank pressure
 * @param reg_out_pressure [Bar] Regulator out pressure
 * @param ox_tank_top_pressure [Bar] OX tank top pressure
 * @param ox_tank_bot_0_pressure [Bar] OX tank bottom 0 pressure
 * @param ox_tank_bot_1_pressure [Bar] OX tank bottom 1 pressure
 * @param combustion_chamber_pressure [Bar] Pressure inside the combustion chamber
 * @param thermocouple_temperature  Thermocouple temperature
 * @param n2_quenching_valve_state  N2 quenching valve state (1: open, 0: close)
 * @param ox_venting_valve_state  OX venting valve state (1: open, 0: close)
 * @param nitrogen_valve_state  Rocket main N2 valve state (1: open, 0: close)
 * @param main_valve_state  Rocket main OX valve state (1: open, 0: close)
 * @param battery_voltage [V] Battery voltage
 * @param current_consumption [A] Current consumption
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @param hil_state  1 if the board is currently in HIL mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float n2_tank_pressure,float reg_out_pressure,float ox_tank_top_pressure,float ox_tank_bot_0_pressure,float ox_tank_bot_1_pressure,float combustion_chamber_pressure,float thermocouple_temperature,uint8_t n2_quenching_valve_state,uint8_t ox_venting_valve_state,uint8_t nitrogen_valve_state,uint8_t main_valve_state,float battery_voltage,float current_consumption,int16_t log_number,uint8_t log_good,uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, n2_tank_pressure);
    _mav_put_float(buf, 12, reg_out_pressure);
    _mav_put_float(buf, 16, ox_tank_top_pressure);
    _mav_put_float(buf, 20, ox_tank_bot_0_pressure);
    _mav_put_float(buf, 24, ox_tank_bot_1_pressure);
    _mav_put_float(buf, 28, combustion_chamber_pressure);
    _mav_put_float(buf, 32, thermocouple_temperature);
    _mav_put_float(buf, 36, battery_voltage);
    _mav_put_float(buf, 40, current_consumption);
    _mav_put_int16_t(buf, 44, log_number);
    _mav_put_uint8_t(buf, 46, n2_quenching_valve_state);
    _mav_put_uint8_t(buf, 47, ox_venting_valve_state);
    _mav_put_uint8_t(buf, 48, nitrogen_valve_state);
    _mav_put_uint8_t(buf, 49, main_valve_state);
    _mav_put_uint8_t(buf, 50, log_good);
    _mav_put_uint8_t(buf, 51, hil_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_TM_LEN);
#else
    mavlink_motor_tm_t packet;
    packet.timestamp = timestamp;
    packet.n2_tank_pressure = n2_tank_pressure;
    packet.reg_out_pressure = reg_out_pressure;
    packet.ox_tank_top_pressure = ox_tank_top_pressure;
    packet.ox_tank_bot_0_pressure = ox_tank_bot_0_pressure;
    packet.ox_tank_bot_1_pressure = ox_tank_bot_1_pressure;
    packet.combustion_chamber_pressure = combustion_chamber_pressure;
    packet.thermocouple_temperature = thermocouple_temperature;
    packet.battery_voltage = battery_voltage;
    packet.current_consumption = current_consumption;
    packet.log_number = log_number;
    packet.n2_quenching_valve_state = n2_quenching_valve_state;
    packet.ox_venting_valve_state = ox_venting_valve_state;
    packet.nitrogen_valve_state = nitrogen_valve_state;
    packet.main_valve_state = main_valve_state;
    packet.log_good = log_good;
    packet.hil_state = hil_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN, MAVLINK_MSG_ID_MOTOR_TM_LEN, MAVLINK_MSG_ID_MOTOR_TM_CRC);
}

/**
 * @brief Encode a motor_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_tm_t* motor_tm)
{
    return mavlink_msg_motor_tm_pack(system_id, component_id, msg, motor_tm->timestamp, motor_tm->n2_tank_pressure, motor_tm->reg_out_pressure, motor_tm->ox_tank_top_pressure, motor_tm->ox_tank_bot_0_pressure, motor_tm->ox_tank_bot_1_pressure, motor_tm->combustion_chamber_pressure, motor_tm->thermocouple_temperature, motor_tm->n2_quenching_valve_state, motor_tm->ox_venting_valve_state, motor_tm->nitrogen_valve_state, motor_tm->main_valve_state, motor_tm->battery_voltage, motor_tm->current_consumption, motor_tm->log_number, motor_tm->log_good, motor_tm->hil_state);
}

/**
 * @brief Encode a motor_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_tm_t* motor_tm)
{
    return mavlink_msg_motor_tm_pack_chan(system_id, component_id, chan, msg, motor_tm->timestamp, motor_tm->n2_tank_pressure, motor_tm->reg_out_pressure, motor_tm->ox_tank_top_pressure, motor_tm->ox_tank_bot_0_pressure, motor_tm->ox_tank_bot_1_pressure, motor_tm->combustion_chamber_pressure, motor_tm->thermocouple_temperature, motor_tm->n2_quenching_valve_state, motor_tm->ox_venting_valve_state, motor_tm->nitrogen_valve_state, motor_tm->main_valve_state, motor_tm->battery_voltage, motor_tm->current_consumption, motor_tm->log_number, motor_tm->log_good, motor_tm->hil_state);
}

/**
 * @brief Send a motor_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param n2_tank_pressure [Bar] N2 tank pressure
 * @param reg_out_pressure [Bar] Regulator out pressure
 * @param ox_tank_top_pressure [Bar] OX tank top pressure
 * @param ox_tank_bot_0_pressure [Bar] OX tank bottom 0 pressure
 * @param ox_tank_bot_1_pressure [Bar] OX tank bottom 1 pressure
 * @param combustion_chamber_pressure [Bar] Pressure inside the combustion chamber
 * @param thermocouple_temperature  Thermocouple temperature
 * @param n2_quenching_valve_state  N2 quenching valve state (1: open, 0: close)
 * @param ox_venting_valve_state  OX venting valve state (1: open, 0: close)
 * @param nitrogen_valve_state  Rocket main N2 valve state (1: open, 0: close)
 * @param main_valve_state  Rocket main OX valve state (1: open, 0: close)
 * @param battery_voltage [V] Battery voltage
 * @param current_consumption [A] Current consumption
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @param hil_state  1 if the board is currently in HIL mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_tm_send(mavlink_channel_t chan, uint64_t timestamp, float n2_tank_pressure, float reg_out_pressure, float ox_tank_top_pressure, float ox_tank_bot_0_pressure, float ox_tank_bot_1_pressure, float combustion_chamber_pressure, float thermocouple_temperature, uint8_t n2_quenching_valve_state, uint8_t ox_venting_valve_state, uint8_t nitrogen_valve_state, uint8_t main_valve_state, float battery_voltage, float current_consumption, int16_t log_number, uint8_t log_good, uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, n2_tank_pressure);
    _mav_put_float(buf, 12, reg_out_pressure);
    _mav_put_float(buf, 16, ox_tank_top_pressure);
    _mav_put_float(buf, 20, ox_tank_bot_0_pressure);
    _mav_put_float(buf, 24, ox_tank_bot_1_pressure);
    _mav_put_float(buf, 28, combustion_chamber_pressure);
    _mav_put_float(buf, 32, thermocouple_temperature);
    _mav_put_float(buf, 36, battery_voltage);
    _mav_put_float(buf, 40, current_consumption);
    _mav_put_int16_t(buf, 44, log_number);
    _mav_put_uint8_t(buf, 46, n2_quenching_valve_state);
    _mav_put_uint8_t(buf, 47, ox_venting_valve_state);
    _mav_put_uint8_t(buf, 48, nitrogen_valve_state);
    _mav_put_uint8_t(buf, 49, main_valve_state);
    _mav_put_uint8_t(buf, 50, log_good);
    _mav_put_uint8_t(buf, 51, hil_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_TM, buf, MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN, MAVLINK_MSG_ID_MOTOR_TM_LEN, MAVLINK_MSG_ID_MOTOR_TM_CRC);
#else
    mavlink_motor_tm_t packet;
    packet.timestamp = timestamp;
    packet.n2_tank_pressure = n2_tank_pressure;
    packet.reg_out_pressure = reg_out_pressure;
    packet.ox_tank_top_pressure = ox_tank_top_pressure;
    packet.ox_tank_bot_0_pressure = ox_tank_bot_0_pressure;
    packet.ox_tank_bot_1_pressure = ox_tank_bot_1_pressure;
    packet.combustion_chamber_pressure = combustion_chamber_pressure;
    packet.thermocouple_temperature = thermocouple_temperature;
    packet.battery_voltage = battery_voltage;
    packet.current_consumption = current_consumption;
    packet.log_number = log_number;
    packet.n2_quenching_valve_state = n2_quenching_valve_state;
    packet.ox_venting_valve_state = ox_venting_valve_state;
    packet.nitrogen_valve_state = nitrogen_valve_state;
    packet.main_valve_state = main_valve_state;
    packet.log_good = log_good;
    packet.hil_state = hil_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_TM, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN, MAVLINK_MSG_ID_MOTOR_TM_LEN, MAVLINK_MSG_ID_MOTOR_TM_CRC);
#endif
}

/**
 * @brief Send a motor_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_motor_tm_send_struct(mavlink_channel_t chan, const mavlink_motor_tm_t* motor_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_motor_tm_send(chan, motor_tm->timestamp, motor_tm->n2_tank_pressure, motor_tm->reg_out_pressure, motor_tm->ox_tank_top_pressure, motor_tm->ox_tank_bot_0_pressure, motor_tm->ox_tank_bot_1_pressure, motor_tm->combustion_chamber_pressure, motor_tm->thermocouple_temperature, motor_tm->n2_quenching_valve_state, motor_tm->ox_venting_valve_state, motor_tm->nitrogen_valve_state, motor_tm->main_valve_state, motor_tm->battery_voltage, motor_tm->current_consumption, motor_tm->log_number, motor_tm->log_good, motor_tm->hil_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_TM, (const char *)motor_tm, MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN, MAVLINK_MSG_ID_MOTOR_TM_LEN, MAVLINK_MSG_ID_MOTOR_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOTOR_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float n2_tank_pressure, float reg_out_pressure, float ox_tank_top_pressure, float ox_tank_bot_0_pressure, float ox_tank_bot_1_pressure, float combustion_chamber_pressure, float thermocouple_temperature, uint8_t n2_quenching_valve_state, uint8_t ox_venting_valve_state, uint8_t nitrogen_valve_state, uint8_t main_valve_state, float battery_voltage, float current_consumption, int16_t log_number, uint8_t log_good, uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, n2_tank_pressure);
    _mav_put_float(buf, 12, reg_out_pressure);
    _mav_put_float(buf, 16, ox_tank_top_pressure);
    _mav_put_float(buf, 20, ox_tank_bot_0_pressure);
    _mav_put_float(buf, 24, ox_tank_bot_1_pressure);
    _mav_put_float(buf, 28, combustion_chamber_pressure);
    _mav_put_float(buf, 32, thermocouple_temperature);
    _mav_put_float(buf, 36, battery_voltage);
    _mav_put_float(buf, 40, current_consumption);
    _mav_put_int16_t(buf, 44, log_number);
    _mav_put_uint8_t(buf, 46, n2_quenching_valve_state);
    _mav_put_uint8_t(buf, 47, ox_venting_valve_state);
    _mav_put_uint8_t(buf, 48, nitrogen_valve_state);
    _mav_put_uint8_t(buf, 49, main_valve_state);
    _mav_put_uint8_t(buf, 50, log_good);
    _mav_put_uint8_t(buf, 51, hil_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_TM, buf, MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN, MAVLINK_MSG_ID_MOTOR_TM_LEN, MAVLINK_MSG_ID_MOTOR_TM_CRC);
#else
    mavlink_motor_tm_t *packet = (mavlink_motor_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->n2_tank_pressure = n2_tank_pressure;
    packet->reg_out_pressure = reg_out_pressure;
    packet->ox_tank_top_pressure = ox_tank_top_pressure;
    packet->ox_tank_bot_0_pressure = ox_tank_bot_0_pressure;
    packet->ox_tank_bot_1_pressure = ox_tank_bot_1_pressure;
    packet->combustion_chamber_pressure = combustion_chamber_pressure;
    packet->thermocouple_temperature = thermocouple_temperature;
    packet->battery_voltage = battery_voltage;
    packet->current_consumption = current_consumption;
    packet->log_number = log_number;
    packet->n2_quenching_valve_state = n2_quenching_valve_state;
    packet->ox_venting_valve_state = ox_venting_valve_state;
    packet->nitrogen_valve_state = nitrogen_valve_state;
    packet->main_valve_state = main_valve_state;
    packet->log_good = log_good;
    packet->hil_state = hil_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_TM, (const char *)packet, MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN, MAVLINK_MSG_ID_MOTOR_TM_LEN, MAVLINK_MSG_ID_MOTOR_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE MOTOR_TM UNPACKING


/**
 * @brief Get field timestamp from motor_tm message
 *
 * @return [us] Timestamp in microseconds
 */
static inline uint64_t mavlink_msg_motor_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field n2_tank_pressure from motor_tm message
 *
 * @return [Bar] N2 tank pressure
 */
static inline float mavlink_msg_motor_tm_get_n2_tank_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field reg_out_pressure from motor_tm message
 *
 * @return [Bar] Regulator out pressure
 */
static inline float mavlink_msg_motor_tm_get_reg_out_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field ox_tank_top_pressure from motor_tm message
 *
 * @return [Bar] OX tank top pressure
 */
static inline float mavlink_msg_motor_tm_get_ox_tank_top_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field ox_tank_bot_0_pressure from motor_tm message
 *
 * @return [Bar] OX tank bottom 0 pressure
 */
static inline float mavlink_msg_motor_tm_get_ox_tank_bot_0_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ox_tank_bot_1_pressure from motor_tm message
 *
 * @return [Bar] OX tank bottom 1 pressure
 */
static inline float mavlink_msg_motor_tm_get_ox_tank_bot_1_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field combustion_chamber_pressure from motor_tm message
 *
 * @return [Bar] Pressure inside the combustion chamber
 */
static inline float mavlink_msg_motor_tm_get_combustion_chamber_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field thermocouple_temperature from motor_tm message
 *
 * @return  Thermocouple temperature
 */
static inline float mavlink_msg_motor_tm_get_thermocouple_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field n2_quenching_valve_state from motor_tm message
 *
 * @return  N2 quenching valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_motor_tm_get_n2_quenching_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field ox_venting_valve_state from motor_tm message
 *
 * @return  OX venting valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_motor_tm_get_ox_venting_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  47);
}

/**
 * @brief Get field nitrogen_valve_state from motor_tm message
 *
 * @return  Rocket main N2 valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_motor_tm_get_nitrogen_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field main_valve_state from motor_tm message
 *
 * @return  Rocket main OX valve state (1: open, 0: close)
 */
static inline uint8_t mavlink_msg_motor_tm_get_main_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field battery_voltage from motor_tm message
 *
 * @return [V] Battery voltage
 */
static inline float mavlink_msg_motor_tm_get_battery_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field current_consumption from motor_tm message
 *
 * @return [A] Current consumption
 */
static inline float mavlink_msg_motor_tm_get_current_consumption(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field log_number from motor_tm message
 *
 * @return  Currently active log file, -1 if the logger is inactive
 */
static inline int16_t mavlink_msg_motor_tm_get_log_number(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  44);
}

/**
 * @brief Get field log_good from motor_tm message
 *
 * @return  0 if log failed, 1 otherwise
 */
static inline uint8_t mavlink_msg_motor_tm_get_log_good(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field hil_state from motor_tm message
 *
 * @return  1 if the board is currently in HIL mode
 */
static inline uint8_t mavlink_msg_motor_tm_get_hil_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Decode a motor_tm message into a struct
 *
 * @param msg The message to decode
 * @param motor_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_tm_decode(const mavlink_message_t* msg, mavlink_motor_tm_t* motor_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    motor_tm->timestamp = mavlink_msg_motor_tm_get_timestamp(msg);
    motor_tm->n2_tank_pressure = mavlink_msg_motor_tm_get_n2_tank_pressure(msg);
    motor_tm->reg_out_pressure = mavlink_msg_motor_tm_get_reg_out_pressure(msg);
    motor_tm->ox_tank_top_pressure = mavlink_msg_motor_tm_get_ox_tank_top_pressure(msg);
    motor_tm->ox_tank_bot_0_pressure = mavlink_msg_motor_tm_get_ox_tank_bot_0_pressure(msg);
    motor_tm->ox_tank_bot_1_pressure = mavlink_msg_motor_tm_get_ox_tank_bot_1_pressure(msg);
    motor_tm->combustion_chamber_pressure = mavlink_msg_motor_tm_get_combustion_chamber_pressure(msg);
    motor_tm->thermocouple_temperature = mavlink_msg_motor_tm_get_thermocouple_temperature(msg);
    motor_tm->battery_voltage = mavlink_msg_motor_tm_get_battery_voltage(msg);
    motor_tm->current_consumption = mavlink_msg_motor_tm_get_current_consumption(msg);
    motor_tm->log_number = mavlink_msg_motor_tm_get_log_number(msg);
    motor_tm->n2_quenching_valve_state = mavlink_msg_motor_tm_get_n2_quenching_valve_state(msg);
    motor_tm->ox_venting_valve_state = mavlink_msg_motor_tm_get_ox_venting_valve_state(msg);
    motor_tm->nitrogen_valve_state = mavlink_msg_motor_tm_get_nitrogen_valve_state(msg);
    motor_tm->main_valve_state = mavlink_msg_motor_tm_get_main_valve_state(msg);
    motor_tm->log_good = mavlink_msg_motor_tm_get_log_good(msg);
    motor_tm->hil_state = mavlink_msg_motor_tm_get_hil_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOTOR_TM_LEN? msg->len : MAVLINK_MSG_ID_MOTOR_TM_LEN;
        memset(motor_tm, 0, MAVLINK_MSG_ID_MOTOR_TM_LEN);
    memcpy(motor_tm, _MAV_PAYLOAD(msg), len);
#endif
}
