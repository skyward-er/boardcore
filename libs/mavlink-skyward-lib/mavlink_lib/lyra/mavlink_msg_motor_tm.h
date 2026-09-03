#pragma once
// MESSAGE MOTOR_TM PACKING

#define MAVLINK_MSG_ID_MOTOR_TM 213


typedef struct __mavlink_motor_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 float top_tank_pressure; /*< [Bar] Tank upper pressure*/
 float bottom_tank_pressure; /*< [Bar] Tank bottom pressure*/
 float combustion_chamber_pressure; /*< [Bar] Pressure inside the combustion chamber used for automatic shutdown*/
 float tank_temperature; /*<  Tank temperature*/
 float battery_voltage; /*< [V] Battery voltage*/
 int32_t log_good; /*<  0 if log failed, 1 otherwise*/
 int16_t log_number; /*<  Currently active log file, -1 if the logger is inactive*/
 uint8_t main_valve_state; /*<  1 If the main valve is open */
 uint8_t venting_valve_state; /*<  1 If the venting valve is open */
 uint8_t hil_state; /*<  1 if the board is currently in HIL mode*/
} mavlink_motor_tm_t;

#define MAVLINK_MSG_ID_MOTOR_TM_LEN 37
#define MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN 37
#define MAVLINK_MSG_ID_213_LEN 37
#define MAVLINK_MSG_ID_213_MIN_LEN 37

#define MAVLINK_MSG_ID_MOTOR_TM_CRC 67
#define MAVLINK_MSG_ID_213_CRC 67



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOTOR_TM { \
    213, \
    "MOTOR_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_motor_tm_t, timestamp) }, \
         { "top_tank_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_tm_t, top_tank_pressure) }, \
         { "bottom_tank_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_motor_tm_t, bottom_tank_pressure) }, \
         { "combustion_chamber_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_motor_tm_t, combustion_chamber_pressure) }, \
         { "tank_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_motor_tm_t, tank_temperature) }, \
         { "main_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_motor_tm_t, main_valve_state) }, \
         { "venting_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_motor_tm_t, venting_valve_state) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_motor_tm_t, battery_voltage) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_motor_tm_t, log_number) }, \
         { "log_good", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_motor_tm_t, log_good) }, \
         { "hil_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_motor_tm_t, hil_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOTOR_TM { \
    "MOTOR_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_motor_tm_t, timestamp) }, \
         { "top_tank_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_tm_t, top_tank_pressure) }, \
         { "bottom_tank_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_motor_tm_t, bottom_tank_pressure) }, \
         { "combustion_chamber_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_motor_tm_t, combustion_chamber_pressure) }, \
         { "tank_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_motor_tm_t, tank_temperature) }, \
         { "main_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_motor_tm_t, main_valve_state) }, \
         { "venting_valve_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_motor_tm_t, venting_valve_state) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_motor_tm_t, battery_voltage) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_motor_tm_t, log_number) }, \
         { "log_good", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_motor_tm_t, log_good) }, \
         { "hil_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_motor_tm_t, hil_state) }, \
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
 * @param top_tank_pressure [Bar] Tank upper pressure
 * @param bottom_tank_pressure [Bar] Tank bottom pressure
 * @param combustion_chamber_pressure [Bar] Pressure inside the combustion chamber used for automatic shutdown
 * @param tank_temperature  Tank temperature
 * @param main_valve_state  1 If the main valve is open 
 * @param venting_valve_state  1 If the venting valve is open 
 * @param battery_voltage [V] Battery voltage
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @param hil_state  1 if the board is currently in HIL mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float top_tank_pressure, float bottom_tank_pressure, float combustion_chamber_pressure, float tank_temperature, uint8_t main_valve_state, uint8_t venting_valve_state, float battery_voltage, int16_t log_number, int32_t log_good, uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, top_tank_pressure);
    _mav_put_float(buf, 12, bottom_tank_pressure);
    _mav_put_float(buf, 16, combustion_chamber_pressure);
    _mav_put_float(buf, 20, tank_temperature);
    _mav_put_float(buf, 24, battery_voltage);
    _mav_put_int32_t(buf, 28, log_good);
    _mav_put_int16_t(buf, 32, log_number);
    _mav_put_uint8_t(buf, 34, main_valve_state);
    _mav_put_uint8_t(buf, 35, venting_valve_state);
    _mav_put_uint8_t(buf, 36, hil_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_TM_LEN);
#else
    mavlink_motor_tm_t packet;
    packet.timestamp = timestamp;
    packet.top_tank_pressure = top_tank_pressure;
    packet.bottom_tank_pressure = bottom_tank_pressure;
    packet.combustion_chamber_pressure = combustion_chamber_pressure;
    packet.tank_temperature = tank_temperature;
    packet.battery_voltage = battery_voltage;
    packet.log_good = log_good;
    packet.log_number = log_number;
    packet.main_valve_state = main_valve_state;
    packet.venting_valve_state = venting_valve_state;
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
 * @param top_tank_pressure [Bar] Tank upper pressure
 * @param bottom_tank_pressure [Bar] Tank bottom pressure
 * @param combustion_chamber_pressure [Bar] Pressure inside the combustion chamber used for automatic shutdown
 * @param tank_temperature  Tank temperature
 * @param main_valve_state  1 If the main valve is open 
 * @param venting_valve_state  1 If the venting valve is open 
 * @param battery_voltage [V] Battery voltage
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @param hil_state  1 if the board is currently in HIL mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float top_tank_pressure,float bottom_tank_pressure,float combustion_chamber_pressure,float tank_temperature,uint8_t main_valve_state,uint8_t venting_valve_state,float battery_voltage,int16_t log_number,int32_t log_good,uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, top_tank_pressure);
    _mav_put_float(buf, 12, bottom_tank_pressure);
    _mav_put_float(buf, 16, combustion_chamber_pressure);
    _mav_put_float(buf, 20, tank_temperature);
    _mav_put_float(buf, 24, battery_voltage);
    _mav_put_int32_t(buf, 28, log_good);
    _mav_put_int16_t(buf, 32, log_number);
    _mav_put_uint8_t(buf, 34, main_valve_state);
    _mav_put_uint8_t(buf, 35, venting_valve_state);
    _mav_put_uint8_t(buf, 36, hil_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_TM_LEN);
#else
    mavlink_motor_tm_t packet;
    packet.timestamp = timestamp;
    packet.top_tank_pressure = top_tank_pressure;
    packet.bottom_tank_pressure = bottom_tank_pressure;
    packet.combustion_chamber_pressure = combustion_chamber_pressure;
    packet.tank_temperature = tank_temperature;
    packet.battery_voltage = battery_voltage;
    packet.log_good = log_good;
    packet.log_number = log_number;
    packet.main_valve_state = main_valve_state;
    packet.venting_valve_state = venting_valve_state;
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
    return mavlink_msg_motor_tm_pack(system_id, component_id, msg, motor_tm->timestamp, motor_tm->top_tank_pressure, motor_tm->bottom_tank_pressure, motor_tm->combustion_chamber_pressure, motor_tm->tank_temperature, motor_tm->main_valve_state, motor_tm->venting_valve_state, motor_tm->battery_voltage, motor_tm->log_number, motor_tm->log_good, motor_tm->hil_state);
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
    return mavlink_msg_motor_tm_pack_chan(system_id, component_id, chan, msg, motor_tm->timestamp, motor_tm->top_tank_pressure, motor_tm->bottom_tank_pressure, motor_tm->combustion_chamber_pressure, motor_tm->tank_temperature, motor_tm->main_valve_state, motor_tm->venting_valve_state, motor_tm->battery_voltage, motor_tm->log_number, motor_tm->log_good, motor_tm->hil_state);
}

/**
 * @brief Send a motor_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param top_tank_pressure [Bar] Tank upper pressure
 * @param bottom_tank_pressure [Bar] Tank bottom pressure
 * @param combustion_chamber_pressure [Bar] Pressure inside the combustion chamber used for automatic shutdown
 * @param tank_temperature  Tank temperature
 * @param main_valve_state  1 If the main valve is open 
 * @param venting_valve_state  1 If the venting valve is open 
 * @param battery_voltage [V] Battery voltage
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param log_good  0 if log failed, 1 otherwise
 * @param hil_state  1 if the board is currently in HIL mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_tm_send(mavlink_channel_t chan, uint64_t timestamp, float top_tank_pressure, float bottom_tank_pressure, float combustion_chamber_pressure, float tank_temperature, uint8_t main_valve_state, uint8_t venting_valve_state, float battery_voltage, int16_t log_number, int32_t log_good, uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, top_tank_pressure);
    _mav_put_float(buf, 12, bottom_tank_pressure);
    _mav_put_float(buf, 16, combustion_chamber_pressure);
    _mav_put_float(buf, 20, tank_temperature);
    _mav_put_float(buf, 24, battery_voltage);
    _mav_put_int32_t(buf, 28, log_good);
    _mav_put_int16_t(buf, 32, log_number);
    _mav_put_uint8_t(buf, 34, main_valve_state);
    _mav_put_uint8_t(buf, 35, venting_valve_state);
    _mav_put_uint8_t(buf, 36, hil_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_TM, buf, MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN, MAVLINK_MSG_ID_MOTOR_TM_LEN, MAVLINK_MSG_ID_MOTOR_TM_CRC);
#else
    mavlink_motor_tm_t packet;
    packet.timestamp = timestamp;
    packet.top_tank_pressure = top_tank_pressure;
    packet.bottom_tank_pressure = bottom_tank_pressure;
    packet.combustion_chamber_pressure = combustion_chamber_pressure;
    packet.tank_temperature = tank_temperature;
    packet.battery_voltage = battery_voltage;
    packet.log_good = log_good;
    packet.log_number = log_number;
    packet.main_valve_state = main_valve_state;
    packet.venting_valve_state = venting_valve_state;
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
    mavlink_msg_motor_tm_send(chan, motor_tm->timestamp, motor_tm->top_tank_pressure, motor_tm->bottom_tank_pressure, motor_tm->combustion_chamber_pressure, motor_tm->tank_temperature, motor_tm->main_valve_state, motor_tm->venting_valve_state, motor_tm->battery_voltage, motor_tm->log_number, motor_tm->log_good, motor_tm->hil_state);
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
static inline void mavlink_msg_motor_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float top_tank_pressure, float bottom_tank_pressure, float combustion_chamber_pressure, float tank_temperature, uint8_t main_valve_state, uint8_t venting_valve_state, float battery_voltage, int16_t log_number, int32_t log_good, uint8_t hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, top_tank_pressure);
    _mav_put_float(buf, 12, bottom_tank_pressure);
    _mav_put_float(buf, 16, combustion_chamber_pressure);
    _mav_put_float(buf, 20, tank_temperature);
    _mav_put_float(buf, 24, battery_voltage);
    _mav_put_int32_t(buf, 28, log_good);
    _mav_put_int16_t(buf, 32, log_number);
    _mav_put_uint8_t(buf, 34, main_valve_state);
    _mav_put_uint8_t(buf, 35, venting_valve_state);
    _mav_put_uint8_t(buf, 36, hil_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_TM, buf, MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN, MAVLINK_MSG_ID_MOTOR_TM_LEN, MAVLINK_MSG_ID_MOTOR_TM_CRC);
#else
    mavlink_motor_tm_t *packet = (mavlink_motor_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->top_tank_pressure = top_tank_pressure;
    packet->bottom_tank_pressure = bottom_tank_pressure;
    packet->combustion_chamber_pressure = combustion_chamber_pressure;
    packet->tank_temperature = tank_temperature;
    packet->battery_voltage = battery_voltage;
    packet->log_good = log_good;
    packet->log_number = log_number;
    packet->main_valve_state = main_valve_state;
    packet->venting_valve_state = venting_valve_state;
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
 * @brief Get field top_tank_pressure from motor_tm message
 *
 * @return [Bar] Tank upper pressure
 */
static inline float mavlink_msg_motor_tm_get_top_tank_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field bottom_tank_pressure from motor_tm message
 *
 * @return [Bar] Tank bottom pressure
 */
static inline float mavlink_msg_motor_tm_get_bottom_tank_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field combustion_chamber_pressure from motor_tm message
 *
 * @return [Bar] Pressure inside the combustion chamber used for automatic shutdown
 */
static inline float mavlink_msg_motor_tm_get_combustion_chamber_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field tank_temperature from motor_tm message
 *
 * @return  Tank temperature
 */
static inline float mavlink_msg_motor_tm_get_tank_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field main_valve_state from motor_tm message
 *
 * @return  1 If the main valve is open 
 */
static inline uint8_t mavlink_msg_motor_tm_get_main_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field venting_valve_state from motor_tm message
 *
 * @return  1 If the venting valve is open 
 */
static inline uint8_t mavlink_msg_motor_tm_get_venting_valve_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field battery_voltage from motor_tm message
 *
 * @return [V] Battery voltage
 */
static inline float mavlink_msg_motor_tm_get_battery_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field log_number from motor_tm message
 *
 * @return  Currently active log file, -1 if the logger is inactive
 */
static inline int16_t mavlink_msg_motor_tm_get_log_number(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field log_good from motor_tm message
 *
 * @return  0 if log failed, 1 otherwise
 */
static inline int32_t mavlink_msg_motor_tm_get_log_good(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field hil_state from motor_tm message
 *
 * @return  1 if the board is currently in HIL mode
 */
static inline uint8_t mavlink_msg_motor_tm_get_hil_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
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
    motor_tm->top_tank_pressure = mavlink_msg_motor_tm_get_top_tank_pressure(msg);
    motor_tm->bottom_tank_pressure = mavlink_msg_motor_tm_get_bottom_tank_pressure(msg);
    motor_tm->combustion_chamber_pressure = mavlink_msg_motor_tm_get_combustion_chamber_pressure(msg);
    motor_tm->tank_temperature = mavlink_msg_motor_tm_get_tank_temperature(msg);
    motor_tm->battery_voltage = mavlink_msg_motor_tm_get_battery_voltage(msg);
    motor_tm->log_good = mavlink_msg_motor_tm_get_log_good(msg);
    motor_tm->log_number = mavlink_msg_motor_tm_get_log_number(msg);
    motor_tm->main_valve_state = mavlink_msg_motor_tm_get_main_valve_state(msg);
    motor_tm->venting_valve_state = mavlink_msg_motor_tm_get_venting_valve_state(msg);
    motor_tm->hil_state = mavlink_msg_motor_tm_get_hil_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOTOR_TM_LEN? msg->len : MAVLINK_MSG_ID_MOTOR_TM_LEN;
        memset(motor_tm, 0, MAVLINK_MSG_ID_MOTOR_TM_LEN);
    memcpy(motor_tm, _MAV_PAYLOAD(msg), len);
#endif
}
