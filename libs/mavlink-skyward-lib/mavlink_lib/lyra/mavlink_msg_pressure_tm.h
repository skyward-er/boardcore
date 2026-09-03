#pragma once
// MESSAGE PRESSURE_TM PACKING

#define MAVLINK_MSG_ID_PRESSURE_TM 105


typedef struct __mavlink_pressure_tm_t {
 uint64_t timestamp; /*< [us] When was this logged*/
 float pressure; /*< [Pa] Pressure of the digital barometer*/
 char sensor_name[20]; /*<  Sensor name*/
} mavlink_pressure_tm_t;

#define MAVLINK_MSG_ID_PRESSURE_TM_LEN 32
#define MAVLINK_MSG_ID_PRESSURE_TM_MIN_LEN 32
#define MAVLINK_MSG_ID_105_LEN 32
#define MAVLINK_MSG_ID_105_MIN_LEN 32

#define MAVLINK_MSG_ID_PRESSURE_TM_CRC 87
#define MAVLINK_MSG_ID_105_CRC 87

#define MAVLINK_MSG_PRESSURE_TM_FIELD_SENSOR_NAME_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PRESSURE_TM { \
    105, \
    "PRESSURE_TM", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_pressure_tm_t, timestamp) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 20, 12, offsetof(mavlink_pressure_tm_t, sensor_name) }, \
         { "pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_pressure_tm_t, pressure) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PRESSURE_TM { \
    "PRESSURE_TM", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_pressure_tm_t, timestamp) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 20, 12, offsetof(mavlink_pressure_tm_t, sensor_name) }, \
         { "pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_pressure_tm_t, pressure) }, \
         } \
}
#endif

/**
 * @brief Pack a pressure_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] When was this logged
 * @param sensor_name  Sensor name
 * @param pressure [Pa] Pressure of the digital barometer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pressure_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, const char *sensor_name, float pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PRESSURE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure);
    _mav_put_char_array(buf, 12, sensor_name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PRESSURE_TM_LEN);
#else
    mavlink_pressure_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure = pressure;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PRESSURE_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PRESSURE_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PRESSURE_TM_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_TM_LEN, MAVLINK_MSG_ID_PRESSURE_TM_CRC);
}

/**
 * @brief Pack a pressure_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] When was this logged
 * @param sensor_name  Sensor name
 * @param pressure [Pa] Pressure of the digital barometer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pressure_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,const char *sensor_name,float pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PRESSURE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure);
    _mav_put_char_array(buf, 12, sensor_name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PRESSURE_TM_LEN);
#else
    mavlink_pressure_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure = pressure;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PRESSURE_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PRESSURE_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PRESSURE_TM_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_TM_LEN, MAVLINK_MSG_ID_PRESSURE_TM_CRC);
}

/**
 * @brief Encode a pressure_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pressure_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pressure_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pressure_tm_t* pressure_tm)
{
    return mavlink_msg_pressure_tm_pack(system_id, component_id, msg, pressure_tm->timestamp, pressure_tm->sensor_name, pressure_tm->pressure);
}

/**
 * @brief Encode a pressure_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pressure_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pressure_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pressure_tm_t* pressure_tm)
{
    return mavlink_msg_pressure_tm_pack_chan(system_id, component_id, chan, msg, pressure_tm->timestamp, pressure_tm->sensor_name, pressure_tm->pressure);
}

/**
 * @brief Send a pressure_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged
 * @param sensor_name  Sensor name
 * @param pressure [Pa] Pressure of the digital barometer
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pressure_tm_send(mavlink_channel_t chan, uint64_t timestamp, const char *sensor_name, float pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PRESSURE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure);
    _mav_put_char_array(buf, 12, sensor_name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE_TM, buf, MAVLINK_MSG_ID_PRESSURE_TM_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_TM_LEN, MAVLINK_MSG_ID_PRESSURE_TM_CRC);
#else
    mavlink_pressure_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure = pressure;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE_TM, (const char *)&packet, MAVLINK_MSG_ID_PRESSURE_TM_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_TM_LEN, MAVLINK_MSG_ID_PRESSURE_TM_CRC);
#endif
}

/**
 * @brief Send a pressure_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_pressure_tm_send_struct(mavlink_channel_t chan, const mavlink_pressure_tm_t* pressure_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pressure_tm_send(chan, pressure_tm->timestamp, pressure_tm->sensor_name, pressure_tm->pressure);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE_TM, (const char *)pressure_tm, MAVLINK_MSG_ID_PRESSURE_TM_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_TM_LEN, MAVLINK_MSG_ID_PRESSURE_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_PRESSURE_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pressure_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, const char *sensor_name, float pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure);
    _mav_put_char_array(buf, 12, sensor_name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE_TM, buf, MAVLINK_MSG_ID_PRESSURE_TM_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_TM_LEN, MAVLINK_MSG_ID_PRESSURE_TM_CRC);
#else
    mavlink_pressure_tm_t *packet = (mavlink_pressure_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->pressure = pressure;
    mav_array_memcpy(packet->sensor_name, sensor_name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE_TM, (const char *)packet, MAVLINK_MSG_ID_PRESSURE_TM_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_TM_LEN, MAVLINK_MSG_ID_PRESSURE_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE PRESSURE_TM UNPACKING


/**
 * @brief Get field timestamp from pressure_tm message
 *
 * @return [us] When was this logged
 */
static inline uint64_t mavlink_msg_pressure_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sensor_name from pressure_tm message
 *
 * @return  Sensor name
 */
static inline uint16_t mavlink_msg_pressure_tm_get_sensor_name(const mavlink_message_t* msg, char *sensor_name)
{
    return _MAV_RETURN_char_array(msg, sensor_name, 20,  12);
}

/**
 * @brief Get field pressure from pressure_tm message
 *
 * @return [Pa] Pressure of the digital barometer
 */
static inline float mavlink_msg_pressure_tm_get_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a pressure_tm message into a struct
 *
 * @param msg The message to decode
 * @param pressure_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_pressure_tm_decode(const mavlink_message_t* msg, mavlink_pressure_tm_t* pressure_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    pressure_tm->timestamp = mavlink_msg_pressure_tm_get_timestamp(msg);
    pressure_tm->pressure = mavlink_msg_pressure_tm_get_pressure(msg);
    mavlink_msg_pressure_tm_get_sensor_name(msg, pressure_tm->sensor_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PRESSURE_TM_LEN? msg->len : MAVLINK_MSG_ID_PRESSURE_TM_LEN;
        memset(pressure_tm, 0, MAVLINK_MSG_ID_PRESSURE_TM_LEN);
    memcpy(pressure_tm, _MAV_PAYLOAD(msg), len);
#endif
}
