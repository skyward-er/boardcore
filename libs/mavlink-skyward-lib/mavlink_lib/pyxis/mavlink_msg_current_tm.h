#pragma once
// MESSAGE CURRENT_TM PACKING

#define MAVLINK_MSG_ID_CURRENT_TM 107


typedef struct __mavlink_current_tm_t {
 uint64_t timestamp; /*< [us] When was this logged*/
 float current; /*< [A] Current*/
 char sensor_id[20]; /*<  Sensor name*/
} mavlink_current_tm_t;

#define MAVLINK_MSG_ID_CURRENT_TM_LEN 32
#define MAVLINK_MSG_ID_CURRENT_TM_MIN_LEN 32
#define MAVLINK_MSG_ID_107_LEN 32
#define MAVLINK_MSG_ID_107_MIN_LEN 32

#define MAVLINK_MSG_ID_CURRENT_TM_CRC 222
#define MAVLINK_MSG_ID_107_CRC 222

#define MAVLINK_MSG_CURRENT_TM_FIELD_SENSOR_ID_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CURRENT_TM { \
    107, \
    "CURRENT_TM", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_current_tm_t, timestamp) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_CHAR, 20, 12, offsetof(mavlink_current_tm_t, sensor_id) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_current_tm_t, current) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CURRENT_TM { \
    "CURRENT_TM", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_current_tm_t, timestamp) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_CHAR, 20, 12, offsetof(mavlink_current_tm_t, sensor_id) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_current_tm_t, current) }, \
         } \
}
#endif

/**
 * @brief Pack a current_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] When was this logged
 * @param sensor_id  Sensor name
 * @param current [A] Current
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_current_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, const char *sensor_id, float current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRENT_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, current);
    _mav_put_char_array(buf, 12, sensor_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CURRENT_TM_LEN);
#else
    mavlink_current_tm_t packet;
    packet.timestamp = timestamp;
    packet.current = current;
    mav_array_memcpy(packet.sensor_id, sensor_id, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CURRENT_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CURRENT_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CURRENT_TM_MIN_LEN, MAVLINK_MSG_ID_CURRENT_TM_LEN, MAVLINK_MSG_ID_CURRENT_TM_CRC);
}

/**
 * @brief Pack a current_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] When was this logged
 * @param sensor_id  Sensor name
 * @param current [A] Current
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_current_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,const char *sensor_id,float current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRENT_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, current);
    _mav_put_char_array(buf, 12, sensor_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CURRENT_TM_LEN);
#else
    mavlink_current_tm_t packet;
    packet.timestamp = timestamp;
    packet.current = current;
    mav_array_memcpy(packet.sensor_id, sensor_id, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CURRENT_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CURRENT_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CURRENT_TM_MIN_LEN, MAVLINK_MSG_ID_CURRENT_TM_LEN, MAVLINK_MSG_ID_CURRENT_TM_CRC);
}

/**
 * @brief Encode a current_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param current_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_current_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_current_tm_t* current_tm)
{
    return mavlink_msg_current_tm_pack(system_id, component_id, msg, current_tm->timestamp, current_tm->sensor_id, current_tm->current);
}

/**
 * @brief Encode a current_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param current_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_current_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_current_tm_t* current_tm)
{
    return mavlink_msg_current_tm_pack_chan(system_id, component_id, chan, msg, current_tm->timestamp, current_tm->sensor_id, current_tm->current);
}

/**
 * @brief Send a current_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged
 * @param sensor_id  Sensor name
 * @param current [A] Current
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_current_tm_send(mavlink_channel_t chan, uint64_t timestamp, const char *sensor_id, float current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRENT_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, current);
    _mav_put_char_array(buf, 12, sensor_id, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_TM, buf, MAVLINK_MSG_ID_CURRENT_TM_MIN_LEN, MAVLINK_MSG_ID_CURRENT_TM_LEN, MAVLINK_MSG_ID_CURRENT_TM_CRC);
#else
    mavlink_current_tm_t packet;
    packet.timestamp = timestamp;
    packet.current = current;
    mav_array_memcpy(packet.sensor_id, sensor_id, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_TM, (const char *)&packet, MAVLINK_MSG_ID_CURRENT_TM_MIN_LEN, MAVLINK_MSG_ID_CURRENT_TM_LEN, MAVLINK_MSG_ID_CURRENT_TM_CRC);
#endif
}

/**
 * @brief Send a current_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_current_tm_send_struct(mavlink_channel_t chan, const mavlink_current_tm_t* current_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_current_tm_send(chan, current_tm->timestamp, current_tm->sensor_id, current_tm->current);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_TM, (const char *)current_tm, MAVLINK_MSG_ID_CURRENT_TM_MIN_LEN, MAVLINK_MSG_ID_CURRENT_TM_LEN, MAVLINK_MSG_ID_CURRENT_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_CURRENT_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_current_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, const char *sensor_id, float current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, current);
    _mav_put_char_array(buf, 12, sensor_id, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_TM, buf, MAVLINK_MSG_ID_CURRENT_TM_MIN_LEN, MAVLINK_MSG_ID_CURRENT_TM_LEN, MAVLINK_MSG_ID_CURRENT_TM_CRC);
#else
    mavlink_current_tm_t *packet = (mavlink_current_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->current = current;
    mav_array_memcpy(packet->sensor_id, sensor_id, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_TM, (const char *)packet, MAVLINK_MSG_ID_CURRENT_TM_MIN_LEN, MAVLINK_MSG_ID_CURRENT_TM_LEN, MAVLINK_MSG_ID_CURRENT_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE CURRENT_TM UNPACKING


/**
 * @brief Get field timestamp from current_tm message
 *
 * @return [us] When was this logged
 */
static inline uint64_t mavlink_msg_current_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sensor_id from current_tm message
 *
 * @return  Sensor name
 */
static inline uint16_t mavlink_msg_current_tm_get_sensor_id(const mavlink_message_t* msg, char *sensor_id)
{
    return _MAV_RETURN_char_array(msg, sensor_id, 20,  12);
}

/**
 * @brief Get field current from current_tm message
 *
 * @return [A] Current
 */
static inline float mavlink_msg_current_tm_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a current_tm message into a struct
 *
 * @param msg The message to decode
 * @param current_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_current_tm_decode(const mavlink_message_t* msg, mavlink_current_tm_t* current_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    current_tm->timestamp = mavlink_msg_current_tm_get_timestamp(msg);
    current_tm->current = mavlink_msg_current_tm_get_current(msg);
    mavlink_msg_current_tm_get_sensor_id(msg, current_tm->sensor_id);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CURRENT_TM_LEN? msg->len : MAVLINK_MSG_ID_CURRENT_TM_LEN;
        memset(current_tm, 0, MAVLINK_MSG_ID_CURRENT_TM_LEN);
    memcpy(current_tm, _MAV_PAYLOAD(msg), len);
#endif
}
