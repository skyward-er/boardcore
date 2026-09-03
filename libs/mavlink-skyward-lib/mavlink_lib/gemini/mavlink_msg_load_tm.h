#pragma once
// MESSAGE LOAD_TM PACKING

#define MAVLINK_MSG_ID_LOAD_TM 109


typedef struct __mavlink_load_tm_t {
 uint64_t timestamp; /*< [us] When was this logged*/
 float load; /*< [kg] Load force*/
 char sensor_name[20]; /*<  Sensor name*/
} mavlink_load_tm_t;

#define MAVLINK_MSG_ID_LOAD_TM_LEN 32
#define MAVLINK_MSG_ID_LOAD_TM_MIN_LEN 32
#define MAVLINK_MSG_ID_109_LEN 32
#define MAVLINK_MSG_ID_109_MIN_LEN 32

#define MAVLINK_MSG_ID_LOAD_TM_CRC 148
#define MAVLINK_MSG_ID_109_CRC 148

#define MAVLINK_MSG_LOAD_TM_FIELD_SENSOR_NAME_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LOAD_TM { \
    109, \
    "LOAD_TM", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_load_tm_t, timestamp) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 20, 12, offsetof(mavlink_load_tm_t, sensor_name) }, \
         { "load", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_load_tm_t, load) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LOAD_TM { \
    "LOAD_TM", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_load_tm_t, timestamp) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 20, 12, offsetof(mavlink_load_tm_t, sensor_name) }, \
         { "load", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_load_tm_t, load) }, \
         } \
}
#endif

/**
 * @brief Pack a load_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] When was this logged
 * @param sensor_name  Sensor name
 * @param load [kg] Load force
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_load_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, const char *sensor_name, float load)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOAD_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, load);
    _mav_put_char_array(buf, 12, sensor_name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOAD_TM_LEN);
#else
    mavlink_load_tm_t packet;
    packet.timestamp = timestamp;
    packet.load = load;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOAD_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOAD_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOAD_TM_MIN_LEN, MAVLINK_MSG_ID_LOAD_TM_LEN, MAVLINK_MSG_ID_LOAD_TM_CRC);
}

/**
 * @brief Pack a load_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] When was this logged
 * @param sensor_name  Sensor name
 * @param load [kg] Load force
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_load_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,const char *sensor_name,float load)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOAD_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, load);
    _mav_put_char_array(buf, 12, sensor_name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOAD_TM_LEN);
#else
    mavlink_load_tm_t packet;
    packet.timestamp = timestamp;
    packet.load = load;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOAD_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOAD_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOAD_TM_MIN_LEN, MAVLINK_MSG_ID_LOAD_TM_LEN, MAVLINK_MSG_ID_LOAD_TM_CRC);
}

/**
 * @brief Encode a load_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param load_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_load_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_load_tm_t* load_tm)
{
    return mavlink_msg_load_tm_pack(system_id, component_id, msg, load_tm->timestamp, load_tm->sensor_name, load_tm->load);
}

/**
 * @brief Encode a load_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param load_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_load_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_load_tm_t* load_tm)
{
    return mavlink_msg_load_tm_pack_chan(system_id, component_id, chan, msg, load_tm->timestamp, load_tm->sensor_name, load_tm->load);
}

/**
 * @brief Send a load_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged
 * @param sensor_name  Sensor name
 * @param load [kg] Load force
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_load_tm_send(mavlink_channel_t chan, uint64_t timestamp, const char *sensor_name, float load)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOAD_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, load);
    _mav_put_char_array(buf, 12, sensor_name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOAD_TM, buf, MAVLINK_MSG_ID_LOAD_TM_MIN_LEN, MAVLINK_MSG_ID_LOAD_TM_LEN, MAVLINK_MSG_ID_LOAD_TM_CRC);
#else
    mavlink_load_tm_t packet;
    packet.timestamp = timestamp;
    packet.load = load;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOAD_TM, (const char *)&packet, MAVLINK_MSG_ID_LOAD_TM_MIN_LEN, MAVLINK_MSG_ID_LOAD_TM_LEN, MAVLINK_MSG_ID_LOAD_TM_CRC);
#endif
}

/**
 * @brief Send a load_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_load_tm_send_struct(mavlink_channel_t chan, const mavlink_load_tm_t* load_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_load_tm_send(chan, load_tm->timestamp, load_tm->sensor_name, load_tm->load);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOAD_TM, (const char *)load_tm, MAVLINK_MSG_ID_LOAD_TM_MIN_LEN, MAVLINK_MSG_ID_LOAD_TM_LEN, MAVLINK_MSG_ID_LOAD_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_LOAD_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_load_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, const char *sensor_name, float load)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, load);
    _mav_put_char_array(buf, 12, sensor_name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOAD_TM, buf, MAVLINK_MSG_ID_LOAD_TM_MIN_LEN, MAVLINK_MSG_ID_LOAD_TM_LEN, MAVLINK_MSG_ID_LOAD_TM_CRC);
#else
    mavlink_load_tm_t *packet = (mavlink_load_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->load = load;
    mav_array_memcpy(packet->sensor_name, sensor_name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOAD_TM, (const char *)packet, MAVLINK_MSG_ID_LOAD_TM_MIN_LEN, MAVLINK_MSG_ID_LOAD_TM_LEN, MAVLINK_MSG_ID_LOAD_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE LOAD_TM UNPACKING


/**
 * @brief Get field timestamp from load_tm message
 *
 * @return [us] When was this logged
 */
static inline uint64_t mavlink_msg_load_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sensor_name from load_tm message
 *
 * @return  Sensor name
 */
static inline uint16_t mavlink_msg_load_tm_get_sensor_name(const mavlink_message_t* msg, char *sensor_name)
{
    return _MAV_RETURN_char_array(msg, sensor_name, 20,  12);
}

/**
 * @brief Get field load from load_tm message
 *
 * @return [kg] Load force
 */
static inline float mavlink_msg_load_tm_get_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a load_tm message into a struct
 *
 * @param msg The message to decode
 * @param load_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_load_tm_decode(const mavlink_message_t* msg, mavlink_load_tm_t* load_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    load_tm->timestamp = mavlink_msg_load_tm_get_timestamp(msg);
    load_tm->load = mavlink_msg_load_tm_get_load(msg);
    mavlink_msg_load_tm_get_sensor_name(msg, load_tm->sensor_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LOAD_TM_LEN? msg->len : MAVLINK_MSG_ID_LOAD_TM_LEN;
        memset(load_tm, 0, MAVLINK_MSG_ID_LOAD_TM_LEN);
    memcpy(load_tm, _MAV_PAYLOAD(msg), len);
#endif
}
