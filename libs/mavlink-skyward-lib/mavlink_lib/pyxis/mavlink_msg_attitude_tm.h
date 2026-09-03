#pragma once
// MESSAGE ATTITUDE_TM PACKING

#define MAVLINK_MSG_ID_ATTITUDE_TM 110


typedef struct __mavlink_attitude_tm_t {
 uint64_t timestamp; /*< [us] When was this logged*/
 float roll; /*< [deg] Roll angle*/
 float pitch; /*< [deg] Pitch angle*/
 float yaw; /*< [deg] Yaw angle*/
 float quat_x; /*<  Quaternion x component*/
 float quat_y; /*<  Quaternion y component*/
 float quat_z; /*<  Quaternion z component*/
 float quat_w; /*<  Quaternion w component*/
 char sensor_id[20]; /*<  Sensor name*/
} mavlink_attitude_tm_t;

#define MAVLINK_MSG_ID_ATTITUDE_TM_LEN 56
#define MAVLINK_MSG_ID_ATTITUDE_TM_MIN_LEN 56
#define MAVLINK_MSG_ID_110_LEN 56
#define MAVLINK_MSG_ID_110_MIN_LEN 56

#define MAVLINK_MSG_ID_ATTITUDE_TM_CRC 170
#define MAVLINK_MSG_ID_110_CRC 170

#define MAVLINK_MSG_ATTITUDE_TM_FIELD_SENSOR_ID_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATTITUDE_TM { \
    110, \
    "ATTITUDE_TM", \
    9, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_attitude_tm_t, timestamp) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_CHAR, 20, 36, offsetof(mavlink_attitude_tm_t, sensor_id) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_tm_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_tm_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_tm_t, yaw) }, \
         { "quat_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_tm_t, quat_x) }, \
         { "quat_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_tm_t, quat_y) }, \
         { "quat_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_attitude_tm_t, quat_z) }, \
         { "quat_w", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_attitude_tm_t, quat_w) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATTITUDE_TM { \
    "ATTITUDE_TM", \
    9, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_attitude_tm_t, timestamp) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_CHAR, 20, 36, offsetof(mavlink_attitude_tm_t, sensor_id) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_tm_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_tm_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_tm_t, yaw) }, \
         { "quat_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_tm_t, quat_x) }, \
         { "quat_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_tm_t, quat_y) }, \
         { "quat_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_attitude_tm_t, quat_z) }, \
         { "quat_w", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_attitude_tm_t, quat_w) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] When was this logged
 * @param sensor_id  Sensor name
 * @param roll [deg] Roll angle
 * @param pitch [deg] Pitch angle
 * @param yaw [deg] Yaw angle
 * @param quat_x  Quaternion x component
 * @param quat_y  Quaternion y component
 * @param quat_z  Quaternion z component
 * @param quat_w  Quaternion w component
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, const char *sensor_id, float roll, float pitch, float yaw, float quat_x, float quat_y, float quat_z, float quat_w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, quat_x);
    _mav_put_float(buf, 24, quat_y);
    _mav_put_float(buf, 28, quat_z);
    _mav_put_float(buf, 32, quat_w);
    _mav_put_char_array(buf, 36, sensor_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_TM_LEN);
#else
    mavlink_attitude_tm_t packet;
    packet.timestamp = timestamp;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.quat_x = quat_x;
    packet.quat_y = quat_y;
    packet.quat_z = quat_z;
    packet.quat_w = quat_w;
    mav_array_memcpy(packet.sensor_id, sensor_id, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_TM_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_CRC);
}

/**
 * @brief Pack a attitude_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] When was this logged
 * @param sensor_id  Sensor name
 * @param roll [deg] Roll angle
 * @param pitch [deg] Pitch angle
 * @param yaw [deg] Yaw angle
 * @param quat_x  Quaternion x component
 * @param quat_y  Quaternion y component
 * @param quat_z  Quaternion z component
 * @param quat_w  Quaternion w component
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,const char *sensor_id,float roll,float pitch,float yaw,float quat_x,float quat_y,float quat_z,float quat_w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, quat_x);
    _mav_put_float(buf, 24, quat_y);
    _mav_put_float(buf, 28, quat_z);
    _mav_put_float(buf, 32, quat_w);
    _mav_put_char_array(buf, 36, sensor_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_TM_LEN);
#else
    mavlink_attitude_tm_t packet;
    packet.timestamp = timestamp;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.quat_x = quat_x;
    packet.quat_y = quat_y;
    packet.quat_z = quat_z;
    packet.quat_w = quat_w;
    mav_array_memcpy(packet.sensor_id, sensor_id, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_TM_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_CRC);
}

/**
 * @brief Encode a attitude_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_tm_t* attitude_tm)
{
    return mavlink_msg_attitude_tm_pack(system_id, component_id, msg, attitude_tm->timestamp, attitude_tm->sensor_id, attitude_tm->roll, attitude_tm->pitch, attitude_tm->yaw, attitude_tm->quat_x, attitude_tm->quat_y, attitude_tm->quat_z, attitude_tm->quat_w);
}

/**
 * @brief Encode a attitude_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude_tm_t* attitude_tm)
{
    return mavlink_msg_attitude_tm_pack_chan(system_id, component_id, chan, msg, attitude_tm->timestamp, attitude_tm->sensor_id, attitude_tm->roll, attitude_tm->pitch, attitude_tm->yaw, attitude_tm->quat_x, attitude_tm->quat_y, attitude_tm->quat_z, attitude_tm->quat_w);
}

/**
 * @brief Send a attitude_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged
 * @param sensor_id  Sensor name
 * @param roll [deg] Roll angle
 * @param pitch [deg] Pitch angle
 * @param yaw [deg] Yaw angle
 * @param quat_x  Quaternion x component
 * @param quat_y  Quaternion y component
 * @param quat_z  Quaternion z component
 * @param quat_w  Quaternion w component
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_tm_send(mavlink_channel_t chan, uint64_t timestamp, const char *sensor_id, float roll, float pitch, float yaw, float quat_x, float quat_y, float quat_z, float quat_w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, quat_x);
    _mav_put_float(buf, 24, quat_y);
    _mav_put_float(buf, 28, quat_z);
    _mav_put_float(buf, 32, quat_w);
    _mav_put_char_array(buf, 36, sensor_id, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TM, buf, MAVLINK_MSG_ID_ATTITUDE_TM_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_CRC);
#else
    mavlink_attitude_tm_t packet;
    packet.timestamp = timestamp;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.quat_x = quat_x;
    packet.quat_y = quat_y;
    packet.quat_z = quat_z;
    packet.quat_w = quat_w;
    mav_array_memcpy(packet.sensor_id, sensor_id, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TM, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_TM_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_CRC);
#endif
}

/**
 * @brief Send a attitude_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_attitude_tm_send_struct(mavlink_channel_t chan, const mavlink_attitude_tm_t* attitude_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_attitude_tm_send(chan, attitude_tm->timestamp, attitude_tm->sensor_id, attitude_tm->roll, attitude_tm->pitch, attitude_tm->yaw, attitude_tm->quat_x, attitude_tm->quat_y, attitude_tm->quat_z, attitude_tm->quat_w);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TM, (const char *)attitude_tm, MAVLINK_MSG_ID_ATTITUDE_TM_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATTITUDE_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_attitude_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, const char *sensor_id, float roll, float pitch, float yaw, float quat_x, float quat_y, float quat_z, float quat_w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, quat_x);
    _mav_put_float(buf, 24, quat_y);
    _mav_put_float(buf, 28, quat_z);
    _mav_put_float(buf, 32, quat_w);
    _mav_put_char_array(buf, 36, sensor_id, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TM, buf, MAVLINK_MSG_ID_ATTITUDE_TM_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_CRC);
#else
    mavlink_attitude_tm_t *packet = (mavlink_attitude_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->quat_x = quat_x;
    packet->quat_y = quat_y;
    packet->quat_z = quat_z;
    packet->quat_w = quat_w;
    mav_array_memcpy(packet->sensor_id, sensor_id, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TM, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_TM_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_LEN, MAVLINK_MSG_ID_ATTITUDE_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE_TM UNPACKING


/**
 * @brief Get field timestamp from attitude_tm message
 *
 * @return [us] When was this logged
 */
static inline uint64_t mavlink_msg_attitude_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sensor_id from attitude_tm message
 *
 * @return  Sensor name
 */
static inline uint16_t mavlink_msg_attitude_tm_get_sensor_id(const mavlink_message_t* msg, char *sensor_id)
{
    return _MAV_RETURN_char_array(msg, sensor_id, 20,  36);
}

/**
 * @brief Get field roll from attitude_tm message
 *
 * @return [deg] Roll angle
 */
static inline float mavlink_msg_attitude_tm_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch from attitude_tm message
 *
 * @return [deg] Pitch angle
 */
static inline float mavlink_msg_attitude_tm_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yaw from attitude_tm message
 *
 * @return [deg] Yaw angle
 */
static inline float mavlink_msg_attitude_tm_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field quat_x from attitude_tm message
 *
 * @return  Quaternion x component
 */
static inline float mavlink_msg_attitude_tm_get_quat_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field quat_y from attitude_tm message
 *
 * @return  Quaternion y component
 */
static inline float mavlink_msg_attitude_tm_get_quat_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field quat_z from attitude_tm message
 *
 * @return  Quaternion z component
 */
static inline float mavlink_msg_attitude_tm_get_quat_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field quat_w from attitude_tm message
 *
 * @return  Quaternion w component
 */
static inline float mavlink_msg_attitude_tm_get_quat_w(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a attitude_tm message into a struct
 *
 * @param msg The message to decode
 * @param attitude_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_tm_decode(const mavlink_message_t* msg, mavlink_attitude_tm_t* attitude_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    attitude_tm->timestamp = mavlink_msg_attitude_tm_get_timestamp(msg);
    attitude_tm->roll = mavlink_msg_attitude_tm_get_roll(msg);
    attitude_tm->pitch = mavlink_msg_attitude_tm_get_pitch(msg);
    attitude_tm->yaw = mavlink_msg_attitude_tm_get_yaw(msg);
    attitude_tm->quat_x = mavlink_msg_attitude_tm_get_quat_x(msg);
    attitude_tm->quat_y = mavlink_msg_attitude_tm_get_quat_y(msg);
    attitude_tm->quat_z = mavlink_msg_attitude_tm_get_quat_z(msg);
    attitude_tm->quat_w = mavlink_msg_attitude_tm_get_quat_w(msg);
    mavlink_msg_attitude_tm_get_sensor_id(msg, attitude_tm->sensor_id);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATTITUDE_TM_LEN? msg->len : MAVLINK_MSG_ID_ATTITUDE_TM_LEN;
        memset(attitude_tm, 0, MAVLINK_MSG_ID_ATTITUDE_TM_LEN);
    memcpy(attitude_tm, _MAV_PAYLOAD(msg), len);
#endif
}
