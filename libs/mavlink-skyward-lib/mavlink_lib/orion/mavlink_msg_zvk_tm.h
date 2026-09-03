#pragma once
// MESSAGE ZVK_TM PACKING

#define MAVLINK_MSG_ID_ZVK_TM 215


typedef struct __mavlink_zvk_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 float acc0_bias_x; /*< [m/s^2] Accelerometer 0 X bias*/
 float acc0_bias_y; /*< [m/s^2] Accelerometer 0 Y bias*/
 float acc0_bias_z; /*< [m/s^2] Accelerometer 0 Z bias*/
 float gyro0_bias_x; /*< [rad/s] Gyroscope 0 X bias*/
 float gyro0_bias_y; /*< [rad/s] Gyroscope 0 Y bias*/
 float gyro0_bias_z; /*< [rad/s] Gyroscope 0 Z bias*/
 float acc1_bias_x; /*< [m/s^2] Accelerometer 1 X bias*/
 float acc1_bias_y; /*< [m/s^2] Accelerometer 1 Y bias*/
 float acc1_bias_z; /*< [m/s^2] Accelerometer 1 Z bias*/
 float gyro1_bias_x; /*< [rad/s] Gyroscope 1 X bias*/
 float gyro1_bias_y; /*< [rad/s] Gyroscope 1 Y bias*/
 float gyro1_bias_z; /*< [rad/s] Gyroscope 1 Z bias*/
} mavlink_zvk_tm_t;

#define MAVLINK_MSG_ID_ZVK_TM_LEN 56
#define MAVLINK_MSG_ID_ZVK_TM_MIN_LEN 56
#define MAVLINK_MSG_ID_215_LEN 56
#define MAVLINK_MSG_ID_215_MIN_LEN 56

#define MAVLINK_MSG_ID_ZVK_TM_CRC 183
#define MAVLINK_MSG_ID_215_CRC 183



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZVK_TM { \
    215, \
    "ZVK_TM", \
    13, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_zvk_tm_t, timestamp) }, \
         { "acc0_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_zvk_tm_t, acc0_bias_x) }, \
         { "acc0_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_zvk_tm_t, acc0_bias_y) }, \
         { "acc0_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_zvk_tm_t, acc0_bias_z) }, \
         { "gyro0_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_zvk_tm_t, gyro0_bias_x) }, \
         { "gyro0_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_zvk_tm_t, gyro0_bias_y) }, \
         { "gyro0_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_zvk_tm_t, gyro0_bias_z) }, \
         { "acc1_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_zvk_tm_t, acc1_bias_x) }, \
         { "acc1_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_zvk_tm_t, acc1_bias_y) }, \
         { "acc1_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_zvk_tm_t, acc1_bias_z) }, \
         { "gyro1_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_zvk_tm_t, gyro1_bias_x) }, \
         { "gyro1_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_zvk_tm_t, gyro1_bias_y) }, \
         { "gyro1_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_zvk_tm_t, gyro1_bias_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZVK_TM { \
    "ZVK_TM", \
    13, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_zvk_tm_t, timestamp) }, \
         { "acc0_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_zvk_tm_t, acc0_bias_x) }, \
         { "acc0_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_zvk_tm_t, acc0_bias_y) }, \
         { "acc0_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_zvk_tm_t, acc0_bias_z) }, \
         { "gyro0_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_zvk_tm_t, gyro0_bias_x) }, \
         { "gyro0_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_zvk_tm_t, gyro0_bias_y) }, \
         { "gyro0_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_zvk_tm_t, gyro0_bias_z) }, \
         { "acc1_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_zvk_tm_t, acc1_bias_x) }, \
         { "acc1_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_zvk_tm_t, acc1_bias_y) }, \
         { "acc1_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_zvk_tm_t, acc1_bias_z) }, \
         { "gyro1_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_zvk_tm_t, gyro1_bias_x) }, \
         { "gyro1_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_zvk_tm_t, gyro1_bias_y) }, \
         { "gyro1_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_zvk_tm_t, gyro1_bias_z) }, \
         } \
}
#endif

/**
 * @brief Pack a zvk_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param acc0_bias_x [m/s^2] Accelerometer 0 X bias
 * @param acc0_bias_y [m/s^2] Accelerometer 0 Y bias
 * @param acc0_bias_z [m/s^2] Accelerometer 0 Z bias
 * @param gyro0_bias_x [rad/s] Gyroscope 0 X bias
 * @param gyro0_bias_y [rad/s] Gyroscope 0 Y bias
 * @param gyro0_bias_z [rad/s] Gyroscope 0 Z bias
 * @param acc1_bias_x [m/s^2] Accelerometer 1 X bias
 * @param acc1_bias_y [m/s^2] Accelerometer 1 Y bias
 * @param acc1_bias_z [m/s^2] Accelerometer 1 Z bias
 * @param gyro1_bias_x [rad/s] Gyroscope 1 X bias
 * @param gyro1_bias_y [rad/s] Gyroscope 1 Y bias
 * @param gyro1_bias_z [rad/s] Gyroscope 1 Z bias
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zvk_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float acc0_bias_x, float acc0_bias_y, float acc0_bias_z, float gyro0_bias_x, float gyro0_bias_y, float gyro0_bias_z, float acc1_bias_x, float acc1_bias_y, float acc1_bias_z, float gyro1_bias_x, float gyro1_bias_y, float gyro1_bias_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZVK_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc0_bias_x);
    _mav_put_float(buf, 12, acc0_bias_y);
    _mav_put_float(buf, 16, acc0_bias_z);
    _mav_put_float(buf, 20, gyro0_bias_x);
    _mav_put_float(buf, 24, gyro0_bias_y);
    _mav_put_float(buf, 28, gyro0_bias_z);
    _mav_put_float(buf, 32, acc1_bias_x);
    _mav_put_float(buf, 36, acc1_bias_y);
    _mav_put_float(buf, 40, acc1_bias_z);
    _mav_put_float(buf, 44, gyro1_bias_x);
    _mav_put_float(buf, 48, gyro1_bias_y);
    _mav_put_float(buf, 52, gyro1_bias_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZVK_TM_LEN);
#else
    mavlink_zvk_tm_t packet;
    packet.timestamp = timestamp;
    packet.acc0_bias_x = acc0_bias_x;
    packet.acc0_bias_y = acc0_bias_y;
    packet.acc0_bias_z = acc0_bias_z;
    packet.gyro0_bias_x = gyro0_bias_x;
    packet.gyro0_bias_y = gyro0_bias_y;
    packet.gyro0_bias_z = gyro0_bias_z;
    packet.acc1_bias_x = acc1_bias_x;
    packet.acc1_bias_y = acc1_bias_y;
    packet.acc1_bias_z = acc1_bias_z;
    packet.gyro1_bias_x = gyro1_bias_x;
    packet.gyro1_bias_y = gyro1_bias_y;
    packet.gyro1_bias_z = gyro1_bias_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZVK_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZVK_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZVK_TM_MIN_LEN, MAVLINK_MSG_ID_ZVK_TM_LEN, MAVLINK_MSG_ID_ZVK_TM_CRC);
}

/**
 * @brief Pack a zvk_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp in microseconds
 * @param acc0_bias_x [m/s^2] Accelerometer 0 X bias
 * @param acc0_bias_y [m/s^2] Accelerometer 0 Y bias
 * @param acc0_bias_z [m/s^2] Accelerometer 0 Z bias
 * @param gyro0_bias_x [rad/s] Gyroscope 0 X bias
 * @param gyro0_bias_y [rad/s] Gyroscope 0 Y bias
 * @param gyro0_bias_z [rad/s] Gyroscope 0 Z bias
 * @param acc1_bias_x [m/s^2] Accelerometer 1 X bias
 * @param acc1_bias_y [m/s^2] Accelerometer 1 Y bias
 * @param acc1_bias_z [m/s^2] Accelerometer 1 Z bias
 * @param gyro1_bias_x [rad/s] Gyroscope 1 X bias
 * @param gyro1_bias_y [rad/s] Gyroscope 1 Y bias
 * @param gyro1_bias_z [rad/s] Gyroscope 1 Z bias
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zvk_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float acc0_bias_x,float acc0_bias_y,float acc0_bias_z,float gyro0_bias_x,float gyro0_bias_y,float gyro0_bias_z,float acc1_bias_x,float acc1_bias_y,float acc1_bias_z,float gyro1_bias_x,float gyro1_bias_y,float gyro1_bias_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZVK_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc0_bias_x);
    _mav_put_float(buf, 12, acc0_bias_y);
    _mav_put_float(buf, 16, acc0_bias_z);
    _mav_put_float(buf, 20, gyro0_bias_x);
    _mav_put_float(buf, 24, gyro0_bias_y);
    _mav_put_float(buf, 28, gyro0_bias_z);
    _mav_put_float(buf, 32, acc1_bias_x);
    _mav_put_float(buf, 36, acc1_bias_y);
    _mav_put_float(buf, 40, acc1_bias_z);
    _mav_put_float(buf, 44, gyro1_bias_x);
    _mav_put_float(buf, 48, gyro1_bias_y);
    _mav_put_float(buf, 52, gyro1_bias_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZVK_TM_LEN);
#else
    mavlink_zvk_tm_t packet;
    packet.timestamp = timestamp;
    packet.acc0_bias_x = acc0_bias_x;
    packet.acc0_bias_y = acc0_bias_y;
    packet.acc0_bias_z = acc0_bias_z;
    packet.gyro0_bias_x = gyro0_bias_x;
    packet.gyro0_bias_y = gyro0_bias_y;
    packet.gyro0_bias_z = gyro0_bias_z;
    packet.acc1_bias_x = acc1_bias_x;
    packet.acc1_bias_y = acc1_bias_y;
    packet.acc1_bias_z = acc1_bias_z;
    packet.gyro1_bias_x = gyro1_bias_x;
    packet.gyro1_bias_y = gyro1_bias_y;
    packet.gyro1_bias_z = gyro1_bias_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZVK_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZVK_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZVK_TM_MIN_LEN, MAVLINK_MSG_ID_ZVK_TM_LEN, MAVLINK_MSG_ID_ZVK_TM_CRC);
}

/**
 * @brief Encode a zvk_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zvk_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zvk_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zvk_tm_t* zvk_tm)
{
    return mavlink_msg_zvk_tm_pack(system_id, component_id, msg, zvk_tm->timestamp, zvk_tm->acc0_bias_x, zvk_tm->acc0_bias_y, zvk_tm->acc0_bias_z, zvk_tm->gyro0_bias_x, zvk_tm->gyro0_bias_y, zvk_tm->gyro0_bias_z, zvk_tm->acc1_bias_x, zvk_tm->acc1_bias_y, zvk_tm->acc1_bias_z, zvk_tm->gyro1_bias_x, zvk_tm->gyro1_bias_y, zvk_tm->gyro1_bias_z);
}

/**
 * @brief Encode a zvk_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zvk_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zvk_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zvk_tm_t* zvk_tm)
{
    return mavlink_msg_zvk_tm_pack_chan(system_id, component_id, chan, msg, zvk_tm->timestamp, zvk_tm->acc0_bias_x, zvk_tm->acc0_bias_y, zvk_tm->acc0_bias_z, zvk_tm->gyro0_bias_x, zvk_tm->gyro0_bias_y, zvk_tm->gyro0_bias_z, zvk_tm->acc1_bias_x, zvk_tm->acc1_bias_y, zvk_tm->acc1_bias_z, zvk_tm->gyro1_bias_x, zvk_tm->gyro1_bias_y, zvk_tm->gyro1_bias_z);
}

/**
 * @brief Send a zvk_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param acc0_bias_x [m/s^2] Accelerometer 0 X bias
 * @param acc0_bias_y [m/s^2] Accelerometer 0 Y bias
 * @param acc0_bias_z [m/s^2] Accelerometer 0 Z bias
 * @param gyro0_bias_x [rad/s] Gyroscope 0 X bias
 * @param gyro0_bias_y [rad/s] Gyroscope 0 Y bias
 * @param gyro0_bias_z [rad/s] Gyroscope 0 Z bias
 * @param acc1_bias_x [m/s^2] Accelerometer 1 X bias
 * @param acc1_bias_y [m/s^2] Accelerometer 1 Y bias
 * @param acc1_bias_z [m/s^2] Accelerometer 1 Z bias
 * @param gyro1_bias_x [rad/s] Gyroscope 1 X bias
 * @param gyro1_bias_y [rad/s] Gyroscope 1 Y bias
 * @param gyro1_bias_z [rad/s] Gyroscope 1 Z bias
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zvk_tm_send(mavlink_channel_t chan, uint64_t timestamp, float acc0_bias_x, float acc0_bias_y, float acc0_bias_z, float gyro0_bias_x, float gyro0_bias_y, float gyro0_bias_z, float acc1_bias_x, float acc1_bias_y, float acc1_bias_z, float gyro1_bias_x, float gyro1_bias_y, float gyro1_bias_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZVK_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc0_bias_x);
    _mav_put_float(buf, 12, acc0_bias_y);
    _mav_put_float(buf, 16, acc0_bias_z);
    _mav_put_float(buf, 20, gyro0_bias_x);
    _mav_put_float(buf, 24, gyro0_bias_y);
    _mav_put_float(buf, 28, gyro0_bias_z);
    _mav_put_float(buf, 32, acc1_bias_x);
    _mav_put_float(buf, 36, acc1_bias_y);
    _mav_put_float(buf, 40, acc1_bias_z);
    _mav_put_float(buf, 44, gyro1_bias_x);
    _mav_put_float(buf, 48, gyro1_bias_y);
    _mav_put_float(buf, 52, gyro1_bias_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZVK_TM, buf, MAVLINK_MSG_ID_ZVK_TM_MIN_LEN, MAVLINK_MSG_ID_ZVK_TM_LEN, MAVLINK_MSG_ID_ZVK_TM_CRC);
#else
    mavlink_zvk_tm_t packet;
    packet.timestamp = timestamp;
    packet.acc0_bias_x = acc0_bias_x;
    packet.acc0_bias_y = acc0_bias_y;
    packet.acc0_bias_z = acc0_bias_z;
    packet.gyro0_bias_x = gyro0_bias_x;
    packet.gyro0_bias_y = gyro0_bias_y;
    packet.gyro0_bias_z = gyro0_bias_z;
    packet.acc1_bias_x = acc1_bias_x;
    packet.acc1_bias_y = acc1_bias_y;
    packet.acc1_bias_z = acc1_bias_z;
    packet.gyro1_bias_x = gyro1_bias_x;
    packet.gyro1_bias_y = gyro1_bias_y;
    packet.gyro1_bias_z = gyro1_bias_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZVK_TM, (const char *)&packet, MAVLINK_MSG_ID_ZVK_TM_MIN_LEN, MAVLINK_MSG_ID_ZVK_TM_LEN, MAVLINK_MSG_ID_ZVK_TM_CRC);
#endif
}

/**
 * @brief Send a zvk_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zvk_tm_send_struct(mavlink_channel_t chan, const mavlink_zvk_tm_t* zvk_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zvk_tm_send(chan, zvk_tm->timestamp, zvk_tm->acc0_bias_x, zvk_tm->acc0_bias_y, zvk_tm->acc0_bias_z, zvk_tm->gyro0_bias_x, zvk_tm->gyro0_bias_y, zvk_tm->gyro0_bias_z, zvk_tm->acc1_bias_x, zvk_tm->acc1_bias_y, zvk_tm->acc1_bias_z, zvk_tm->gyro1_bias_x, zvk_tm->gyro1_bias_y, zvk_tm->gyro1_bias_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZVK_TM, (const char *)zvk_tm, MAVLINK_MSG_ID_ZVK_TM_MIN_LEN, MAVLINK_MSG_ID_ZVK_TM_LEN, MAVLINK_MSG_ID_ZVK_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZVK_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zvk_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float acc0_bias_x, float acc0_bias_y, float acc0_bias_z, float gyro0_bias_x, float gyro0_bias_y, float gyro0_bias_z, float acc1_bias_x, float acc1_bias_y, float acc1_bias_z, float gyro1_bias_x, float gyro1_bias_y, float gyro1_bias_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc0_bias_x);
    _mav_put_float(buf, 12, acc0_bias_y);
    _mav_put_float(buf, 16, acc0_bias_z);
    _mav_put_float(buf, 20, gyro0_bias_x);
    _mav_put_float(buf, 24, gyro0_bias_y);
    _mav_put_float(buf, 28, gyro0_bias_z);
    _mav_put_float(buf, 32, acc1_bias_x);
    _mav_put_float(buf, 36, acc1_bias_y);
    _mav_put_float(buf, 40, acc1_bias_z);
    _mav_put_float(buf, 44, gyro1_bias_x);
    _mav_put_float(buf, 48, gyro1_bias_y);
    _mav_put_float(buf, 52, gyro1_bias_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZVK_TM, buf, MAVLINK_MSG_ID_ZVK_TM_MIN_LEN, MAVLINK_MSG_ID_ZVK_TM_LEN, MAVLINK_MSG_ID_ZVK_TM_CRC);
#else
    mavlink_zvk_tm_t *packet = (mavlink_zvk_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->acc0_bias_x = acc0_bias_x;
    packet->acc0_bias_y = acc0_bias_y;
    packet->acc0_bias_z = acc0_bias_z;
    packet->gyro0_bias_x = gyro0_bias_x;
    packet->gyro0_bias_y = gyro0_bias_y;
    packet->gyro0_bias_z = gyro0_bias_z;
    packet->acc1_bias_x = acc1_bias_x;
    packet->acc1_bias_y = acc1_bias_y;
    packet->acc1_bias_z = acc1_bias_z;
    packet->gyro1_bias_x = gyro1_bias_x;
    packet->gyro1_bias_y = gyro1_bias_y;
    packet->gyro1_bias_z = gyro1_bias_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZVK_TM, (const char *)packet, MAVLINK_MSG_ID_ZVK_TM_MIN_LEN, MAVLINK_MSG_ID_ZVK_TM_LEN, MAVLINK_MSG_ID_ZVK_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ZVK_TM UNPACKING


/**
 * @brief Get field timestamp from zvk_tm message
 *
 * @return [us] Timestamp in microseconds
 */
static inline uint64_t mavlink_msg_zvk_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field acc0_bias_x from zvk_tm message
 *
 * @return [m/s^2] Accelerometer 0 X bias
 */
static inline float mavlink_msg_zvk_tm_get_acc0_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field acc0_bias_y from zvk_tm message
 *
 * @return [m/s^2] Accelerometer 0 Y bias
 */
static inline float mavlink_msg_zvk_tm_get_acc0_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field acc0_bias_z from zvk_tm message
 *
 * @return [m/s^2] Accelerometer 0 Z bias
 */
static inline float mavlink_msg_zvk_tm_get_acc0_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gyro0_bias_x from zvk_tm message
 *
 * @return [rad/s] Gyroscope 0 X bias
 */
static inline float mavlink_msg_zvk_tm_get_gyro0_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field gyro0_bias_y from zvk_tm message
 *
 * @return [rad/s] Gyroscope 0 Y bias
 */
static inline float mavlink_msg_zvk_tm_get_gyro0_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field gyro0_bias_z from zvk_tm message
 *
 * @return [rad/s] Gyroscope 0 Z bias
 */
static inline float mavlink_msg_zvk_tm_get_gyro0_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field acc1_bias_x from zvk_tm message
 *
 * @return [m/s^2] Accelerometer 1 X bias
 */
static inline float mavlink_msg_zvk_tm_get_acc1_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field acc1_bias_y from zvk_tm message
 *
 * @return [m/s^2] Accelerometer 1 Y bias
 */
static inline float mavlink_msg_zvk_tm_get_acc1_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field acc1_bias_z from zvk_tm message
 *
 * @return [m/s^2] Accelerometer 1 Z bias
 */
static inline float mavlink_msg_zvk_tm_get_acc1_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field gyro1_bias_x from zvk_tm message
 *
 * @return [rad/s] Gyroscope 1 X bias
 */
static inline float mavlink_msg_zvk_tm_get_gyro1_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field gyro1_bias_y from zvk_tm message
 *
 * @return [rad/s] Gyroscope 1 Y bias
 */
static inline float mavlink_msg_zvk_tm_get_gyro1_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field gyro1_bias_z from zvk_tm message
 *
 * @return [rad/s] Gyroscope 1 Z bias
 */
static inline float mavlink_msg_zvk_tm_get_gyro1_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Decode a zvk_tm message into a struct
 *
 * @param msg The message to decode
 * @param zvk_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_zvk_tm_decode(const mavlink_message_t* msg, mavlink_zvk_tm_t* zvk_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zvk_tm->timestamp = mavlink_msg_zvk_tm_get_timestamp(msg);
    zvk_tm->acc0_bias_x = mavlink_msg_zvk_tm_get_acc0_bias_x(msg);
    zvk_tm->acc0_bias_y = mavlink_msg_zvk_tm_get_acc0_bias_y(msg);
    zvk_tm->acc0_bias_z = mavlink_msg_zvk_tm_get_acc0_bias_z(msg);
    zvk_tm->gyro0_bias_x = mavlink_msg_zvk_tm_get_gyro0_bias_x(msg);
    zvk_tm->gyro0_bias_y = mavlink_msg_zvk_tm_get_gyro0_bias_y(msg);
    zvk_tm->gyro0_bias_z = mavlink_msg_zvk_tm_get_gyro0_bias_z(msg);
    zvk_tm->acc1_bias_x = mavlink_msg_zvk_tm_get_acc1_bias_x(msg);
    zvk_tm->acc1_bias_y = mavlink_msg_zvk_tm_get_acc1_bias_y(msg);
    zvk_tm->acc1_bias_z = mavlink_msg_zvk_tm_get_acc1_bias_z(msg);
    zvk_tm->gyro1_bias_x = mavlink_msg_zvk_tm_get_gyro1_bias_x(msg);
    zvk_tm->gyro1_bias_y = mavlink_msg_zvk_tm_get_gyro1_bias_y(msg);
    zvk_tm->gyro1_bias_z = mavlink_msg_zvk_tm_get_gyro1_bias_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZVK_TM_LEN? msg->len : MAVLINK_MSG_ID_ZVK_TM_LEN;
        memset(zvk_tm, 0, MAVLINK_MSG_ID_ZVK_TM_LEN);
    memcpy(zvk_tm, _MAV_PAYLOAD(msg), len);
#endif
}
