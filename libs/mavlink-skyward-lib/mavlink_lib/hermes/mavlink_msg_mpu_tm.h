#pragma once
// MESSAGE MPU_TM PACKING

#define MAVLINK_MSG_ID_MPU_TM 171

MAVPACKED(
typedef struct __mavlink_mpu_tm_t {
 uint64_t timestamp; /*<  When was this logged*/
 float acc_x; /*<  X axis acceleration*/
 float acc_y; /*<  Y axis acceleration*/
 float acc_z; /*<  Z axis acceleration*/
 float gyro_x; /*<  X axis gyro*/
 float gyro_y; /*<  Y axis gyro*/
 float gyro_z; /*<  Z axis gyro*/
 float compass_x; /*<  X axis compass*/
 float compass_y; /*<  Y axis compass*/
 float compass_z; /*<  Z axis compass*/
 float temp; /*<  Temperature*/
}) mavlink_mpu_tm_t;

#define MAVLINK_MSG_ID_MPU_TM_LEN 48
#define MAVLINK_MSG_ID_MPU_TM_MIN_LEN 48
#define MAVLINK_MSG_ID_171_LEN 48
#define MAVLINK_MSG_ID_171_MIN_LEN 48

#define MAVLINK_MSG_ID_MPU_TM_CRC 57
#define MAVLINK_MSG_ID_171_CRC 57



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MPU_TM { \
    171, \
    "MPU_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mpu_tm_t, timestamp) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mpu_tm_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mpu_tm_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mpu_tm_t, acc_z) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mpu_tm_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mpu_tm_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_mpu_tm_t, gyro_z) }, \
         { "compass_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_mpu_tm_t, compass_x) }, \
         { "compass_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_mpu_tm_t, compass_y) }, \
         { "compass_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_mpu_tm_t, compass_z) }, \
         { "temp", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_mpu_tm_t, temp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MPU_TM { \
    "MPU_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mpu_tm_t, timestamp) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mpu_tm_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mpu_tm_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mpu_tm_t, acc_z) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mpu_tm_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mpu_tm_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_mpu_tm_t, gyro_z) }, \
         { "compass_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_mpu_tm_t, compass_x) }, \
         { "compass_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_mpu_tm_t, compass_y) }, \
         { "compass_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_mpu_tm_t, compass_z) }, \
         { "temp", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_mpu_tm_t, temp) }, \
         } \
}
#endif

/**
 * @brief Pack a mpu_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  When was this logged
 * @param acc_x  X axis acceleration
 * @param acc_y  Y axis acceleration
 * @param acc_z  Z axis acceleration
 * @param gyro_x  X axis gyro
 * @param gyro_y  Y axis gyro
 * @param gyro_z  Z axis gyro
 * @param compass_x  X axis compass
 * @param compass_y  Y axis compass
 * @param compass_z  Z axis compass
 * @param temp  Temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mpu_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float compass_x, float compass_y, float compass_z, float temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPU_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc_x);
    _mav_put_float(buf, 12, acc_y);
    _mav_put_float(buf, 16, acc_z);
    _mav_put_float(buf, 20, gyro_x);
    _mav_put_float(buf, 24, gyro_y);
    _mav_put_float(buf, 28, gyro_z);
    _mav_put_float(buf, 32, compass_x);
    _mav_put_float(buf, 36, compass_y);
    _mav_put_float(buf, 40, compass_z);
    _mav_put_float(buf, 44, temp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MPU_TM_LEN);
#else
    mavlink_mpu_tm_t packet;
    packet.timestamp = timestamp;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.compass_x = compass_x;
    packet.compass_y = compass_y;
    packet.compass_z = compass_z;
    packet.temp = temp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MPU_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MPU_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MPU_TM_MIN_LEN, MAVLINK_MSG_ID_MPU_TM_LEN, MAVLINK_MSG_ID_MPU_TM_CRC);
}

/**
 * @brief Pack a mpu_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  When was this logged
 * @param acc_x  X axis acceleration
 * @param acc_y  Y axis acceleration
 * @param acc_z  Z axis acceleration
 * @param gyro_x  X axis gyro
 * @param gyro_y  Y axis gyro
 * @param gyro_z  Z axis gyro
 * @param compass_x  X axis compass
 * @param compass_y  Y axis compass
 * @param compass_z  Z axis compass
 * @param temp  Temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mpu_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float acc_x,float acc_y,float acc_z,float gyro_x,float gyro_y,float gyro_z,float compass_x,float compass_y,float compass_z,float temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPU_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc_x);
    _mav_put_float(buf, 12, acc_y);
    _mav_put_float(buf, 16, acc_z);
    _mav_put_float(buf, 20, gyro_x);
    _mav_put_float(buf, 24, gyro_y);
    _mav_put_float(buf, 28, gyro_z);
    _mav_put_float(buf, 32, compass_x);
    _mav_put_float(buf, 36, compass_y);
    _mav_put_float(buf, 40, compass_z);
    _mav_put_float(buf, 44, temp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MPU_TM_LEN);
#else
    mavlink_mpu_tm_t packet;
    packet.timestamp = timestamp;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.compass_x = compass_x;
    packet.compass_y = compass_y;
    packet.compass_z = compass_z;
    packet.temp = temp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MPU_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MPU_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MPU_TM_MIN_LEN, MAVLINK_MSG_ID_MPU_TM_LEN, MAVLINK_MSG_ID_MPU_TM_CRC);
}

/**
 * @brief Encode a mpu_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mpu_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mpu_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mpu_tm_t* mpu_tm)
{
    return mavlink_msg_mpu_tm_pack(system_id, component_id, msg, mpu_tm->timestamp, mpu_tm->acc_x, mpu_tm->acc_y, mpu_tm->acc_z, mpu_tm->gyro_x, mpu_tm->gyro_y, mpu_tm->gyro_z, mpu_tm->compass_x, mpu_tm->compass_y, mpu_tm->compass_z, mpu_tm->temp);
}

/**
 * @brief Encode a mpu_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mpu_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mpu_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mpu_tm_t* mpu_tm)
{
    return mavlink_msg_mpu_tm_pack_chan(system_id, component_id, chan, msg, mpu_tm->timestamp, mpu_tm->acc_x, mpu_tm->acc_y, mpu_tm->acc_z, mpu_tm->gyro_x, mpu_tm->gyro_y, mpu_tm->gyro_z, mpu_tm->compass_x, mpu_tm->compass_y, mpu_tm->compass_z, mpu_tm->temp);
}

/**
 * @brief Send a mpu_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  When was this logged
 * @param acc_x  X axis acceleration
 * @param acc_y  Y axis acceleration
 * @param acc_z  Z axis acceleration
 * @param gyro_x  X axis gyro
 * @param gyro_y  Y axis gyro
 * @param gyro_z  Z axis gyro
 * @param compass_x  X axis compass
 * @param compass_y  Y axis compass
 * @param compass_z  Z axis compass
 * @param temp  Temperature
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mpu_tm_send(mavlink_channel_t chan, uint64_t timestamp, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float compass_x, float compass_y, float compass_z, float temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPU_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc_x);
    _mav_put_float(buf, 12, acc_y);
    _mav_put_float(buf, 16, acc_z);
    _mav_put_float(buf, 20, gyro_x);
    _mav_put_float(buf, 24, gyro_y);
    _mav_put_float(buf, 28, gyro_z);
    _mav_put_float(buf, 32, compass_x);
    _mav_put_float(buf, 36, compass_y);
    _mav_put_float(buf, 40, compass_z);
    _mav_put_float(buf, 44, temp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPU_TM, buf, MAVLINK_MSG_ID_MPU_TM_MIN_LEN, MAVLINK_MSG_ID_MPU_TM_LEN, MAVLINK_MSG_ID_MPU_TM_CRC);
#else
    mavlink_mpu_tm_t packet;
    packet.timestamp = timestamp;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.compass_x = compass_x;
    packet.compass_y = compass_y;
    packet.compass_z = compass_z;
    packet.temp = temp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPU_TM, (const char *)&packet, MAVLINK_MSG_ID_MPU_TM_MIN_LEN, MAVLINK_MSG_ID_MPU_TM_LEN, MAVLINK_MSG_ID_MPU_TM_CRC);
#endif
}

/**
 * @brief Send a mpu_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mpu_tm_send_struct(mavlink_channel_t chan, const mavlink_mpu_tm_t* mpu_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mpu_tm_send(chan, mpu_tm->timestamp, mpu_tm->acc_x, mpu_tm->acc_y, mpu_tm->acc_z, mpu_tm->gyro_x, mpu_tm->gyro_y, mpu_tm->gyro_z, mpu_tm->compass_x, mpu_tm->compass_y, mpu_tm->compass_z, mpu_tm->temp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPU_TM, (const char *)mpu_tm, MAVLINK_MSG_ID_MPU_TM_MIN_LEN, MAVLINK_MSG_ID_MPU_TM_LEN, MAVLINK_MSG_ID_MPU_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_MPU_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mpu_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float compass_x, float compass_y, float compass_z, float temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc_x);
    _mav_put_float(buf, 12, acc_y);
    _mav_put_float(buf, 16, acc_z);
    _mav_put_float(buf, 20, gyro_x);
    _mav_put_float(buf, 24, gyro_y);
    _mav_put_float(buf, 28, gyro_z);
    _mav_put_float(buf, 32, compass_x);
    _mav_put_float(buf, 36, compass_y);
    _mav_put_float(buf, 40, compass_z);
    _mav_put_float(buf, 44, temp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPU_TM, buf, MAVLINK_MSG_ID_MPU_TM_MIN_LEN, MAVLINK_MSG_ID_MPU_TM_LEN, MAVLINK_MSG_ID_MPU_TM_CRC);
#else
    mavlink_mpu_tm_t *packet = (mavlink_mpu_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->acc_x = acc_x;
    packet->acc_y = acc_y;
    packet->acc_z = acc_z;
    packet->gyro_x = gyro_x;
    packet->gyro_y = gyro_y;
    packet->gyro_z = gyro_z;
    packet->compass_x = compass_x;
    packet->compass_y = compass_y;
    packet->compass_z = compass_z;
    packet->temp = temp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPU_TM, (const char *)packet, MAVLINK_MSG_ID_MPU_TM_MIN_LEN, MAVLINK_MSG_ID_MPU_TM_LEN, MAVLINK_MSG_ID_MPU_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE MPU_TM UNPACKING


/**
 * @brief Get field timestamp from mpu_tm message
 *
 * @return  When was this logged
 */
static inline uint64_t mavlink_msg_mpu_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field acc_x from mpu_tm message
 *
 * @return  X axis acceleration
 */
static inline float mavlink_msg_mpu_tm_get_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field acc_y from mpu_tm message
 *
 * @return  Y axis acceleration
 */
static inline float mavlink_msg_mpu_tm_get_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field acc_z from mpu_tm message
 *
 * @return  Z axis acceleration
 */
static inline float mavlink_msg_mpu_tm_get_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gyro_x from mpu_tm message
 *
 * @return  X axis gyro
 */
static inline float mavlink_msg_mpu_tm_get_gyro_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field gyro_y from mpu_tm message
 *
 * @return  Y axis gyro
 */
static inline float mavlink_msg_mpu_tm_get_gyro_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field gyro_z from mpu_tm message
 *
 * @return  Z axis gyro
 */
static inline float mavlink_msg_mpu_tm_get_gyro_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field compass_x from mpu_tm message
 *
 * @return  X axis compass
 */
static inline float mavlink_msg_mpu_tm_get_compass_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field compass_y from mpu_tm message
 *
 * @return  Y axis compass
 */
static inline float mavlink_msg_mpu_tm_get_compass_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field compass_z from mpu_tm message
 *
 * @return  Z axis compass
 */
static inline float mavlink_msg_mpu_tm_get_compass_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field temp from mpu_tm message
 *
 * @return  Temperature
 */
static inline float mavlink_msg_mpu_tm_get_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a mpu_tm message into a struct
 *
 * @param msg The message to decode
 * @param mpu_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_mpu_tm_decode(const mavlink_message_t* msg, mavlink_mpu_tm_t* mpu_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mpu_tm->timestamp = mavlink_msg_mpu_tm_get_timestamp(msg);
    mpu_tm->acc_x = mavlink_msg_mpu_tm_get_acc_x(msg);
    mpu_tm->acc_y = mavlink_msg_mpu_tm_get_acc_y(msg);
    mpu_tm->acc_z = mavlink_msg_mpu_tm_get_acc_z(msg);
    mpu_tm->gyro_x = mavlink_msg_mpu_tm_get_gyro_x(msg);
    mpu_tm->gyro_y = mavlink_msg_mpu_tm_get_gyro_y(msg);
    mpu_tm->gyro_z = mavlink_msg_mpu_tm_get_gyro_z(msg);
    mpu_tm->compass_x = mavlink_msg_mpu_tm_get_compass_x(msg);
    mpu_tm->compass_y = mavlink_msg_mpu_tm_get_compass_y(msg);
    mpu_tm->compass_z = mavlink_msg_mpu_tm_get_compass_z(msg);
    mpu_tm->temp = mavlink_msg_mpu_tm_get_temp(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MPU_TM_LEN? msg->len : MAVLINK_MSG_ID_MPU_TM_LEN;
        memset(mpu_tm, 0, MAVLINK_MSG_ID_MPU_TM_LEN);
    memcpy(mpu_tm, _MAV_PAYLOAD(msg), len);
#endif
}
