#pragma once
// MESSAGE IMU_TM PACKING

#define MAVLINK_MSG_ID_IMU_TM 104


typedef struct __mavlink_imu_tm_t {
 uint64_t timestamp; /*< [us] When was this logged*/
 float acc_x; /*< [m/s2] X axis acceleration*/
 float acc_y; /*< [m/s2] Y axis acceleration*/
 float acc_z; /*< [m/s2] Z axis acceleration*/
 float gyro_x; /*< [rad/s] X axis gyro*/
 float gyro_y; /*< [rad/s] Y axis gyro*/
 float gyro_z; /*< [rad/s] Z axis gyro*/
 float mag_x; /*< [uT] X axis compass*/
 float mag_y; /*< [uT] Y axis compass*/
 float mag_z; /*< [uT] Z axis compass*/
 char sensor_name[20]; /*<  Sensor name*/
} mavlink_imu_tm_t;

#define MAVLINK_MSG_ID_IMU_TM_LEN 64
#define MAVLINK_MSG_ID_IMU_TM_MIN_LEN 64
#define MAVLINK_MSG_ID_104_LEN 64
#define MAVLINK_MSG_ID_104_MIN_LEN 64

#define MAVLINK_MSG_ID_IMU_TM_CRC 72
#define MAVLINK_MSG_ID_104_CRC 72

#define MAVLINK_MSG_IMU_TM_FIELD_SENSOR_NAME_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IMU_TM { \
    104, \
    "IMU_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_imu_tm_t, timestamp) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 20, 44, offsetof(mavlink_imu_tm_t, sensor_name) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_imu_tm_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_imu_tm_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_imu_tm_t, acc_z) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_imu_tm_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_imu_tm_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_imu_tm_t, gyro_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_imu_tm_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_imu_tm_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_imu_tm_t, mag_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IMU_TM { \
    "IMU_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_imu_tm_t, timestamp) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 20, 44, offsetof(mavlink_imu_tm_t, sensor_name) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_imu_tm_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_imu_tm_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_imu_tm_t, acc_z) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_imu_tm_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_imu_tm_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_imu_tm_t, gyro_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_imu_tm_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_imu_tm_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_imu_tm_t, mag_z) }, \
         } \
}
#endif

/**
 * @brief Pack a imu_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] When was this logged
 * @param sensor_name  Sensor name
 * @param acc_x [m/s2] X axis acceleration
 * @param acc_y [m/s2] Y axis acceleration
 * @param acc_z [m/s2] Z axis acceleration
 * @param gyro_x [rad/s] X axis gyro
 * @param gyro_y [rad/s] Y axis gyro
 * @param gyro_z [rad/s] Z axis gyro
 * @param mag_x [uT] X axis compass
 * @param mag_y [uT] Y axis compass
 * @param mag_z [uT] Z axis compass
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, const char *sensor_name, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc_x);
    _mav_put_float(buf, 12, acc_y);
    _mav_put_float(buf, 16, acc_z);
    _mav_put_float(buf, 20, gyro_x);
    _mav_put_float(buf, 24, gyro_y);
    _mav_put_float(buf, 28, gyro_z);
    _mav_put_float(buf, 32, mag_x);
    _mav_put_float(buf, 36, mag_y);
    _mav_put_float(buf, 40, mag_z);
    _mav_put_char_array(buf, 44, sensor_name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_TM_LEN);
#else
    mavlink_imu_tm_t packet;
    packet.timestamp = timestamp;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMU_TM_MIN_LEN, MAVLINK_MSG_ID_IMU_TM_LEN, MAVLINK_MSG_ID_IMU_TM_CRC);
}

/**
 * @brief Pack a imu_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] When was this logged
 * @param sensor_name  Sensor name
 * @param acc_x [m/s2] X axis acceleration
 * @param acc_y [m/s2] Y axis acceleration
 * @param acc_z [m/s2] Z axis acceleration
 * @param gyro_x [rad/s] X axis gyro
 * @param gyro_y [rad/s] Y axis gyro
 * @param gyro_z [rad/s] Z axis gyro
 * @param mag_x [uT] X axis compass
 * @param mag_y [uT] Y axis compass
 * @param mag_z [uT] Z axis compass
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,const char *sensor_name,float acc_x,float acc_y,float acc_z,float gyro_x,float gyro_y,float gyro_z,float mag_x,float mag_y,float mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc_x);
    _mav_put_float(buf, 12, acc_y);
    _mav_put_float(buf, 16, acc_z);
    _mav_put_float(buf, 20, gyro_x);
    _mav_put_float(buf, 24, gyro_y);
    _mav_put_float(buf, 28, gyro_z);
    _mav_put_float(buf, 32, mag_x);
    _mav_put_float(buf, 36, mag_y);
    _mav_put_float(buf, 40, mag_z);
    _mav_put_char_array(buf, 44, sensor_name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_TM_LEN);
#else
    mavlink_imu_tm_t packet;
    packet.timestamp = timestamp;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMU_TM_MIN_LEN, MAVLINK_MSG_ID_IMU_TM_LEN, MAVLINK_MSG_ID_IMU_TM_CRC);
}

/**
 * @brief Encode a imu_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param imu_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_imu_tm_t* imu_tm)
{
    return mavlink_msg_imu_tm_pack(system_id, component_id, msg, imu_tm->timestamp, imu_tm->sensor_name, imu_tm->acc_x, imu_tm->acc_y, imu_tm->acc_z, imu_tm->gyro_x, imu_tm->gyro_y, imu_tm->gyro_z, imu_tm->mag_x, imu_tm->mag_y, imu_tm->mag_z);
}

/**
 * @brief Encode a imu_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param imu_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_imu_tm_t* imu_tm)
{
    return mavlink_msg_imu_tm_pack_chan(system_id, component_id, chan, msg, imu_tm->timestamp, imu_tm->sensor_name, imu_tm->acc_x, imu_tm->acc_y, imu_tm->acc_z, imu_tm->gyro_x, imu_tm->gyro_y, imu_tm->gyro_z, imu_tm->mag_x, imu_tm->mag_y, imu_tm->mag_z);
}

/**
 * @brief Send a imu_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged
 * @param sensor_name  Sensor name
 * @param acc_x [m/s2] X axis acceleration
 * @param acc_y [m/s2] Y axis acceleration
 * @param acc_z [m/s2] Z axis acceleration
 * @param gyro_x [rad/s] X axis gyro
 * @param gyro_y [rad/s] Y axis gyro
 * @param gyro_z [rad/s] Z axis gyro
 * @param mag_x [uT] X axis compass
 * @param mag_y [uT] Y axis compass
 * @param mag_z [uT] Z axis compass
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_imu_tm_send(mavlink_channel_t chan, uint64_t timestamp, const char *sensor_name, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc_x);
    _mav_put_float(buf, 12, acc_y);
    _mav_put_float(buf, 16, acc_z);
    _mav_put_float(buf, 20, gyro_x);
    _mav_put_float(buf, 24, gyro_y);
    _mav_put_float(buf, 28, gyro_z);
    _mav_put_float(buf, 32, mag_x);
    _mav_put_float(buf, 36, mag_y);
    _mav_put_float(buf, 40, mag_z);
    _mav_put_char_array(buf, 44, sensor_name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_TM, buf, MAVLINK_MSG_ID_IMU_TM_MIN_LEN, MAVLINK_MSG_ID_IMU_TM_LEN, MAVLINK_MSG_ID_IMU_TM_CRC);
#else
    mavlink_imu_tm_t packet;
    packet.timestamp = timestamp;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_TM, (const char *)&packet, MAVLINK_MSG_ID_IMU_TM_MIN_LEN, MAVLINK_MSG_ID_IMU_TM_LEN, MAVLINK_MSG_ID_IMU_TM_CRC);
#endif
}

/**
 * @brief Send a imu_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_imu_tm_send_struct(mavlink_channel_t chan, const mavlink_imu_tm_t* imu_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_imu_tm_send(chan, imu_tm->timestamp, imu_tm->sensor_name, imu_tm->acc_x, imu_tm->acc_y, imu_tm->acc_z, imu_tm->gyro_x, imu_tm->gyro_y, imu_tm->gyro_z, imu_tm->mag_x, imu_tm->mag_y, imu_tm->mag_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_TM, (const char *)imu_tm, MAVLINK_MSG_ID_IMU_TM_MIN_LEN, MAVLINK_MSG_ID_IMU_TM_LEN, MAVLINK_MSG_ID_IMU_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_IMU_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_imu_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, const char *sensor_name, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z)
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
    _mav_put_float(buf, 32, mag_x);
    _mav_put_float(buf, 36, mag_y);
    _mav_put_float(buf, 40, mag_z);
    _mav_put_char_array(buf, 44, sensor_name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_TM, buf, MAVLINK_MSG_ID_IMU_TM_MIN_LEN, MAVLINK_MSG_ID_IMU_TM_LEN, MAVLINK_MSG_ID_IMU_TM_CRC);
#else
    mavlink_imu_tm_t *packet = (mavlink_imu_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->acc_x = acc_x;
    packet->acc_y = acc_y;
    packet->acc_z = acc_z;
    packet->gyro_x = gyro_x;
    packet->gyro_y = gyro_y;
    packet->gyro_z = gyro_z;
    packet->mag_x = mag_x;
    packet->mag_y = mag_y;
    packet->mag_z = mag_z;
    mav_array_memcpy(packet->sensor_name, sensor_name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_TM, (const char *)packet, MAVLINK_MSG_ID_IMU_TM_MIN_LEN, MAVLINK_MSG_ID_IMU_TM_LEN, MAVLINK_MSG_ID_IMU_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE IMU_TM UNPACKING


/**
 * @brief Get field timestamp from imu_tm message
 *
 * @return [us] When was this logged
 */
static inline uint64_t mavlink_msg_imu_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sensor_name from imu_tm message
 *
 * @return  Sensor name
 */
static inline uint16_t mavlink_msg_imu_tm_get_sensor_name(const mavlink_message_t* msg, char *sensor_name)
{
    return _MAV_RETURN_char_array(msg, sensor_name, 20,  44);
}

/**
 * @brief Get field acc_x from imu_tm message
 *
 * @return [m/s2] X axis acceleration
 */
static inline float mavlink_msg_imu_tm_get_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field acc_y from imu_tm message
 *
 * @return [m/s2] Y axis acceleration
 */
static inline float mavlink_msg_imu_tm_get_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field acc_z from imu_tm message
 *
 * @return [m/s2] Z axis acceleration
 */
static inline float mavlink_msg_imu_tm_get_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gyro_x from imu_tm message
 *
 * @return [rad/s] X axis gyro
 */
static inline float mavlink_msg_imu_tm_get_gyro_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field gyro_y from imu_tm message
 *
 * @return [rad/s] Y axis gyro
 */
static inline float mavlink_msg_imu_tm_get_gyro_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field gyro_z from imu_tm message
 *
 * @return [rad/s] Z axis gyro
 */
static inline float mavlink_msg_imu_tm_get_gyro_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field mag_x from imu_tm message
 *
 * @return [uT] X axis compass
 */
static inline float mavlink_msg_imu_tm_get_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field mag_y from imu_tm message
 *
 * @return [uT] Y axis compass
 */
static inline float mavlink_msg_imu_tm_get_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field mag_z from imu_tm message
 *
 * @return [uT] Z axis compass
 */
static inline float mavlink_msg_imu_tm_get_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a imu_tm message into a struct
 *
 * @param msg The message to decode
 * @param imu_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_imu_tm_decode(const mavlink_message_t* msg, mavlink_imu_tm_t* imu_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    imu_tm->timestamp = mavlink_msg_imu_tm_get_timestamp(msg);
    imu_tm->acc_x = mavlink_msg_imu_tm_get_acc_x(msg);
    imu_tm->acc_y = mavlink_msg_imu_tm_get_acc_y(msg);
    imu_tm->acc_z = mavlink_msg_imu_tm_get_acc_z(msg);
    imu_tm->gyro_x = mavlink_msg_imu_tm_get_gyro_x(msg);
    imu_tm->gyro_y = mavlink_msg_imu_tm_get_gyro_y(msg);
    imu_tm->gyro_z = mavlink_msg_imu_tm_get_gyro_z(msg);
    imu_tm->mag_x = mavlink_msg_imu_tm_get_mag_x(msg);
    imu_tm->mag_y = mavlink_msg_imu_tm_get_mag_y(msg);
    imu_tm->mag_z = mavlink_msg_imu_tm_get_mag_z(msg);
    mavlink_msg_imu_tm_get_sensor_name(msg, imu_tm->sensor_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IMU_TM_LEN? msg->len : MAVLINK_MSG_ID_IMU_TM_LEN;
        memset(imu_tm, 0, MAVLINK_MSG_ID_IMU_TM_LEN);
    memcpy(imu_tm, _MAV_PAYLOAD(msg), len);
#endif
}
