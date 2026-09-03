#pragma once
// MESSAGE LIS3MDL_TM PACKING

#define MAVLINK_MSG_ID_LIS3MDL_TM 174

MAVPACKED(
typedef struct __mavlink_lis3mdl_tm_t {
 uint64_t timestamp; /*< [ms] When was this logged*/
 float mag_x; /*< [rad/s] X axis compass*/
 float mag_y; /*< [rad/s] Y axis compass*/
 float mag_z; /*< [rad/s] Z axis compass*/
 float temp; /*< [deg] Temperature*/
}) mavlink_lis3mdl_tm_t;

#define MAVLINK_MSG_ID_LIS3MDL_TM_LEN 24
#define MAVLINK_MSG_ID_LIS3MDL_TM_MIN_LEN 24
#define MAVLINK_MSG_ID_174_LEN 24
#define MAVLINK_MSG_ID_174_MIN_LEN 24

#define MAVLINK_MSG_ID_LIS3MDL_TM_CRC 129
#define MAVLINK_MSG_ID_174_CRC 129



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LIS3MDL_TM { \
    174, \
    "LIS3MDL_TM", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lis3mdl_tm_t, timestamp) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_lis3mdl_tm_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_lis3mdl_tm_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_lis3mdl_tm_t, mag_z) }, \
         { "temp", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_lis3mdl_tm_t, temp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LIS3MDL_TM { \
    "LIS3MDL_TM", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lis3mdl_tm_t, timestamp) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_lis3mdl_tm_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_lis3mdl_tm_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_lis3mdl_tm_t, mag_z) }, \
         { "temp", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_lis3mdl_tm_t, temp) }, \
         } \
}
#endif

/**
 * @brief Pack a lis3mdl_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] When was this logged
 * @param mag_x [rad/s] X axis compass
 * @param mag_y [rad/s] Y axis compass
 * @param mag_z [rad/s] Z axis compass
 * @param temp [deg] Temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lis3mdl_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float mag_x, float mag_y, float mag_z, float temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LIS3MDL_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, mag_x);
    _mav_put_float(buf, 12, mag_y);
    _mav_put_float(buf, 16, mag_z);
    _mav_put_float(buf, 20, temp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LIS3MDL_TM_LEN);
#else
    mavlink_lis3mdl_tm_t packet;
    packet.timestamp = timestamp;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.temp = temp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LIS3MDL_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LIS3MDL_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LIS3MDL_TM_MIN_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_CRC);
}

/**
 * @brief Pack a lis3mdl_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] When was this logged
 * @param mag_x [rad/s] X axis compass
 * @param mag_y [rad/s] Y axis compass
 * @param mag_z [rad/s] Z axis compass
 * @param temp [deg] Temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lis3mdl_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float mag_x,float mag_y,float mag_z,float temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LIS3MDL_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, mag_x);
    _mav_put_float(buf, 12, mag_y);
    _mav_put_float(buf, 16, mag_z);
    _mav_put_float(buf, 20, temp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LIS3MDL_TM_LEN);
#else
    mavlink_lis3mdl_tm_t packet;
    packet.timestamp = timestamp;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.temp = temp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LIS3MDL_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LIS3MDL_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LIS3MDL_TM_MIN_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_CRC);
}

/**
 * @brief Encode a lis3mdl_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lis3mdl_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lis3mdl_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lis3mdl_tm_t* lis3mdl_tm)
{
    return mavlink_msg_lis3mdl_tm_pack(system_id, component_id, msg, lis3mdl_tm->timestamp, lis3mdl_tm->mag_x, lis3mdl_tm->mag_y, lis3mdl_tm->mag_z, lis3mdl_tm->temp);
}

/**
 * @brief Encode a lis3mdl_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lis3mdl_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lis3mdl_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lis3mdl_tm_t* lis3mdl_tm)
{
    return mavlink_msg_lis3mdl_tm_pack_chan(system_id, component_id, chan, msg, lis3mdl_tm->timestamp, lis3mdl_tm->mag_x, lis3mdl_tm->mag_y, lis3mdl_tm->mag_z, lis3mdl_tm->temp);
}

/**
 * @brief Send a lis3mdl_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] When was this logged
 * @param mag_x [rad/s] X axis compass
 * @param mag_y [rad/s] Y axis compass
 * @param mag_z [rad/s] Z axis compass
 * @param temp [deg] Temperature
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lis3mdl_tm_send(mavlink_channel_t chan, uint64_t timestamp, float mag_x, float mag_y, float mag_z, float temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LIS3MDL_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, mag_x);
    _mav_put_float(buf, 12, mag_y);
    _mav_put_float(buf, 16, mag_z);
    _mav_put_float(buf, 20, temp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIS3MDL_TM, buf, MAVLINK_MSG_ID_LIS3MDL_TM_MIN_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_CRC);
#else
    mavlink_lis3mdl_tm_t packet;
    packet.timestamp = timestamp;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.temp = temp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIS3MDL_TM, (const char *)&packet, MAVLINK_MSG_ID_LIS3MDL_TM_MIN_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_CRC);
#endif
}

/**
 * @brief Send a lis3mdl_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lis3mdl_tm_send_struct(mavlink_channel_t chan, const mavlink_lis3mdl_tm_t* lis3mdl_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lis3mdl_tm_send(chan, lis3mdl_tm->timestamp, lis3mdl_tm->mag_x, lis3mdl_tm->mag_y, lis3mdl_tm->mag_z, lis3mdl_tm->temp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIS3MDL_TM, (const char *)lis3mdl_tm, MAVLINK_MSG_ID_LIS3MDL_TM_MIN_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_LIS3MDL_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lis3mdl_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float mag_x, float mag_y, float mag_z, float temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, mag_x);
    _mav_put_float(buf, 12, mag_y);
    _mav_put_float(buf, 16, mag_z);
    _mav_put_float(buf, 20, temp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIS3MDL_TM, buf, MAVLINK_MSG_ID_LIS3MDL_TM_MIN_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_CRC);
#else
    mavlink_lis3mdl_tm_t *packet = (mavlink_lis3mdl_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->mag_x = mag_x;
    packet->mag_y = mag_y;
    packet->mag_z = mag_z;
    packet->temp = temp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIS3MDL_TM, (const char *)packet, MAVLINK_MSG_ID_LIS3MDL_TM_MIN_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_LEN, MAVLINK_MSG_ID_LIS3MDL_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE LIS3MDL_TM UNPACKING


/**
 * @brief Get field timestamp from lis3mdl_tm message
 *
 * @return [ms] When was this logged
 */
static inline uint64_t mavlink_msg_lis3mdl_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field mag_x from lis3mdl_tm message
 *
 * @return [rad/s] X axis compass
 */
static inline float mavlink_msg_lis3mdl_tm_get_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field mag_y from lis3mdl_tm message
 *
 * @return [rad/s] Y axis compass
 */
static inline float mavlink_msg_lis3mdl_tm_get_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field mag_z from lis3mdl_tm message
 *
 * @return [rad/s] Z axis compass
 */
static inline float mavlink_msg_lis3mdl_tm_get_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field temp from lis3mdl_tm message
 *
 * @return [deg] Temperature
 */
static inline float mavlink_msg_lis3mdl_tm_get_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a lis3mdl_tm message into a struct
 *
 * @param msg The message to decode
 * @param lis3mdl_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_lis3mdl_tm_decode(const mavlink_message_t* msg, mavlink_lis3mdl_tm_t* lis3mdl_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lis3mdl_tm->timestamp = mavlink_msg_lis3mdl_tm_get_timestamp(msg);
    lis3mdl_tm->mag_x = mavlink_msg_lis3mdl_tm_get_mag_x(msg);
    lis3mdl_tm->mag_y = mavlink_msg_lis3mdl_tm_get_mag_y(msg);
    lis3mdl_tm->mag_z = mavlink_msg_lis3mdl_tm_get_mag_z(msg);
    lis3mdl_tm->temp = mavlink_msg_lis3mdl_tm_get_temp(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LIS3MDL_TM_LEN? msg->len : MAVLINK_MSG_ID_LIS3MDL_TM_LEN;
        memset(lis3mdl_tm, 0, MAVLINK_MSG_ID_LIS3MDL_TM_LEN);
    memcpy(lis3mdl_tm, _MAV_PAYLOAD(msg), len);
#endif
}
