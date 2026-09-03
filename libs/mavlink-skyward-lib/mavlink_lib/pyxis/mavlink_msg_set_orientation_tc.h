#pragma once
// MESSAGE SET_ORIENTATION_TC PACKING

#define MAVLINK_MSG_ID_SET_ORIENTATION_TC 11


typedef struct __mavlink_set_orientation_tc_t {
 float yaw; /*< [deg] Yaw angle*/
 float pitch; /*< [deg] Pitch angle*/
 float roll; /*< [deg] Roll angle*/
} mavlink_set_orientation_tc_t;

#define MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN 12
#define MAVLINK_MSG_ID_SET_ORIENTATION_TC_MIN_LEN 12
#define MAVLINK_MSG_ID_11_LEN 12
#define MAVLINK_MSG_ID_11_MIN_LEN 12

#define MAVLINK_MSG_ID_SET_ORIENTATION_TC_CRC 71
#define MAVLINK_MSG_ID_11_CRC 71



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_ORIENTATION_TC { \
    11, \
    "SET_ORIENTATION_TC", \
    3, \
    {  { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_orientation_tc_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_orientation_tc_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_orientation_tc_t, roll) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_ORIENTATION_TC { \
    "SET_ORIENTATION_TC", \
    3, \
    {  { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_orientation_tc_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_orientation_tc_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_orientation_tc_t, roll) }, \
         } \
}
#endif

/**
 * @brief Pack a set_orientation_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param yaw [deg] Yaw angle
 * @param pitch [deg] Pitch angle
 * @param roll [deg] Roll angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_orientation_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float yaw, float pitch, float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, roll);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN);
#else
    mavlink_set_orientation_tc_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ORIENTATION_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_ORIENTATION_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_CRC);
}

/**
 * @brief Pack a set_orientation_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param yaw [deg] Yaw angle
 * @param pitch [deg] Pitch angle
 * @param roll [deg] Roll angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_orientation_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float yaw,float pitch,float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, roll);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN);
#else
    mavlink_set_orientation_tc_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ORIENTATION_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_ORIENTATION_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_CRC);
}

/**
 * @brief Encode a set_orientation_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_orientation_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_orientation_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_orientation_tc_t* set_orientation_tc)
{
    return mavlink_msg_set_orientation_tc_pack(system_id, component_id, msg, set_orientation_tc->yaw, set_orientation_tc->pitch, set_orientation_tc->roll);
}

/**
 * @brief Encode a set_orientation_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_orientation_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_orientation_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_orientation_tc_t* set_orientation_tc)
{
    return mavlink_msg_set_orientation_tc_pack_chan(system_id, component_id, chan, msg, set_orientation_tc->yaw, set_orientation_tc->pitch, set_orientation_tc->roll);
}

/**
 * @brief Send a set_orientation_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param yaw [deg] Yaw angle
 * @param pitch [deg] Pitch angle
 * @param roll [deg] Roll angle
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_orientation_tc_send(mavlink_channel_t chan, float yaw, float pitch, float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, roll);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ORIENTATION_TC, buf, MAVLINK_MSG_ID_SET_ORIENTATION_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_CRC);
#else
    mavlink_set_orientation_tc_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ORIENTATION_TC, (const char *)&packet, MAVLINK_MSG_ID_SET_ORIENTATION_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_CRC);
#endif
}

/**
 * @brief Send a set_orientation_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_orientation_tc_send_struct(mavlink_channel_t chan, const mavlink_set_orientation_tc_t* set_orientation_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_orientation_tc_send(chan, set_orientation_tc->yaw, set_orientation_tc->pitch, set_orientation_tc->roll);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ORIENTATION_TC, (const char *)set_orientation_tc, MAVLINK_MSG_ID_SET_ORIENTATION_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_orientation_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float yaw, float pitch, float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, roll);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ORIENTATION_TC, buf, MAVLINK_MSG_ID_SET_ORIENTATION_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_CRC);
#else
    mavlink_set_orientation_tc_t *packet = (mavlink_set_orientation_tc_t *)msgbuf;
    packet->yaw = yaw;
    packet->pitch = pitch;
    packet->roll = roll;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ORIENTATION_TC, (const char *)packet, MAVLINK_MSG_ID_SET_ORIENTATION_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_ORIENTATION_TC UNPACKING


/**
 * @brief Get field yaw from set_orientation_tc message
 *
 * @return [deg] Yaw angle
 */
static inline float mavlink_msg_set_orientation_tc_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from set_orientation_tc message
 *
 * @return [deg] Pitch angle
 */
static inline float mavlink_msg_set_orientation_tc_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field roll from set_orientation_tc message
 *
 * @return [deg] Roll angle
 */
static inline float mavlink_msg_set_orientation_tc_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a set_orientation_tc message into a struct
 *
 * @param msg The message to decode
 * @param set_orientation_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_orientation_tc_decode(const mavlink_message_t* msg, mavlink_set_orientation_tc_t* set_orientation_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_orientation_tc->yaw = mavlink_msg_set_orientation_tc_get_yaw(msg);
    set_orientation_tc->pitch = mavlink_msg_set_orientation_tc_get_pitch(msg);
    set_orientation_tc->roll = mavlink_msg_set_orientation_tc_get_roll(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN? msg->len : MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN;
        memset(set_orientation_tc, 0, MAVLINK_MSG_ID_SET_ORIENTATION_TC_LEN);
    memcpy(set_orientation_tc, _MAV_PAYLOAD(msg), len);
#endif
}
