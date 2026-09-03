#pragma once
// MESSAGE SET_ORIENTATION_QUAT_TC PACKING

#define MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC 12


typedef struct __mavlink_set_orientation_quat_tc_t {
 float quat_x; /*<  Quaternion x component*/
 float quat_y; /*<  Quaternion y component*/
 float quat_z; /*<  Quaternion z component*/
 float quat_w; /*<  Quaternion w component*/
} mavlink_set_orientation_quat_tc_t;

#define MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN 16
#define MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_MIN_LEN 16
#define MAVLINK_MSG_ID_12_LEN 16
#define MAVLINK_MSG_ID_12_MIN_LEN 16

#define MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_CRC 168
#define MAVLINK_MSG_ID_12_CRC 168



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_ORIENTATION_QUAT_TC { \
    12, \
    "SET_ORIENTATION_QUAT_TC", \
    4, \
    {  { "quat_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_orientation_quat_tc_t, quat_x) }, \
         { "quat_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_orientation_quat_tc_t, quat_y) }, \
         { "quat_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_orientation_quat_tc_t, quat_z) }, \
         { "quat_w", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_orientation_quat_tc_t, quat_w) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_ORIENTATION_QUAT_TC { \
    "SET_ORIENTATION_QUAT_TC", \
    4, \
    {  { "quat_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_orientation_quat_tc_t, quat_x) }, \
         { "quat_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_orientation_quat_tc_t, quat_y) }, \
         { "quat_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_orientation_quat_tc_t, quat_z) }, \
         { "quat_w", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_orientation_quat_tc_t, quat_w) }, \
         } \
}
#endif

/**
 * @brief Pack a set_orientation_quat_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param quat_x  Quaternion x component
 * @param quat_y  Quaternion y component
 * @param quat_z  Quaternion z component
 * @param quat_w  Quaternion w component
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_orientation_quat_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float quat_x, float quat_y, float quat_z, float quat_w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN];
    _mav_put_float(buf, 0, quat_x);
    _mav_put_float(buf, 4, quat_y);
    _mav_put_float(buf, 8, quat_z);
    _mav_put_float(buf, 12, quat_w);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN);
#else
    mavlink_set_orientation_quat_tc_t packet;
    packet.quat_x = quat_x;
    packet.quat_y = quat_y;
    packet.quat_z = quat_z;
    packet.quat_w = quat_w;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_CRC);
}

/**
 * @brief Pack a set_orientation_quat_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param quat_x  Quaternion x component
 * @param quat_y  Quaternion y component
 * @param quat_z  Quaternion z component
 * @param quat_w  Quaternion w component
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_orientation_quat_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float quat_x,float quat_y,float quat_z,float quat_w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN];
    _mav_put_float(buf, 0, quat_x);
    _mav_put_float(buf, 4, quat_y);
    _mav_put_float(buf, 8, quat_z);
    _mav_put_float(buf, 12, quat_w);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN);
#else
    mavlink_set_orientation_quat_tc_t packet;
    packet.quat_x = quat_x;
    packet.quat_y = quat_y;
    packet.quat_z = quat_z;
    packet.quat_w = quat_w;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_CRC);
}

/**
 * @brief Encode a set_orientation_quat_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_orientation_quat_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_orientation_quat_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_orientation_quat_tc_t* set_orientation_quat_tc)
{
    return mavlink_msg_set_orientation_quat_tc_pack(system_id, component_id, msg, set_orientation_quat_tc->quat_x, set_orientation_quat_tc->quat_y, set_orientation_quat_tc->quat_z, set_orientation_quat_tc->quat_w);
}

/**
 * @brief Encode a set_orientation_quat_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_orientation_quat_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_orientation_quat_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_orientation_quat_tc_t* set_orientation_quat_tc)
{
    return mavlink_msg_set_orientation_quat_tc_pack_chan(system_id, component_id, chan, msg, set_orientation_quat_tc->quat_x, set_orientation_quat_tc->quat_y, set_orientation_quat_tc->quat_z, set_orientation_quat_tc->quat_w);
}

/**
 * @brief Send a set_orientation_quat_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param quat_x  Quaternion x component
 * @param quat_y  Quaternion y component
 * @param quat_z  Quaternion z component
 * @param quat_w  Quaternion w component
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_orientation_quat_tc_send(mavlink_channel_t chan, float quat_x, float quat_y, float quat_z, float quat_w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN];
    _mav_put_float(buf, 0, quat_x);
    _mav_put_float(buf, 4, quat_y);
    _mav_put_float(buf, 8, quat_z);
    _mav_put_float(buf, 12, quat_w);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC, buf, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_CRC);
#else
    mavlink_set_orientation_quat_tc_t packet;
    packet.quat_x = quat_x;
    packet.quat_y = quat_y;
    packet.quat_z = quat_z;
    packet.quat_w = quat_w;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC, (const char *)&packet, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_CRC);
#endif
}

/**
 * @brief Send a set_orientation_quat_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_orientation_quat_tc_send_struct(mavlink_channel_t chan, const mavlink_set_orientation_quat_tc_t* set_orientation_quat_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_orientation_quat_tc_send(chan, set_orientation_quat_tc->quat_x, set_orientation_quat_tc->quat_y, set_orientation_quat_tc->quat_z, set_orientation_quat_tc->quat_w);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC, (const char *)set_orientation_quat_tc, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_orientation_quat_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float quat_x, float quat_y, float quat_z, float quat_w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, quat_x);
    _mav_put_float(buf, 4, quat_y);
    _mav_put_float(buf, 8, quat_z);
    _mav_put_float(buf, 12, quat_w);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC, buf, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_CRC);
#else
    mavlink_set_orientation_quat_tc_t *packet = (mavlink_set_orientation_quat_tc_t *)msgbuf;
    packet->quat_x = quat_x;
    packet->quat_y = quat_y;
    packet->quat_z = quat_z;
    packet->quat_w = quat_w;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC, (const char *)packet, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_ORIENTATION_QUAT_TC UNPACKING


/**
 * @brief Get field quat_x from set_orientation_quat_tc message
 *
 * @return  Quaternion x component
 */
static inline float mavlink_msg_set_orientation_quat_tc_get_quat_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field quat_y from set_orientation_quat_tc message
 *
 * @return  Quaternion y component
 */
static inline float mavlink_msg_set_orientation_quat_tc_get_quat_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field quat_z from set_orientation_quat_tc message
 *
 * @return  Quaternion z component
 */
static inline float mavlink_msg_set_orientation_quat_tc_get_quat_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field quat_w from set_orientation_quat_tc message
 *
 * @return  Quaternion w component
 */
static inline float mavlink_msg_set_orientation_quat_tc_get_quat_w(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a set_orientation_quat_tc message into a struct
 *
 * @param msg The message to decode
 * @param set_orientation_quat_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_orientation_quat_tc_decode(const mavlink_message_t* msg, mavlink_set_orientation_quat_tc_t* set_orientation_quat_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_orientation_quat_tc->quat_x = mavlink_msg_set_orientation_quat_tc_get_quat_x(msg);
    set_orientation_quat_tc->quat_y = mavlink_msg_set_orientation_quat_tc_get_quat_y(msg);
    set_orientation_quat_tc->quat_z = mavlink_msg_set_orientation_quat_tc_get_quat_z(msg);
    set_orientation_quat_tc->quat_w = mavlink_msg_set_orientation_quat_tc_get_quat_w(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN? msg->len : MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN;
        memset(set_orientation_quat_tc, 0, MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_LEN);
    memcpy(set_orientation_quat_tc, _MAV_PAYLOAD(msg), len);
#endif
}
