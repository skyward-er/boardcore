#pragma once
// MESSAGE SET_AEROBRAKE_ANGLE_TC PACKING

#define MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC 22

MAVPACKED(
typedef struct __mavlink_set_aerobrake_angle_tc_t {
 float angle; /*< [deg] Aerobrake angle*/
}) mavlink_set_aerobrake_angle_tc_t;

#define MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN 4
#define MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_MIN_LEN 4
#define MAVLINK_MSG_ID_22_LEN 4
#define MAVLINK_MSG_ID_22_MIN_LEN 4

#define MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_CRC 183
#define MAVLINK_MSG_ID_22_CRC 183



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_AEROBRAKE_ANGLE_TC { \
    22, \
    "SET_AEROBRAKE_ANGLE_TC", \
    1, \
    {  { "angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_aerobrake_angle_tc_t, angle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_AEROBRAKE_ANGLE_TC { \
    "SET_AEROBRAKE_ANGLE_TC", \
    1, \
    {  { "angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_aerobrake_angle_tc_t, angle) }, \
         } \
}
#endif

/**
 * @brief Pack a set_aerobrake_angle_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param angle [deg] Aerobrake angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_aerobrake_angle_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN];
    _mav_put_float(buf, 0, angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN);
#else
    mavlink_set_aerobrake_angle_tc_t packet;
    packet.angle = angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_CRC);
}

/**
 * @brief Pack a set_aerobrake_angle_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param angle [deg] Aerobrake angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_aerobrake_angle_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN];
    _mav_put_float(buf, 0, angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN);
#else
    mavlink_set_aerobrake_angle_tc_t packet;
    packet.angle = angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_CRC);
}

/**
 * @brief Encode a set_aerobrake_angle_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_aerobrake_angle_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_aerobrake_angle_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_aerobrake_angle_tc_t* set_aerobrake_angle_tc)
{
    return mavlink_msg_set_aerobrake_angle_tc_pack(system_id, component_id, msg, set_aerobrake_angle_tc->angle);
}

/**
 * @brief Encode a set_aerobrake_angle_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_aerobrake_angle_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_aerobrake_angle_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_aerobrake_angle_tc_t* set_aerobrake_angle_tc)
{
    return mavlink_msg_set_aerobrake_angle_tc_pack_chan(system_id, component_id, chan, msg, set_aerobrake_angle_tc->angle);
}

/**
 * @brief Send a set_aerobrake_angle_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param angle [deg] Aerobrake angle
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_aerobrake_angle_tc_send(mavlink_channel_t chan, float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN];
    _mav_put_float(buf, 0, angle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC, buf, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_CRC);
#else
    mavlink_set_aerobrake_angle_tc_t packet;
    packet.angle = angle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC, (const char *)&packet, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_CRC);
#endif
}

/**
 * @brief Send a set_aerobrake_angle_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_aerobrake_angle_tc_send_struct(mavlink_channel_t chan, const mavlink_set_aerobrake_angle_tc_t* set_aerobrake_angle_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_aerobrake_angle_tc_send(chan, set_aerobrake_angle_tc->angle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC, (const char *)set_aerobrake_angle_tc, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_aerobrake_angle_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, angle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC, buf, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_CRC);
#else
    mavlink_set_aerobrake_angle_tc_t *packet = (mavlink_set_aerobrake_angle_tc_t *)msgbuf;
    packet->angle = angle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC, (const char *)packet, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_AEROBRAKE_ANGLE_TC UNPACKING


/**
 * @brief Get field angle from set_aerobrake_angle_tc message
 *
 * @return [deg] Aerobrake angle
 */
static inline float mavlink_msg_set_aerobrake_angle_tc_get_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a set_aerobrake_angle_tc message into a struct
 *
 * @param msg The message to decode
 * @param set_aerobrake_angle_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_aerobrake_angle_tc_decode(const mavlink_message_t* msg, mavlink_set_aerobrake_angle_tc_t* set_aerobrake_angle_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_aerobrake_angle_tc->angle = mavlink_msg_set_aerobrake_angle_tc_get_angle(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN? msg->len : MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN;
        memset(set_aerobrake_angle_tc, 0, MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_LEN);
    memcpy(set_aerobrake_angle_tc, _MAV_PAYLOAD(msg), len);
#endif
}
