#pragma once
// MESSAGE SET_STEPPER_ANGLE_TC PACKING

#define MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC 60


typedef struct __mavlink_set_stepper_angle_tc_t {
 float angle; /*<  Stepper angle in degrees*/
 uint8_t stepper_id; /*<  A member of the StepperList enum*/
} mavlink_set_stepper_angle_tc_t;

#define MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN 5
#define MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_MIN_LEN 5
#define MAVLINK_MSG_ID_60_LEN 5
#define MAVLINK_MSG_ID_60_MIN_LEN 5

#define MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_CRC 180
#define MAVLINK_MSG_ID_60_CRC 180



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_STEPPER_ANGLE_TC { \
    60, \
    "SET_STEPPER_ANGLE_TC", \
    2, \
    {  { "stepper_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_set_stepper_angle_tc_t, stepper_id) }, \
         { "angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_stepper_angle_tc_t, angle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_STEPPER_ANGLE_TC { \
    "SET_STEPPER_ANGLE_TC", \
    2, \
    {  { "stepper_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_set_stepper_angle_tc_t, stepper_id) }, \
         { "angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_stepper_angle_tc_t, angle) }, \
         } \
}
#endif

/**
 * @brief Pack a set_stepper_angle_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param stepper_id  A member of the StepperList enum
 * @param angle  Stepper angle in degrees
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_stepper_angle_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t stepper_id, float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN];
    _mav_put_float(buf, 0, angle);
    _mav_put_uint8_t(buf, 4, stepper_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN);
#else
    mavlink_set_stepper_angle_tc_t packet;
    packet.angle = angle;
    packet.stepper_id = stepper_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_CRC);
}

/**
 * @brief Pack a set_stepper_angle_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param stepper_id  A member of the StepperList enum
 * @param angle  Stepper angle in degrees
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_stepper_angle_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t stepper_id,float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN];
    _mav_put_float(buf, 0, angle);
    _mav_put_uint8_t(buf, 4, stepper_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN);
#else
    mavlink_set_stepper_angle_tc_t packet;
    packet.angle = angle;
    packet.stepper_id = stepper_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_CRC);
}

/**
 * @brief Encode a set_stepper_angle_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_stepper_angle_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_stepper_angle_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_stepper_angle_tc_t* set_stepper_angle_tc)
{
    return mavlink_msg_set_stepper_angle_tc_pack(system_id, component_id, msg, set_stepper_angle_tc->stepper_id, set_stepper_angle_tc->angle);
}

/**
 * @brief Encode a set_stepper_angle_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_stepper_angle_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_stepper_angle_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_stepper_angle_tc_t* set_stepper_angle_tc)
{
    return mavlink_msg_set_stepper_angle_tc_pack_chan(system_id, component_id, chan, msg, set_stepper_angle_tc->stepper_id, set_stepper_angle_tc->angle);
}

/**
 * @brief Send a set_stepper_angle_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param stepper_id  A member of the StepperList enum
 * @param angle  Stepper angle in degrees
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_stepper_angle_tc_send(mavlink_channel_t chan, uint8_t stepper_id, float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN];
    _mav_put_float(buf, 0, angle);
    _mav_put_uint8_t(buf, 4, stepper_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC, buf, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_CRC);
#else
    mavlink_set_stepper_angle_tc_t packet;
    packet.angle = angle;
    packet.stepper_id = stepper_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC, (const char *)&packet, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_CRC);
#endif
}

/**
 * @brief Send a set_stepper_angle_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_stepper_angle_tc_send_struct(mavlink_channel_t chan, const mavlink_set_stepper_angle_tc_t* set_stepper_angle_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_stepper_angle_tc_send(chan, set_stepper_angle_tc->stepper_id, set_stepper_angle_tc->angle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC, (const char *)set_stepper_angle_tc, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_stepper_angle_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t stepper_id, float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, angle);
    _mav_put_uint8_t(buf, 4, stepper_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC, buf, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_CRC);
#else
    mavlink_set_stepper_angle_tc_t *packet = (mavlink_set_stepper_angle_tc_t *)msgbuf;
    packet->angle = angle;
    packet->stepper_id = stepper_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC, (const char *)packet, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_STEPPER_ANGLE_TC UNPACKING


/**
 * @brief Get field stepper_id from set_stepper_angle_tc message
 *
 * @return  A member of the StepperList enum
 */
static inline uint8_t mavlink_msg_set_stepper_angle_tc_get_stepper_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field angle from set_stepper_angle_tc message
 *
 * @return  Stepper angle in degrees
 */
static inline float mavlink_msg_set_stepper_angle_tc_get_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a set_stepper_angle_tc message into a struct
 *
 * @param msg The message to decode
 * @param set_stepper_angle_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_stepper_angle_tc_decode(const mavlink_message_t* msg, mavlink_set_stepper_angle_tc_t* set_stepper_angle_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_stepper_angle_tc->angle = mavlink_msg_set_stepper_angle_tc_get_angle(msg);
    set_stepper_angle_tc->stepper_id = mavlink_msg_set_stepper_angle_tc_get_stepper_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN? msg->len : MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN;
        memset(set_stepper_angle_tc, 0, MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_LEN);
    memcpy(set_stepper_angle_tc, _MAV_PAYLOAD(msg), len);
#endif
}
