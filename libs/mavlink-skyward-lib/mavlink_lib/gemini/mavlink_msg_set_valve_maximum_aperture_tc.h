#pragma once
// MESSAGE SET_VALVE_MAXIMUM_APERTURE_TC PACKING

#define MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC 18


typedef struct __mavlink_set_valve_maximum_aperture_tc_t {
 float maximum_aperture; /*<  Maximum aperture*/
 uint8_t servo_id; /*<  A member of the ServosList enum*/
} mavlink_set_valve_maximum_aperture_tc_t;

#define MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN 5
#define MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_MIN_LEN 5
#define MAVLINK_MSG_ID_18_LEN 5
#define MAVLINK_MSG_ID_18_MIN_LEN 5

#define MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_CRC 22
#define MAVLINK_MSG_ID_18_CRC 22



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_VALVE_MAXIMUM_APERTURE_TC { \
    18, \
    "SET_VALVE_MAXIMUM_APERTURE_TC", \
    2, \
    {  { "servo_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_set_valve_maximum_aperture_tc_t, servo_id) }, \
         { "maximum_aperture", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_valve_maximum_aperture_tc_t, maximum_aperture) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_VALVE_MAXIMUM_APERTURE_TC { \
    "SET_VALVE_MAXIMUM_APERTURE_TC", \
    2, \
    {  { "servo_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_set_valve_maximum_aperture_tc_t, servo_id) }, \
         { "maximum_aperture", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_valve_maximum_aperture_tc_t, maximum_aperture) }, \
         } \
}
#endif

/**
 * @brief Pack a set_valve_maximum_aperture_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param servo_id  A member of the ServosList enum
 * @param maximum_aperture  Maximum aperture
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_valve_maximum_aperture_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t servo_id, float maximum_aperture)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN];
    _mav_put_float(buf, 0, maximum_aperture);
    _mav_put_uint8_t(buf, 4, servo_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN);
#else
    mavlink_set_valve_maximum_aperture_tc_t packet;
    packet.maximum_aperture = maximum_aperture;
    packet.servo_id = servo_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_CRC);
}

/**
 * @brief Pack a set_valve_maximum_aperture_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param servo_id  A member of the ServosList enum
 * @param maximum_aperture  Maximum aperture
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_valve_maximum_aperture_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t servo_id,float maximum_aperture)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN];
    _mav_put_float(buf, 0, maximum_aperture);
    _mav_put_uint8_t(buf, 4, servo_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN);
#else
    mavlink_set_valve_maximum_aperture_tc_t packet;
    packet.maximum_aperture = maximum_aperture;
    packet.servo_id = servo_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_CRC);
}

/**
 * @brief Encode a set_valve_maximum_aperture_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_valve_maximum_aperture_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_valve_maximum_aperture_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_valve_maximum_aperture_tc_t* set_valve_maximum_aperture_tc)
{
    return mavlink_msg_set_valve_maximum_aperture_tc_pack(system_id, component_id, msg, set_valve_maximum_aperture_tc->servo_id, set_valve_maximum_aperture_tc->maximum_aperture);
}

/**
 * @brief Encode a set_valve_maximum_aperture_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_valve_maximum_aperture_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_valve_maximum_aperture_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_valve_maximum_aperture_tc_t* set_valve_maximum_aperture_tc)
{
    return mavlink_msg_set_valve_maximum_aperture_tc_pack_chan(system_id, component_id, chan, msg, set_valve_maximum_aperture_tc->servo_id, set_valve_maximum_aperture_tc->maximum_aperture);
}

/**
 * @brief Send a set_valve_maximum_aperture_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param servo_id  A member of the ServosList enum
 * @param maximum_aperture  Maximum aperture
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_valve_maximum_aperture_tc_send(mavlink_channel_t chan, uint8_t servo_id, float maximum_aperture)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN];
    _mav_put_float(buf, 0, maximum_aperture);
    _mav_put_uint8_t(buf, 4, servo_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC, buf, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_CRC);
#else
    mavlink_set_valve_maximum_aperture_tc_t packet;
    packet.maximum_aperture = maximum_aperture;
    packet.servo_id = servo_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC, (const char *)&packet, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_CRC);
#endif
}

/**
 * @brief Send a set_valve_maximum_aperture_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_valve_maximum_aperture_tc_send_struct(mavlink_channel_t chan, const mavlink_set_valve_maximum_aperture_tc_t* set_valve_maximum_aperture_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_valve_maximum_aperture_tc_send(chan, set_valve_maximum_aperture_tc->servo_id, set_valve_maximum_aperture_tc->maximum_aperture);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC, (const char *)set_valve_maximum_aperture_tc, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_valve_maximum_aperture_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t servo_id, float maximum_aperture)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, maximum_aperture);
    _mav_put_uint8_t(buf, 4, servo_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC, buf, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_CRC);
#else
    mavlink_set_valve_maximum_aperture_tc_t *packet = (mavlink_set_valve_maximum_aperture_tc_t *)msgbuf;
    packet->maximum_aperture = maximum_aperture;
    packet->servo_id = servo_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC, (const char *)packet, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_MIN_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_VALVE_MAXIMUM_APERTURE_TC UNPACKING


/**
 * @brief Get field servo_id from set_valve_maximum_aperture_tc message
 *
 * @return  A member of the ServosList enum
 */
static inline uint8_t mavlink_msg_set_valve_maximum_aperture_tc_get_servo_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field maximum_aperture from set_valve_maximum_aperture_tc message
 *
 * @return  Maximum aperture
 */
static inline float mavlink_msg_set_valve_maximum_aperture_tc_get_maximum_aperture(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a set_valve_maximum_aperture_tc message into a struct
 *
 * @param msg The message to decode
 * @param set_valve_maximum_aperture_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_valve_maximum_aperture_tc_decode(const mavlink_message_t* msg, mavlink_set_valve_maximum_aperture_tc_t* set_valve_maximum_aperture_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_valve_maximum_aperture_tc->maximum_aperture = mavlink_msg_set_valve_maximum_aperture_tc_get_maximum_aperture(msg);
    set_valve_maximum_aperture_tc->servo_id = mavlink_msg_set_valve_maximum_aperture_tc_get_servo_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN? msg->len : MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN;
        memset(set_valve_maximum_aperture_tc, 0, MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_LEN);
    memcpy(set_valve_maximum_aperture_tc, _MAV_PAYLOAD(msg), len);
#endif
}
