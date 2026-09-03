#pragma once
// MESSAGE SET_TARS3_PARAMS_TC PACKING

#define MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC 38


typedef struct __mavlink_set_tars3_params_tc_t {
 float mass_target; /*< [kg] Target mass*/
 float pressure_target; /*< [Bar] Target pressure*/
} mavlink_set_tars3_params_tc_t;

#define MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN 8
#define MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_MIN_LEN 8
#define MAVLINK_MSG_ID_38_LEN 8
#define MAVLINK_MSG_ID_38_MIN_LEN 8

#define MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_CRC 250
#define MAVLINK_MSG_ID_38_CRC 250



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_TARS3_PARAMS_TC { \
    38, \
    "SET_TARS3_PARAMS_TC", \
    2, \
    {  { "mass_target", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_tars3_params_tc_t, mass_target) }, \
         { "pressure_target", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_tars3_params_tc_t, pressure_target) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_TARS3_PARAMS_TC { \
    "SET_TARS3_PARAMS_TC", \
    2, \
    {  { "mass_target", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_tars3_params_tc_t, mass_target) }, \
         { "pressure_target", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_tars3_params_tc_t, pressure_target) }, \
         } \
}
#endif

/**
 * @brief Pack a set_tars3_params_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mass_target [kg] Target mass
 * @param pressure_target [Bar] Target pressure
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_tars3_params_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float mass_target, float pressure_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN];
    _mav_put_float(buf, 0, mass_target);
    _mav_put_float(buf, 4, pressure_target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN);
#else
    mavlink_set_tars3_params_tc_t packet;
    packet.mass_target = mass_target;
    packet.pressure_target = pressure_target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_MIN_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_CRC);
}

/**
 * @brief Pack a set_tars3_params_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mass_target [kg] Target mass
 * @param pressure_target [Bar] Target pressure
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_tars3_params_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float mass_target,float pressure_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN];
    _mav_put_float(buf, 0, mass_target);
    _mav_put_float(buf, 4, pressure_target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN);
#else
    mavlink_set_tars3_params_tc_t packet;
    packet.mass_target = mass_target;
    packet.pressure_target = pressure_target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_MIN_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_CRC);
}

/**
 * @brief Encode a set_tars3_params_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_tars3_params_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_tars3_params_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_tars3_params_tc_t* set_tars3_params_tc)
{
    return mavlink_msg_set_tars3_params_tc_pack(system_id, component_id, msg, set_tars3_params_tc->mass_target, set_tars3_params_tc->pressure_target);
}

/**
 * @brief Encode a set_tars3_params_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_tars3_params_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_tars3_params_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_tars3_params_tc_t* set_tars3_params_tc)
{
    return mavlink_msg_set_tars3_params_tc_pack_chan(system_id, component_id, chan, msg, set_tars3_params_tc->mass_target, set_tars3_params_tc->pressure_target);
}

/**
 * @brief Send a set_tars3_params_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param mass_target [kg] Target mass
 * @param pressure_target [Bar] Target pressure
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_tars3_params_tc_send(mavlink_channel_t chan, float mass_target, float pressure_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN];
    _mav_put_float(buf, 0, mass_target);
    _mav_put_float(buf, 4, pressure_target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC, buf, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_MIN_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_CRC);
#else
    mavlink_set_tars3_params_tc_t packet;
    packet.mass_target = mass_target;
    packet.pressure_target = pressure_target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC, (const char *)&packet, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_MIN_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_CRC);
#endif
}

/**
 * @brief Send a set_tars3_params_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_tars3_params_tc_send_struct(mavlink_channel_t chan, const mavlink_set_tars3_params_tc_t* set_tars3_params_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_tars3_params_tc_send(chan, set_tars3_params_tc->mass_target, set_tars3_params_tc->pressure_target);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC, (const char *)set_tars3_params_tc, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_MIN_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_tars3_params_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float mass_target, float pressure_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, mass_target);
    _mav_put_float(buf, 4, pressure_target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC, buf, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_MIN_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_CRC);
#else
    mavlink_set_tars3_params_tc_t *packet = (mavlink_set_tars3_params_tc_t *)msgbuf;
    packet->mass_target = mass_target;
    packet->pressure_target = pressure_target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC, (const char *)packet, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_MIN_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_TARS3_PARAMS_TC UNPACKING


/**
 * @brief Get field mass_target from set_tars3_params_tc message
 *
 * @return [kg] Target mass
 */
static inline float mavlink_msg_set_tars3_params_tc_get_mass_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pressure_target from set_tars3_params_tc message
 *
 * @return [Bar] Target pressure
 */
static inline float mavlink_msg_set_tars3_params_tc_get_pressure_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a set_tars3_params_tc message into a struct
 *
 * @param msg The message to decode
 * @param set_tars3_params_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_tars3_params_tc_decode(const mavlink_message_t* msg, mavlink_set_tars3_params_tc_t* set_tars3_params_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_tars3_params_tc->mass_target = mavlink_msg_set_tars3_params_tc_get_mass_target(msg);
    set_tars3_params_tc->pressure_target = mavlink_msg_set_tars3_params_tc_get_pressure_target(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN? msg->len : MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN;
        memset(set_tars3_params_tc, 0, MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC_LEN);
    memcpy(set_tars3_params_tc, _MAV_PAYLOAD(msg), len);
#endif
}
