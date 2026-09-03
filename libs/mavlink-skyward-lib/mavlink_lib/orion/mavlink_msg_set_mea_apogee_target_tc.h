#pragma once
// MESSAGE SET_MEA_APOGEE_TARGET_TC PACKING

#define MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC 20


typedef struct __mavlink_set_mea_apogee_target_tc_t {
 float apogee_target; /*< [m] Apogee target*/
} mavlink_set_mea_apogee_target_tc_t;

#define MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN 4
#define MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_MIN_LEN 4
#define MAVLINK_MSG_ID_20_LEN 4
#define MAVLINK_MSG_ID_20_MIN_LEN 4

#define MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_CRC 205
#define MAVLINK_MSG_ID_20_CRC 205



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_MEA_APOGEE_TARGET_TC { \
    20, \
    "SET_MEA_APOGEE_TARGET_TC", \
    1, \
    {  { "apogee_target", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_mea_apogee_target_tc_t, apogee_target) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_MEA_APOGEE_TARGET_TC { \
    "SET_MEA_APOGEE_TARGET_TC", \
    1, \
    {  { "apogee_target", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_mea_apogee_target_tc_t, apogee_target) }, \
         } \
}
#endif

/**
 * @brief Pack a set_mea_apogee_target_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param apogee_target [m] Apogee target
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_mea_apogee_target_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float apogee_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN];
    _mav_put_float(buf, 0, apogee_target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN);
#else
    mavlink_set_mea_apogee_target_tc_t packet;
    packet.apogee_target = apogee_target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_MIN_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_CRC);
}

/**
 * @brief Pack a set_mea_apogee_target_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param apogee_target [m] Apogee target
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_mea_apogee_target_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float apogee_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN];
    _mav_put_float(buf, 0, apogee_target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN);
#else
    mavlink_set_mea_apogee_target_tc_t packet;
    packet.apogee_target = apogee_target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_MIN_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_CRC);
}

/**
 * @brief Encode a set_mea_apogee_target_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_mea_apogee_target_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_mea_apogee_target_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_mea_apogee_target_tc_t* set_mea_apogee_target_tc)
{
    return mavlink_msg_set_mea_apogee_target_tc_pack(system_id, component_id, msg, set_mea_apogee_target_tc->apogee_target);
}

/**
 * @brief Encode a set_mea_apogee_target_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_mea_apogee_target_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_mea_apogee_target_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_mea_apogee_target_tc_t* set_mea_apogee_target_tc)
{
    return mavlink_msg_set_mea_apogee_target_tc_pack_chan(system_id, component_id, chan, msg, set_mea_apogee_target_tc->apogee_target);
}

/**
 * @brief Send a set_mea_apogee_target_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param apogee_target [m] Apogee target
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_mea_apogee_target_tc_send(mavlink_channel_t chan, float apogee_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN];
    _mav_put_float(buf, 0, apogee_target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC, buf, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_MIN_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_CRC);
#else
    mavlink_set_mea_apogee_target_tc_t packet;
    packet.apogee_target = apogee_target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC, (const char *)&packet, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_MIN_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_CRC);
#endif
}

/**
 * @brief Send a set_mea_apogee_target_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_mea_apogee_target_tc_send_struct(mavlink_channel_t chan, const mavlink_set_mea_apogee_target_tc_t* set_mea_apogee_target_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_mea_apogee_target_tc_send(chan, set_mea_apogee_target_tc->apogee_target);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC, (const char *)set_mea_apogee_target_tc, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_MIN_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_mea_apogee_target_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float apogee_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, apogee_target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC, buf, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_MIN_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_CRC);
#else
    mavlink_set_mea_apogee_target_tc_t *packet = (mavlink_set_mea_apogee_target_tc_t *)msgbuf;
    packet->apogee_target = apogee_target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC, (const char *)packet, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_MIN_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_MEA_APOGEE_TARGET_TC UNPACKING


/**
 * @brief Get field apogee_target from set_mea_apogee_target_tc message
 *
 * @return [m] Apogee target
 */
static inline float mavlink_msg_set_mea_apogee_target_tc_get_apogee_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a set_mea_apogee_target_tc message into a struct
 *
 * @param msg The message to decode
 * @param set_mea_apogee_target_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_mea_apogee_target_tc_decode(const mavlink_message_t* msg, mavlink_set_mea_apogee_target_tc_t* set_mea_apogee_target_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_mea_apogee_target_tc->apogee_target = mavlink_msg_set_mea_apogee_target_tc_get_apogee_target(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN? msg->len : MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN;
        memset(set_mea_apogee_target_tc, 0, MAVLINK_MSG_ID_SET_MEA_APOGEE_TARGET_TC_LEN);
    memcpy(set_mea_apogee_target_tc, _MAV_PAYLOAD(msg), len);
#endif
}
