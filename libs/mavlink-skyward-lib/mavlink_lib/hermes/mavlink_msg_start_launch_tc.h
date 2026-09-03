#pragma once
// MESSAGE START_LAUNCH_TC PACKING

#define MAVLINK_MSG_ID_START_LAUNCH_TC 20

MAVPACKED(
typedef struct __mavlink_start_launch_tc_t {
 uint64_t launch_code; /*<  64bit launch code.*/
}) mavlink_start_launch_tc_t;

#define MAVLINK_MSG_ID_START_LAUNCH_TC_LEN 8
#define MAVLINK_MSG_ID_START_LAUNCH_TC_MIN_LEN 8
#define MAVLINK_MSG_ID_20_LEN 8
#define MAVLINK_MSG_ID_20_MIN_LEN 8

#define MAVLINK_MSG_ID_START_LAUNCH_TC_CRC 43
#define MAVLINK_MSG_ID_20_CRC 43



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_START_LAUNCH_TC { \
    20, \
    "START_LAUNCH_TC", \
    1, \
    {  { "launch_code", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_start_launch_tc_t, launch_code) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_START_LAUNCH_TC { \
    "START_LAUNCH_TC", \
    1, \
    {  { "launch_code", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_start_launch_tc_t, launch_code) }, \
         } \
}
#endif

/**
 * @brief Pack a start_launch_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param launch_code  64bit launch code.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_start_launch_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t launch_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_START_LAUNCH_TC_LEN];
    _mav_put_uint64_t(buf, 0, launch_code);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_START_LAUNCH_TC_LEN);
#else
    mavlink_start_launch_tc_t packet;
    packet.launch_code = launch_code;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_START_LAUNCH_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_START_LAUNCH_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_START_LAUNCH_TC_MIN_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_CRC);
}

/**
 * @brief Pack a start_launch_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param launch_code  64bit launch code.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_start_launch_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t launch_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_START_LAUNCH_TC_LEN];
    _mav_put_uint64_t(buf, 0, launch_code);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_START_LAUNCH_TC_LEN);
#else
    mavlink_start_launch_tc_t packet;
    packet.launch_code = launch_code;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_START_LAUNCH_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_START_LAUNCH_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_START_LAUNCH_TC_MIN_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_CRC);
}

/**
 * @brief Encode a start_launch_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param start_launch_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_start_launch_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_start_launch_tc_t* start_launch_tc)
{
    return mavlink_msg_start_launch_tc_pack(system_id, component_id, msg, start_launch_tc->launch_code);
}

/**
 * @brief Encode a start_launch_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param start_launch_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_start_launch_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_start_launch_tc_t* start_launch_tc)
{
    return mavlink_msg_start_launch_tc_pack_chan(system_id, component_id, chan, msg, start_launch_tc->launch_code);
}

/**
 * @brief Send a start_launch_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param launch_code  64bit launch code.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_start_launch_tc_send(mavlink_channel_t chan, uint64_t launch_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_START_LAUNCH_TC_LEN];
    _mav_put_uint64_t(buf, 0, launch_code);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_START_LAUNCH_TC, buf, MAVLINK_MSG_ID_START_LAUNCH_TC_MIN_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_CRC);
#else
    mavlink_start_launch_tc_t packet;
    packet.launch_code = launch_code;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_START_LAUNCH_TC, (const char *)&packet, MAVLINK_MSG_ID_START_LAUNCH_TC_MIN_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_CRC);
#endif
}

/**
 * @brief Send a start_launch_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_start_launch_tc_send_struct(mavlink_channel_t chan, const mavlink_start_launch_tc_t* start_launch_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_start_launch_tc_send(chan, start_launch_tc->launch_code);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_START_LAUNCH_TC, (const char *)start_launch_tc, MAVLINK_MSG_ID_START_LAUNCH_TC_MIN_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_START_LAUNCH_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_start_launch_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t launch_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, launch_code);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_START_LAUNCH_TC, buf, MAVLINK_MSG_ID_START_LAUNCH_TC_MIN_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_CRC);
#else
    mavlink_start_launch_tc_t *packet = (mavlink_start_launch_tc_t *)msgbuf;
    packet->launch_code = launch_code;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_START_LAUNCH_TC, (const char *)packet, MAVLINK_MSG_ID_START_LAUNCH_TC_MIN_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_LEN, MAVLINK_MSG_ID_START_LAUNCH_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE START_LAUNCH_TC UNPACKING


/**
 * @brief Get field launch_code from start_launch_tc message
 *
 * @return  64bit launch code.
 */
static inline uint64_t mavlink_msg_start_launch_tc_get_launch_code(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Decode a start_launch_tc message into a struct
 *
 * @param msg The message to decode
 * @param start_launch_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_start_launch_tc_decode(const mavlink_message_t* msg, mavlink_start_launch_tc_t* start_launch_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    start_launch_tc->launch_code = mavlink_msg_start_launch_tc_get_launch_code(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_START_LAUNCH_TC_LEN? msg->len : MAVLINK_MSG_ID_START_LAUNCH_TC_LEN;
        memset(start_launch_tc, 0, MAVLINK_MSG_ID_START_LAUNCH_TC_LEN);
    memcpy(start_launch_tc, _MAV_PAYLOAD(msg), len);
#endif
}
