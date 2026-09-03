#pragma once
// MESSAGE PING_TC PACKING

#define MAVLINK_MSG_ID_PING_TC 1


typedef struct __mavlink_ping_tc_t {
 uint64_t timestamp; /*<  Timestamp to identify when it was sent*/
} mavlink_ping_tc_t;

#define MAVLINK_MSG_ID_PING_TC_LEN 8
#define MAVLINK_MSG_ID_PING_TC_MIN_LEN 8
#define MAVLINK_MSG_ID_1_LEN 8
#define MAVLINK_MSG_ID_1_MIN_LEN 8

#define MAVLINK_MSG_ID_PING_TC_CRC 136
#define MAVLINK_MSG_ID_1_CRC 136



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PING_TC { \
    1, \
    "PING_TC", \
    1, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ping_tc_t, timestamp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PING_TC { \
    "PING_TC", \
    1, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ping_tc_t, timestamp) }, \
         } \
}
#endif

/**
 * @brief Pack a ping_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Timestamp to identify when it was sent
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ping_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PING_TC_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PING_TC_LEN);
#else
    mavlink_ping_tc_t packet;
    packet.timestamp = timestamp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PING_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PING_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PING_TC_MIN_LEN, MAVLINK_MSG_ID_PING_TC_LEN, MAVLINK_MSG_ID_PING_TC_CRC);
}

/**
 * @brief Pack a ping_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Timestamp to identify when it was sent
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ping_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PING_TC_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PING_TC_LEN);
#else
    mavlink_ping_tc_t packet;
    packet.timestamp = timestamp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PING_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PING_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PING_TC_MIN_LEN, MAVLINK_MSG_ID_PING_TC_LEN, MAVLINK_MSG_ID_PING_TC_CRC);
}

/**
 * @brief Encode a ping_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ping_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ping_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ping_tc_t* ping_tc)
{
    return mavlink_msg_ping_tc_pack(system_id, component_id, msg, ping_tc->timestamp);
}

/**
 * @brief Encode a ping_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ping_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ping_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ping_tc_t* ping_tc)
{
    return mavlink_msg_ping_tc_pack_chan(system_id, component_id, chan, msg, ping_tc->timestamp);
}

/**
 * @brief Send a ping_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Timestamp to identify when it was sent
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ping_tc_send(mavlink_channel_t chan, uint64_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PING_TC_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PING_TC, buf, MAVLINK_MSG_ID_PING_TC_MIN_LEN, MAVLINK_MSG_ID_PING_TC_LEN, MAVLINK_MSG_ID_PING_TC_CRC);
#else
    mavlink_ping_tc_t packet;
    packet.timestamp = timestamp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PING_TC, (const char *)&packet, MAVLINK_MSG_ID_PING_TC_MIN_LEN, MAVLINK_MSG_ID_PING_TC_LEN, MAVLINK_MSG_ID_PING_TC_CRC);
#endif
}

/**
 * @brief Send a ping_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ping_tc_send_struct(mavlink_channel_t chan, const mavlink_ping_tc_t* ping_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ping_tc_send(chan, ping_tc->timestamp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PING_TC, (const char *)ping_tc, MAVLINK_MSG_ID_PING_TC_MIN_LEN, MAVLINK_MSG_ID_PING_TC_LEN, MAVLINK_MSG_ID_PING_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_PING_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ping_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PING_TC, buf, MAVLINK_MSG_ID_PING_TC_MIN_LEN, MAVLINK_MSG_ID_PING_TC_LEN, MAVLINK_MSG_ID_PING_TC_CRC);
#else
    mavlink_ping_tc_t *packet = (mavlink_ping_tc_t *)msgbuf;
    packet->timestamp = timestamp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PING_TC, (const char *)packet, MAVLINK_MSG_ID_PING_TC_MIN_LEN, MAVLINK_MSG_ID_PING_TC_LEN, MAVLINK_MSG_ID_PING_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE PING_TC UNPACKING


/**
 * @brief Get field timestamp from ping_tc message
 *
 * @return  Timestamp to identify when it was sent
 */
static inline uint64_t mavlink_msg_ping_tc_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Decode a ping_tc message into a struct
 *
 * @param msg The message to decode
 * @param ping_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_ping_tc_decode(const mavlink_message_t* msg, mavlink_ping_tc_t* ping_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ping_tc->timestamp = mavlink_msg_ping_tc_get_timestamp(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PING_TC_LEN? msg->len : MAVLINK_MSG_ID_PING_TC_LEN;
        memset(ping_tc, 0, MAVLINK_MSG_ID_PING_TC_LEN);
    memcpy(ping_tc, _MAV_PAYLOAD(msg), len);
#endif
}
