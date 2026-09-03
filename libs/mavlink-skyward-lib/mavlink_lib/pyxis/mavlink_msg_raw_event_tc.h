#pragma once
// MESSAGE RAW_EVENT_TC PACKING

#define MAVLINK_MSG_ID_RAW_EVENT_TC 13


typedef struct __mavlink_raw_event_tc_t {
 uint8_t topic_id; /*<  Id of the topic to which the event should be posted*/
 uint8_t event_id; /*<  Id of the event to be posted*/
} mavlink_raw_event_tc_t;

#define MAVLINK_MSG_ID_RAW_EVENT_TC_LEN 2
#define MAVLINK_MSG_ID_RAW_EVENT_TC_MIN_LEN 2
#define MAVLINK_MSG_ID_13_LEN 2
#define MAVLINK_MSG_ID_13_MIN_LEN 2

#define MAVLINK_MSG_ID_RAW_EVENT_TC_CRC 218
#define MAVLINK_MSG_ID_13_CRC 218



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RAW_EVENT_TC { \
    13, \
    "RAW_EVENT_TC", \
    2, \
    {  { "topic_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_raw_event_tc_t, topic_id) }, \
         { "event_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_raw_event_tc_t, event_id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RAW_EVENT_TC { \
    "RAW_EVENT_TC", \
    2, \
    {  { "topic_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_raw_event_tc_t, topic_id) }, \
         { "event_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_raw_event_tc_t, event_id) }, \
         } \
}
#endif

/**
 * @brief Pack a raw_event_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param topic_id  Id of the topic to which the event should be posted
 * @param event_id  Id of the event to be posted
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_event_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t topic_id, uint8_t event_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RAW_EVENT_TC_LEN];
    _mav_put_uint8_t(buf, 0, topic_id);
    _mav_put_uint8_t(buf, 1, event_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RAW_EVENT_TC_LEN);
#else
    mavlink_raw_event_tc_t packet;
    packet.topic_id = topic_id;
    packet.event_id = event_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RAW_EVENT_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RAW_EVENT_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RAW_EVENT_TC_MIN_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_CRC);
}

/**
 * @brief Pack a raw_event_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param topic_id  Id of the topic to which the event should be posted
 * @param event_id  Id of the event to be posted
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_event_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t topic_id,uint8_t event_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RAW_EVENT_TC_LEN];
    _mav_put_uint8_t(buf, 0, topic_id);
    _mav_put_uint8_t(buf, 1, event_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RAW_EVENT_TC_LEN);
#else
    mavlink_raw_event_tc_t packet;
    packet.topic_id = topic_id;
    packet.event_id = event_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RAW_EVENT_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RAW_EVENT_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RAW_EVENT_TC_MIN_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_CRC);
}

/**
 * @brief Encode a raw_event_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raw_event_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raw_event_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_event_tc_t* raw_event_tc)
{
    return mavlink_msg_raw_event_tc_pack(system_id, component_id, msg, raw_event_tc->topic_id, raw_event_tc->event_id);
}

/**
 * @brief Encode a raw_event_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param raw_event_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raw_event_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_raw_event_tc_t* raw_event_tc)
{
    return mavlink_msg_raw_event_tc_pack_chan(system_id, component_id, chan, msg, raw_event_tc->topic_id, raw_event_tc->event_id);
}

/**
 * @brief Send a raw_event_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param topic_id  Id of the topic to which the event should be posted
 * @param event_id  Id of the event to be posted
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_event_tc_send(mavlink_channel_t chan, uint8_t topic_id, uint8_t event_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RAW_EVENT_TC_LEN];
    _mav_put_uint8_t(buf, 0, topic_id);
    _mav_put_uint8_t(buf, 1, event_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_EVENT_TC, buf, MAVLINK_MSG_ID_RAW_EVENT_TC_MIN_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_CRC);
#else
    mavlink_raw_event_tc_t packet;
    packet.topic_id = topic_id;
    packet.event_id = event_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_EVENT_TC, (const char *)&packet, MAVLINK_MSG_ID_RAW_EVENT_TC_MIN_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_CRC);
#endif
}

/**
 * @brief Send a raw_event_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_raw_event_tc_send_struct(mavlink_channel_t chan, const mavlink_raw_event_tc_t* raw_event_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_raw_event_tc_send(chan, raw_event_tc->topic_id, raw_event_tc->event_id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_EVENT_TC, (const char *)raw_event_tc, MAVLINK_MSG_ID_RAW_EVENT_TC_MIN_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_RAW_EVENT_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_raw_event_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t topic_id, uint8_t event_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, topic_id);
    _mav_put_uint8_t(buf, 1, event_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_EVENT_TC, buf, MAVLINK_MSG_ID_RAW_EVENT_TC_MIN_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_CRC);
#else
    mavlink_raw_event_tc_t *packet = (mavlink_raw_event_tc_t *)msgbuf;
    packet->topic_id = topic_id;
    packet->event_id = event_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_EVENT_TC, (const char *)packet, MAVLINK_MSG_ID_RAW_EVENT_TC_MIN_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_LEN, MAVLINK_MSG_ID_RAW_EVENT_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE RAW_EVENT_TC UNPACKING


/**
 * @brief Get field topic_id from raw_event_tc message
 *
 * @return  Id of the topic to which the event should be posted
 */
static inline uint8_t mavlink_msg_raw_event_tc_get_topic_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field event_id from raw_event_tc message
 *
 * @return  Id of the event to be posted
 */
static inline uint8_t mavlink_msg_raw_event_tc_get_event_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a raw_event_tc message into a struct
 *
 * @param msg The message to decode
 * @param raw_event_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_raw_event_tc_decode(const mavlink_message_t* msg, mavlink_raw_event_tc_t* raw_event_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    raw_event_tc->topic_id = mavlink_msg_raw_event_tc_get_topic_id(msg);
    raw_event_tc->event_id = mavlink_msg_raw_event_tc_get_event_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RAW_EVENT_TC_LEN? msg->len : MAVLINK_MSG_ID_RAW_EVENT_TC_LEN;
        memset(raw_event_tc, 0, MAVLINK_MSG_ID_RAW_EVENT_TC_LEN);
    memcpy(raw_event_tc, _MAV_PAYLOAD(msg), len);
#endif
}
