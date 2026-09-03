#pragma once
// MESSAGE CAN_TM PACKING

#define MAVLINK_MSG_ID_CAN_TM 207


typedef struct __mavlink_can_tm_t {
 uint64_t timestamp; /*< [us] When was this logged*/
 uint64_t last_sent_ts; /*<  Timestamp of the last sent message*/
 uint64_t last_rcv_ts; /*<  Timestamp of the last received message*/
 uint16_t n_sent; /*<  Number of sent messages*/
 uint16_t n_rcv; /*<  Number of received messages*/
 uint8_t last_sent; /*<  Id of the last sent message*/
 uint8_t last_rcv; /*<  Id of the last received message*/
} mavlink_can_tm_t;

#define MAVLINK_MSG_ID_CAN_TM_LEN 30
#define MAVLINK_MSG_ID_CAN_TM_MIN_LEN 30
#define MAVLINK_MSG_ID_207_LEN 30
#define MAVLINK_MSG_ID_207_MIN_LEN 30

#define MAVLINK_MSG_ID_CAN_TM_CRC 169
#define MAVLINK_MSG_ID_207_CRC 169



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAN_TM { \
    207, \
    "CAN_TM", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_can_tm_t, timestamp) }, \
         { "n_sent", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_can_tm_t, n_sent) }, \
         { "n_rcv", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_can_tm_t, n_rcv) }, \
         { "last_sent", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_can_tm_t, last_sent) }, \
         { "last_rcv", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_can_tm_t, last_rcv) }, \
         { "last_sent_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_can_tm_t, last_sent_ts) }, \
         { "last_rcv_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_can_tm_t, last_rcv_ts) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAN_TM { \
    "CAN_TM", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_can_tm_t, timestamp) }, \
         { "n_sent", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_can_tm_t, n_sent) }, \
         { "n_rcv", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_can_tm_t, n_rcv) }, \
         { "last_sent", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_can_tm_t, last_sent) }, \
         { "last_rcv", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_can_tm_t, last_rcv) }, \
         { "last_sent_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_can_tm_t, last_sent_ts) }, \
         { "last_rcv_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_can_tm_t, last_rcv_ts) }, \
         } \
}
#endif

/**
 * @brief Pack a can_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] When was this logged
 * @param n_sent  Number of sent messages
 * @param n_rcv  Number of received messages
 * @param last_sent  Id of the last sent message
 * @param last_rcv  Id of the last received message
 * @param last_sent_ts  Timestamp of the last sent message
 * @param last_rcv_ts  Timestamp of the last received message
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint16_t n_sent, uint16_t n_rcv, uint8_t last_sent, uint8_t last_rcv, uint64_t last_sent_ts, uint64_t last_rcv_ts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_sent_ts);
    _mav_put_uint64_t(buf, 16, last_rcv_ts);
    _mav_put_uint16_t(buf, 24, n_sent);
    _mav_put_uint16_t(buf, 26, n_rcv);
    _mav_put_uint8_t(buf, 28, last_sent);
    _mav_put_uint8_t(buf, 29, last_rcv);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_TM_LEN);
#else
    mavlink_can_tm_t packet;
    packet.timestamp = timestamp;
    packet.last_sent_ts = last_sent_ts;
    packet.last_rcv_ts = last_rcv_ts;
    packet.n_sent = n_sent;
    packet.n_rcv = n_rcv;
    packet.last_sent = last_sent;
    packet.last_rcv = last_rcv;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAN_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_TM_LEN, MAVLINK_MSG_ID_CAN_TM_CRC);
}

/**
 * @brief Pack a can_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] When was this logged
 * @param n_sent  Number of sent messages
 * @param n_rcv  Number of received messages
 * @param last_sent  Id of the last sent message
 * @param last_rcv  Id of the last received message
 * @param last_sent_ts  Timestamp of the last sent message
 * @param last_rcv_ts  Timestamp of the last received message
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint16_t n_sent,uint16_t n_rcv,uint8_t last_sent,uint8_t last_rcv,uint64_t last_sent_ts,uint64_t last_rcv_ts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_sent_ts);
    _mav_put_uint64_t(buf, 16, last_rcv_ts);
    _mav_put_uint16_t(buf, 24, n_sent);
    _mav_put_uint16_t(buf, 26, n_rcv);
    _mav_put_uint8_t(buf, 28, last_sent);
    _mav_put_uint8_t(buf, 29, last_rcv);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_TM_LEN);
#else
    mavlink_can_tm_t packet;
    packet.timestamp = timestamp;
    packet.last_sent_ts = last_sent_ts;
    packet.last_rcv_ts = last_rcv_ts;
    packet.n_sent = n_sent;
    packet.n_rcv = n_rcv;
    packet.last_sent = last_sent;
    packet.last_rcv = last_rcv;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAN_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_TM_LEN, MAVLINK_MSG_ID_CAN_TM_CRC);
}

/**
 * @brief Encode a can_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param can_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_can_tm_t* can_tm)
{
    return mavlink_msg_can_tm_pack(system_id, component_id, msg, can_tm->timestamp, can_tm->n_sent, can_tm->n_rcv, can_tm->last_sent, can_tm->last_rcv, can_tm->last_sent_ts, can_tm->last_rcv_ts);
}

/**
 * @brief Encode a can_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param can_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_can_tm_t* can_tm)
{
    return mavlink_msg_can_tm_pack_chan(system_id, component_id, chan, msg, can_tm->timestamp, can_tm->n_sent, can_tm->n_rcv, can_tm->last_sent, can_tm->last_rcv, can_tm->last_sent_ts, can_tm->last_rcv_ts);
}

/**
 * @brief Send a can_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged
 * @param n_sent  Number of sent messages
 * @param n_rcv  Number of received messages
 * @param last_sent  Id of the last sent message
 * @param last_rcv  Id of the last received message
 * @param last_sent_ts  Timestamp of the last sent message
 * @param last_rcv_ts  Timestamp of the last received message
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_can_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint16_t n_sent, uint16_t n_rcv, uint8_t last_sent, uint8_t last_rcv, uint64_t last_sent_ts, uint64_t last_rcv_ts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_sent_ts);
    _mav_put_uint64_t(buf, 16, last_rcv_ts);
    _mav_put_uint16_t(buf, 24, n_sent);
    _mav_put_uint16_t(buf, 26, n_rcv);
    _mav_put_uint8_t(buf, 28, last_sent);
    _mav_put_uint8_t(buf, 29, last_rcv);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_TM, buf, MAVLINK_MSG_ID_CAN_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_TM_LEN, MAVLINK_MSG_ID_CAN_TM_CRC);
#else
    mavlink_can_tm_t packet;
    packet.timestamp = timestamp;
    packet.last_sent_ts = last_sent_ts;
    packet.last_rcv_ts = last_rcv_ts;
    packet.n_sent = n_sent;
    packet.n_rcv = n_rcv;
    packet.last_sent = last_sent;
    packet.last_rcv = last_rcv;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_TM, (const char *)&packet, MAVLINK_MSG_ID_CAN_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_TM_LEN, MAVLINK_MSG_ID_CAN_TM_CRC);
#endif
}

/**
 * @brief Send a can_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_can_tm_send_struct(mavlink_channel_t chan, const mavlink_can_tm_t* can_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_can_tm_send(chan, can_tm->timestamp, can_tm->n_sent, can_tm->n_rcv, can_tm->last_sent, can_tm->last_rcv, can_tm->last_sent_ts, can_tm->last_rcv_ts);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_TM, (const char *)can_tm, MAVLINK_MSG_ID_CAN_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_TM_LEN, MAVLINK_MSG_ID_CAN_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAN_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_can_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint16_t n_sent, uint16_t n_rcv, uint8_t last_sent, uint8_t last_rcv, uint64_t last_sent_ts, uint64_t last_rcv_ts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_sent_ts);
    _mav_put_uint64_t(buf, 16, last_rcv_ts);
    _mav_put_uint16_t(buf, 24, n_sent);
    _mav_put_uint16_t(buf, 26, n_rcv);
    _mav_put_uint8_t(buf, 28, last_sent);
    _mav_put_uint8_t(buf, 29, last_rcv);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_TM, buf, MAVLINK_MSG_ID_CAN_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_TM_LEN, MAVLINK_MSG_ID_CAN_TM_CRC);
#else
    mavlink_can_tm_t *packet = (mavlink_can_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->last_sent_ts = last_sent_ts;
    packet->last_rcv_ts = last_rcv_ts;
    packet->n_sent = n_sent;
    packet->n_rcv = n_rcv;
    packet->last_sent = last_sent;
    packet->last_rcv = last_rcv;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_TM, (const char *)packet, MAVLINK_MSG_ID_CAN_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_TM_LEN, MAVLINK_MSG_ID_CAN_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE CAN_TM UNPACKING


/**
 * @brief Get field timestamp from can_tm message
 *
 * @return [us] When was this logged
 */
static inline uint64_t mavlink_msg_can_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field n_sent from can_tm message
 *
 * @return  Number of sent messages
 */
static inline uint16_t mavlink_msg_can_tm_get_n_sent(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field n_rcv from can_tm message
 *
 * @return  Number of received messages
 */
static inline uint16_t mavlink_msg_can_tm_get_n_rcv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field last_sent from can_tm message
 *
 * @return  Id of the last sent message
 */
static inline uint8_t mavlink_msg_can_tm_get_last_sent(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field last_rcv from can_tm message
 *
 * @return  Id of the last received message
 */
static inline uint8_t mavlink_msg_can_tm_get_last_rcv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field last_sent_ts from can_tm message
 *
 * @return  Timestamp of the last sent message
 */
static inline uint64_t mavlink_msg_can_tm_get_last_sent_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field last_rcv_ts from can_tm message
 *
 * @return  Timestamp of the last received message
 */
static inline uint64_t mavlink_msg_can_tm_get_last_rcv_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Decode a can_tm message into a struct
 *
 * @param msg The message to decode
 * @param can_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_can_tm_decode(const mavlink_message_t* msg, mavlink_can_tm_t* can_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    can_tm->timestamp = mavlink_msg_can_tm_get_timestamp(msg);
    can_tm->last_sent_ts = mavlink_msg_can_tm_get_last_sent_ts(msg);
    can_tm->last_rcv_ts = mavlink_msg_can_tm_get_last_rcv_ts(msg);
    can_tm->n_sent = mavlink_msg_can_tm_get_n_sent(msg);
    can_tm->n_rcv = mavlink_msg_can_tm_get_n_rcv(msg);
    can_tm->last_sent = mavlink_msg_can_tm_get_last_sent(msg);
    can_tm->last_rcv = mavlink_msg_can_tm_get_last_rcv(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAN_TM_LEN? msg->len : MAVLINK_MSG_ID_CAN_TM_LEN;
        memset(can_tm, 0, MAVLINK_MSG_ID_CAN_TM_LEN);
    memcpy(can_tm, _MAV_PAYLOAD(msg), len);
#endif
}
