#pragma once
// MESSAGE WACK_TM PACKING

#define MAVLINK_MSG_ID_WACK_TM 102


typedef struct __mavlink_wack_tm_t {
 uint16_t err_id; /*<  Error core that generated the WACK*/
 uint8_t recv_msgid; /*<  Message id of the received message*/
 uint8_t seq_ack; /*<  Sequence number of the received message*/
} mavlink_wack_tm_t;

#define MAVLINK_MSG_ID_WACK_TM_LEN 4
#define MAVLINK_MSG_ID_WACK_TM_MIN_LEN 4
#define MAVLINK_MSG_ID_102_LEN 4
#define MAVLINK_MSG_ID_102_MIN_LEN 4

#define MAVLINK_MSG_ID_WACK_TM_CRC 51
#define MAVLINK_MSG_ID_102_CRC 51



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WACK_TM { \
    102, \
    "WACK_TM", \
    3, \
    {  { "recv_msgid", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_wack_tm_t, recv_msgid) }, \
         { "seq_ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_wack_tm_t, seq_ack) }, \
         { "err_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_wack_tm_t, err_id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WACK_TM { \
    "WACK_TM", \
    3, \
    {  { "recv_msgid", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_wack_tm_t, recv_msgid) }, \
         { "seq_ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_wack_tm_t, seq_ack) }, \
         { "err_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_wack_tm_t, err_id) }, \
         } \
}
#endif

/**
 * @brief Pack a wack_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param recv_msgid  Message id of the received message
 * @param seq_ack  Sequence number of the received message
 * @param err_id  Error core that generated the WACK
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wack_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t recv_msgid, uint8_t seq_ack, uint16_t err_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WACK_TM_LEN];
    _mav_put_uint16_t(buf, 0, err_id);
    _mav_put_uint8_t(buf, 2, recv_msgid);
    _mav_put_uint8_t(buf, 3, seq_ack);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WACK_TM_LEN);
#else
    mavlink_wack_tm_t packet;
    packet.err_id = err_id;
    packet.recv_msgid = recv_msgid;
    packet.seq_ack = seq_ack;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WACK_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WACK_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WACK_TM_MIN_LEN, MAVLINK_MSG_ID_WACK_TM_LEN, MAVLINK_MSG_ID_WACK_TM_CRC);
}

/**
 * @brief Pack a wack_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param recv_msgid  Message id of the received message
 * @param seq_ack  Sequence number of the received message
 * @param err_id  Error core that generated the WACK
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wack_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t recv_msgid,uint8_t seq_ack,uint16_t err_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WACK_TM_LEN];
    _mav_put_uint16_t(buf, 0, err_id);
    _mav_put_uint8_t(buf, 2, recv_msgid);
    _mav_put_uint8_t(buf, 3, seq_ack);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WACK_TM_LEN);
#else
    mavlink_wack_tm_t packet;
    packet.err_id = err_id;
    packet.recv_msgid = recv_msgid;
    packet.seq_ack = seq_ack;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WACK_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WACK_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WACK_TM_MIN_LEN, MAVLINK_MSG_ID_WACK_TM_LEN, MAVLINK_MSG_ID_WACK_TM_CRC);
}

/**
 * @brief Encode a wack_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wack_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wack_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wack_tm_t* wack_tm)
{
    return mavlink_msg_wack_tm_pack(system_id, component_id, msg, wack_tm->recv_msgid, wack_tm->seq_ack, wack_tm->err_id);
}

/**
 * @brief Encode a wack_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wack_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wack_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_wack_tm_t* wack_tm)
{
    return mavlink_msg_wack_tm_pack_chan(system_id, component_id, chan, msg, wack_tm->recv_msgid, wack_tm->seq_ack, wack_tm->err_id);
}

/**
 * @brief Send a wack_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param recv_msgid  Message id of the received message
 * @param seq_ack  Sequence number of the received message
 * @param err_id  Error core that generated the WACK
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wack_tm_send(mavlink_channel_t chan, uint8_t recv_msgid, uint8_t seq_ack, uint16_t err_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WACK_TM_LEN];
    _mav_put_uint16_t(buf, 0, err_id);
    _mav_put_uint8_t(buf, 2, recv_msgid);
    _mav_put_uint8_t(buf, 3, seq_ack);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WACK_TM, buf, MAVLINK_MSG_ID_WACK_TM_MIN_LEN, MAVLINK_MSG_ID_WACK_TM_LEN, MAVLINK_MSG_ID_WACK_TM_CRC);
#else
    mavlink_wack_tm_t packet;
    packet.err_id = err_id;
    packet.recv_msgid = recv_msgid;
    packet.seq_ack = seq_ack;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WACK_TM, (const char *)&packet, MAVLINK_MSG_ID_WACK_TM_MIN_LEN, MAVLINK_MSG_ID_WACK_TM_LEN, MAVLINK_MSG_ID_WACK_TM_CRC);
#endif
}

/**
 * @brief Send a wack_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_wack_tm_send_struct(mavlink_channel_t chan, const mavlink_wack_tm_t* wack_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_wack_tm_send(chan, wack_tm->recv_msgid, wack_tm->seq_ack, wack_tm->err_id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WACK_TM, (const char *)wack_tm, MAVLINK_MSG_ID_WACK_TM_MIN_LEN, MAVLINK_MSG_ID_WACK_TM_LEN, MAVLINK_MSG_ID_WACK_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_WACK_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_wack_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t recv_msgid, uint8_t seq_ack, uint16_t err_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, err_id);
    _mav_put_uint8_t(buf, 2, recv_msgid);
    _mav_put_uint8_t(buf, 3, seq_ack);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WACK_TM, buf, MAVLINK_MSG_ID_WACK_TM_MIN_LEN, MAVLINK_MSG_ID_WACK_TM_LEN, MAVLINK_MSG_ID_WACK_TM_CRC);
#else
    mavlink_wack_tm_t *packet = (mavlink_wack_tm_t *)msgbuf;
    packet->err_id = err_id;
    packet->recv_msgid = recv_msgid;
    packet->seq_ack = seq_ack;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WACK_TM, (const char *)packet, MAVLINK_MSG_ID_WACK_TM_MIN_LEN, MAVLINK_MSG_ID_WACK_TM_LEN, MAVLINK_MSG_ID_WACK_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE WACK_TM UNPACKING


/**
 * @brief Get field recv_msgid from wack_tm message
 *
 * @return  Message id of the received message
 */
static inline uint8_t mavlink_msg_wack_tm_get_recv_msgid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field seq_ack from wack_tm message
 *
 * @return  Sequence number of the received message
 */
static inline uint8_t mavlink_msg_wack_tm_get_seq_ack(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field err_id from wack_tm message
 *
 * @return  Error core that generated the WACK
 */
static inline uint16_t mavlink_msg_wack_tm_get_err_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a wack_tm message into a struct
 *
 * @param msg The message to decode
 * @param wack_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_wack_tm_decode(const mavlink_message_t* msg, mavlink_wack_tm_t* wack_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    wack_tm->err_id = mavlink_msg_wack_tm_get_err_id(msg);
    wack_tm->recv_msgid = mavlink_msg_wack_tm_get_recv_msgid(msg);
    wack_tm->seq_ack = mavlink_msg_wack_tm_get_seq_ack(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WACK_TM_LEN? msg->len : MAVLINK_MSG_ID_WACK_TM_LEN;
        memset(wack_tm, 0, MAVLINK_MSG_ID_WACK_TM_LEN);
    memcpy(wack_tm, _MAV_PAYLOAD(msg), len);
#endif
}
