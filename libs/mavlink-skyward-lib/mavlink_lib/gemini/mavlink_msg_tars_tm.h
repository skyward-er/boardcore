#pragma once
// MESSAGE TARS_TM PACKING

#define MAVLINK_MSG_ID_TARS_TM 207


typedef struct __mavlink_tars_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 uint8_t algorithm_number; /*<  Algorithm selection number*/
} mavlink_tars_tm_t;

#define MAVLINK_MSG_ID_TARS_TM_LEN 9
#define MAVLINK_MSG_ID_TARS_TM_MIN_LEN 9
#define MAVLINK_MSG_ID_207_LEN 9
#define MAVLINK_MSG_ID_207_MIN_LEN 9

#define MAVLINK_MSG_ID_TARS_TM_CRC 122
#define MAVLINK_MSG_ID_207_CRC 122



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TARS_TM { \
    207, \
    "TARS_TM", \
    2, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tars_tm_t, timestamp) }, \
         { "algorithm_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_tars_tm_t, algorithm_number) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TARS_TM { \
    "TARS_TM", \
    2, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tars_tm_t, timestamp) }, \
         { "algorithm_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_tars_tm_t, algorithm_number) }, \
         } \
}
#endif

/**
 * @brief Pack a tars_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param algorithm_number  Algorithm selection number
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tars_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t algorithm_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, algorithm_number);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARS_TM_LEN);
#else
    mavlink_tars_tm_t packet;
    packet.timestamp = timestamp;
    packet.algorithm_number = algorithm_number;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TARS_TM_MIN_LEN, MAVLINK_MSG_ID_TARS_TM_LEN, MAVLINK_MSG_ID_TARS_TM_CRC);
}

/**
 * @brief Pack a tars_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp in microseconds
 * @param algorithm_number  Algorithm selection number
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tars_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t algorithm_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, algorithm_number);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARS_TM_LEN);
#else
    mavlink_tars_tm_t packet;
    packet.timestamp = timestamp;
    packet.algorithm_number = algorithm_number;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TARS_TM_MIN_LEN, MAVLINK_MSG_ID_TARS_TM_LEN, MAVLINK_MSG_ID_TARS_TM_CRC);
}

/**
 * @brief Encode a tars_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tars_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tars_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tars_tm_t* tars_tm)
{
    return mavlink_msg_tars_tm_pack(system_id, component_id, msg, tars_tm->timestamp, tars_tm->algorithm_number);
}

/**
 * @brief Encode a tars_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tars_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tars_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tars_tm_t* tars_tm)
{
    return mavlink_msg_tars_tm_pack_chan(system_id, component_id, chan, msg, tars_tm->timestamp, tars_tm->algorithm_number);
}

/**
 * @brief Send a tars_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param algorithm_number  Algorithm selection number
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tars_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t algorithm_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, algorithm_number);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARS_TM, buf, MAVLINK_MSG_ID_TARS_TM_MIN_LEN, MAVLINK_MSG_ID_TARS_TM_LEN, MAVLINK_MSG_ID_TARS_TM_CRC);
#else
    mavlink_tars_tm_t packet;
    packet.timestamp = timestamp;
    packet.algorithm_number = algorithm_number;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARS_TM, (const char *)&packet, MAVLINK_MSG_ID_TARS_TM_MIN_LEN, MAVLINK_MSG_ID_TARS_TM_LEN, MAVLINK_MSG_ID_TARS_TM_CRC);
#endif
}

/**
 * @brief Send a tars_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tars_tm_send_struct(mavlink_channel_t chan, const mavlink_tars_tm_t* tars_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tars_tm_send(chan, tars_tm->timestamp, tars_tm->algorithm_number);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARS_TM, (const char *)tars_tm, MAVLINK_MSG_ID_TARS_TM_MIN_LEN, MAVLINK_MSG_ID_TARS_TM_LEN, MAVLINK_MSG_ID_TARS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_TARS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tars_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t algorithm_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, algorithm_number);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARS_TM, buf, MAVLINK_MSG_ID_TARS_TM_MIN_LEN, MAVLINK_MSG_ID_TARS_TM_LEN, MAVLINK_MSG_ID_TARS_TM_CRC);
#else
    mavlink_tars_tm_t *packet = (mavlink_tars_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->algorithm_number = algorithm_number;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARS_TM, (const char *)packet, MAVLINK_MSG_ID_TARS_TM_MIN_LEN, MAVLINK_MSG_ID_TARS_TM_LEN, MAVLINK_MSG_ID_TARS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE TARS_TM UNPACKING


/**
 * @brief Get field timestamp from tars_tm message
 *
 * @return [us] Timestamp in microseconds
 */
static inline uint64_t mavlink_msg_tars_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field algorithm_number from tars_tm message
 *
 * @return  Algorithm selection number
 */
static inline uint8_t mavlink_msg_tars_tm_get_algorithm_number(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a tars_tm message into a struct
 *
 * @param msg The message to decode
 * @param tars_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_tars_tm_decode(const mavlink_message_t* msg, mavlink_tars_tm_t* tars_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tars_tm->timestamp = mavlink_msg_tars_tm_get_timestamp(msg);
    tars_tm->algorithm_number = mavlink_msg_tars_tm_get_algorithm_number(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TARS_TM_LEN? msg->len : MAVLINK_MSG_ID_TARS_TM_LEN;
        memset(tars_tm, 0, MAVLINK_MSG_ID_TARS_TM_LEN);
    memcpy(tars_tm, _MAV_PAYLOAD(msg), len);
#endif
}
