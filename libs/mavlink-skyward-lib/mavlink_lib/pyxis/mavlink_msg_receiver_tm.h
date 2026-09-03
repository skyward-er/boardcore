#pragma once
// MESSAGE RECEIVER_TM PACKING

#define MAVLINK_MSG_ID_RECEIVER_TM 150


typedef struct __mavlink_receiver_tm_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 uint64_t rx_bitrate; /*< [kb/s] Rx bitrate*/
 uint64_t tx_bitrate; /*< [kb/s] Tx bitrate*/
 float rssi; /*< [dBm] Rssi*/
 float fei; /*< [Hz] Fei*/
} mavlink_receiver_tm_t;

#define MAVLINK_MSG_ID_RECEIVER_TM_LEN 32
#define MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN 32
#define MAVLINK_MSG_ID_150_LEN 32
#define MAVLINK_MSG_ID_150_MIN_LEN 32

#define MAVLINK_MSG_ID_RECEIVER_TM_CRC 109
#define MAVLINK_MSG_ID_150_CRC 109



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RECEIVER_TM { \
    150, \
    "RECEIVER_TM", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_receiver_tm_t, timestamp) }, \
         { "rssi", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_receiver_tm_t, rssi) }, \
         { "fei", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_receiver_tm_t, fei) }, \
         { "rx_bitrate", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_receiver_tm_t, rx_bitrate) }, \
         { "tx_bitrate", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_receiver_tm_t, tx_bitrate) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RECEIVER_TM { \
    "RECEIVER_TM", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_receiver_tm_t, timestamp) }, \
         { "rssi", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_receiver_tm_t, rssi) }, \
         { "fei", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_receiver_tm_t, fei) }, \
         { "rx_bitrate", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_receiver_tm_t, rx_bitrate) }, \
         { "tx_bitrate", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_receiver_tm_t, tx_bitrate) }, \
         } \
}
#endif

/**
 * @brief Pack a receiver_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param rssi [dBm] Rssi
 * @param fei [Hz] Fei
 * @param rx_bitrate [kb/s] Rx bitrate
 * @param tx_bitrate [kb/s] Tx bitrate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_receiver_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float rssi, float fei, uint64_t rx_bitrate, uint64_t tx_bitrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RECEIVER_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, rx_bitrate);
    _mav_put_uint64_t(buf, 16, tx_bitrate);
    _mav_put_float(buf, 24, rssi);
    _mav_put_float(buf, 28, fei);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RECEIVER_TM_LEN);
#else
    mavlink_receiver_tm_t packet;
    packet.timestamp = timestamp;
    packet.rx_bitrate = rx_bitrate;
    packet.tx_bitrate = tx_bitrate;
    packet.rssi = rssi;
    packet.fei = fei;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RECEIVER_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RECEIVER_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN, MAVLINK_MSG_ID_RECEIVER_TM_LEN, MAVLINK_MSG_ID_RECEIVER_TM_CRC);
}

/**
 * @brief Pack a receiver_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param rssi [dBm] Rssi
 * @param fei [Hz] Fei
 * @param rx_bitrate [kb/s] Rx bitrate
 * @param tx_bitrate [kb/s] Tx bitrate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_receiver_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float rssi,float fei,uint64_t rx_bitrate,uint64_t tx_bitrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RECEIVER_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, rx_bitrate);
    _mav_put_uint64_t(buf, 16, tx_bitrate);
    _mav_put_float(buf, 24, rssi);
    _mav_put_float(buf, 28, fei);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RECEIVER_TM_LEN);
#else
    mavlink_receiver_tm_t packet;
    packet.timestamp = timestamp;
    packet.rx_bitrate = rx_bitrate;
    packet.tx_bitrate = tx_bitrate;
    packet.rssi = rssi;
    packet.fei = fei;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RECEIVER_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RECEIVER_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN, MAVLINK_MSG_ID_RECEIVER_TM_LEN, MAVLINK_MSG_ID_RECEIVER_TM_CRC);
}

/**
 * @brief Encode a receiver_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param receiver_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_receiver_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_receiver_tm_t* receiver_tm)
{
    return mavlink_msg_receiver_tm_pack(system_id, component_id, msg, receiver_tm->timestamp, receiver_tm->rssi, receiver_tm->fei, receiver_tm->rx_bitrate, receiver_tm->tx_bitrate);
}

/**
 * @brief Encode a receiver_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param receiver_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_receiver_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_receiver_tm_t* receiver_tm)
{
    return mavlink_msg_receiver_tm_pack_chan(system_id, component_id, chan, msg, receiver_tm->timestamp, receiver_tm->rssi, receiver_tm->fei, receiver_tm->rx_bitrate, receiver_tm->tx_bitrate);
}

/**
 * @brief Send a receiver_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param rssi [dBm] Rssi
 * @param fei [Hz] Fei
 * @param rx_bitrate [kb/s] Rx bitrate
 * @param tx_bitrate [kb/s] Tx bitrate
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_receiver_tm_send(mavlink_channel_t chan, uint64_t timestamp, float rssi, float fei, uint64_t rx_bitrate, uint64_t tx_bitrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RECEIVER_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, rx_bitrate);
    _mav_put_uint64_t(buf, 16, tx_bitrate);
    _mav_put_float(buf, 24, rssi);
    _mav_put_float(buf, 28, fei);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RECEIVER_TM, buf, MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN, MAVLINK_MSG_ID_RECEIVER_TM_LEN, MAVLINK_MSG_ID_RECEIVER_TM_CRC);
#else
    mavlink_receiver_tm_t packet;
    packet.timestamp = timestamp;
    packet.rx_bitrate = rx_bitrate;
    packet.tx_bitrate = tx_bitrate;
    packet.rssi = rssi;
    packet.fei = fei;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RECEIVER_TM, (const char *)&packet, MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN, MAVLINK_MSG_ID_RECEIVER_TM_LEN, MAVLINK_MSG_ID_RECEIVER_TM_CRC);
#endif
}

/**
 * @brief Send a receiver_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_receiver_tm_send_struct(mavlink_channel_t chan, const mavlink_receiver_tm_t* receiver_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_receiver_tm_send(chan, receiver_tm->timestamp, receiver_tm->rssi, receiver_tm->fei, receiver_tm->rx_bitrate, receiver_tm->tx_bitrate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RECEIVER_TM, (const char *)receiver_tm, MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN, MAVLINK_MSG_ID_RECEIVER_TM_LEN, MAVLINK_MSG_ID_RECEIVER_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_RECEIVER_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_receiver_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float rssi, float fei, uint64_t rx_bitrate, uint64_t tx_bitrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, rx_bitrate);
    _mav_put_uint64_t(buf, 16, tx_bitrate);
    _mav_put_float(buf, 24, rssi);
    _mav_put_float(buf, 28, fei);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RECEIVER_TM, buf, MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN, MAVLINK_MSG_ID_RECEIVER_TM_LEN, MAVLINK_MSG_ID_RECEIVER_TM_CRC);
#else
    mavlink_receiver_tm_t *packet = (mavlink_receiver_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->rx_bitrate = rx_bitrate;
    packet->tx_bitrate = tx_bitrate;
    packet->rssi = rssi;
    packet->fei = fei;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RECEIVER_TM, (const char *)packet, MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN, MAVLINK_MSG_ID_RECEIVER_TM_LEN, MAVLINK_MSG_ID_RECEIVER_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE RECEIVER_TM UNPACKING


/**
 * @brief Get field timestamp from receiver_tm message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_receiver_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field rssi from receiver_tm message
 *
 * @return [dBm] Rssi
 */
static inline float mavlink_msg_receiver_tm_get_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field fei from receiver_tm message
 *
 * @return [Hz] Fei
 */
static inline float mavlink_msg_receiver_tm_get_fei(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field rx_bitrate from receiver_tm message
 *
 * @return [kb/s] Rx bitrate
 */
static inline uint64_t mavlink_msg_receiver_tm_get_rx_bitrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field tx_bitrate from receiver_tm message
 *
 * @return [kb/s] Tx bitrate
 */
static inline uint64_t mavlink_msg_receiver_tm_get_tx_bitrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Decode a receiver_tm message into a struct
 *
 * @param msg The message to decode
 * @param receiver_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_receiver_tm_decode(const mavlink_message_t* msg, mavlink_receiver_tm_t* receiver_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    receiver_tm->timestamp = mavlink_msg_receiver_tm_get_timestamp(msg);
    receiver_tm->rx_bitrate = mavlink_msg_receiver_tm_get_rx_bitrate(msg);
    receiver_tm->tx_bitrate = mavlink_msg_receiver_tm_get_tx_bitrate(msg);
    receiver_tm->rssi = mavlink_msg_receiver_tm_get_rssi(msg);
    receiver_tm->fei = mavlink_msg_receiver_tm_get_fei(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RECEIVER_TM_LEN? msg->len : MAVLINK_MSG_ID_RECEIVER_TM_LEN;
        memset(receiver_tm, 0, MAVLINK_MSG_ID_RECEIVER_TM_LEN);
    memcpy(receiver_tm, _MAV_PAYLOAD(msg), len);
#endif
}
