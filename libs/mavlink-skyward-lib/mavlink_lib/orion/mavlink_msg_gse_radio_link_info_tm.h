#pragma once
// MESSAGE GSE_RADIO_LINK_INFO_TM PACKING

#define MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM 243


typedef struct __mavlink_gse_radio_link_info_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 uint32_t frequency; /*< [Hz] Radio frequency*/
 uint16_t rx_success_count; /*<  Radio received packet count*/
 uint16_t rx_drop_count; /*<  Radio dropped packet count (e.g. CRC mismatch)*/
 uint16_t bitrate; /*< [b/s] Radio bitrate*/
 int8_t rssi; /*< [dBm] Radio RSSI*/
 int8_t snr; /*< [dB] Radio Signal to Noise Ratio*/
 uint8_t ethernet_status; /*<  Status flag indicating the status of the ethernet PHY*/
} mavlink_gse_radio_link_info_tm_t;

#define MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN 21
#define MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_MIN_LEN 21
#define MAVLINK_MSG_ID_243_LEN 21
#define MAVLINK_MSG_ID_243_MIN_LEN 21

#define MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_CRC 8
#define MAVLINK_MSG_ID_243_CRC 8



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GSE_RADIO_LINK_INFO_TM { \
    243, \
    "GSE_RADIO_LINK_INFO_TM", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gse_radio_link_info_tm_t, timestamp) }, \
         { "rssi", NULL, MAVLINK_TYPE_INT8_T, 0, 18, offsetof(mavlink_gse_radio_link_info_tm_t, rssi) }, \
         { "snr", NULL, MAVLINK_TYPE_INT8_T, 0, 19, offsetof(mavlink_gse_radio_link_info_tm_t, snr) }, \
         { "rx_success_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_gse_radio_link_info_tm_t, rx_success_count) }, \
         { "rx_drop_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_gse_radio_link_info_tm_t, rx_drop_count) }, \
         { "bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_gse_radio_link_info_tm_t, bitrate) }, \
         { "frequency", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_gse_radio_link_info_tm_t, frequency) }, \
         { "ethernet_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_gse_radio_link_info_tm_t, ethernet_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GSE_RADIO_LINK_INFO_TM { \
    "GSE_RADIO_LINK_INFO_TM", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gse_radio_link_info_tm_t, timestamp) }, \
         { "rssi", NULL, MAVLINK_TYPE_INT8_T, 0, 18, offsetof(mavlink_gse_radio_link_info_tm_t, rssi) }, \
         { "snr", NULL, MAVLINK_TYPE_INT8_T, 0, 19, offsetof(mavlink_gse_radio_link_info_tm_t, snr) }, \
         { "rx_success_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_gse_radio_link_info_tm_t, rx_success_count) }, \
         { "rx_drop_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_gse_radio_link_info_tm_t, rx_drop_count) }, \
         { "bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_gse_radio_link_info_tm_t, bitrate) }, \
         { "frequency", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_gse_radio_link_info_tm_t, frequency) }, \
         { "ethernet_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_gse_radio_link_info_tm_t, ethernet_status) }, \
         } \
}
#endif

/**
 * @brief Pack a gse_radio_link_info_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param rssi [dBm] Radio RSSI
 * @param snr [dB] Radio Signal to Noise Ratio
 * @param rx_success_count  Radio received packet count
 * @param rx_drop_count  Radio dropped packet count (e.g. CRC mismatch)
 * @param bitrate [b/s] Radio bitrate
 * @param frequency [Hz] Radio frequency
 * @param ethernet_status  Status flag indicating the status of the ethernet PHY
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gse_radio_link_info_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, int8_t rssi, int8_t snr, uint16_t rx_success_count, uint16_t rx_drop_count, uint16_t bitrate, uint32_t frequency, uint8_t ethernet_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, frequency);
    _mav_put_uint16_t(buf, 12, rx_success_count);
    _mav_put_uint16_t(buf, 14, rx_drop_count);
    _mav_put_uint16_t(buf, 16, bitrate);
    _mav_put_int8_t(buf, 18, rssi);
    _mav_put_int8_t(buf, 19, snr);
    _mav_put_uint8_t(buf, 20, ethernet_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN);
#else
    mavlink_gse_radio_link_info_tm_t packet;
    packet.timestamp = timestamp;
    packet.frequency = frequency;
    packet.rx_success_count = rx_success_count;
    packet.rx_drop_count = rx_drop_count;
    packet.bitrate = bitrate;
    packet.rssi = rssi;
    packet.snr = snr;
    packet.ethernet_status = ethernet_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_CRC);
}

/**
 * @brief Pack a gse_radio_link_info_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp in microseconds
 * @param rssi [dBm] Radio RSSI
 * @param snr [dB] Radio Signal to Noise Ratio
 * @param rx_success_count  Radio received packet count
 * @param rx_drop_count  Radio dropped packet count (e.g. CRC mismatch)
 * @param bitrate [b/s] Radio bitrate
 * @param frequency [Hz] Radio frequency
 * @param ethernet_status  Status flag indicating the status of the ethernet PHY
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gse_radio_link_info_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,int8_t rssi,int8_t snr,uint16_t rx_success_count,uint16_t rx_drop_count,uint16_t bitrate,uint32_t frequency,uint8_t ethernet_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, frequency);
    _mav_put_uint16_t(buf, 12, rx_success_count);
    _mav_put_uint16_t(buf, 14, rx_drop_count);
    _mav_put_uint16_t(buf, 16, bitrate);
    _mav_put_int8_t(buf, 18, rssi);
    _mav_put_int8_t(buf, 19, snr);
    _mav_put_uint8_t(buf, 20, ethernet_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN);
#else
    mavlink_gse_radio_link_info_tm_t packet;
    packet.timestamp = timestamp;
    packet.frequency = frequency;
    packet.rx_success_count = rx_success_count;
    packet.rx_drop_count = rx_drop_count;
    packet.bitrate = bitrate;
    packet.rssi = rssi;
    packet.snr = snr;
    packet.ethernet_status = ethernet_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_CRC);
}

/**
 * @brief Encode a gse_radio_link_info_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gse_radio_link_info_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gse_radio_link_info_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gse_radio_link_info_tm_t* gse_radio_link_info_tm)
{
    return mavlink_msg_gse_radio_link_info_tm_pack(system_id, component_id, msg, gse_radio_link_info_tm->timestamp, gse_radio_link_info_tm->rssi, gse_radio_link_info_tm->snr, gse_radio_link_info_tm->rx_success_count, gse_radio_link_info_tm->rx_drop_count, gse_radio_link_info_tm->bitrate, gse_radio_link_info_tm->frequency, gse_radio_link_info_tm->ethernet_status);
}

/**
 * @brief Encode a gse_radio_link_info_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gse_radio_link_info_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gse_radio_link_info_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gse_radio_link_info_tm_t* gse_radio_link_info_tm)
{
    return mavlink_msg_gse_radio_link_info_tm_pack_chan(system_id, component_id, chan, msg, gse_radio_link_info_tm->timestamp, gse_radio_link_info_tm->rssi, gse_radio_link_info_tm->snr, gse_radio_link_info_tm->rx_success_count, gse_radio_link_info_tm->rx_drop_count, gse_radio_link_info_tm->bitrate, gse_radio_link_info_tm->frequency, gse_radio_link_info_tm->ethernet_status);
}

/**
 * @brief Send a gse_radio_link_info_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param rssi [dBm] Radio RSSI
 * @param snr [dB] Radio Signal to Noise Ratio
 * @param rx_success_count  Radio received packet count
 * @param rx_drop_count  Radio dropped packet count (e.g. CRC mismatch)
 * @param bitrate [b/s] Radio bitrate
 * @param frequency [Hz] Radio frequency
 * @param ethernet_status  Status flag indicating the status of the ethernet PHY
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gse_radio_link_info_tm_send(mavlink_channel_t chan, uint64_t timestamp, int8_t rssi, int8_t snr, uint16_t rx_success_count, uint16_t rx_drop_count, uint16_t bitrate, uint32_t frequency, uint8_t ethernet_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, frequency);
    _mav_put_uint16_t(buf, 12, rx_success_count);
    _mav_put_uint16_t(buf, 14, rx_drop_count);
    _mav_put_uint16_t(buf, 16, bitrate);
    _mav_put_int8_t(buf, 18, rssi);
    _mav_put_int8_t(buf, 19, snr);
    _mav_put_uint8_t(buf, 20, ethernet_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM, buf, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_CRC);
#else
    mavlink_gse_radio_link_info_tm_t packet;
    packet.timestamp = timestamp;
    packet.frequency = frequency;
    packet.rx_success_count = rx_success_count;
    packet.rx_drop_count = rx_drop_count;
    packet.bitrate = bitrate;
    packet.rssi = rssi;
    packet.snr = snr;
    packet.ethernet_status = ethernet_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM, (const char *)&packet, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_CRC);
#endif
}

/**
 * @brief Send a gse_radio_link_info_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gse_radio_link_info_tm_send_struct(mavlink_channel_t chan, const mavlink_gse_radio_link_info_tm_t* gse_radio_link_info_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gse_radio_link_info_tm_send(chan, gse_radio_link_info_tm->timestamp, gse_radio_link_info_tm->rssi, gse_radio_link_info_tm->snr, gse_radio_link_info_tm->rx_success_count, gse_radio_link_info_tm->rx_drop_count, gse_radio_link_info_tm->bitrate, gse_radio_link_info_tm->frequency, gse_radio_link_info_tm->ethernet_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM, (const char *)gse_radio_link_info_tm, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gse_radio_link_info_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, int8_t rssi, int8_t snr, uint16_t rx_success_count, uint16_t rx_drop_count, uint16_t bitrate, uint32_t frequency, uint8_t ethernet_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, frequency);
    _mav_put_uint16_t(buf, 12, rx_success_count);
    _mav_put_uint16_t(buf, 14, rx_drop_count);
    _mav_put_uint16_t(buf, 16, bitrate);
    _mav_put_int8_t(buf, 18, rssi);
    _mav_put_int8_t(buf, 19, snr);
    _mav_put_uint8_t(buf, 20, ethernet_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM, buf, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_CRC);
#else
    mavlink_gse_radio_link_info_tm_t *packet = (mavlink_gse_radio_link_info_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->frequency = frequency;
    packet->rx_success_count = rx_success_count;
    packet->rx_drop_count = rx_drop_count;
    packet->bitrate = bitrate;
    packet->rssi = rssi;
    packet->snr = snr;
    packet->ethernet_status = ethernet_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM, (const char *)packet, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE GSE_RADIO_LINK_INFO_TM UNPACKING


/**
 * @brief Get field timestamp from gse_radio_link_info_tm message
 *
 * @return [us] Timestamp in microseconds
 */
static inline uint64_t mavlink_msg_gse_radio_link_info_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field rssi from gse_radio_link_info_tm message
 *
 * @return [dBm] Radio RSSI
 */
static inline int8_t mavlink_msg_gse_radio_link_info_tm_get_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  18);
}

/**
 * @brief Get field snr from gse_radio_link_info_tm message
 *
 * @return [dB] Radio Signal to Noise Ratio
 */
static inline int8_t mavlink_msg_gse_radio_link_info_tm_get_snr(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  19);
}

/**
 * @brief Get field rx_success_count from gse_radio_link_info_tm message
 *
 * @return  Radio received packet count
 */
static inline uint16_t mavlink_msg_gse_radio_link_info_tm_get_rx_success_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field rx_drop_count from gse_radio_link_info_tm message
 *
 * @return  Radio dropped packet count (e.g. CRC mismatch)
 */
static inline uint16_t mavlink_msg_gse_radio_link_info_tm_get_rx_drop_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field bitrate from gse_radio_link_info_tm message
 *
 * @return [b/s] Radio bitrate
 */
static inline uint16_t mavlink_msg_gse_radio_link_info_tm_get_bitrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field frequency from gse_radio_link_info_tm message
 *
 * @return [Hz] Radio frequency
 */
static inline uint32_t mavlink_msg_gse_radio_link_info_tm_get_frequency(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field ethernet_status from gse_radio_link_info_tm message
 *
 * @return  Status flag indicating the status of the ethernet PHY
 */
static inline uint8_t mavlink_msg_gse_radio_link_info_tm_get_ethernet_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a gse_radio_link_info_tm message into a struct
 *
 * @param msg The message to decode
 * @param gse_radio_link_info_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_gse_radio_link_info_tm_decode(const mavlink_message_t* msg, mavlink_gse_radio_link_info_tm_t* gse_radio_link_info_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gse_radio_link_info_tm->timestamp = mavlink_msg_gse_radio_link_info_tm_get_timestamp(msg);
    gse_radio_link_info_tm->frequency = mavlink_msg_gse_radio_link_info_tm_get_frequency(msg);
    gse_radio_link_info_tm->rx_success_count = mavlink_msg_gse_radio_link_info_tm_get_rx_success_count(msg);
    gse_radio_link_info_tm->rx_drop_count = mavlink_msg_gse_radio_link_info_tm_get_rx_drop_count(msg);
    gse_radio_link_info_tm->bitrate = mavlink_msg_gse_radio_link_info_tm_get_bitrate(msg);
    gse_radio_link_info_tm->rssi = mavlink_msg_gse_radio_link_info_tm_get_rssi(msg);
    gse_radio_link_info_tm->snr = mavlink_msg_gse_radio_link_info_tm_get_snr(msg);
    gse_radio_link_info_tm->ethernet_status = mavlink_msg_gse_radio_link_info_tm_get_ethernet_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN? msg->len : MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN;
        memset(gse_radio_link_info_tm, 0, MAVLINK_MSG_ID_GSE_RADIO_LINK_INFO_TM_LEN);
    memcpy(gse_radio_link_info_tm, _MAV_PAYLOAD(msg), len);
#endif
}
