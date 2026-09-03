#pragma once
// MESSAGE ROCKET_RADIO_LINK_INFO_TM PACKING

#define MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM 242


typedef struct __mavlink_rocket_radio_link_info_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 uint32_t main_frequency; /*< [Hz] Main radio frequency*/
 uint32_t payload_frequency; /*< [Hz] Payload radio frequency*/
 uint16_t main_rx_success_count; /*<  Main radio received packet count*/
 uint16_t main_rx_drop_count; /*<  Main radio dropped packet count (e.g. CRC mismatch)*/
 uint16_t main_bitrate; /*< [b/s] Main radio bitrate*/
 uint16_t payload_rx_success_count; /*<  Payload radio received packet count*/
 uint16_t payload_rx_drop_count; /*<  Payload radio dropped packet count (e.g. CRC mismatch)*/
 uint16_t payload_bitrate; /*< [b/s] Payload radio bitrate*/
 uint8_t radio_433_type; /*<  Radio 433 type of the device*/
 uint8_t radio_868_type; /*<  Radio 868 type of the device*/
 int8_t main_rssi; /*< [dBm] Main radio RSSI*/
 int8_t payload_rssi; /*< [dBm] Payload radio RSSI*/
 uint8_t ethernet_status; /*<  Status flag indicating the status of the ethernet PHY*/
} mavlink_rocket_radio_link_info_tm_t;

#define MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN 33
#define MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_MIN_LEN 33
#define MAVLINK_MSG_ID_242_LEN 33
#define MAVLINK_MSG_ID_242_MIN_LEN 33

#define MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_CRC 97
#define MAVLINK_MSG_ID_242_CRC 97



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROCKET_RADIO_LINK_INFO_TM { \
    242, \
    "ROCKET_RADIO_LINK_INFO_TM", \
    14, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rocket_radio_link_info_tm_t, timestamp) }, \
         { "radio_433_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_rocket_radio_link_info_tm_t, radio_433_type) }, \
         { "radio_868_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_rocket_radio_link_info_tm_t, radio_868_type) }, \
         { "main_rssi", NULL, MAVLINK_TYPE_INT8_T, 0, 30, offsetof(mavlink_rocket_radio_link_info_tm_t, main_rssi) }, \
         { "main_rx_success_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_rocket_radio_link_info_tm_t, main_rx_success_count) }, \
         { "main_rx_drop_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_rocket_radio_link_info_tm_t, main_rx_drop_count) }, \
         { "main_bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_rocket_radio_link_info_tm_t, main_bitrate) }, \
         { "main_frequency", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_rocket_radio_link_info_tm_t, main_frequency) }, \
         { "payload_rssi", NULL, MAVLINK_TYPE_INT8_T, 0, 31, offsetof(mavlink_rocket_radio_link_info_tm_t, payload_rssi) }, \
         { "payload_rx_success_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_rocket_radio_link_info_tm_t, payload_rx_success_count) }, \
         { "payload_rx_drop_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_rocket_radio_link_info_tm_t, payload_rx_drop_count) }, \
         { "payload_bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_rocket_radio_link_info_tm_t, payload_bitrate) }, \
         { "payload_frequency", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_rocket_radio_link_info_tm_t, payload_frequency) }, \
         { "ethernet_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_rocket_radio_link_info_tm_t, ethernet_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROCKET_RADIO_LINK_INFO_TM { \
    "ROCKET_RADIO_LINK_INFO_TM", \
    14, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rocket_radio_link_info_tm_t, timestamp) }, \
         { "radio_433_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_rocket_radio_link_info_tm_t, radio_433_type) }, \
         { "radio_868_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_rocket_radio_link_info_tm_t, radio_868_type) }, \
         { "main_rssi", NULL, MAVLINK_TYPE_INT8_T, 0, 30, offsetof(mavlink_rocket_radio_link_info_tm_t, main_rssi) }, \
         { "main_rx_success_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_rocket_radio_link_info_tm_t, main_rx_success_count) }, \
         { "main_rx_drop_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_rocket_radio_link_info_tm_t, main_rx_drop_count) }, \
         { "main_bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_rocket_radio_link_info_tm_t, main_bitrate) }, \
         { "main_frequency", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_rocket_radio_link_info_tm_t, main_frequency) }, \
         { "payload_rssi", NULL, MAVLINK_TYPE_INT8_T, 0, 31, offsetof(mavlink_rocket_radio_link_info_tm_t, payload_rssi) }, \
         { "payload_rx_success_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_rocket_radio_link_info_tm_t, payload_rx_success_count) }, \
         { "payload_rx_drop_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_rocket_radio_link_info_tm_t, payload_rx_drop_count) }, \
         { "payload_bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_rocket_radio_link_info_tm_t, payload_bitrate) }, \
         { "payload_frequency", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_rocket_radio_link_info_tm_t, payload_frequency) }, \
         { "ethernet_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_rocket_radio_link_info_tm_t, ethernet_status) }, \
         } \
}
#endif

/**
 * @brief Pack a rocket_radio_link_info_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param radio_433_type  Radio 433 type of the device
 * @param radio_868_type  Radio 868 type of the device
 * @param main_rssi [dBm] Main radio RSSI
 * @param main_rx_success_count  Main radio received packet count
 * @param main_rx_drop_count  Main radio dropped packet count (e.g. CRC mismatch)
 * @param main_bitrate [b/s] Main radio bitrate
 * @param main_frequency [Hz] Main radio frequency
 * @param payload_rssi [dBm] Payload radio RSSI
 * @param payload_rx_success_count  Payload radio received packet count
 * @param payload_rx_drop_count  Payload radio dropped packet count (e.g. CRC mismatch)
 * @param payload_bitrate [b/s] Payload radio bitrate
 * @param payload_frequency [Hz] Payload radio frequency
 * @param ethernet_status  Status flag indicating the status of the ethernet PHY
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rocket_radio_link_info_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t radio_433_type, uint8_t radio_868_type, int8_t main_rssi, uint16_t main_rx_success_count, uint16_t main_rx_drop_count, uint16_t main_bitrate, uint32_t main_frequency, int8_t payload_rssi, uint16_t payload_rx_success_count, uint16_t payload_rx_drop_count, uint16_t payload_bitrate, uint32_t payload_frequency, uint8_t ethernet_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, main_frequency);
    _mav_put_uint32_t(buf, 12, payload_frequency);
    _mav_put_uint16_t(buf, 16, main_rx_success_count);
    _mav_put_uint16_t(buf, 18, main_rx_drop_count);
    _mav_put_uint16_t(buf, 20, main_bitrate);
    _mav_put_uint16_t(buf, 22, payload_rx_success_count);
    _mav_put_uint16_t(buf, 24, payload_rx_drop_count);
    _mav_put_uint16_t(buf, 26, payload_bitrate);
    _mav_put_uint8_t(buf, 28, radio_433_type);
    _mav_put_uint8_t(buf, 29, radio_868_type);
    _mav_put_int8_t(buf, 30, main_rssi);
    _mav_put_int8_t(buf, 31, payload_rssi);
    _mav_put_uint8_t(buf, 32, ethernet_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN);
#else
    mavlink_rocket_radio_link_info_tm_t packet;
    packet.timestamp = timestamp;
    packet.main_frequency = main_frequency;
    packet.payload_frequency = payload_frequency;
    packet.main_rx_success_count = main_rx_success_count;
    packet.main_rx_drop_count = main_rx_drop_count;
    packet.main_bitrate = main_bitrate;
    packet.payload_rx_success_count = payload_rx_success_count;
    packet.payload_rx_drop_count = payload_rx_drop_count;
    packet.payload_bitrate = payload_bitrate;
    packet.radio_433_type = radio_433_type;
    packet.radio_868_type = radio_868_type;
    packet.main_rssi = main_rssi;
    packet.payload_rssi = payload_rssi;
    packet.ethernet_status = ethernet_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_CRC);
}

/**
 * @brief Pack a rocket_radio_link_info_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp in microseconds
 * @param radio_433_type  Radio 433 type of the device
 * @param radio_868_type  Radio 868 type of the device
 * @param main_rssi [dBm] Main radio RSSI
 * @param main_rx_success_count  Main radio received packet count
 * @param main_rx_drop_count  Main radio dropped packet count (e.g. CRC mismatch)
 * @param main_bitrate [b/s] Main radio bitrate
 * @param main_frequency [Hz] Main radio frequency
 * @param payload_rssi [dBm] Payload radio RSSI
 * @param payload_rx_success_count  Payload radio received packet count
 * @param payload_rx_drop_count  Payload radio dropped packet count (e.g. CRC mismatch)
 * @param payload_bitrate [b/s] Payload radio bitrate
 * @param payload_frequency [Hz] Payload radio frequency
 * @param ethernet_status  Status flag indicating the status of the ethernet PHY
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rocket_radio_link_info_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t radio_433_type,uint8_t radio_868_type,int8_t main_rssi,uint16_t main_rx_success_count,uint16_t main_rx_drop_count,uint16_t main_bitrate,uint32_t main_frequency,int8_t payload_rssi,uint16_t payload_rx_success_count,uint16_t payload_rx_drop_count,uint16_t payload_bitrate,uint32_t payload_frequency,uint8_t ethernet_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, main_frequency);
    _mav_put_uint32_t(buf, 12, payload_frequency);
    _mav_put_uint16_t(buf, 16, main_rx_success_count);
    _mav_put_uint16_t(buf, 18, main_rx_drop_count);
    _mav_put_uint16_t(buf, 20, main_bitrate);
    _mav_put_uint16_t(buf, 22, payload_rx_success_count);
    _mav_put_uint16_t(buf, 24, payload_rx_drop_count);
    _mav_put_uint16_t(buf, 26, payload_bitrate);
    _mav_put_uint8_t(buf, 28, radio_433_type);
    _mav_put_uint8_t(buf, 29, radio_868_type);
    _mav_put_int8_t(buf, 30, main_rssi);
    _mav_put_int8_t(buf, 31, payload_rssi);
    _mav_put_uint8_t(buf, 32, ethernet_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN);
#else
    mavlink_rocket_radio_link_info_tm_t packet;
    packet.timestamp = timestamp;
    packet.main_frequency = main_frequency;
    packet.payload_frequency = payload_frequency;
    packet.main_rx_success_count = main_rx_success_count;
    packet.main_rx_drop_count = main_rx_drop_count;
    packet.main_bitrate = main_bitrate;
    packet.payload_rx_success_count = payload_rx_success_count;
    packet.payload_rx_drop_count = payload_rx_drop_count;
    packet.payload_bitrate = payload_bitrate;
    packet.radio_433_type = radio_433_type;
    packet.radio_868_type = radio_868_type;
    packet.main_rssi = main_rssi;
    packet.payload_rssi = payload_rssi;
    packet.ethernet_status = ethernet_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_CRC);
}

/**
 * @brief Encode a rocket_radio_link_info_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rocket_radio_link_info_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rocket_radio_link_info_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rocket_radio_link_info_tm_t* rocket_radio_link_info_tm)
{
    return mavlink_msg_rocket_radio_link_info_tm_pack(system_id, component_id, msg, rocket_radio_link_info_tm->timestamp, rocket_radio_link_info_tm->radio_433_type, rocket_radio_link_info_tm->radio_868_type, rocket_radio_link_info_tm->main_rssi, rocket_radio_link_info_tm->main_rx_success_count, rocket_radio_link_info_tm->main_rx_drop_count, rocket_radio_link_info_tm->main_bitrate, rocket_radio_link_info_tm->main_frequency, rocket_radio_link_info_tm->payload_rssi, rocket_radio_link_info_tm->payload_rx_success_count, rocket_radio_link_info_tm->payload_rx_drop_count, rocket_radio_link_info_tm->payload_bitrate, rocket_radio_link_info_tm->payload_frequency, rocket_radio_link_info_tm->ethernet_status);
}

/**
 * @brief Encode a rocket_radio_link_info_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rocket_radio_link_info_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rocket_radio_link_info_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rocket_radio_link_info_tm_t* rocket_radio_link_info_tm)
{
    return mavlink_msg_rocket_radio_link_info_tm_pack_chan(system_id, component_id, chan, msg, rocket_radio_link_info_tm->timestamp, rocket_radio_link_info_tm->radio_433_type, rocket_radio_link_info_tm->radio_868_type, rocket_radio_link_info_tm->main_rssi, rocket_radio_link_info_tm->main_rx_success_count, rocket_radio_link_info_tm->main_rx_drop_count, rocket_radio_link_info_tm->main_bitrate, rocket_radio_link_info_tm->main_frequency, rocket_radio_link_info_tm->payload_rssi, rocket_radio_link_info_tm->payload_rx_success_count, rocket_radio_link_info_tm->payload_rx_drop_count, rocket_radio_link_info_tm->payload_bitrate, rocket_radio_link_info_tm->payload_frequency, rocket_radio_link_info_tm->ethernet_status);
}

/**
 * @brief Send a rocket_radio_link_info_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param radio_433_type  Radio 433 type of the device
 * @param radio_868_type  Radio 868 type of the device
 * @param main_rssi [dBm] Main radio RSSI
 * @param main_rx_success_count  Main radio received packet count
 * @param main_rx_drop_count  Main radio dropped packet count (e.g. CRC mismatch)
 * @param main_bitrate [b/s] Main radio bitrate
 * @param main_frequency [Hz] Main radio frequency
 * @param payload_rssi [dBm] Payload radio RSSI
 * @param payload_rx_success_count  Payload radio received packet count
 * @param payload_rx_drop_count  Payload radio dropped packet count (e.g. CRC mismatch)
 * @param payload_bitrate [b/s] Payload radio bitrate
 * @param payload_frequency [Hz] Payload radio frequency
 * @param ethernet_status  Status flag indicating the status of the ethernet PHY
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rocket_radio_link_info_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t radio_433_type, uint8_t radio_868_type, int8_t main_rssi, uint16_t main_rx_success_count, uint16_t main_rx_drop_count, uint16_t main_bitrate, uint32_t main_frequency, int8_t payload_rssi, uint16_t payload_rx_success_count, uint16_t payload_rx_drop_count, uint16_t payload_bitrate, uint32_t payload_frequency, uint8_t ethernet_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, main_frequency);
    _mav_put_uint32_t(buf, 12, payload_frequency);
    _mav_put_uint16_t(buf, 16, main_rx_success_count);
    _mav_put_uint16_t(buf, 18, main_rx_drop_count);
    _mav_put_uint16_t(buf, 20, main_bitrate);
    _mav_put_uint16_t(buf, 22, payload_rx_success_count);
    _mav_put_uint16_t(buf, 24, payload_rx_drop_count);
    _mav_put_uint16_t(buf, 26, payload_bitrate);
    _mav_put_uint8_t(buf, 28, radio_433_type);
    _mav_put_uint8_t(buf, 29, radio_868_type);
    _mav_put_int8_t(buf, 30, main_rssi);
    _mav_put_int8_t(buf, 31, payload_rssi);
    _mav_put_uint8_t(buf, 32, ethernet_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM, buf, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_CRC);
#else
    mavlink_rocket_radio_link_info_tm_t packet;
    packet.timestamp = timestamp;
    packet.main_frequency = main_frequency;
    packet.payload_frequency = payload_frequency;
    packet.main_rx_success_count = main_rx_success_count;
    packet.main_rx_drop_count = main_rx_drop_count;
    packet.main_bitrate = main_bitrate;
    packet.payload_rx_success_count = payload_rx_success_count;
    packet.payload_rx_drop_count = payload_rx_drop_count;
    packet.payload_bitrate = payload_bitrate;
    packet.radio_433_type = radio_433_type;
    packet.radio_868_type = radio_868_type;
    packet.main_rssi = main_rssi;
    packet.payload_rssi = payload_rssi;
    packet.ethernet_status = ethernet_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM, (const char *)&packet, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_CRC);
#endif
}

/**
 * @brief Send a rocket_radio_link_info_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rocket_radio_link_info_tm_send_struct(mavlink_channel_t chan, const mavlink_rocket_radio_link_info_tm_t* rocket_radio_link_info_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rocket_radio_link_info_tm_send(chan, rocket_radio_link_info_tm->timestamp, rocket_radio_link_info_tm->radio_433_type, rocket_radio_link_info_tm->radio_868_type, rocket_radio_link_info_tm->main_rssi, rocket_radio_link_info_tm->main_rx_success_count, rocket_radio_link_info_tm->main_rx_drop_count, rocket_radio_link_info_tm->main_bitrate, rocket_radio_link_info_tm->main_frequency, rocket_radio_link_info_tm->payload_rssi, rocket_radio_link_info_tm->payload_rx_success_count, rocket_radio_link_info_tm->payload_rx_drop_count, rocket_radio_link_info_tm->payload_bitrate, rocket_radio_link_info_tm->payload_frequency, rocket_radio_link_info_tm->ethernet_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM, (const char *)rocket_radio_link_info_tm, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rocket_radio_link_info_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t radio_433_type, uint8_t radio_868_type, int8_t main_rssi, uint16_t main_rx_success_count, uint16_t main_rx_drop_count, uint16_t main_bitrate, uint32_t main_frequency, int8_t payload_rssi, uint16_t payload_rx_success_count, uint16_t payload_rx_drop_count, uint16_t payload_bitrate, uint32_t payload_frequency, uint8_t ethernet_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, main_frequency);
    _mav_put_uint32_t(buf, 12, payload_frequency);
    _mav_put_uint16_t(buf, 16, main_rx_success_count);
    _mav_put_uint16_t(buf, 18, main_rx_drop_count);
    _mav_put_uint16_t(buf, 20, main_bitrate);
    _mav_put_uint16_t(buf, 22, payload_rx_success_count);
    _mav_put_uint16_t(buf, 24, payload_rx_drop_count);
    _mav_put_uint16_t(buf, 26, payload_bitrate);
    _mav_put_uint8_t(buf, 28, radio_433_type);
    _mav_put_uint8_t(buf, 29, radio_868_type);
    _mav_put_int8_t(buf, 30, main_rssi);
    _mav_put_int8_t(buf, 31, payload_rssi);
    _mav_put_uint8_t(buf, 32, ethernet_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM, buf, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_CRC);
#else
    mavlink_rocket_radio_link_info_tm_t *packet = (mavlink_rocket_radio_link_info_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->main_frequency = main_frequency;
    packet->payload_frequency = payload_frequency;
    packet->main_rx_success_count = main_rx_success_count;
    packet->main_rx_drop_count = main_rx_drop_count;
    packet->main_bitrate = main_bitrate;
    packet->payload_rx_success_count = payload_rx_success_count;
    packet->payload_rx_drop_count = payload_rx_drop_count;
    packet->payload_bitrate = payload_bitrate;
    packet->radio_433_type = radio_433_type;
    packet->radio_868_type = radio_868_type;
    packet->main_rssi = main_rssi;
    packet->payload_rssi = payload_rssi;
    packet->ethernet_status = ethernet_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM, (const char *)packet, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ROCKET_RADIO_LINK_INFO_TM UNPACKING


/**
 * @brief Get field timestamp from rocket_radio_link_info_tm message
 *
 * @return [us] Timestamp in microseconds
 */
static inline uint64_t mavlink_msg_rocket_radio_link_info_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field radio_433_type from rocket_radio_link_info_tm message
 *
 * @return  Radio 433 type of the device
 */
static inline uint8_t mavlink_msg_rocket_radio_link_info_tm_get_radio_433_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field radio_868_type from rocket_radio_link_info_tm message
 *
 * @return  Radio 868 type of the device
 */
static inline uint8_t mavlink_msg_rocket_radio_link_info_tm_get_radio_868_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field main_rssi from rocket_radio_link_info_tm message
 *
 * @return [dBm] Main radio RSSI
 */
static inline int8_t mavlink_msg_rocket_radio_link_info_tm_get_main_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  30);
}

/**
 * @brief Get field main_rx_success_count from rocket_radio_link_info_tm message
 *
 * @return  Main radio received packet count
 */
static inline uint16_t mavlink_msg_rocket_radio_link_info_tm_get_main_rx_success_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field main_rx_drop_count from rocket_radio_link_info_tm message
 *
 * @return  Main radio dropped packet count (e.g. CRC mismatch)
 */
static inline uint16_t mavlink_msg_rocket_radio_link_info_tm_get_main_rx_drop_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field main_bitrate from rocket_radio_link_info_tm message
 *
 * @return [b/s] Main radio bitrate
 */
static inline uint16_t mavlink_msg_rocket_radio_link_info_tm_get_main_bitrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field main_frequency from rocket_radio_link_info_tm message
 *
 * @return [Hz] Main radio frequency
 */
static inline uint32_t mavlink_msg_rocket_radio_link_info_tm_get_main_frequency(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field payload_rssi from rocket_radio_link_info_tm message
 *
 * @return [dBm] Payload radio RSSI
 */
static inline int8_t mavlink_msg_rocket_radio_link_info_tm_get_payload_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  31);
}

/**
 * @brief Get field payload_rx_success_count from rocket_radio_link_info_tm message
 *
 * @return  Payload radio received packet count
 */
static inline uint16_t mavlink_msg_rocket_radio_link_info_tm_get_payload_rx_success_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field payload_rx_drop_count from rocket_radio_link_info_tm message
 *
 * @return  Payload radio dropped packet count (e.g. CRC mismatch)
 */
static inline uint16_t mavlink_msg_rocket_radio_link_info_tm_get_payload_rx_drop_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field payload_bitrate from rocket_radio_link_info_tm message
 *
 * @return [b/s] Payload radio bitrate
 */
static inline uint16_t mavlink_msg_rocket_radio_link_info_tm_get_payload_bitrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field payload_frequency from rocket_radio_link_info_tm message
 *
 * @return [Hz] Payload radio frequency
 */
static inline uint32_t mavlink_msg_rocket_radio_link_info_tm_get_payload_frequency(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field ethernet_status from rocket_radio_link_info_tm message
 *
 * @return  Status flag indicating the status of the ethernet PHY
 */
static inline uint8_t mavlink_msg_rocket_radio_link_info_tm_get_ethernet_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Decode a rocket_radio_link_info_tm message into a struct
 *
 * @param msg The message to decode
 * @param rocket_radio_link_info_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_rocket_radio_link_info_tm_decode(const mavlink_message_t* msg, mavlink_rocket_radio_link_info_tm_t* rocket_radio_link_info_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rocket_radio_link_info_tm->timestamp = mavlink_msg_rocket_radio_link_info_tm_get_timestamp(msg);
    rocket_radio_link_info_tm->main_frequency = mavlink_msg_rocket_radio_link_info_tm_get_main_frequency(msg);
    rocket_radio_link_info_tm->payload_frequency = mavlink_msg_rocket_radio_link_info_tm_get_payload_frequency(msg);
    rocket_radio_link_info_tm->main_rx_success_count = mavlink_msg_rocket_radio_link_info_tm_get_main_rx_success_count(msg);
    rocket_radio_link_info_tm->main_rx_drop_count = mavlink_msg_rocket_radio_link_info_tm_get_main_rx_drop_count(msg);
    rocket_radio_link_info_tm->main_bitrate = mavlink_msg_rocket_radio_link_info_tm_get_main_bitrate(msg);
    rocket_radio_link_info_tm->payload_rx_success_count = mavlink_msg_rocket_radio_link_info_tm_get_payload_rx_success_count(msg);
    rocket_radio_link_info_tm->payload_rx_drop_count = mavlink_msg_rocket_radio_link_info_tm_get_payload_rx_drop_count(msg);
    rocket_radio_link_info_tm->payload_bitrate = mavlink_msg_rocket_radio_link_info_tm_get_payload_bitrate(msg);
    rocket_radio_link_info_tm->radio_433_type = mavlink_msg_rocket_radio_link_info_tm_get_radio_433_type(msg);
    rocket_radio_link_info_tm->radio_868_type = mavlink_msg_rocket_radio_link_info_tm_get_radio_868_type(msg);
    rocket_radio_link_info_tm->main_rssi = mavlink_msg_rocket_radio_link_info_tm_get_main_rssi(msg);
    rocket_radio_link_info_tm->payload_rssi = mavlink_msg_rocket_radio_link_info_tm_get_payload_rssi(msg);
    rocket_radio_link_info_tm->ethernet_status = mavlink_msg_rocket_radio_link_info_tm_get_ethernet_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN? msg->len : MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN;
        memset(rocket_radio_link_info_tm, 0, MAVLINK_MSG_ID_ROCKET_RADIO_LINK_INFO_TM_LEN);
    memcpy(rocket_radio_link_info_tm, _MAV_PAYLOAD(msg), len);
#endif
}
