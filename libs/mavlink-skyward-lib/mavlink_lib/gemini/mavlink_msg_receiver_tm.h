#pragma once
// MESSAGE RECEIVER_TM PACKING

#define MAVLINK_MSG_ID_RECEIVER_TM 150


typedef struct __mavlink_receiver_tm_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 float main_rx_rssi; /*< [dBm] Receive RSSI*/
 float main_rx_fei; /*< [Hz] Receive frequency error index*/
 float payload_rx_rssi; /*< [dBm] Receive RSSI*/
 float payload_rx_fei; /*< [Hz] Receive frequency error index*/
 float battery_voltage; /*< [V] Battery voltage*/
 uint16_t main_packet_tx_error_count; /*<  Number of errors during send*/
 uint16_t main_tx_bitrate; /*< [b/s] Send bitrate*/
 uint16_t main_packet_rx_success_count; /*<  Number of succesfull received mavlink packets*/
 uint16_t main_packet_rx_drop_count; /*<  Number of dropped mavlink packets*/
 uint16_t main_rx_bitrate; /*< [b/s] Receive bitrate*/
 uint16_t payload_packet_tx_error_count; /*<  Number of errors during send*/
 uint16_t payload_tx_bitrate; /*< [b/s] Send bitrate*/
 uint16_t payload_packet_rx_success_count; /*<  Number of succesfull received mavlink packets*/
 uint16_t payload_packet_rx_drop_count; /*<  Number of dropped mavlink packets*/
 uint16_t payload_rx_bitrate; /*< [b/s] Receive bitrate*/
 uint8_t main_radio_present; /*<  Boolean indicating the presence of the main radio*/
 uint8_t payload_radio_present; /*<  Boolean indicating the presence of the payload radio*/
 uint8_t ethernet_present; /*<  Boolean indicating the presence of the ethernet module*/
 uint8_t ethernet_status; /*<  Status flag indicating the status of the ethernet PHY*/
} mavlink_receiver_tm_t;

#define MAVLINK_MSG_ID_RECEIVER_TM_LEN 52
#define MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN 52
#define MAVLINK_MSG_ID_150_LEN 52
#define MAVLINK_MSG_ID_150_MIN_LEN 52

#define MAVLINK_MSG_ID_RECEIVER_TM_CRC 117
#define MAVLINK_MSG_ID_150_CRC 117



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RECEIVER_TM { \
    150, \
    "RECEIVER_TM", \
    20, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_receiver_tm_t, timestamp) }, \
         { "main_radio_present", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_receiver_tm_t, main_radio_present) }, \
         { "main_packet_tx_error_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_receiver_tm_t, main_packet_tx_error_count) }, \
         { "main_tx_bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_receiver_tm_t, main_tx_bitrate) }, \
         { "main_packet_rx_success_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_receiver_tm_t, main_packet_rx_success_count) }, \
         { "main_packet_rx_drop_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_receiver_tm_t, main_packet_rx_drop_count) }, \
         { "main_rx_bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_receiver_tm_t, main_rx_bitrate) }, \
         { "main_rx_rssi", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_receiver_tm_t, main_rx_rssi) }, \
         { "main_rx_fei", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_receiver_tm_t, main_rx_fei) }, \
         { "payload_radio_present", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_receiver_tm_t, payload_radio_present) }, \
         { "payload_packet_tx_error_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_receiver_tm_t, payload_packet_tx_error_count) }, \
         { "payload_tx_bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_receiver_tm_t, payload_tx_bitrate) }, \
         { "payload_packet_rx_success_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_receiver_tm_t, payload_packet_rx_success_count) }, \
         { "payload_packet_rx_drop_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_receiver_tm_t, payload_packet_rx_drop_count) }, \
         { "payload_rx_bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 46, offsetof(mavlink_receiver_tm_t, payload_rx_bitrate) }, \
         { "payload_rx_rssi", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_receiver_tm_t, payload_rx_rssi) }, \
         { "payload_rx_fei", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_receiver_tm_t, payload_rx_fei) }, \
         { "ethernet_present", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_receiver_tm_t, ethernet_present) }, \
         { "ethernet_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_receiver_tm_t, ethernet_status) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_receiver_tm_t, battery_voltage) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RECEIVER_TM { \
    "RECEIVER_TM", \
    20, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_receiver_tm_t, timestamp) }, \
         { "main_radio_present", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_receiver_tm_t, main_radio_present) }, \
         { "main_packet_tx_error_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_receiver_tm_t, main_packet_tx_error_count) }, \
         { "main_tx_bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_receiver_tm_t, main_tx_bitrate) }, \
         { "main_packet_rx_success_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_receiver_tm_t, main_packet_rx_success_count) }, \
         { "main_packet_rx_drop_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_receiver_tm_t, main_packet_rx_drop_count) }, \
         { "main_rx_bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_receiver_tm_t, main_rx_bitrate) }, \
         { "main_rx_rssi", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_receiver_tm_t, main_rx_rssi) }, \
         { "main_rx_fei", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_receiver_tm_t, main_rx_fei) }, \
         { "payload_radio_present", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_receiver_tm_t, payload_radio_present) }, \
         { "payload_packet_tx_error_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_receiver_tm_t, payload_packet_tx_error_count) }, \
         { "payload_tx_bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_receiver_tm_t, payload_tx_bitrate) }, \
         { "payload_packet_rx_success_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_receiver_tm_t, payload_packet_rx_success_count) }, \
         { "payload_packet_rx_drop_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_receiver_tm_t, payload_packet_rx_drop_count) }, \
         { "payload_rx_bitrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 46, offsetof(mavlink_receiver_tm_t, payload_rx_bitrate) }, \
         { "payload_rx_rssi", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_receiver_tm_t, payload_rx_rssi) }, \
         { "payload_rx_fei", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_receiver_tm_t, payload_rx_fei) }, \
         { "ethernet_present", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_receiver_tm_t, ethernet_present) }, \
         { "ethernet_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_receiver_tm_t, ethernet_status) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_receiver_tm_t, battery_voltage) }, \
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
 * @param main_radio_present  Boolean indicating the presence of the main radio
 * @param main_packet_tx_error_count  Number of errors during send
 * @param main_tx_bitrate [b/s] Send bitrate
 * @param main_packet_rx_success_count  Number of succesfull received mavlink packets
 * @param main_packet_rx_drop_count  Number of dropped mavlink packets
 * @param main_rx_bitrate [b/s] Receive bitrate
 * @param main_rx_rssi [dBm] Receive RSSI
 * @param main_rx_fei [Hz] Receive frequency error index
 * @param payload_radio_present  Boolean indicating the presence of the payload radio
 * @param payload_packet_tx_error_count  Number of errors during send
 * @param payload_tx_bitrate [b/s] Send bitrate
 * @param payload_packet_rx_success_count  Number of succesfull received mavlink packets
 * @param payload_packet_rx_drop_count  Number of dropped mavlink packets
 * @param payload_rx_bitrate [b/s] Receive bitrate
 * @param payload_rx_rssi [dBm] Receive RSSI
 * @param payload_rx_fei [Hz] Receive frequency error index
 * @param ethernet_present  Boolean indicating the presence of the ethernet module
 * @param ethernet_status  Status flag indicating the status of the ethernet PHY
 * @param battery_voltage [V] Battery voltage
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_receiver_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t main_radio_present, uint16_t main_packet_tx_error_count, uint16_t main_tx_bitrate, uint16_t main_packet_rx_success_count, uint16_t main_packet_rx_drop_count, uint16_t main_rx_bitrate, float main_rx_rssi, float main_rx_fei, uint8_t payload_radio_present, uint16_t payload_packet_tx_error_count, uint16_t payload_tx_bitrate, uint16_t payload_packet_rx_success_count, uint16_t payload_packet_rx_drop_count, uint16_t payload_rx_bitrate, float payload_rx_rssi, float payload_rx_fei, uint8_t ethernet_present, uint8_t ethernet_status, float battery_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RECEIVER_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, main_rx_rssi);
    _mav_put_float(buf, 12, main_rx_fei);
    _mav_put_float(buf, 16, payload_rx_rssi);
    _mav_put_float(buf, 20, payload_rx_fei);
    _mav_put_float(buf, 24, battery_voltage);
    _mav_put_uint16_t(buf, 28, main_packet_tx_error_count);
    _mav_put_uint16_t(buf, 30, main_tx_bitrate);
    _mav_put_uint16_t(buf, 32, main_packet_rx_success_count);
    _mav_put_uint16_t(buf, 34, main_packet_rx_drop_count);
    _mav_put_uint16_t(buf, 36, main_rx_bitrate);
    _mav_put_uint16_t(buf, 38, payload_packet_tx_error_count);
    _mav_put_uint16_t(buf, 40, payload_tx_bitrate);
    _mav_put_uint16_t(buf, 42, payload_packet_rx_success_count);
    _mav_put_uint16_t(buf, 44, payload_packet_rx_drop_count);
    _mav_put_uint16_t(buf, 46, payload_rx_bitrate);
    _mav_put_uint8_t(buf, 48, main_radio_present);
    _mav_put_uint8_t(buf, 49, payload_radio_present);
    _mav_put_uint8_t(buf, 50, ethernet_present);
    _mav_put_uint8_t(buf, 51, ethernet_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RECEIVER_TM_LEN);
#else
    mavlink_receiver_tm_t packet;
    packet.timestamp = timestamp;
    packet.main_rx_rssi = main_rx_rssi;
    packet.main_rx_fei = main_rx_fei;
    packet.payload_rx_rssi = payload_rx_rssi;
    packet.payload_rx_fei = payload_rx_fei;
    packet.battery_voltage = battery_voltage;
    packet.main_packet_tx_error_count = main_packet_tx_error_count;
    packet.main_tx_bitrate = main_tx_bitrate;
    packet.main_packet_rx_success_count = main_packet_rx_success_count;
    packet.main_packet_rx_drop_count = main_packet_rx_drop_count;
    packet.main_rx_bitrate = main_rx_bitrate;
    packet.payload_packet_tx_error_count = payload_packet_tx_error_count;
    packet.payload_tx_bitrate = payload_tx_bitrate;
    packet.payload_packet_rx_success_count = payload_packet_rx_success_count;
    packet.payload_packet_rx_drop_count = payload_packet_rx_drop_count;
    packet.payload_rx_bitrate = payload_rx_bitrate;
    packet.main_radio_present = main_radio_present;
    packet.payload_radio_present = payload_radio_present;
    packet.ethernet_present = ethernet_present;
    packet.ethernet_status = ethernet_status;

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
 * @param main_radio_present  Boolean indicating the presence of the main radio
 * @param main_packet_tx_error_count  Number of errors during send
 * @param main_tx_bitrate [b/s] Send bitrate
 * @param main_packet_rx_success_count  Number of succesfull received mavlink packets
 * @param main_packet_rx_drop_count  Number of dropped mavlink packets
 * @param main_rx_bitrate [b/s] Receive bitrate
 * @param main_rx_rssi [dBm] Receive RSSI
 * @param main_rx_fei [Hz] Receive frequency error index
 * @param payload_radio_present  Boolean indicating the presence of the payload radio
 * @param payload_packet_tx_error_count  Number of errors during send
 * @param payload_tx_bitrate [b/s] Send bitrate
 * @param payload_packet_rx_success_count  Number of succesfull received mavlink packets
 * @param payload_packet_rx_drop_count  Number of dropped mavlink packets
 * @param payload_rx_bitrate [b/s] Receive bitrate
 * @param payload_rx_rssi [dBm] Receive RSSI
 * @param payload_rx_fei [Hz] Receive frequency error index
 * @param ethernet_present  Boolean indicating the presence of the ethernet module
 * @param ethernet_status  Status flag indicating the status of the ethernet PHY
 * @param battery_voltage [V] Battery voltage
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_receiver_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t main_radio_present,uint16_t main_packet_tx_error_count,uint16_t main_tx_bitrate,uint16_t main_packet_rx_success_count,uint16_t main_packet_rx_drop_count,uint16_t main_rx_bitrate,float main_rx_rssi,float main_rx_fei,uint8_t payload_radio_present,uint16_t payload_packet_tx_error_count,uint16_t payload_tx_bitrate,uint16_t payload_packet_rx_success_count,uint16_t payload_packet_rx_drop_count,uint16_t payload_rx_bitrate,float payload_rx_rssi,float payload_rx_fei,uint8_t ethernet_present,uint8_t ethernet_status,float battery_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RECEIVER_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, main_rx_rssi);
    _mav_put_float(buf, 12, main_rx_fei);
    _mav_put_float(buf, 16, payload_rx_rssi);
    _mav_put_float(buf, 20, payload_rx_fei);
    _mav_put_float(buf, 24, battery_voltage);
    _mav_put_uint16_t(buf, 28, main_packet_tx_error_count);
    _mav_put_uint16_t(buf, 30, main_tx_bitrate);
    _mav_put_uint16_t(buf, 32, main_packet_rx_success_count);
    _mav_put_uint16_t(buf, 34, main_packet_rx_drop_count);
    _mav_put_uint16_t(buf, 36, main_rx_bitrate);
    _mav_put_uint16_t(buf, 38, payload_packet_tx_error_count);
    _mav_put_uint16_t(buf, 40, payload_tx_bitrate);
    _mav_put_uint16_t(buf, 42, payload_packet_rx_success_count);
    _mav_put_uint16_t(buf, 44, payload_packet_rx_drop_count);
    _mav_put_uint16_t(buf, 46, payload_rx_bitrate);
    _mav_put_uint8_t(buf, 48, main_radio_present);
    _mav_put_uint8_t(buf, 49, payload_radio_present);
    _mav_put_uint8_t(buf, 50, ethernet_present);
    _mav_put_uint8_t(buf, 51, ethernet_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RECEIVER_TM_LEN);
#else
    mavlink_receiver_tm_t packet;
    packet.timestamp = timestamp;
    packet.main_rx_rssi = main_rx_rssi;
    packet.main_rx_fei = main_rx_fei;
    packet.payload_rx_rssi = payload_rx_rssi;
    packet.payload_rx_fei = payload_rx_fei;
    packet.battery_voltage = battery_voltage;
    packet.main_packet_tx_error_count = main_packet_tx_error_count;
    packet.main_tx_bitrate = main_tx_bitrate;
    packet.main_packet_rx_success_count = main_packet_rx_success_count;
    packet.main_packet_rx_drop_count = main_packet_rx_drop_count;
    packet.main_rx_bitrate = main_rx_bitrate;
    packet.payload_packet_tx_error_count = payload_packet_tx_error_count;
    packet.payload_tx_bitrate = payload_tx_bitrate;
    packet.payload_packet_rx_success_count = payload_packet_rx_success_count;
    packet.payload_packet_rx_drop_count = payload_packet_rx_drop_count;
    packet.payload_rx_bitrate = payload_rx_bitrate;
    packet.main_radio_present = main_radio_present;
    packet.payload_radio_present = payload_radio_present;
    packet.ethernet_present = ethernet_present;
    packet.ethernet_status = ethernet_status;

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
    return mavlink_msg_receiver_tm_pack(system_id, component_id, msg, receiver_tm->timestamp, receiver_tm->main_radio_present, receiver_tm->main_packet_tx_error_count, receiver_tm->main_tx_bitrate, receiver_tm->main_packet_rx_success_count, receiver_tm->main_packet_rx_drop_count, receiver_tm->main_rx_bitrate, receiver_tm->main_rx_rssi, receiver_tm->main_rx_fei, receiver_tm->payload_radio_present, receiver_tm->payload_packet_tx_error_count, receiver_tm->payload_tx_bitrate, receiver_tm->payload_packet_rx_success_count, receiver_tm->payload_packet_rx_drop_count, receiver_tm->payload_rx_bitrate, receiver_tm->payload_rx_rssi, receiver_tm->payload_rx_fei, receiver_tm->ethernet_present, receiver_tm->ethernet_status, receiver_tm->battery_voltage);
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
    return mavlink_msg_receiver_tm_pack_chan(system_id, component_id, chan, msg, receiver_tm->timestamp, receiver_tm->main_radio_present, receiver_tm->main_packet_tx_error_count, receiver_tm->main_tx_bitrate, receiver_tm->main_packet_rx_success_count, receiver_tm->main_packet_rx_drop_count, receiver_tm->main_rx_bitrate, receiver_tm->main_rx_rssi, receiver_tm->main_rx_fei, receiver_tm->payload_radio_present, receiver_tm->payload_packet_tx_error_count, receiver_tm->payload_tx_bitrate, receiver_tm->payload_packet_rx_success_count, receiver_tm->payload_packet_rx_drop_count, receiver_tm->payload_rx_bitrate, receiver_tm->payload_rx_rssi, receiver_tm->payload_rx_fei, receiver_tm->ethernet_present, receiver_tm->ethernet_status, receiver_tm->battery_voltage);
}

/**
 * @brief Send a receiver_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param main_radio_present  Boolean indicating the presence of the main radio
 * @param main_packet_tx_error_count  Number of errors during send
 * @param main_tx_bitrate [b/s] Send bitrate
 * @param main_packet_rx_success_count  Number of succesfull received mavlink packets
 * @param main_packet_rx_drop_count  Number of dropped mavlink packets
 * @param main_rx_bitrate [b/s] Receive bitrate
 * @param main_rx_rssi [dBm] Receive RSSI
 * @param main_rx_fei [Hz] Receive frequency error index
 * @param payload_radio_present  Boolean indicating the presence of the payload radio
 * @param payload_packet_tx_error_count  Number of errors during send
 * @param payload_tx_bitrate [b/s] Send bitrate
 * @param payload_packet_rx_success_count  Number of succesfull received mavlink packets
 * @param payload_packet_rx_drop_count  Number of dropped mavlink packets
 * @param payload_rx_bitrate [b/s] Receive bitrate
 * @param payload_rx_rssi [dBm] Receive RSSI
 * @param payload_rx_fei [Hz] Receive frequency error index
 * @param ethernet_present  Boolean indicating the presence of the ethernet module
 * @param ethernet_status  Status flag indicating the status of the ethernet PHY
 * @param battery_voltage [V] Battery voltage
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_receiver_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t main_radio_present, uint16_t main_packet_tx_error_count, uint16_t main_tx_bitrate, uint16_t main_packet_rx_success_count, uint16_t main_packet_rx_drop_count, uint16_t main_rx_bitrate, float main_rx_rssi, float main_rx_fei, uint8_t payload_radio_present, uint16_t payload_packet_tx_error_count, uint16_t payload_tx_bitrate, uint16_t payload_packet_rx_success_count, uint16_t payload_packet_rx_drop_count, uint16_t payload_rx_bitrate, float payload_rx_rssi, float payload_rx_fei, uint8_t ethernet_present, uint8_t ethernet_status, float battery_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RECEIVER_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, main_rx_rssi);
    _mav_put_float(buf, 12, main_rx_fei);
    _mav_put_float(buf, 16, payload_rx_rssi);
    _mav_put_float(buf, 20, payload_rx_fei);
    _mav_put_float(buf, 24, battery_voltage);
    _mav_put_uint16_t(buf, 28, main_packet_tx_error_count);
    _mav_put_uint16_t(buf, 30, main_tx_bitrate);
    _mav_put_uint16_t(buf, 32, main_packet_rx_success_count);
    _mav_put_uint16_t(buf, 34, main_packet_rx_drop_count);
    _mav_put_uint16_t(buf, 36, main_rx_bitrate);
    _mav_put_uint16_t(buf, 38, payload_packet_tx_error_count);
    _mav_put_uint16_t(buf, 40, payload_tx_bitrate);
    _mav_put_uint16_t(buf, 42, payload_packet_rx_success_count);
    _mav_put_uint16_t(buf, 44, payload_packet_rx_drop_count);
    _mav_put_uint16_t(buf, 46, payload_rx_bitrate);
    _mav_put_uint8_t(buf, 48, main_radio_present);
    _mav_put_uint8_t(buf, 49, payload_radio_present);
    _mav_put_uint8_t(buf, 50, ethernet_present);
    _mav_put_uint8_t(buf, 51, ethernet_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RECEIVER_TM, buf, MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN, MAVLINK_MSG_ID_RECEIVER_TM_LEN, MAVLINK_MSG_ID_RECEIVER_TM_CRC);
#else
    mavlink_receiver_tm_t packet;
    packet.timestamp = timestamp;
    packet.main_rx_rssi = main_rx_rssi;
    packet.main_rx_fei = main_rx_fei;
    packet.payload_rx_rssi = payload_rx_rssi;
    packet.payload_rx_fei = payload_rx_fei;
    packet.battery_voltage = battery_voltage;
    packet.main_packet_tx_error_count = main_packet_tx_error_count;
    packet.main_tx_bitrate = main_tx_bitrate;
    packet.main_packet_rx_success_count = main_packet_rx_success_count;
    packet.main_packet_rx_drop_count = main_packet_rx_drop_count;
    packet.main_rx_bitrate = main_rx_bitrate;
    packet.payload_packet_tx_error_count = payload_packet_tx_error_count;
    packet.payload_tx_bitrate = payload_tx_bitrate;
    packet.payload_packet_rx_success_count = payload_packet_rx_success_count;
    packet.payload_packet_rx_drop_count = payload_packet_rx_drop_count;
    packet.payload_rx_bitrate = payload_rx_bitrate;
    packet.main_radio_present = main_radio_present;
    packet.payload_radio_present = payload_radio_present;
    packet.ethernet_present = ethernet_present;
    packet.ethernet_status = ethernet_status;

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
    mavlink_msg_receiver_tm_send(chan, receiver_tm->timestamp, receiver_tm->main_radio_present, receiver_tm->main_packet_tx_error_count, receiver_tm->main_tx_bitrate, receiver_tm->main_packet_rx_success_count, receiver_tm->main_packet_rx_drop_count, receiver_tm->main_rx_bitrate, receiver_tm->main_rx_rssi, receiver_tm->main_rx_fei, receiver_tm->payload_radio_present, receiver_tm->payload_packet_tx_error_count, receiver_tm->payload_tx_bitrate, receiver_tm->payload_packet_rx_success_count, receiver_tm->payload_packet_rx_drop_count, receiver_tm->payload_rx_bitrate, receiver_tm->payload_rx_rssi, receiver_tm->payload_rx_fei, receiver_tm->ethernet_present, receiver_tm->ethernet_status, receiver_tm->battery_voltage);
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
static inline void mavlink_msg_receiver_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t main_radio_present, uint16_t main_packet_tx_error_count, uint16_t main_tx_bitrate, uint16_t main_packet_rx_success_count, uint16_t main_packet_rx_drop_count, uint16_t main_rx_bitrate, float main_rx_rssi, float main_rx_fei, uint8_t payload_radio_present, uint16_t payload_packet_tx_error_count, uint16_t payload_tx_bitrate, uint16_t payload_packet_rx_success_count, uint16_t payload_packet_rx_drop_count, uint16_t payload_rx_bitrate, float payload_rx_rssi, float payload_rx_fei, uint8_t ethernet_present, uint8_t ethernet_status, float battery_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, main_rx_rssi);
    _mav_put_float(buf, 12, main_rx_fei);
    _mav_put_float(buf, 16, payload_rx_rssi);
    _mav_put_float(buf, 20, payload_rx_fei);
    _mav_put_float(buf, 24, battery_voltage);
    _mav_put_uint16_t(buf, 28, main_packet_tx_error_count);
    _mav_put_uint16_t(buf, 30, main_tx_bitrate);
    _mav_put_uint16_t(buf, 32, main_packet_rx_success_count);
    _mav_put_uint16_t(buf, 34, main_packet_rx_drop_count);
    _mav_put_uint16_t(buf, 36, main_rx_bitrate);
    _mav_put_uint16_t(buf, 38, payload_packet_tx_error_count);
    _mav_put_uint16_t(buf, 40, payload_tx_bitrate);
    _mav_put_uint16_t(buf, 42, payload_packet_rx_success_count);
    _mav_put_uint16_t(buf, 44, payload_packet_rx_drop_count);
    _mav_put_uint16_t(buf, 46, payload_rx_bitrate);
    _mav_put_uint8_t(buf, 48, main_radio_present);
    _mav_put_uint8_t(buf, 49, payload_radio_present);
    _mav_put_uint8_t(buf, 50, ethernet_present);
    _mav_put_uint8_t(buf, 51, ethernet_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RECEIVER_TM, buf, MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN, MAVLINK_MSG_ID_RECEIVER_TM_LEN, MAVLINK_MSG_ID_RECEIVER_TM_CRC);
#else
    mavlink_receiver_tm_t *packet = (mavlink_receiver_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->main_rx_rssi = main_rx_rssi;
    packet->main_rx_fei = main_rx_fei;
    packet->payload_rx_rssi = payload_rx_rssi;
    packet->payload_rx_fei = payload_rx_fei;
    packet->battery_voltage = battery_voltage;
    packet->main_packet_tx_error_count = main_packet_tx_error_count;
    packet->main_tx_bitrate = main_tx_bitrate;
    packet->main_packet_rx_success_count = main_packet_rx_success_count;
    packet->main_packet_rx_drop_count = main_packet_rx_drop_count;
    packet->main_rx_bitrate = main_rx_bitrate;
    packet->payload_packet_tx_error_count = payload_packet_tx_error_count;
    packet->payload_tx_bitrate = payload_tx_bitrate;
    packet->payload_packet_rx_success_count = payload_packet_rx_success_count;
    packet->payload_packet_rx_drop_count = payload_packet_rx_drop_count;
    packet->payload_rx_bitrate = payload_rx_bitrate;
    packet->main_radio_present = main_radio_present;
    packet->payload_radio_present = payload_radio_present;
    packet->ethernet_present = ethernet_present;
    packet->ethernet_status = ethernet_status;

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
 * @brief Get field main_radio_present from receiver_tm message
 *
 * @return  Boolean indicating the presence of the main radio
 */
static inline uint8_t mavlink_msg_receiver_tm_get_main_radio_present(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field main_packet_tx_error_count from receiver_tm message
 *
 * @return  Number of errors during send
 */
static inline uint16_t mavlink_msg_receiver_tm_get_main_packet_tx_error_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field main_tx_bitrate from receiver_tm message
 *
 * @return [b/s] Send bitrate
 */
static inline uint16_t mavlink_msg_receiver_tm_get_main_tx_bitrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field main_packet_rx_success_count from receiver_tm message
 *
 * @return  Number of succesfull received mavlink packets
 */
static inline uint16_t mavlink_msg_receiver_tm_get_main_packet_rx_success_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field main_packet_rx_drop_count from receiver_tm message
 *
 * @return  Number of dropped mavlink packets
 */
static inline uint16_t mavlink_msg_receiver_tm_get_main_packet_rx_drop_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field main_rx_bitrate from receiver_tm message
 *
 * @return [b/s] Receive bitrate
 */
static inline uint16_t mavlink_msg_receiver_tm_get_main_rx_bitrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field main_rx_rssi from receiver_tm message
 *
 * @return [dBm] Receive RSSI
 */
static inline float mavlink_msg_receiver_tm_get_main_rx_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field main_rx_fei from receiver_tm message
 *
 * @return [Hz] Receive frequency error index
 */
static inline float mavlink_msg_receiver_tm_get_main_rx_fei(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field payload_radio_present from receiver_tm message
 *
 * @return  Boolean indicating the presence of the payload radio
 */
static inline uint8_t mavlink_msg_receiver_tm_get_payload_radio_present(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field payload_packet_tx_error_count from receiver_tm message
 *
 * @return  Number of errors during send
 */
static inline uint16_t mavlink_msg_receiver_tm_get_payload_packet_tx_error_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  38);
}

/**
 * @brief Get field payload_tx_bitrate from receiver_tm message
 *
 * @return [b/s] Send bitrate
 */
static inline uint16_t mavlink_msg_receiver_tm_get_payload_tx_bitrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field payload_packet_rx_success_count from receiver_tm message
 *
 * @return  Number of succesfull received mavlink packets
 */
static inline uint16_t mavlink_msg_receiver_tm_get_payload_packet_rx_success_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  42);
}

/**
 * @brief Get field payload_packet_rx_drop_count from receiver_tm message
 *
 * @return  Number of dropped mavlink packets
 */
static inline uint16_t mavlink_msg_receiver_tm_get_payload_packet_rx_drop_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  44);
}

/**
 * @brief Get field payload_rx_bitrate from receiver_tm message
 *
 * @return [b/s] Receive bitrate
 */
static inline uint16_t mavlink_msg_receiver_tm_get_payload_rx_bitrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  46);
}

/**
 * @brief Get field payload_rx_rssi from receiver_tm message
 *
 * @return [dBm] Receive RSSI
 */
static inline float mavlink_msg_receiver_tm_get_payload_rx_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field payload_rx_fei from receiver_tm message
 *
 * @return [Hz] Receive frequency error index
 */
static inline float mavlink_msg_receiver_tm_get_payload_rx_fei(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ethernet_present from receiver_tm message
 *
 * @return  Boolean indicating the presence of the ethernet module
 */
static inline uint8_t mavlink_msg_receiver_tm_get_ethernet_present(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field ethernet_status from receiver_tm message
 *
 * @return  Status flag indicating the status of the ethernet PHY
 */
static inline uint8_t mavlink_msg_receiver_tm_get_ethernet_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Get field battery_voltage from receiver_tm message
 *
 * @return [V] Battery voltage
 */
static inline float mavlink_msg_receiver_tm_get_battery_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
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
    receiver_tm->main_rx_rssi = mavlink_msg_receiver_tm_get_main_rx_rssi(msg);
    receiver_tm->main_rx_fei = mavlink_msg_receiver_tm_get_main_rx_fei(msg);
    receiver_tm->payload_rx_rssi = mavlink_msg_receiver_tm_get_payload_rx_rssi(msg);
    receiver_tm->payload_rx_fei = mavlink_msg_receiver_tm_get_payload_rx_fei(msg);
    receiver_tm->battery_voltage = mavlink_msg_receiver_tm_get_battery_voltage(msg);
    receiver_tm->main_packet_tx_error_count = mavlink_msg_receiver_tm_get_main_packet_tx_error_count(msg);
    receiver_tm->main_tx_bitrate = mavlink_msg_receiver_tm_get_main_tx_bitrate(msg);
    receiver_tm->main_packet_rx_success_count = mavlink_msg_receiver_tm_get_main_packet_rx_success_count(msg);
    receiver_tm->main_packet_rx_drop_count = mavlink_msg_receiver_tm_get_main_packet_rx_drop_count(msg);
    receiver_tm->main_rx_bitrate = mavlink_msg_receiver_tm_get_main_rx_bitrate(msg);
    receiver_tm->payload_packet_tx_error_count = mavlink_msg_receiver_tm_get_payload_packet_tx_error_count(msg);
    receiver_tm->payload_tx_bitrate = mavlink_msg_receiver_tm_get_payload_tx_bitrate(msg);
    receiver_tm->payload_packet_rx_success_count = mavlink_msg_receiver_tm_get_payload_packet_rx_success_count(msg);
    receiver_tm->payload_packet_rx_drop_count = mavlink_msg_receiver_tm_get_payload_packet_rx_drop_count(msg);
    receiver_tm->payload_rx_bitrate = mavlink_msg_receiver_tm_get_payload_rx_bitrate(msg);
    receiver_tm->main_radio_present = mavlink_msg_receiver_tm_get_main_radio_present(msg);
    receiver_tm->payload_radio_present = mavlink_msg_receiver_tm_get_payload_radio_present(msg);
    receiver_tm->ethernet_present = mavlink_msg_receiver_tm_get_ethernet_present(msg);
    receiver_tm->ethernet_status = mavlink_msg_receiver_tm_get_ethernet_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RECEIVER_TM_LEN? msg->len : MAVLINK_MSG_ID_RECEIVER_TM_LEN;
        memset(receiver_tm, 0, MAVLINK_MSG_ID_RECEIVER_TM_LEN);
    memcpy(receiver_tm, _MAV_PAYLOAD(msg), len);
#endif
}
