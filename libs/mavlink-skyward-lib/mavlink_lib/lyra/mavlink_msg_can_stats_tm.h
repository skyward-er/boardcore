#pragma once
// MESSAGE CAN_STATS_TM PACKING

#define MAVLINK_MSG_ID_CAN_STATS_TM 203


typedef struct __mavlink_can_stats_tm_t {
 uint64_t timestamp; /*< [us] When was this logged*/
 float payload_bit_rate; /*<  Payload only bitrate*/
 float total_bit_rate; /*<  Total bitrate*/
 float load_percent; /*<  Load percent of the BUS*/
} mavlink_can_stats_tm_t;

#define MAVLINK_MSG_ID_CAN_STATS_TM_LEN 20
#define MAVLINK_MSG_ID_CAN_STATS_TM_MIN_LEN 20
#define MAVLINK_MSG_ID_203_LEN 20
#define MAVLINK_MSG_ID_203_MIN_LEN 20

#define MAVLINK_MSG_ID_CAN_STATS_TM_CRC 39
#define MAVLINK_MSG_ID_203_CRC 39



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAN_STATS_TM { \
    203, \
    "CAN_STATS_TM", \
    4, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_can_stats_tm_t, timestamp) }, \
         { "payload_bit_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_can_stats_tm_t, payload_bit_rate) }, \
         { "total_bit_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_can_stats_tm_t, total_bit_rate) }, \
         { "load_percent", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_can_stats_tm_t, load_percent) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAN_STATS_TM { \
    "CAN_STATS_TM", \
    4, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_can_stats_tm_t, timestamp) }, \
         { "payload_bit_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_can_stats_tm_t, payload_bit_rate) }, \
         { "total_bit_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_can_stats_tm_t, total_bit_rate) }, \
         { "load_percent", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_can_stats_tm_t, load_percent) }, \
         } \
}
#endif

/**
 * @brief Pack a can_stats_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] When was this logged
 * @param payload_bit_rate  Payload only bitrate
 * @param total_bit_rate  Total bitrate
 * @param load_percent  Load percent of the BUS
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_stats_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float payload_bit_rate, float total_bit_rate, float load_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, payload_bit_rate);
    _mav_put_float(buf, 12, total_bit_rate);
    _mav_put_float(buf, 16, load_percent);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_STATS_TM_LEN);
#else
    mavlink_can_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.payload_bit_rate = payload_bit_rate;
    packet.total_bit_rate = total_bit_rate;
    packet.load_percent = load_percent;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_STATS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_STATS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAN_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_CRC);
}

/**
 * @brief Pack a can_stats_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] When was this logged
 * @param payload_bit_rate  Payload only bitrate
 * @param total_bit_rate  Total bitrate
 * @param load_percent  Load percent of the BUS
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_stats_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float payload_bit_rate,float total_bit_rate,float load_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, payload_bit_rate);
    _mav_put_float(buf, 12, total_bit_rate);
    _mav_put_float(buf, 16, load_percent);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_STATS_TM_LEN);
#else
    mavlink_can_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.payload_bit_rate = payload_bit_rate;
    packet.total_bit_rate = total_bit_rate;
    packet.load_percent = load_percent;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_STATS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_STATS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAN_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_CRC);
}

/**
 * @brief Encode a can_stats_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param can_stats_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_stats_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_can_stats_tm_t* can_stats_tm)
{
    return mavlink_msg_can_stats_tm_pack(system_id, component_id, msg, can_stats_tm->timestamp, can_stats_tm->payload_bit_rate, can_stats_tm->total_bit_rate, can_stats_tm->load_percent);
}

/**
 * @brief Encode a can_stats_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param can_stats_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_stats_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_can_stats_tm_t* can_stats_tm)
{
    return mavlink_msg_can_stats_tm_pack_chan(system_id, component_id, chan, msg, can_stats_tm->timestamp, can_stats_tm->payload_bit_rate, can_stats_tm->total_bit_rate, can_stats_tm->load_percent);
}

/**
 * @brief Send a can_stats_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged
 * @param payload_bit_rate  Payload only bitrate
 * @param total_bit_rate  Total bitrate
 * @param load_percent  Load percent of the BUS
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_can_stats_tm_send(mavlink_channel_t chan, uint64_t timestamp, float payload_bit_rate, float total_bit_rate, float load_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, payload_bit_rate);
    _mav_put_float(buf, 12, total_bit_rate);
    _mav_put_float(buf, 16, load_percent);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_STATS_TM, buf, MAVLINK_MSG_ID_CAN_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_CRC);
#else
    mavlink_can_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.payload_bit_rate = payload_bit_rate;
    packet.total_bit_rate = total_bit_rate;
    packet.load_percent = load_percent;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_STATS_TM, (const char *)&packet, MAVLINK_MSG_ID_CAN_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_CRC);
#endif
}

/**
 * @brief Send a can_stats_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_can_stats_tm_send_struct(mavlink_channel_t chan, const mavlink_can_stats_tm_t* can_stats_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_can_stats_tm_send(chan, can_stats_tm->timestamp, can_stats_tm->payload_bit_rate, can_stats_tm->total_bit_rate, can_stats_tm->load_percent);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_STATS_TM, (const char *)can_stats_tm, MAVLINK_MSG_ID_CAN_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAN_STATS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_can_stats_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float payload_bit_rate, float total_bit_rate, float load_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, payload_bit_rate);
    _mav_put_float(buf, 12, total_bit_rate);
    _mav_put_float(buf, 16, load_percent);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_STATS_TM, buf, MAVLINK_MSG_ID_CAN_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_CRC);
#else
    mavlink_can_stats_tm_t *packet = (mavlink_can_stats_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->payload_bit_rate = payload_bit_rate;
    packet->total_bit_rate = total_bit_rate;
    packet->load_percent = load_percent;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_STATS_TM, (const char *)packet, MAVLINK_MSG_ID_CAN_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_LEN, MAVLINK_MSG_ID_CAN_STATS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE CAN_STATS_TM UNPACKING


/**
 * @brief Get field timestamp from can_stats_tm message
 *
 * @return [us] When was this logged
 */
static inline uint64_t mavlink_msg_can_stats_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field payload_bit_rate from can_stats_tm message
 *
 * @return  Payload only bitrate
 */
static inline float mavlink_msg_can_stats_tm_get_payload_bit_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field total_bit_rate from can_stats_tm message
 *
 * @return  Total bitrate
 */
static inline float mavlink_msg_can_stats_tm_get_total_bit_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field load_percent from can_stats_tm message
 *
 * @return  Load percent of the BUS
 */
static inline float mavlink_msg_can_stats_tm_get_load_percent(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a can_stats_tm message into a struct
 *
 * @param msg The message to decode
 * @param can_stats_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_can_stats_tm_decode(const mavlink_message_t* msg, mavlink_can_stats_tm_t* can_stats_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    can_stats_tm->timestamp = mavlink_msg_can_stats_tm_get_timestamp(msg);
    can_stats_tm->payload_bit_rate = mavlink_msg_can_stats_tm_get_payload_bit_rate(msg);
    can_stats_tm->total_bit_rate = mavlink_msg_can_stats_tm_get_total_bit_rate(msg);
    can_stats_tm->load_percent = mavlink_msg_can_stats_tm_get_load_percent(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAN_STATS_TM_LEN? msg->len : MAVLINK_MSG_ID_CAN_STATS_TM_LEN;
        memset(can_stats_tm, 0, MAVLINK_MSG_ID_CAN_STATS_TM_LEN);
    memcpy(can_stats_tm, _MAV_PAYLOAD(msg), len);
#endif
}
