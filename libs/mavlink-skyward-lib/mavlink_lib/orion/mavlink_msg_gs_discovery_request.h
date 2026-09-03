#pragma once
// MESSAGE GS_DISCOVERY_REQUEST PACKING

#define MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST 240


typedef struct __mavlink_gs_discovery_request_t {
 uint32_t source_ip; /*<  IP address of the requesting device*/
 uint16_t source_port; /*<  Port of the requesting device*/
} mavlink_gs_discovery_request_t;

#define MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN 6
#define MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_MIN_LEN 6
#define MAVLINK_MSG_ID_240_LEN 6
#define MAVLINK_MSG_ID_240_MIN_LEN 6

#define MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_CRC 234
#define MAVLINK_MSG_ID_240_CRC 234



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GS_DISCOVERY_REQUEST { \
    240, \
    "GS_DISCOVERY_REQUEST", \
    2, \
    {  { "source_ip", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gs_discovery_request_t, source_ip) }, \
         { "source_port", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_gs_discovery_request_t, source_port) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GS_DISCOVERY_REQUEST { \
    "GS_DISCOVERY_REQUEST", \
    2, \
    {  { "source_ip", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gs_discovery_request_t, source_ip) }, \
         { "source_port", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_gs_discovery_request_t, source_port) }, \
         } \
}
#endif

/**
 * @brief Pack a gs_discovery_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param source_ip  IP address of the requesting device
 * @param source_port  Port of the requesting device
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gs_discovery_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t source_ip, uint16_t source_port)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN];
    _mav_put_uint32_t(buf, 0, source_ip);
    _mav_put_uint16_t(buf, 4, source_port);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN);
#else
    mavlink_gs_discovery_request_t packet;
    packet.source_ip = source_ip;
    packet.source_port = source_port;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_CRC);
}

/**
 * @brief Pack a gs_discovery_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param source_ip  IP address of the requesting device
 * @param source_port  Port of the requesting device
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gs_discovery_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t source_ip,uint16_t source_port)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN];
    _mav_put_uint32_t(buf, 0, source_ip);
    _mav_put_uint16_t(buf, 4, source_port);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN);
#else
    mavlink_gs_discovery_request_t packet;
    packet.source_ip = source_ip;
    packet.source_port = source_port;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_CRC);
}

/**
 * @brief Encode a gs_discovery_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gs_discovery_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gs_discovery_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gs_discovery_request_t* gs_discovery_request)
{
    return mavlink_msg_gs_discovery_request_pack(system_id, component_id, msg, gs_discovery_request->source_ip, gs_discovery_request->source_port);
}

/**
 * @brief Encode a gs_discovery_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gs_discovery_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gs_discovery_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gs_discovery_request_t* gs_discovery_request)
{
    return mavlink_msg_gs_discovery_request_pack_chan(system_id, component_id, chan, msg, gs_discovery_request->source_ip, gs_discovery_request->source_port);
}

/**
 * @brief Send a gs_discovery_request message
 * @param chan MAVLink channel to send the message
 *
 * @param source_ip  IP address of the requesting device
 * @param source_port  Port of the requesting device
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gs_discovery_request_send(mavlink_channel_t chan, uint32_t source_ip, uint16_t source_port)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN];
    _mav_put_uint32_t(buf, 0, source_ip);
    _mav_put_uint16_t(buf, 4, source_port);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST, buf, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_CRC);
#else
    mavlink_gs_discovery_request_t packet;
    packet.source_ip = source_ip;
    packet.source_port = source_port;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_CRC);
#endif
}

/**
 * @brief Send a gs_discovery_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gs_discovery_request_send_struct(mavlink_channel_t chan, const mavlink_gs_discovery_request_t* gs_discovery_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gs_discovery_request_send(chan, gs_discovery_request->source_ip, gs_discovery_request->source_port);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST, (const char *)gs_discovery_request, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gs_discovery_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t source_ip, uint16_t source_port)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, source_ip);
    _mav_put_uint16_t(buf, 4, source_port);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST, buf, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_CRC);
#else
    mavlink_gs_discovery_request_t *packet = (mavlink_gs_discovery_request_t *)msgbuf;
    packet->source_ip = source_ip;
    packet->source_port = source_port;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST, (const char *)packet, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE GS_DISCOVERY_REQUEST UNPACKING


/**
 * @brief Get field source_ip from gs_discovery_request message
 *
 * @return  IP address of the requesting device
 */
static inline uint32_t mavlink_msg_gs_discovery_request_get_source_ip(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field source_port from gs_discovery_request message
 *
 * @return  Port of the requesting device
 */
static inline uint16_t mavlink_msg_gs_discovery_request_get_source_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a gs_discovery_request message into a struct
 *
 * @param msg The message to decode
 * @param gs_discovery_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_gs_discovery_request_decode(const mavlink_message_t* msg, mavlink_gs_discovery_request_t* gs_discovery_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gs_discovery_request->source_ip = mavlink_msg_gs_discovery_request_get_source_ip(msg);
    gs_discovery_request->source_port = mavlink_msg_gs_discovery_request_get_source_port(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN;
        memset(gs_discovery_request, 0, MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST_LEN);
    memcpy(gs_discovery_request, _MAV_PAYLOAD(msg), len);
#endif
}
