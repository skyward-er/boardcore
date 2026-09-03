#pragma once
// MESSAGE HR_TM PACKING

#define MAVLINK_MSG_ID_HR_TM 180

MAVPACKED(
typedef struct __mavlink_hr_tm_t {
 uint8_t payload[104]; /*<  Payload of the packet. Payload contains various bit-packet data*/
}) mavlink_hr_tm_t;

#define MAVLINK_MSG_ID_HR_TM_LEN 104
#define MAVLINK_MSG_ID_HR_TM_MIN_LEN 104
#define MAVLINK_MSG_ID_180_LEN 104
#define MAVLINK_MSG_ID_180_MIN_LEN 104

#define MAVLINK_MSG_ID_HR_TM_CRC 142
#define MAVLINK_MSG_ID_180_CRC 142

#define MAVLINK_MSG_HR_TM_FIELD_PAYLOAD_LEN 104

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HR_TM { \
    180, \
    "HR_TM", \
    1, \
    {  { "payload", NULL, MAVLINK_TYPE_UINT8_T, 104, 0, offsetof(mavlink_hr_tm_t, payload) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HR_TM { \
    "HR_TM", \
    1, \
    {  { "payload", NULL, MAVLINK_TYPE_UINT8_T, 104, 0, offsetof(mavlink_hr_tm_t, payload) }, \
         } \
}
#endif

/**
 * @brief Pack a hr_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param payload  Payload of the packet. Payload contains various bit-packet data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hr_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HR_TM_LEN];

    _mav_put_uint8_t_array(buf, 0, payload, 104);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HR_TM_LEN);
#else
    mavlink_hr_tm_t packet;

    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*104);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HR_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HR_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
}

/**
 * @brief Pack a hr_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param payload  Payload of the packet. Payload contains various bit-packet data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hr_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HR_TM_LEN];

    _mav_put_uint8_t_array(buf, 0, payload, 104);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HR_TM_LEN);
#else
    mavlink_hr_tm_t packet;

    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*104);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HR_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HR_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
}

/**
 * @brief Encode a hr_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hr_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hr_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hr_tm_t* hr_tm)
{
    return mavlink_msg_hr_tm_pack(system_id, component_id, msg, hr_tm->payload);
}

/**
 * @brief Encode a hr_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hr_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hr_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hr_tm_t* hr_tm)
{
    return mavlink_msg_hr_tm_pack_chan(system_id, component_id, chan, msg, hr_tm->payload);
}

/**
 * @brief Send a hr_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param payload  Payload of the packet. Payload contains various bit-packet data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hr_tm_send(mavlink_channel_t chan, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HR_TM_LEN];

    _mav_put_uint8_t_array(buf, 0, payload, 104);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_TM, buf, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
#else
    mavlink_hr_tm_t packet;

    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*104);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_TM, (const char *)&packet, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
#endif
}

/**
 * @brief Send a hr_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hr_tm_send_struct(mavlink_channel_t chan, const mavlink_hr_tm_t* hr_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hr_tm_send(chan, hr_tm->payload);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_TM, (const char *)hr_tm, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_HR_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hr_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint8_t_array(buf, 0, payload, 104);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_TM, buf, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
#else
    mavlink_hr_tm_t *packet = (mavlink_hr_tm_t *)msgbuf;

    mav_array_memcpy(packet->payload, payload, sizeof(uint8_t)*104);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_TM, (const char *)packet, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE HR_TM UNPACKING


/**
 * @brief Get field payload from hr_tm message
 *
 * @return  Payload of the packet. Payload contains various bit-packet data
 */
static inline uint16_t mavlink_msg_hr_tm_get_payload(const mavlink_message_t* msg, uint8_t *payload)
{
    return _MAV_RETURN_uint8_t_array(msg, payload, 104,  0);
}

/**
 * @brief Decode a hr_tm message into a struct
 *
 * @param msg The message to decode
 * @param hr_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_hr_tm_decode(const mavlink_message_t* msg, mavlink_hr_tm_t* hr_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hr_tm_get_payload(msg, hr_tm->payload);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HR_TM_LEN? msg->len : MAVLINK_MSG_ID_HR_TM_LEN;
        memset(hr_tm, 0, MAVLINK_MSG_ID_HR_TM_LEN);
    memcpy(hr_tm, _MAV_PAYLOAD(msg), len);
#endif
}
