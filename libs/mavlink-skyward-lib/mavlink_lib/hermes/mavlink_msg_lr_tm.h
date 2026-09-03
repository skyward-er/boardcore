#pragma once
// MESSAGE LR_TM PACKING

#define MAVLINK_MSG_ID_LR_TM 181

MAVPACKED(
typedef struct __mavlink_lr_tm_t {
 uint8_t payload[40]; /*<  Payload of the packet. Payload contains various bit-packet data*/
}) mavlink_lr_tm_t;

#define MAVLINK_MSG_ID_LR_TM_LEN 40
#define MAVLINK_MSG_ID_LR_TM_MIN_LEN 40
#define MAVLINK_MSG_ID_181_LEN 40
#define MAVLINK_MSG_ID_181_MIN_LEN 40

#define MAVLINK_MSG_ID_LR_TM_CRC 249
#define MAVLINK_MSG_ID_181_CRC 249

#define MAVLINK_MSG_LR_TM_FIELD_PAYLOAD_LEN 40

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LR_TM { \
    181, \
    "LR_TM", \
    1, \
    {  { "payload", NULL, MAVLINK_TYPE_UINT8_T, 40, 0, offsetof(mavlink_lr_tm_t, payload) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LR_TM { \
    "LR_TM", \
    1, \
    {  { "payload", NULL, MAVLINK_TYPE_UINT8_T, 40, 0, offsetof(mavlink_lr_tm_t, payload) }, \
         } \
}
#endif

/**
 * @brief Pack a lr_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param payload  Payload of the packet. Payload contains various bit-packet data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lr_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LR_TM_LEN];

    _mav_put_uint8_t_array(buf, 0, payload, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LR_TM_LEN);
#else
    mavlink_lr_tm_t packet;

    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LR_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LR_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
}

/**
 * @brief Pack a lr_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param payload  Payload of the packet. Payload contains various bit-packet data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lr_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LR_TM_LEN];

    _mav_put_uint8_t_array(buf, 0, payload, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LR_TM_LEN);
#else
    mavlink_lr_tm_t packet;

    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LR_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LR_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
}

/**
 * @brief Encode a lr_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lr_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lr_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lr_tm_t* lr_tm)
{
    return mavlink_msg_lr_tm_pack(system_id, component_id, msg, lr_tm->payload);
}

/**
 * @brief Encode a lr_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lr_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lr_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lr_tm_t* lr_tm)
{
    return mavlink_msg_lr_tm_pack_chan(system_id, component_id, chan, msg, lr_tm->payload);
}

/**
 * @brief Send a lr_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param payload  Payload of the packet. Payload contains various bit-packet data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lr_tm_send(mavlink_channel_t chan, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LR_TM_LEN];

    _mav_put_uint8_t_array(buf, 0, payload, 40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_TM, buf, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
#else
    mavlink_lr_tm_t packet;

    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_TM, (const char *)&packet, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
#endif
}

/**
 * @brief Send a lr_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lr_tm_send_struct(mavlink_channel_t chan, const mavlink_lr_tm_t* lr_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lr_tm_send(chan, lr_tm->payload);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_TM, (const char *)lr_tm, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_LR_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lr_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint8_t_array(buf, 0, payload, 40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_TM, buf, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
#else
    mavlink_lr_tm_t *packet = (mavlink_lr_tm_t *)msgbuf;

    mav_array_memcpy(packet->payload, payload, sizeof(uint8_t)*40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_TM, (const char *)packet, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE LR_TM UNPACKING


/**
 * @brief Get field payload from lr_tm message
 *
 * @return  Payload of the packet. Payload contains various bit-packet data
 */
static inline uint16_t mavlink_msg_lr_tm_get_payload(const mavlink_message_t* msg, uint8_t *payload)
{
    return _MAV_RETURN_uint8_t_array(msg, payload, 40,  0);
}

/**
 * @brief Decode a lr_tm message into a struct
 *
 * @param msg The message to decode
 * @param lr_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_lr_tm_decode(const mavlink_message_t* msg, mavlink_lr_tm_t* lr_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lr_tm_get_payload(msg, lr_tm->payload);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LR_TM_LEN? msg->len : MAVLINK_MSG_ID_LR_TM_LEN;
        memset(lr_tm, 0, MAVLINK_MSG_ID_LR_TM_LEN);
    memcpy(lr_tm, _MAV_PAYLOAD(msg), len);
#endif
}
