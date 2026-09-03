#pragma once
// MESSAGE IGNITION_STATUS_TM PACKING

#define MAVLINK_MSG_ID_IGNITION_STATUS_TM 142

MAVPACKED(
typedef struct __mavlink_ignition_status_tm_t {
 uint8_t connected; /*<  If False the other value are not valid.*/
 uint8_t ignition_state; /*<  Value of IGNITION_STATUS_ENUM.*/
}) mavlink_ignition_status_tm_t;

#define MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN 2
#define MAVLINK_MSG_ID_IGNITION_STATUS_TM_MIN_LEN 2
#define MAVLINK_MSG_ID_142_LEN 2
#define MAVLINK_MSG_ID_142_MIN_LEN 2

#define MAVLINK_MSG_ID_IGNITION_STATUS_TM_CRC 55
#define MAVLINK_MSG_ID_142_CRC 55



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IGNITION_STATUS_TM { \
    142, \
    "IGNITION_STATUS_TM", \
    2, \
    {  { "connected", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_ignition_status_tm_t, connected) }, \
         { "ignition_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_ignition_status_tm_t, ignition_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IGNITION_STATUS_TM { \
    "IGNITION_STATUS_TM", \
    2, \
    {  { "connected", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_ignition_status_tm_t, connected) }, \
         { "ignition_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_ignition_status_tm_t, ignition_state) }, \
         } \
}
#endif

/**
 * @brief Pack a ignition_status_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param connected  If False the other value are not valid.
 * @param ignition_state  Value of IGNITION_STATUS_ENUM.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ignition_status_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t connected, uint8_t ignition_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN];
    _mav_put_uint8_t(buf, 0, connected);
    _mav_put_uint8_t(buf, 1, ignition_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN);
#else
    mavlink_ignition_status_tm_t packet;
    packet.connected = connected;
    packet.ignition_state = ignition_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IGNITION_STATUS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IGNITION_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_CRC);
}

/**
 * @brief Pack a ignition_status_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param connected  If False the other value are not valid.
 * @param ignition_state  Value of IGNITION_STATUS_ENUM.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ignition_status_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t connected,uint8_t ignition_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN];
    _mav_put_uint8_t(buf, 0, connected);
    _mav_put_uint8_t(buf, 1, ignition_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN);
#else
    mavlink_ignition_status_tm_t packet;
    packet.connected = connected;
    packet.ignition_state = ignition_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IGNITION_STATUS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IGNITION_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_CRC);
}

/**
 * @brief Encode a ignition_status_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ignition_status_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ignition_status_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ignition_status_tm_t* ignition_status_tm)
{
    return mavlink_msg_ignition_status_tm_pack(system_id, component_id, msg, ignition_status_tm->connected, ignition_status_tm->ignition_state);
}

/**
 * @brief Encode a ignition_status_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ignition_status_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ignition_status_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ignition_status_tm_t* ignition_status_tm)
{
    return mavlink_msg_ignition_status_tm_pack_chan(system_id, component_id, chan, msg, ignition_status_tm->connected, ignition_status_tm->ignition_state);
}

/**
 * @brief Send a ignition_status_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param connected  If False the other value are not valid.
 * @param ignition_state  Value of IGNITION_STATUS_ENUM.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ignition_status_tm_send(mavlink_channel_t chan, uint8_t connected, uint8_t ignition_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN];
    _mav_put_uint8_t(buf, 0, connected);
    _mav_put_uint8_t(buf, 1, ignition_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IGNITION_STATUS_TM, buf, MAVLINK_MSG_ID_IGNITION_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_CRC);
#else
    mavlink_ignition_status_tm_t packet;
    packet.connected = connected;
    packet.ignition_state = ignition_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IGNITION_STATUS_TM, (const char *)&packet, MAVLINK_MSG_ID_IGNITION_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_CRC);
#endif
}

/**
 * @brief Send a ignition_status_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ignition_status_tm_send_struct(mavlink_channel_t chan, const mavlink_ignition_status_tm_t* ignition_status_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ignition_status_tm_send(chan, ignition_status_tm->connected, ignition_status_tm->ignition_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IGNITION_STATUS_TM, (const char *)ignition_status_tm, MAVLINK_MSG_ID_IGNITION_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ignition_status_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t connected, uint8_t ignition_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, connected);
    _mav_put_uint8_t(buf, 1, ignition_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IGNITION_STATUS_TM, buf, MAVLINK_MSG_ID_IGNITION_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_CRC);
#else
    mavlink_ignition_status_tm_t *packet = (mavlink_ignition_status_tm_t *)msgbuf;
    packet->connected = connected;
    packet->ignition_state = ignition_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IGNITION_STATUS_TM, (const char *)packet, MAVLINK_MSG_ID_IGNITION_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN, MAVLINK_MSG_ID_IGNITION_STATUS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE IGNITION_STATUS_TM UNPACKING


/**
 * @brief Get field connected from ignition_status_tm message
 *
 * @return  If False the other value are not valid.
 */
static inline uint8_t mavlink_msg_ignition_status_tm_get_connected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ignition_state from ignition_status_tm message
 *
 * @return  Value of IGNITION_STATUS_ENUM.
 */
static inline uint8_t mavlink_msg_ignition_status_tm_get_ignition_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a ignition_status_tm message into a struct
 *
 * @param msg The message to decode
 * @param ignition_status_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_ignition_status_tm_decode(const mavlink_message_t* msg, mavlink_ignition_status_tm_t* ignition_status_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ignition_status_tm->connected = mavlink_msg_ignition_status_tm_get_connected(msg);
    ignition_status_tm->ignition_state = mavlink_msg_ignition_status_tm_get_ignition_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN? msg->len : MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN;
        memset(ignition_status_tm, 0, MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN);
    memcpy(ignition_status_tm, _MAV_PAYLOAD(msg), len);
#endif
}
