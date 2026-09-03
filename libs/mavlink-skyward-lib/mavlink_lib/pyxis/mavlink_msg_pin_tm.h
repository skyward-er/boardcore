#pragma once
// MESSAGE PIN_TM PACKING

#define MAVLINK_MSG_ID_PIN_TM 113


typedef struct __mavlink_pin_tm_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 uint64_t last_change_timestamp; /*<  Last change timestamp of pin*/
 uint8_t pin_id; /*<  A member of the PinsList enum*/
 uint8_t changes_counter; /*<  Number of changes of pin*/
 uint8_t current_state; /*<  Current state of pin*/
} mavlink_pin_tm_t;

#define MAVLINK_MSG_ID_PIN_TM_LEN 19
#define MAVLINK_MSG_ID_PIN_TM_MIN_LEN 19
#define MAVLINK_MSG_ID_113_LEN 19
#define MAVLINK_MSG_ID_113_MIN_LEN 19

#define MAVLINK_MSG_ID_PIN_TM_CRC 255
#define MAVLINK_MSG_ID_113_CRC 255



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PIN_TM { \
    113, \
    "PIN_TM", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_pin_tm_t, timestamp) }, \
         { "pin_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_pin_tm_t, pin_id) }, \
         { "last_change_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_pin_tm_t, last_change_timestamp) }, \
         { "changes_counter", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_pin_tm_t, changes_counter) }, \
         { "current_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_pin_tm_t, current_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PIN_TM { \
    "PIN_TM", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_pin_tm_t, timestamp) }, \
         { "pin_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_pin_tm_t, pin_id) }, \
         { "last_change_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_pin_tm_t, last_change_timestamp) }, \
         { "changes_counter", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_pin_tm_t, changes_counter) }, \
         { "current_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_pin_tm_t, current_state) }, \
         } \
}
#endif

/**
 * @brief Pack a pin_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param pin_id  A member of the PinsList enum
 * @param last_change_timestamp  Last change timestamp of pin
 * @param changes_counter  Number of changes of pin
 * @param current_state  Current state of pin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pin_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t pin_id, uint64_t last_change_timestamp, uint8_t changes_counter, uint8_t current_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIN_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_change_timestamp);
    _mav_put_uint8_t(buf, 16, pin_id);
    _mav_put_uint8_t(buf, 17, changes_counter);
    _mav_put_uint8_t(buf, 18, current_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIN_TM_LEN);
#else
    mavlink_pin_tm_t packet;
    packet.timestamp = timestamp;
    packet.last_change_timestamp = last_change_timestamp;
    packet.pin_id = pin_id;
    packet.changes_counter = changes_counter;
    packet.current_state = current_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIN_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PIN_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PIN_TM_MIN_LEN, MAVLINK_MSG_ID_PIN_TM_LEN, MAVLINK_MSG_ID_PIN_TM_CRC);
}

/**
 * @brief Pack a pin_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param pin_id  A member of the PinsList enum
 * @param last_change_timestamp  Last change timestamp of pin
 * @param changes_counter  Number of changes of pin
 * @param current_state  Current state of pin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pin_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t pin_id,uint64_t last_change_timestamp,uint8_t changes_counter,uint8_t current_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIN_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_change_timestamp);
    _mav_put_uint8_t(buf, 16, pin_id);
    _mav_put_uint8_t(buf, 17, changes_counter);
    _mav_put_uint8_t(buf, 18, current_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIN_TM_LEN);
#else
    mavlink_pin_tm_t packet;
    packet.timestamp = timestamp;
    packet.last_change_timestamp = last_change_timestamp;
    packet.pin_id = pin_id;
    packet.changes_counter = changes_counter;
    packet.current_state = current_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIN_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PIN_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PIN_TM_MIN_LEN, MAVLINK_MSG_ID_PIN_TM_LEN, MAVLINK_MSG_ID_PIN_TM_CRC);
}

/**
 * @brief Encode a pin_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pin_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pin_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pin_tm_t* pin_tm)
{
    return mavlink_msg_pin_tm_pack(system_id, component_id, msg, pin_tm->timestamp, pin_tm->pin_id, pin_tm->last_change_timestamp, pin_tm->changes_counter, pin_tm->current_state);
}

/**
 * @brief Encode a pin_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pin_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pin_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pin_tm_t* pin_tm)
{
    return mavlink_msg_pin_tm_pack_chan(system_id, component_id, chan, msg, pin_tm->timestamp, pin_tm->pin_id, pin_tm->last_change_timestamp, pin_tm->changes_counter, pin_tm->current_state);
}

/**
 * @brief Send a pin_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param pin_id  A member of the PinsList enum
 * @param last_change_timestamp  Last change timestamp of pin
 * @param changes_counter  Number of changes of pin
 * @param current_state  Current state of pin
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pin_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t pin_id, uint64_t last_change_timestamp, uint8_t changes_counter, uint8_t current_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIN_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_change_timestamp);
    _mav_put_uint8_t(buf, 16, pin_id);
    _mav_put_uint8_t(buf, 17, changes_counter);
    _mav_put_uint8_t(buf, 18, current_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIN_TM, buf, MAVLINK_MSG_ID_PIN_TM_MIN_LEN, MAVLINK_MSG_ID_PIN_TM_LEN, MAVLINK_MSG_ID_PIN_TM_CRC);
#else
    mavlink_pin_tm_t packet;
    packet.timestamp = timestamp;
    packet.last_change_timestamp = last_change_timestamp;
    packet.pin_id = pin_id;
    packet.changes_counter = changes_counter;
    packet.current_state = current_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIN_TM, (const char *)&packet, MAVLINK_MSG_ID_PIN_TM_MIN_LEN, MAVLINK_MSG_ID_PIN_TM_LEN, MAVLINK_MSG_ID_PIN_TM_CRC);
#endif
}

/**
 * @brief Send a pin_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_pin_tm_send_struct(mavlink_channel_t chan, const mavlink_pin_tm_t* pin_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pin_tm_send(chan, pin_tm->timestamp, pin_tm->pin_id, pin_tm->last_change_timestamp, pin_tm->changes_counter, pin_tm->current_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIN_TM, (const char *)pin_tm, MAVLINK_MSG_ID_PIN_TM_MIN_LEN, MAVLINK_MSG_ID_PIN_TM_LEN, MAVLINK_MSG_ID_PIN_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_PIN_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pin_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t pin_id, uint64_t last_change_timestamp, uint8_t changes_counter, uint8_t current_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_change_timestamp);
    _mav_put_uint8_t(buf, 16, pin_id);
    _mav_put_uint8_t(buf, 17, changes_counter);
    _mav_put_uint8_t(buf, 18, current_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIN_TM, buf, MAVLINK_MSG_ID_PIN_TM_MIN_LEN, MAVLINK_MSG_ID_PIN_TM_LEN, MAVLINK_MSG_ID_PIN_TM_CRC);
#else
    mavlink_pin_tm_t *packet = (mavlink_pin_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->last_change_timestamp = last_change_timestamp;
    packet->pin_id = pin_id;
    packet->changes_counter = changes_counter;
    packet->current_state = current_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIN_TM, (const char *)packet, MAVLINK_MSG_ID_PIN_TM_MIN_LEN, MAVLINK_MSG_ID_PIN_TM_LEN, MAVLINK_MSG_ID_PIN_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE PIN_TM UNPACKING


/**
 * @brief Get field timestamp from pin_tm message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_pin_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field pin_id from pin_tm message
 *
 * @return  A member of the PinsList enum
 */
static inline uint8_t mavlink_msg_pin_tm_get_pin_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field last_change_timestamp from pin_tm message
 *
 * @return  Last change timestamp of pin
 */
static inline uint64_t mavlink_msg_pin_tm_get_last_change_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field changes_counter from pin_tm message
 *
 * @return  Number of changes of pin
 */
static inline uint8_t mavlink_msg_pin_tm_get_changes_counter(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field current_state from pin_tm message
 *
 * @return  Current state of pin
 */
static inline uint8_t mavlink_msg_pin_tm_get_current_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Decode a pin_tm message into a struct
 *
 * @param msg The message to decode
 * @param pin_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_pin_tm_decode(const mavlink_message_t* msg, mavlink_pin_tm_t* pin_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    pin_tm->timestamp = mavlink_msg_pin_tm_get_timestamp(msg);
    pin_tm->last_change_timestamp = mavlink_msg_pin_tm_get_last_change_timestamp(msg);
    pin_tm->pin_id = mavlink_msg_pin_tm_get_pin_id(msg);
    pin_tm->changes_counter = mavlink_msg_pin_tm_get_changes_counter(msg);
    pin_tm->current_state = mavlink_msg_pin_tm_get_current_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PIN_TM_LEN? msg->len : MAVLINK_MSG_ID_PIN_TM_LEN;
        memset(pin_tm, 0, MAVLINK_MSG_ID_PIN_TM_LEN);
    memcpy(pin_tm, _MAV_PAYLOAD(msg), len);
#endif
}
