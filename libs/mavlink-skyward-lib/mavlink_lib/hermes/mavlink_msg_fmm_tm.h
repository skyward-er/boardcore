#pragma once
// MESSAGE FMM_TM PACKING

#define MAVLINK_MSG_ID_FMM_TM 161

MAVPACKED(
typedef struct __mavlink_fmm_tm_t {
 uint64_t timestamp; /*<  Timestamp*/
 uint64_t pin_launch_last_change; /*<  Last change of launch pin*/
 uint64_t pin_nosecone_last_change; /*<  Last change of nosecone pin*/
 uint8_t state; /*<  FMM State*/
 uint8_t pin_launch_num_changes; /*<  Number of changes of the launch pin*/
 uint8_t pin_launch_state; /*<  Current state of the launch pin*/
 uint8_t pin_nosecone_num_changes; /*<  Number of changes of the nosecone pin*/
 uint8_t pin_nosecone_state; /*<  Current state of the nosecone pin*/
}) mavlink_fmm_tm_t;

#define MAVLINK_MSG_ID_FMM_TM_LEN 29
#define MAVLINK_MSG_ID_FMM_TM_MIN_LEN 29
#define MAVLINK_MSG_ID_161_LEN 29
#define MAVLINK_MSG_ID_161_MIN_LEN 29

#define MAVLINK_MSG_ID_FMM_TM_CRC 87
#define MAVLINK_MSG_ID_161_CRC 87



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FMM_TM { \
    161, \
    "FMM_TM", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_fmm_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_fmm_tm_t, state) }, \
         { "pin_launch_last_change", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_fmm_tm_t, pin_launch_last_change) }, \
         { "pin_launch_num_changes", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_fmm_tm_t, pin_launch_num_changes) }, \
         { "pin_launch_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_fmm_tm_t, pin_launch_state) }, \
         { "pin_nosecone_last_change", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_fmm_tm_t, pin_nosecone_last_change) }, \
         { "pin_nosecone_num_changes", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_fmm_tm_t, pin_nosecone_num_changes) }, \
         { "pin_nosecone_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_fmm_tm_t, pin_nosecone_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FMM_TM { \
    "FMM_TM", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_fmm_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_fmm_tm_t, state) }, \
         { "pin_launch_last_change", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_fmm_tm_t, pin_launch_last_change) }, \
         { "pin_launch_num_changes", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_fmm_tm_t, pin_launch_num_changes) }, \
         { "pin_launch_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_fmm_tm_t, pin_launch_state) }, \
         { "pin_nosecone_last_change", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_fmm_tm_t, pin_nosecone_last_change) }, \
         { "pin_nosecone_num_changes", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_fmm_tm_t, pin_nosecone_num_changes) }, \
         { "pin_nosecone_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_fmm_tm_t, pin_nosecone_state) }, \
         } \
}
#endif

/**
 * @brief Pack a fmm_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Timestamp
 * @param state  FMM State
 * @param pin_launch_last_change  Last change of launch pin
 * @param pin_launch_num_changes  Number of changes of the launch pin
 * @param pin_launch_state  Current state of the launch pin
 * @param pin_nosecone_last_change  Last change of nosecone pin
 * @param pin_nosecone_num_changes  Number of changes of the nosecone pin
 * @param pin_nosecone_state  Current state of the nosecone pin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fmm_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t state, uint64_t pin_launch_last_change, uint8_t pin_launch_num_changes, uint8_t pin_launch_state, uint64_t pin_nosecone_last_change, uint8_t pin_nosecone_num_changes, uint8_t pin_nosecone_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FMM_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, pin_launch_last_change);
    _mav_put_uint64_t(buf, 16, pin_nosecone_last_change);
    _mav_put_uint8_t(buf, 24, state);
    _mav_put_uint8_t(buf, 25, pin_launch_num_changes);
    _mav_put_uint8_t(buf, 26, pin_launch_state);
    _mav_put_uint8_t(buf, 27, pin_nosecone_num_changes);
    _mav_put_uint8_t(buf, 28, pin_nosecone_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FMM_TM_LEN);
#else
    mavlink_fmm_tm_t packet;
    packet.timestamp = timestamp;
    packet.pin_launch_last_change = pin_launch_last_change;
    packet.pin_nosecone_last_change = pin_nosecone_last_change;
    packet.state = state;
    packet.pin_launch_num_changes = pin_launch_num_changes;
    packet.pin_launch_state = pin_launch_state;
    packet.pin_nosecone_num_changes = pin_nosecone_num_changes;
    packet.pin_nosecone_state = pin_nosecone_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FMM_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FMM_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FMM_TM_MIN_LEN, MAVLINK_MSG_ID_FMM_TM_LEN, MAVLINK_MSG_ID_FMM_TM_CRC);
}

/**
 * @brief Pack a fmm_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Timestamp
 * @param state  FMM State
 * @param pin_launch_last_change  Last change of launch pin
 * @param pin_launch_num_changes  Number of changes of the launch pin
 * @param pin_launch_state  Current state of the launch pin
 * @param pin_nosecone_last_change  Last change of nosecone pin
 * @param pin_nosecone_num_changes  Number of changes of the nosecone pin
 * @param pin_nosecone_state  Current state of the nosecone pin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fmm_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t state,uint64_t pin_launch_last_change,uint8_t pin_launch_num_changes,uint8_t pin_launch_state,uint64_t pin_nosecone_last_change,uint8_t pin_nosecone_num_changes,uint8_t pin_nosecone_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FMM_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, pin_launch_last_change);
    _mav_put_uint64_t(buf, 16, pin_nosecone_last_change);
    _mav_put_uint8_t(buf, 24, state);
    _mav_put_uint8_t(buf, 25, pin_launch_num_changes);
    _mav_put_uint8_t(buf, 26, pin_launch_state);
    _mav_put_uint8_t(buf, 27, pin_nosecone_num_changes);
    _mav_put_uint8_t(buf, 28, pin_nosecone_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FMM_TM_LEN);
#else
    mavlink_fmm_tm_t packet;
    packet.timestamp = timestamp;
    packet.pin_launch_last_change = pin_launch_last_change;
    packet.pin_nosecone_last_change = pin_nosecone_last_change;
    packet.state = state;
    packet.pin_launch_num_changes = pin_launch_num_changes;
    packet.pin_launch_state = pin_launch_state;
    packet.pin_nosecone_num_changes = pin_nosecone_num_changes;
    packet.pin_nosecone_state = pin_nosecone_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FMM_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FMM_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FMM_TM_MIN_LEN, MAVLINK_MSG_ID_FMM_TM_LEN, MAVLINK_MSG_ID_FMM_TM_CRC);
}

/**
 * @brief Encode a fmm_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fmm_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fmm_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fmm_tm_t* fmm_tm)
{
    return mavlink_msg_fmm_tm_pack(system_id, component_id, msg, fmm_tm->timestamp, fmm_tm->state, fmm_tm->pin_launch_last_change, fmm_tm->pin_launch_num_changes, fmm_tm->pin_launch_state, fmm_tm->pin_nosecone_last_change, fmm_tm->pin_nosecone_num_changes, fmm_tm->pin_nosecone_state);
}

/**
 * @brief Encode a fmm_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fmm_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fmm_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fmm_tm_t* fmm_tm)
{
    return mavlink_msg_fmm_tm_pack_chan(system_id, component_id, chan, msg, fmm_tm->timestamp, fmm_tm->state, fmm_tm->pin_launch_last_change, fmm_tm->pin_launch_num_changes, fmm_tm->pin_launch_state, fmm_tm->pin_nosecone_last_change, fmm_tm->pin_nosecone_num_changes, fmm_tm->pin_nosecone_state);
}

/**
 * @brief Send a fmm_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Timestamp
 * @param state  FMM State
 * @param pin_launch_last_change  Last change of launch pin
 * @param pin_launch_num_changes  Number of changes of the launch pin
 * @param pin_launch_state  Current state of the launch pin
 * @param pin_nosecone_last_change  Last change of nosecone pin
 * @param pin_nosecone_num_changes  Number of changes of the nosecone pin
 * @param pin_nosecone_state  Current state of the nosecone pin
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fmm_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t state, uint64_t pin_launch_last_change, uint8_t pin_launch_num_changes, uint8_t pin_launch_state, uint64_t pin_nosecone_last_change, uint8_t pin_nosecone_num_changes, uint8_t pin_nosecone_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FMM_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, pin_launch_last_change);
    _mav_put_uint64_t(buf, 16, pin_nosecone_last_change);
    _mav_put_uint8_t(buf, 24, state);
    _mav_put_uint8_t(buf, 25, pin_launch_num_changes);
    _mav_put_uint8_t(buf, 26, pin_launch_state);
    _mav_put_uint8_t(buf, 27, pin_nosecone_num_changes);
    _mav_put_uint8_t(buf, 28, pin_nosecone_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FMM_TM, buf, MAVLINK_MSG_ID_FMM_TM_MIN_LEN, MAVLINK_MSG_ID_FMM_TM_LEN, MAVLINK_MSG_ID_FMM_TM_CRC);
#else
    mavlink_fmm_tm_t packet;
    packet.timestamp = timestamp;
    packet.pin_launch_last_change = pin_launch_last_change;
    packet.pin_nosecone_last_change = pin_nosecone_last_change;
    packet.state = state;
    packet.pin_launch_num_changes = pin_launch_num_changes;
    packet.pin_launch_state = pin_launch_state;
    packet.pin_nosecone_num_changes = pin_nosecone_num_changes;
    packet.pin_nosecone_state = pin_nosecone_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FMM_TM, (const char *)&packet, MAVLINK_MSG_ID_FMM_TM_MIN_LEN, MAVLINK_MSG_ID_FMM_TM_LEN, MAVLINK_MSG_ID_FMM_TM_CRC);
#endif
}

/**
 * @brief Send a fmm_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_fmm_tm_send_struct(mavlink_channel_t chan, const mavlink_fmm_tm_t* fmm_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_fmm_tm_send(chan, fmm_tm->timestamp, fmm_tm->state, fmm_tm->pin_launch_last_change, fmm_tm->pin_launch_num_changes, fmm_tm->pin_launch_state, fmm_tm->pin_nosecone_last_change, fmm_tm->pin_nosecone_num_changes, fmm_tm->pin_nosecone_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FMM_TM, (const char *)fmm_tm, MAVLINK_MSG_ID_FMM_TM_MIN_LEN, MAVLINK_MSG_ID_FMM_TM_LEN, MAVLINK_MSG_ID_FMM_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_FMM_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fmm_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t state, uint64_t pin_launch_last_change, uint8_t pin_launch_num_changes, uint8_t pin_launch_state, uint64_t pin_nosecone_last_change, uint8_t pin_nosecone_num_changes, uint8_t pin_nosecone_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, pin_launch_last_change);
    _mav_put_uint64_t(buf, 16, pin_nosecone_last_change);
    _mav_put_uint8_t(buf, 24, state);
    _mav_put_uint8_t(buf, 25, pin_launch_num_changes);
    _mav_put_uint8_t(buf, 26, pin_launch_state);
    _mav_put_uint8_t(buf, 27, pin_nosecone_num_changes);
    _mav_put_uint8_t(buf, 28, pin_nosecone_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FMM_TM, buf, MAVLINK_MSG_ID_FMM_TM_MIN_LEN, MAVLINK_MSG_ID_FMM_TM_LEN, MAVLINK_MSG_ID_FMM_TM_CRC);
#else
    mavlink_fmm_tm_t *packet = (mavlink_fmm_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->pin_launch_last_change = pin_launch_last_change;
    packet->pin_nosecone_last_change = pin_nosecone_last_change;
    packet->state = state;
    packet->pin_launch_num_changes = pin_launch_num_changes;
    packet->pin_launch_state = pin_launch_state;
    packet->pin_nosecone_num_changes = pin_nosecone_num_changes;
    packet->pin_nosecone_state = pin_nosecone_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FMM_TM, (const char *)packet, MAVLINK_MSG_ID_FMM_TM_MIN_LEN, MAVLINK_MSG_ID_FMM_TM_LEN, MAVLINK_MSG_ID_FMM_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE FMM_TM UNPACKING


/**
 * @brief Get field timestamp from fmm_tm message
 *
 * @return  Timestamp
 */
static inline uint64_t mavlink_msg_fmm_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field state from fmm_tm message
 *
 * @return  FMM State
 */
static inline uint8_t mavlink_msg_fmm_tm_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field pin_launch_last_change from fmm_tm message
 *
 * @return  Last change of launch pin
 */
static inline uint64_t mavlink_msg_fmm_tm_get_pin_launch_last_change(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field pin_launch_num_changes from fmm_tm message
 *
 * @return  Number of changes of the launch pin
 */
static inline uint8_t mavlink_msg_fmm_tm_get_pin_launch_num_changes(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field pin_launch_state from fmm_tm message
 *
 * @return  Current state of the launch pin
 */
static inline uint8_t mavlink_msg_fmm_tm_get_pin_launch_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field pin_nosecone_last_change from fmm_tm message
 *
 * @return  Last change of nosecone pin
 */
static inline uint64_t mavlink_msg_fmm_tm_get_pin_nosecone_last_change(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Get field pin_nosecone_num_changes from fmm_tm message
 *
 * @return  Number of changes of the nosecone pin
 */
static inline uint8_t mavlink_msg_fmm_tm_get_pin_nosecone_num_changes(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field pin_nosecone_state from fmm_tm message
 *
 * @return  Current state of the nosecone pin
 */
static inline uint8_t mavlink_msg_fmm_tm_get_pin_nosecone_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Decode a fmm_tm message into a struct
 *
 * @param msg The message to decode
 * @param fmm_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_fmm_tm_decode(const mavlink_message_t* msg, mavlink_fmm_tm_t* fmm_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    fmm_tm->timestamp = mavlink_msg_fmm_tm_get_timestamp(msg);
    fmm_tm->pin_launch_last_change = mavlink_msg_fmm_tm_get_pin_launch_last_change(msg);
    fmm_tm->pin_nosecone_last_change = mavlink_msg_fmm_tm_get_pin_nosecone_last_change(msg);
    fmm_tm->state = mavlink_msg_fmm_tm_get_state(msg);
    fmm_tm->pin_launch_num_changes = mavlink_msg_fmm_tm_get_pin_launch_num_changes(msg);
    fmm_tm->pin_launch_state = mavlink_msg_fmm_tm_get_pin_launch_state(msg);
    fmm_tm->pin_nosecone_num_changes = mavlink_msg_fmm_tm_get_pin_nosecone_num_changes(msg);
    fmm_tm->pin_nosecone_state = mavlink_msg_fmm_tm_get_pin_nosecone_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FMM_TM_LEN? msg->len : MAVLINK_MSG_ID_FMM_TM_LEN;
        memset(fmm_tm, 0, MAVLINK_MSG_ID_FMM_TM_LEN);
    memcpy(fmm_tm, _MAV_PAYLOAD(msg), len);
#endif
}
