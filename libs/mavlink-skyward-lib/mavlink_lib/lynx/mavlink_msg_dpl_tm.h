#pragma once
// MESSAGE DPL_TM PACKING

#define MAVLINK_MSG_ID_DPL_TM 166

MAVPACKED(
typedef struct __mavlink_dpl_tm_t {
 uint64_t timestamp; /*< [ms] When was this logged*/
 float servo_position; /*< [deg] DPL servo position (angle)*/
 uint8_t fsm_state; /*<  Current state of the dpl controller*/
 uint8_t cutters_enabled; /*<  Cutters state (enabled/disabled)*/
}) mavlink_dpl_tm_t;

#define MAVLINK_MSG_ID_DPL_TM_LEN 14
#define MAVLINK_MSG_ID_DPL_TM_MIN_LEN 14
#define MAVLINK_MSG_ID_166_LEN 14
#define MAVLINK_MSG_ID_166_MIN_LEN 14

#define MAVLINK_MSG_ID_DPL_TM_CRC 161
#define MAVLINK_MSG_ID_166_CRC 161



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DPL_TM { \
    166, \
    "DPL_TM", \
    4, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_dpl_tm_t, timestamp) }, \
         { "fsm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_dpl_tm_t, fsm_state) }, \
         { "cutters_enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_dpl_tm_t, cutters_enabled) }, \
         { "servo_position", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_dpl_tm_t, servo_position) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DPL_TM { \
    "DPL_TM", \
    4, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_dpl_tm_t, timestamp) }, \
         { "fsm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_dpl_tm_t, fsm_state) }, \
         { "cutters_enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_dpl_tm_t, cutters_enabled) }, \
         { "servo_position", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_dpl_tm_t, servo_position) }, \
         } \
}
#endif

/**
 * @brief Pack a dpl_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] When was this logged
 * @param fsm_state  Current state of the dpl controller
 * @param cutters_enabled  Cutters state (enabled/disabled)
 * @param servo_position [deg] DPL servo position (angle)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dpl_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t fsm_state, uint8_t cutters_enabled, float servo_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DPL_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, servo_position);
    _mav_put_uint8_t(buf, 12, fsm_state);
    _mav_put_uint8_t(buf, 13, cutters_enabled);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DPL_TM_LEN);
#else
    mavlink_dpl_tm_t packet;
    packet.timestamp = timestamp;
    packet.servo_position = servo_position;
    packet.fsm_state = fsm_state;
    packet.cutters_enabled = cutters_enabled;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DPL_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DPL_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DPL_TM_MIN_LEN, MAVLINK_MSG_ID_DPL_TM_LEN, MAVLINK_MSG_ID_DPL_TM_CRC);
}

/**
 * @brief Pack a dpl_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] When was this logged
 * @param fsm_state  Current state of the dpl controller
 * @param cutters_enabled  Cutters state (enabled/disabled)
 * @param servo_position [deg] DPL servo position (angle)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dpl_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t fsm_state,uint8_t cutters_enabled,float servo_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DPL_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, servo_position);
    _mav_put_uint8_t(buf, 12, fsm_state);
    _mav_put_uint8_t(buf, 13, cutters_enabled);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DPL_TM_LEN);
#else
    mavlink_dpl_tm_t packet;
    packet.timestamp = timestamp;
    packet.servo_position = servo_position;
    packet.fsm_state = fsm_state;
    packet.cutters_enabled = cutters_enabled;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DPL_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DPL_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DPL_TM_MIN_LEN, MAVLINK_MSG_ID_DPL_TM_LEN, MAVLINK_MSG_ID_DPL_TM_CRC);
}

/**
 * @brief Encode a dpl_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dpl_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dpl_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dpl_tm_t* dpl_tm)
{
    return mavlink_msg_dpl_tm_pack(system_id, component_id, msg, dpl_tm->timestamp, dpl_tm->fsm_state, dpl_tm->cutters_enabled, dpl_tm->servo_position);
}

/**
 * @brief Encode a dpl_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dpl_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dpl_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dpl_tm_t* dpl_tm)
{
    return mavlink_msg_dpl_tm_pack_chan(system_id, component_id, chan, msg, dpl_tm->timestamp, dpl_tm->fsm_state, dpl_tm->cutters_enabled, dpl_tm->servo_position);
}

/**
 * @brief Send a dpl_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] When was this logged
 * @param fsm_state  Current state of the dpl controller
 * @param cutters_enabled  Cutters state (enabled/disabled)
 * @param servo_position [deg] DPL servo position (angle)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dpl_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t fsm_state, uint8_t cutters_enabled, float servo_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DPL_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, servo_position);
    _mav_put_uint8_t(buf, 12, fsm_state);
    _mav_put_uint8_t(buf, 13, cutters_enabled);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DPL_TM, buf, MAVLINK_MSG_ID_DPL_TM_MIN_LEN, MAVLINK_MSG_ID_DPL_TM_LEN, MAVLINK_MSG_ID_DPL_TM_CRC);
#else
    mavlink_dpl_tm_t packet;
    packet.timestamp = timestamp;
    packet.servo_position = servo_position;
    packet.fsm_state = fsm_state;
    packet.cutters_enabled = cutters_enabled;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DPL_TM, (const char *)&packet, MAVLINK_MSG_ID_DPL_TM_MIN_LEN, MAVLINK_MSG_ID_DPL_TM_LEN, MAVLINK_MSG_ID_DPL_TM_CRC);
#endif
}

/**
 * @brief Send a dpl_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_dpl_tm_send_struct(mavlink_channel_t chan, const mavlink_dpl_tm_t* dpl_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_dpl_tm_send(chan, dpl_tm->timestamp, dpl_tm->fsm_state, dpl_tm->cutters_enabled, dpl_tm->servo_position);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DPL_TM, (const char *)dpl_tm, MAVLINK_MSG_ID_DPL_TM_MIN_LEN, MAVLINK_MSG_ID_DPL_TM_LEN, MAVLINK_MSG_ID_DPL_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_DPL_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_dpl_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t fsm_state, uint8_t cutters_enabled, float servo_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, servo_position);
    _mav_put_uint8_t(buf, 12, fsm_state);
    _mav_put_uint8_t(buf, 13, cutters_enabled);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DPL_TM, buf, MAVLINK_MSG_ID_DPL_TM_MIN_LEN, MAVLINK_MSG_ID_DPL_TM_LEN, MAVLINK_MSG_ID_DPL_TM_CRC);
#else
    mavlink_dpl_tm_t *packet = (mavlink_dpl_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->servo_position = servo_position;
    packet->fsm_state = fsm_state;
    packet->cutters_enabled = cutters_enabled;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DPL_TM, (const char *)packet, MAVLINK_MSG_ID_DPL_TM_MIN_LEN, MAVLINK_MSG_ID_DPL_TM_LEN, MAVLINK_MSG_ID_DPL_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE DPL_TM UNPACKING


/**
 * @brief Get field timestamp from dpl_tm message
 *
 * @return [ms] When was this logged
 */
static inline uint64_t mavlink_msg_dpl_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field fsm_state from dpl_tm message
 *
 * @return  Current state of the dpl controller
 */
static inline uint8_t mavlink_msg_dpl_tm_get_fsm_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field cutters_enabled from dpl_tm message
 *
 * @return  Cutters state (enabled/disabled)
 */
static inline uint8_t mavlink_msg_dpl_tm_get_cutters_enabled(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field servo_position from dpl_tm message
 *
 * @return [deg] DPL servo position (angle)
 */
static inline float mavlink_msg_dpl_tm_get_servo_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a dpl_tm message into a struct
 *
 * @param msg The message to decode
 * @param dpl_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_dpl_tm_decode(const mavlink_message_t* msg, mavlink_dpl_tm_t* dpl_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    dpl_tm->timestamp = mavlink_msg_dpl_tm_get_timestamp(msg);
    dpl_tm->servo_position = mavlink_msg_dpl_tm_get_servo_position(msg);
    dpl_tm->fsm_state = mavlink_msg_dpl_tm_get_fsm_state(msg);
    dpl_tm->cutters_enabled = mavlink_msg_dpl_tm_get_cutters_enabled(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DPL_TM_LEN? msg->len : MAVLINK_MSG_ID_DPL_TM_LEN;
        memset(dpl_tm, 0, MAVLINK_MSG_ID_DPL_TM_LEN);
    memcpy(dpl_tm, _MAV_PAYLOAD(msg), len);
#endif
}
