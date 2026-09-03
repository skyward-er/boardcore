#pragma once
// MESSAGE VALVE_INFO_TM PACKING

#define MAVLINK_MSG_ID_VALVE_INFO_TM 37


typedef struct __mavlink_valve_info_tm_t {
 uint32_t time_to_close; /*< [ms] Time remaining until the valve closes (0 if the valve is closed)*/
 uint32_t timing; /*< [ms] Time the valve will stay open*/
 uint8_t servo_id; /*<  The ID of the valve*/
 uint8_t state; /*<  State of the valve (1 = open, 0 = closed)*/
 uint8_t aperture; /*< [%] Maximum valve aperture (open position) [0-100]*/
} mavlink_valve_info_tm_t;

#define MAVLINK_MSG_ID_VALVE_INFO_TM_LEN 11
#define MAVLINK_MSG_ID_VALVE_INFO_TM_MIN_LEN 11
#define MAVLINK_MSG_ID_37_LEN 11
#define MAVLINK_MSG_ID_37_MIN_LEN 11

#define MAVLINK_MSG_ID_VALVE_INFO_TM_CRC 146
#define MAVLINK_MSG_ID_37_CRC 146



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VALVE_INFO_TM { \
    37, \
    "VALVE_INFO_TM", \
    5, \
    {  { "servo_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_valve_info_tm_t, servo_id) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_valve_info_tm_t, state) }, \
         { "time_to_close", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_valve_info_tm_t, time_to_close) }, \
         { "timing", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_valve_info_tm_t, timing) }, \
         { "aperture", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_valve_info_tm_t, aperture) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VALVE_INFO_TM { \
    "VALVE_INFO_TM", \
    5, \
    {  { "servo_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_valve_info_tm_t, servo_id) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_valve_info_tm_t, state) }, \
         { "time_to_close", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_valve_info_tm_t, time_to_close) }, \
         { "timing", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_valve_info_tm_t, timing) }, \
         { "aperture", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_valve_info_tm_t, aperture) }, \
         } \
}
#endif

/**
 * @brief Pack a valve_info_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param servo_id  The ID of the valve
 * @param state  State of the valve (1 = open, 0 = closed)
 * @param time_to_close [ms] Time remaining until the valve closes (0 if the valve is closed)
 * @param timing [ms] Time the valve will stay open
 * @param aperture [%] Maximum valve aperture (open position) [0-100]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_valve_info_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t servo_id, uint8_t state, uint32_t time_to_close, uint32_t timing, uint8_t aperture)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VALVE_INFO_TM_LEN];
    _mav_put_uint32_t(buf, 0, time_to_close);
    _mav_put_uint32_t(buf, 4, timing);
    _mav_put_uint8_t(buf, 8, servo_id);
    _mav_put_uint8_t(buf, 9, state);
    _mav_put_uint8_t(buf, 10, aperture);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VALVE_INFO_TM_LEN);
#else
    mavlink_valve_info_tm_t packet;
    packet.time_to_close = time_to_close;
    packet.timing = timing;
    packet.servo_id = servo_id;
    packet.state = state;
    packet.aperture = aperture;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VALVE_INFO_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VALVE_INFO_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VALVE_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_CRC);
}

/**
 * @brief Pack a valve_info_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param servo_id  The ID of the valve
 * @param state  State of the valve (1 = open, 0 = closed)
 * @param time_to_close [ms] Time remaining until the valve closes (0 if the valve is closed)
 * @param timing [ms] Time the valve will stay open
 * @param aperture [%] Maximum valve aperture (open position) [0-100]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_valve_info_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t servo_id,uint8_t state,uint32_t time_to_close,uint32_t timing,uint8_t aperture)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VALVE_INFO_TM_LEN];
    _mav_put_uint32_t(buf, 0, time_to_close);
    _mav_put_uint32_t(buf, 4, timing);
    _mav_put_uint8_t(buf, 8, servo_id);
    _mav_put_uint8_t(buf, 9, state);
    _mav_put_uint8_t(buf, 10, aperture);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VALVE_INFO_TM_LEN);
#else
    mavlink_valve_info_tm_t packet;
    packet.time_to_close = time_to_close;
    packet.timing = timing;
    packet.servo_id = servo_id;
    packet.state = state;
    packet.aperture = aperture;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VALVE_INFO_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VALVE_INFO_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VALVE_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_CRC);
}

/**
 * @brief Encode a valve_info_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param valve_info_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_valve_info_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_valve_info_tm_t* valve_info_tm)
{
    return mavlink_msg_valve_info_tm_pack(system_id, component_id, msg, valve_info_tm->servo_id, valve_info_tm->state, valve_info_tm->time_to_close, valve_info_tm->timing, valve_info_tm->aperture);
}

/**
 * @brief Encode a valve_info_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param valve_info_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_valve_info_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_valve_info_tm_t* valve_info_tm)
{
    return mavlink_msg_valve_info_tm_pack_chan(system_id, component_id, chan, msg, valve_info_tm->servo_id, valve_info_tm->state, valve_info_tm->time_to_close, valve_info_tm->timing, valve_info_tm->aperture);
}

/**
 * @brief Send a valve_info_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param servo_id  The ID of the valve
 * @param state  State of the valve (1 = open, 0 = closed)
 * @param time_to_close [ms] Time remaining until the valve closes (0 if the valve is closed)
 * @param timing [ms] Time the valve will stay open
 * @param aperture [%] Maximum valve aperture (open position) [0-100]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_valve_info_tm_send(mavlink_channel_t chan, uint8_t servo_id, uint8_t state, uint32_t time_to_close, uint32_t timing, uint8_t aperture)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VALVE_INFO_TM_LEN];
    _mav_put_uint32_t(buf, 0, time_to_close);
    _mav_put_uint32_t(buf, 4, timing);
    _mav_put_uint8_t(buf, 8, servo_id);
    _mav_put_uint8_t(buf, 9, state);
    _mav_put_uint8_t(buf, 10, aperture);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VALVE_INFO_TM, buf, MAVLINK_MSG_ID_VALVE_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_CRC);
#else
    mavlink_valve_info_tm_t packet;
    packet.time_to_close = time_to_close;
    packet.timing = timing;
    packet.servo_id = servo_id;
    packet.state = state;
    packet.aperture = aperture;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VALVE_INFO_TM, (const char *)&packet, MAVLINK_MSG_ID_VALVE_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_CRC);
#endif
}

/**
 * @brief Send a valve_info_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_valve_info_tm_send_struct(mavlink_channel_t chan, const mavlink_valve_info_tm_t* valve_info_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_valve_info_tm_send(chan, valve_info_tm->servo_id, valve_info_tm->state, valve_info_tm->time_to_close, valve_info_tm->timing, valve_info_tm->aperture);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VALVE_INFO_TM, (const char *)valve_info_tm, MAVLINK_MSG_ID_VALVE_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_VALVE_INFO_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_valve_info_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t servo_id, uint8_t state, uint32_t time_to_close, uint32_t timing, uint8_t aperture)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_to_close);
    _mav_put_uint32_t(buf, 4, timing);
    _mav_put_uint8_t(buf, 8, servo_id);
    _mav_put_uint8_t(buf, 9, state);
    _mav_put_uint8_t(buf, 10, aperture);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VALVE_INFO_TM, buf, MAVLINK_MSG_ID_VALVE_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_CRC);
#else
    mavlink_valve_info_tm_t *packet = (mavlink_valve_info_tm_t *)msgbuf;
    packet->time_to_close = time_to_close;
    packet->timing = timing;
    packet->servo_id = servo_id;
    packet->state = state;
    packet->aperture = aperture;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VALVE_INFO_TM, (const char *)packet, MAVLINK_MSG_ID_VALVE_INFO_TM_MIN_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_LEN, MAVLINK_MSG_ID_VALVE_INFO_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE VALVE_INFO_TM UNPACKING


/**
 * @brief Get field servo_id from valve_info_tm message
 *
 * @return  The ID of the valve
 */
static inline uint8_t mavlink_msg_valve_info_tm_get_servo_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field state from valve_info_tm message
 *
 * @return  State of the valve (1 = open, 0 = closed)
 */
static inline uint8_t mavlink_msg_valve_info_tm_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field time_to_close from valve_info_tm message
 *
 * @return [ms] Time remaining until the valve closes (0 if the valve is closed)
 */
static inline uint32_t mavlink_msg_valve_info_tm_get_time_to_close(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field timing from valve_info_tm message
 *
 * @return [ms] Time the valve will stay open
 */
static inline uint32_t mavlink_msg_valve_info_tm_get_timing(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field aperture from valve_info_tm message
 *
 * @return [%] Maximum valve aperture (open position) [0-100]
 */
static inline uint8_t mavlink_msg_valve_info_tm_get_aperture(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Decode a valve_info_tm message into a struct
 *
 * @param msg The message to decode
 * @param valve_info_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_valve_info_tm_decode(const mavlink_message_t* msg, mavlink_valve_info_tm_t* valve_info_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    valve_info_tm->time_to_close = mavlink_msg_valve_info_tm_get_time_to_close(msg);
    valve_info_tm->timing = mavlink_msg_valve_info_tm_get_timing(msg);
    valve_info_tm->servo_id = mavlink_msg_valve_info_tm_get_servo_id(msg);
    valve_info_tm->state = mavlink_msg_valve_info_tm_get_state(msg);
    valve_info_tm->aperture = mavlink_msg_valve_info_tm_get_aperture(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VALVE_INFO_TM_LEN? msg->len : MAVLINK_MSG_ID_VALVE_INFO_TM_LEN;
        memset(valve_info_tm, 0, MAVLINK_MSG_ID_VALVE_INFO_TM_LEN);
    memcpy(valve_info_tm, _MAV_PAYLOAD(msg), len);
#endif
}
