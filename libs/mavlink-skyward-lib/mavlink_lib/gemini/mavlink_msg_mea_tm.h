#pragma once
// MESSAGE MEA_TM PACKING

#define MAVLINK_MSG_ID_MEA_TM 207


typedef struct __mavlink_mea_tm_t {
 uint64_t timestamp; /*< [us] When was this logged*/
 float kalman_x0; /*<  Kalman state variable 0 (corrected pressure)*/
 float kalman_x1; /*<  Kalman state variable 1 */
 float kalman_x2; /*<  Kalman state variable 2 (mass)*/
 float mass; /*< [kg] Mass estimated*/
 float corrected_pressure; /*< [kg] Corrected pressure*/
 uint8_t state; /*<  MEA current state*/
} mavlink_mea_tm_t;

#define MAVLINK_MSG_ID_MEA_TM_LEN 29
#define MAVLINK_MSG_ID_MEA_TM_MIN_LEN 29
#define MAVLINK_MSG_ID_207_LEN 29
#define MAVLINK_MSG_ID_207_MIN_LEN 29

#define MAVLINK_MSG_ID_MEA_TM_CRC 11
#define MAVLINK_MSG_ID_207_CRC 11



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MEA_TM { \
    207, \
    "MEA_TM", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mea_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_mea_tm_t, state) }, \
         { "kalman_x0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mea_tm_t, kalman_x0) }, \
         { "kalman_x1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mea_tm_t, kalman_x1) }, \
         { "kalman_x2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mea_tm_t, kalman_x2) }, \
         { "mass", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mea_tm_t, mass) }, \
         { "corrected_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mea_tm_t, corrected_pressure) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MEA_TM { \
    "MEA_TM", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mea_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_mea_tm_t, state) }, \
         { "kalman_x0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mea_tm_t, kalman_x0) }, \
         { "kalman_x1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mea_tm_t, kalman_x1) }, \
         { "kalman_x2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mea_tm_t, kalman_x2) }, \
         { "mass", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mea_tm_t, mass) }, \
         { "corrected_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mea_tm_t, corrected_pressure) }, \
         } \
}
#endif

/**
 * @brief Pack a mea_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] When was this logged
 * @param state  MEA current state
 * @param kalman_x0  Kalman state variable 0 (corrected pressure)
 * @param kalman_x1  Kalman state variable 1 
 * @param kalman_x2  Kalman state variable 2 (mass)
 * @param mass [kg] Mass estimated
 * @param corrected_pressure [kg] Corrected pressure
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mea_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t state, float kalman_x0, float kalman_x1, float kalman_x2, float mass, float corrected_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MEA_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, kalman_x0);
    _mav_put_float(buf, 12, kalman_x1);
    _mav_put_float(buf, 16, kalman_x2);
    _mav_put_float(buf, 20, mass);
    _mav_put_float(buf, 24, corrected_pressure);
    _mav_put_uint8_t(buf, 28, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MEA_TM_LEN);
#else
    mavlink_mea_tm_t packet;
    packet.timestamp = timestamp;
    packet.kalman_x0 = kalman_x0;
    packet.kalman_x1 = kalman_x1;
    packet.kalman_x2 = kalman_x2;
    packet.mass = mass;
    packet.corrected_pressure = corrected_pressure;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MEA_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MEA_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MEA_TM_MIN_LEN, MAVLINK_MSG_ID_MEA_TM_LEN, MAVLINK_MSG_ID_MEA_TM_CRC);
}

/**
 * @brief Pack a mea_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] When was this logged
 * @param state  MEA current state
 * @param kalman_x0  Kalman state variable 0 (corrected pressure)
 * @param kalman_x1  Kalman state variable 1 
 * @param kalman_x2  Kalman state variable 2 (mass)
 * @param mass [kg] Mass estimated
 * @param corrected_pressure [kg] Corrected pressure
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mea_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t state,float kalman_x0,float kalman_x1,float kalman_x2,float mass,float corrected_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MEA_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, kalman_x0);
    _mav_put_float(buf, 12, kalman_x1);
    _mav_put_float(buf, 16, kalman_x2);
    _mav_put_float(buf, 20, mass);
    _mav_put_float(buf, 24, corrected_pressure);
    _mav_put_uint8_t(buf, 28, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MEA_TM_LEN);
#else
    mavlink_mea_tm_t packet;
    packet.timestamp = timestamp;
    packet.kalman_x0 = kalman_x0;
    packet.kalman_x1 = kalman_x1;
    packet.kalman_x2 = kalman_x2;
    packet.mass = mass;
    packet.corrected_pressure = corrected_pressure;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MEA_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MEA_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MEA_TM_MIN_LEN, MAVLINK_MSG_ID_MEA_TM_LEN, MAVLINK_MSG_ID_MEA_TM_CRC);
}

/**
 * @brief Encode a mea_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mea_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mea_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mea_tm_t* mea_tm)
{
    return mavlink_msg_mea_tm_pack(system_id, component_id, msg, mea_tm->timestamp, mea_tm->state, mea_tm->kalman_x0, mea_tm->kalman_x1, mea_tm->kalman_x2, mea_tm->mass, mea_tm->corrected_pressure);
}

/**
 * @brief Encode a mea_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mea_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mea_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mea_tm_t* mea_tm)
{
    return mavlink_msg_mea_tm_pack_chan(system_id, component_id, chan, msg, mea_tm->timestamp, mea_tm->state, mea_tm->kalman_x0, mea_tm->kalman_x1, mea_tm->kalman_x2, mea_tm->mass, mea_tm->corrected_pressure);
}

/**
 * @brief Send a mea_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged
 * @param state  MEA current state
 * @param kalman_x0  Kalman state variable 0 (corrected pressure)
 * @param kalman_x1  Kalman state variable 1 
 * @param kalman_x2  Kalman state variable 2 (mass)
 * @param mass [kg] Mass estimated
 * @param corrected_pressure [kg] Corrected pressure
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mea_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t state, float kalman_x0, float kalman_x1, float kalman_x2, float mass, float corrected_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MEA_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, kalman_x0);
    _mav_put_float(buf, 12, kalman_x1);
    _mav_put_float(buf, 16, kalman_x2);
    _mav_put_float(buf, 20, mass);
    _mav_put_float(buf, 24, corrected_pressure);
    _mav_put_uint8_t(buf, 28, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEA_TM, buf, MAVLINK_MSG_ID_MEA_TM_MIN_LEN, MAVLINK_MSG_ID_MEA_TM_LEN, MAVLINK_MSG_ID_MEA_TM_CRC);
#else
    mavlink_mea_tm_t packet;
    packet.timestamp = timestamp;
    packet.kalman_x0 = kalman_x0;
    packet.kalman_x1 = kalman_x1;
    packet.kalman_x2 = kalman_x2;
    packet.mass = mass;
    packet.corrected_pressure = corrected_pressure;
    packet.state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEA_TM, (const char *)&packet, MAVLINK_MSG_ID_MEA_TM_MIN_LEN, MAVLINK_MSG_ID_MEA_TM_LEN, MAVLINK_MSG_ID_MEA_TM_CRC);
#endif
}

/**
 * @brief Send a mea_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mea_tm_send_struct(mavlink_channel_t chan, const mavlink_mea_tm_t* mea_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mea_tm_send(chan, mea_tm->timestamp, mea_tm->state, mea_tm->kalman_x0, mea_tm->kalman_x1, mea_tm->kalman_x2, mea_tm->mass, mea_tm->corrected_pressure);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEA_TM, (const char *)mea_tm, MAVLINK_MSG_ID_MEA_TM_MIN_LEN, MAVLINK_MSG_ID_MEA_TM_LEN, MAVLINK_MSG_ID_MEA_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_MEA_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mea_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t state, float kalman_x0, float kalman_x1, float kalman_x2, float mass, float corrected_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, kalman_x0);
    _mav_put_float(buf, 12, kalman_x1);
    _mav_put_float(buf, 16, kalman_x2);
    _mav_put_float(buf, 20, mass);
    _mav_put_float(buf, 24, corrected_pressure);
    _mav_put_uint8_t(buf, 28, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEA_TM, buf, MAVLINK_MSG_ID_MEA_TM_MIN_LEN, MAVLINK_MSG_ID_MEA_TM_LEN, MAVLINK_MSG_ID_MEA_TM_CRC);
#else
    mavlink_mea_tm_t *packet = (mavlink_mea_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->kalman_x0 = kalman_x0;
    packet->kalman_x1 = kalman_x1;
    packet->kalman_x2 = kalman_x2;
    packet->mass = mass;
    packet->corrected_pressure = corrected_pressure;
    packet->state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEA_TM, (const char *)packet, MAVLINK_MSG_ID_MEA_TM_MIN_LEN, MAVLINK_MSG_ID_MEA_TM_LEN, MAVLINK_MSG_ID_MEA_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE MEA_TM UNPACKING


/**
 * @brief Get field timestamp from mea_tm message
 *
 * @return [us] When was this logged
 */
static inline uint64_t mavlink_msg_mea_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field state from mea_tm message
 *
 * @return  MEA current state
 */
static inline uint8_t mavlink_msg_mea_tm_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field kalman_x0 from mea_tm message
 *
 * @return  Kalman state variable 0 (corrected pressure)
 */
static inline float mavlink_msg_mea_tm_get_kalman_x0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field kalman_x1 from mea_tm message
 *
 * @return  Kalman state variable 1 
 */
static inline float mavlink_msg_mea_tm_get_kalman_x1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field kalman_x2 from mea_tm message
 *
 * @return  Kalman state variable 2 (mass)
 */
static inline float mavlink_msg_mea_tm_get_kalman_x2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field mass from mea_tm message
 *
 * @return [kg] Mass estimated
 */
static inline float mavlink_msg_mea_tm_get_mass(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field corrected_pressure from mea_tm message
 *
 * @return [kg] Corrected pressure
 */
static inline float mavlink_msg_mea_tm_get_corrected_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a mea_tm message into a struct
 *
 * @param msg The message to decode
 * @param mea_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_mea_tm_decode(const mavlink_message_t* msg, mavlink_mea_tm_t* mea_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mea_tm->timestamp = mavlink_msg_mea_tm_get_timestamp(msg);
    mea_tm->kalman_x0 = mavlink_msg_mea_tm_get_kalman_x0(msg);
    mea_tm->kalman_x1 = mavlink_msg_mea_tm_get_kalman_x1(msg);
    mea_tm->kalman_x2 = mavlink_msg_mea_tm_get_kalman_x2(msg);
    mea_tm->mass = mavlink_msg_mea_tm_get_mass(msg);
    mea_tm->corrected_pressure = mavlink_msg_mea_tm_get_corrected_pressure(msg);
    mea_tm->state = mavlink_msg_mea_tm_get_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MEA_TM_LEN? msg->len : MAVLINK_MSG_ID_MEA_TM_LEN;
        memset(mea_tm, 0, MAVLINK_MSG_ID_MEA_TM_LEN);
    memcpy(mea_tm, _MAV_PAYLOAD(msg), len);
#endif
}
