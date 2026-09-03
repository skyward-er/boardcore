#pragma once
// MESSAGE ABK_TM PACKING

#define MAVLINK_MSG_ID_ABK_TM 168

MAVPACKED(
typedef struct __mavlink_abk_tm_t {
 uint64_t timestamp; /*< [ms] When was this logged*/
 float servo_position; /*< [deg] Aerobrakes opening angle*/
 float estimated_cd; /*<  Drag estimated by the control algorithm*/
 float pid_error; /*<  Error of the PID controller at each step*/
 float z; /*< [m] Input altitude*/
 float vz; /*< [m/s] Input vertical speed*/
 float v_mod; /*< [m/s] Input speed magnitude*/
 uint8_t state; /*<  Aerobrakes FSM state*/
 uint8_t trajectory; /*<  Chosen trajectory to be followed*/
}) mavlink_abk_tm_t;

#define MAVLINK_MSG_ID_ABK_TM_LEN 34
#define MAVLINK_MSG_ID_ABK_TM_MIN_LEN 34
#define MAVLINK_MSG_ID_168_LEN 34
#define MAVLINK_MSG_ID_168_MIN_LEN 34

#define MAVLINK_MSG_ID_ABK_TM_CRC 130
#define MAVLINK_MSG_ID_168_CRC 130



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ABK_TM { \
    168, \
    "ABK_TM", \
    9, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_abk_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_abk_tm_t, state) }, \
         { "servo_position", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_abk_tm_t, servo_position) }, \
         { "estimated_cd", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_abk_tm_t, estimated_cd) }, \
         { "pid_error", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_abk_tm_t, pid_error) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_abk_tm_t, z) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_abk_tm_t, vz) }, \
         { "v_mod", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_abk_tm_t, v_mod) }, \
         { "trajectory", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_abk_tm_t, trajectory) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ABK_TM { \
    "ABK_TM", \
    9, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_abk_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_abk_tm_t, state) }, \
         { "servo_position", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_abk_tm_t, servo_position) }, \
         { "estimated_cd", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_abk_tm_t, estimated_cd) }, \
         { "pid_error", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_abk_tm_t, pid_error) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_abk_tm_t, z) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_abk_tm_t, vz) }, \
         { "v_mod", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_abk_tm_t, v_mod) }, \
         { "trajectory", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_abk_tm_t, trajectory) }, \
         } \
}
#endif

/**
 * @brief Pack a abk_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] When was this logged
 * @param state  Aerobrakes FSM state
 * @param servo_position [deg] Aerobrakes opening angle
 * @param estimated_cd  Drag estimated by the control algorithm
 * @param pid_error  Error of the PID controller at each step
 * @param z [m] Input altitude
 * @param vz [m/s] Input vertical speed
 * @param v_mod [m/s] Input speed magnitude
 * @param trajectory  Chosen trajectory to be followed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_abk_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t state, float servo_position, float estimated_cd, float pid_error, float z, float vz, float v_mod, uint8_t trajectory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ABK_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, servo_position);
    _mav_put_float(buf, 12, estimated_cd);
    _mav_put_float(buf, 16, pid_error);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 24, vz);
    _mav_put_float(buf, 28, v_mod);
    _mav_put_uint8_t(buf, 32, state);
    _mav_put_uint8_t(buf, 33, trajectory);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ABK_TM_LEN);
#else
    mavlink_abk_tm_t packet;
    packet.timestamp = timestamp;
    packet.servo_position = servo_position;
    packet.estimated_cd = estimated_cd;
    packet.pid_error = pid_error;
    packet.z = z;
    packet.vz = vz;
    packet.v_mod = v_mod;
    packet.state = state;
    packet.trajectory = trajectory;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ABK_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ABK_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ABK_TM_MIN_LEN, MAVLINK_MSG_ID_ABK_TM_LEN, MAVLINK_MSG_ID_ABK_TM_CRC);
}

/**
 * @brief Pack a abk_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] When was this logged
 * @param state  Aerobrakes FSM state
 * @param servo_position [deg] Aerobrakes opening angle
 * @param estimated_cd  Drag estimated by the control algorithm
 * @param pid_error  Error of the PID controller at each step
 * @param z [m] Input altitude
 * @param vz [m/s] Input vertical speed
 * @param v_mod [m/s] Input speed magnitude
 * @param trajectory  Chosen trajectory to be followed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_abk_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t state,float servo_position,float estimated_cd,float pid_error,float z,float vz,float v_mod,uint8_t trajectory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ABK_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, servo_position);
    _mav_put_float(buf, 12, estimated_cd);
    _mav_put_float(buf, 16, pid_error);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 24, vz);
    _mav_put_float(buf, 28, v_mod);
    _mav_put_uint8_t(buf, 32, state);
    _mav_put_uint8_t(buf, 33, trajectory);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ABK_TM_LEN);
#else
    mavlink_abk_tm_t packet;
    packet.timestamp = timestamp;
    packet.servo_position = servo_position;
    packet.estimated_cd = estimated_cd;
    packet.pid_error = pid_error;
    packet.z = z;
    packet.vz = vz;
    packet.v_mod = v_mod;
    packet.state = state;
    packet.trajectory = trajectory;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ABK_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ABK_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ABK_TM_MIN_LEN, MAVLINK_MSG_ID_ABK_TM_LEN, MAVLINK_MSG_ID_ABK_TM_CRC);
}

/**
 * @brief Encode a abk_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param abk_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_abk_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_abk_tm_t* abk_tm)
{
    return mavlink_msg_abk_tm_pack(system_id, component_id, msg, abk_tm->timestamp, abk_tm->state, abk_tm->servo_position, abk_tm->estimated_cd, abk_tm->pid_error, abk_tm->z, abk_tm->vz, abk_tm->v_mod, abk_tm->trajectory);
}

/**
 * @brief Encode a abk_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param abk_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_abk_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_abk_tm_t* abk_tm)
{
    return mavlink_msg_abk_tm_pack_chan(system_id, component_id, chan, msg, abk_tm->timestamp, abk_tm->state, abk_tm->servo_position, abk_tm->estimated_cd, abk_tm->pid_error, abk_tm->z, abk_tm->vz, abk_tm->v_mod, abk_tm->trajectory);
}

/**
 * @brief Send a abk_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] When was this logged
 * @param state  Aerobrakes FSM state
 * @param servo_position [deg] Aerobrakes opening angle
 * @param estimated_cd  Drag estimated by the control algorithm
 * @param pid_error  Error of the PID controller at each step
 * @param z [m] Input altitude
 * @param vz [m/s] Input vertical speed
 * @param v_mod [m/s] Input speed magnitude
 * @param trajectory  Chosen trajectory to be followed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_abk_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t state, float servo_position, float estimated_cd, float pid_error, float z, float vz, float v_mod, uint8_t trajectory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ABK_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, servo_position);
    _mav_put_float(buf, 12, estimated_cd);
    _mav_put_float(buf, 16, pid_error);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 24, vz);
    _mav_put_float(buf, 28, v_mod);
    _mav_put_uint8_t(buf, 32, state);
    _mav_put_uint8_t(buf, 33, trajectory);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ABK_TM, buf, MAVLINK_MSG_ID_ABK_TM_MIN_LEN, MAVLINK_MSG_ID_ABK_TM_LEN, MAVLINK_MSG_ID_ABK_TM_CRC);
#else
    mavlink_abk_tm_t packet;
    packet.timestamp = timestamp;
    packet.servo_position = servo_position;
    packet.estimated_cd = estimated_cd;
    packet.pid_error = pid_error;
    packet.z = z;
    packet.vz = vz;
    packet.v_mod = v_mod;
    packet.state = state;
    packet.trajectory = trajectory;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ABK_TM, (const char *)&packet, MAVLINK_MSG_ID_ABK_TM_MIN_LEN, MAVLINK_MSG_ID_ABK_TM_LEN, MAVLINK_MSG_ID_ABK_TM_CRC);
#endif
}

/**
 * @brief Send a abk_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_abk_tm_send_struct(mavlink_channel_t chan, const mavlink_abk_tm_t* abk_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_abk_tm_send(chan, abk_tm->timestamp, abk_tm->state, abk_tm->servo_position, abk_tm->estimated_cd, abk_tm->pid_error, abk_tm->z, abk_tm->vz, abk_tm->v_mod, abk_tm->trajectory);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ABK_TM, (const char *)abk_tm, MAVLINK_MSG_ID_ABK_TM_MIN_LEN, MAVLINK_MSG_ID_ABK_TM_LEN, MAVLINK_MSG_ID_ABK_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_ABK_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_abk_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t state, float servo_position, float estimated_cd, float pid_error, float z, float vz, float v_mod, uint8_t trajectory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, servo_position);
    _mav_put_float(buf, 12, estimated_cd);
    _mav_put_float(buf, 16, pid_error);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 24, vz);
    _mav_put_float(buf, 28, v_mod);
    _mav_put_uint8_t(buf, 32, state);
    _mav_put_uint8_t(buf, 33, trajectory);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ABK_TM, buf, MAVLINK_MSG_ID_ABK_TM_MIN_LEN, MAVLINK_MSG_ID_ABK_TM_LEN, MAVLINK_MSG_ID_ABK_TM_CRC);
#else
    mavlink_abk_tm_t *packet = (mavlink_abk_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->servo_position = servo_position;
    packet->estimated_cd = estimated_cd;
    packet->pid_error = pid_error;
    packet->z = z;
    packet->vz = vz;
    packet->v_mod = v_mod;
    packet->state = state;
    packet->trajectory = trajectory;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ABK_TM, (const char *)packet, MAVLINK_MSG_ID_ABK_TM_MIN_LEN, MAVLINK_MSG_ID_ABK_TM_LEN, MAVLINK_MSG_ID_ABK_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ABK_TM UNPACKING


/**
 * @brief Get field timestamp from abk_tm message
 *
 * @return [ms] When was this logged
 */
static inline uint64_t mavlink_msg_abk_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field state from abk_tm message
 *
 * @return  Aerobrakes FSM state
 */
static inline uint8_t mavlink_msg_abk_tm_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field servo_position from abk_tm message
 *
 * @return [deg] Aerobrakes opening angle
 */
static inline float mavlink_msg_abk_tm_get_servo_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field estimated_cd from abk_tm message
 *
 * @return  Drag estimated by the control algorithm
 */
static inline float mavlink_msg_abk_tm_get_estimated_cd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pid_error from abk_tm message
 *
 * @return  Error of the PID controller at each step
 */
static inline float mavlink_msg_abk_tm_get_pid_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field z from abk_tm message
 *
 * @return [m] Input altitude
 */
static inline float mavlink_msg_abk_tm_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vz from abk_tm message
 *
 * @return [m/s] Input vertical speed
 */
static inline float mavlink_msg_abk_tm_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field v_mod from abk_tm message
 *
 * @return [m/s] Input speed magnitude
 */
static inline float mavlink_msg_abk_tm_get_v_mod(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field trajectory from abk_tm message
 *
 * @return  Chosen trajectory to be followed
 */
static inline uint8_t mavlink_msg_abk_tm_get_trajectory(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Decode a abk_tm message into a struct
 *
 * @param msg The message to decode
 * @param abk_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_abk_tm_decode(const mavlink_message_t* msg, mavlink_abk_tm_t* abk_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    abk_tm->timestamp = mavlink_msg_abk_tm_get_timestamp(msg);
    abk_tm->servo_position = mavlink_msg_abk_tm_get_servo_position(msg);
    abk_tm->estimated_cd = mavlink_msg_abk_tm_get_estimated_cd(msg);
    abk_tm->pid_error = mavlink_msg_abk_tm_get_pid_error(msg);
    abk_tm->z = mavlink_msg_abk_tm_get_z(msg);
    abk_tm->vz = mavlink_msg_abk_tm_get_vz(msg);
    abk_tm->v_mod = mavlink_msg_abk_tm_get_v_mod(msg);
    abk_tm->state = mavlink_msg_abk_tm_get_state(msg);
    abk_tm->trajectory = mavlink_msg_abk_tm_get_trajectory(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ABK_TM_LEN? msg->len : MAVLINK_MSG_ID_ABK_TM_LEN;
        memset(abk_tm, 0, MAVLINK_MSG_ID_ABK_TM_LEN);
    memcpy(abk_tm, _MAV_PAYLOAD(msg), len);
#endif
}
