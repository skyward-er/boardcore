#pragma once
// MESSAGE WING_TM PACKING

#define MAVLINK_MSG_ID_WING_TM 215


typedef struct __mavlink_wing_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 float target_lat; /*< [deg] Target latitude*/
 float target_lon; /*< [deg] Target longitude*/
 float target_n; /*< [m] Target N*/
 float target_e; /*< [m] Target E*/
 float emc_n; /*< [m] EMC N*/
 float emc_e; /*< [m] EMC E*/
 float m1_n; /*< [m] M1 N*/
 float m1_e; /*< [m] M1 E*/
 float m2_n; /*< [m] M2 N*/
 float m2_e; /*< [m] M2 E*/
} mavlink_wing_tm_t;

#define MAVLINK_MSG_ID_WING_TM_LEN 48
#define MAVLINK_MSG_ID_WING_TM_MIN_LEN 48
#define MAVLINK_MSG_ID_215_LEN 48
#define MAVLINK_MSG_ID_215_MIN_LEN 48

#define MAVLINK_MSG_ID_WING_TM_CRC 113
#define MAVLINK_MSG_ID_215_CRC 113



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WING_TM { \
    215, \
    "WING_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_wing_tm_t, timestamp) }, \
         { "target_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_wing_tm_t, target_lat) }, \
         { "target_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_wing_tm_t, target_lon) }, \
         { "target_n", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_wing_tm_t, target_n) }, \
         { "target_e", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_wing_tm_t, target_e) }, \
         { "emc_n", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_wing_tm_t, emc_n) }, \
         { "emc_e", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_wing_tm_t, emc_e) }, \
         { "m1_n", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_wing_tm_t, m1_n) }, \
         { "m1_e", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_wing_tm_t, m1_e) }, \
         { "m2_n", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_wing_tm_t, m2_n) }, \
         { "m2_e", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_wing_tm_t, m2_e) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WING_TM { \
    "WING_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_wing_tm_t, timestamp) }, \
         { "target_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_wing_tm_t, target_lat) }, \
         { "target_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_wing_tm_t, target_lon) }, \
         { "target_n", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_wing_tm_t, target_n) }, \
         { "target_e", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_wing_tm_t, target_e) }, \
         { "emc_n", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_wing_tm_t, emc_n) }, \
         { "emc_e", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_wing_tm_t, emc_e) }, \
         { "m1_n", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_wing_tm_t, m1_n) }, \
         { "m1_e", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_wing_tm_t, m1_e) }, \
         { "m2_n", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_wing_tm_t, m2_n) }, \
         { "m2_e", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_wing_tm_t, m2_e) }, \
         } \
}
#endif

/**
 * @brief Pack a wing_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param target_lat [deg] Target latitude
 * @param target_lon [deg] Target longitude
 * @param target_n [m] Target N
 * @param target_e [m] Target E
 * @param emc_n [m] EMC N
 * @param emc_e [m] EMC E
 * @param m1_n [m] M1 N
 * @param m1_e [m] M1 E
 * @param m2_n [m] M2 N
 * @param m2_e [m] M2 E
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wing_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float target_lat, float target_lon, float target_n, float target_e, float emc_n, float emc_e, float m1_n, float m1_e, float m2_n, float m2_e)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WING_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, target_lat);
    _mav_put_float(buf, 12, target_lon);
    _mav_put_float(buf, 16, target_n);
    _mav_put_float(buf, 20, target_e);
    _mav_put_float(buf, 24, emc_n);
    _mav_put_float(buf, 28, emc_e);
    _mav_put_float(buf, 32, m1_n);
    _mav_put_float(buf, 36, m1_e);
    _mav_put_float(buf, 40, m2_n);
    _mav_put_float(buf, 44, m2_e);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WING_TM_LEN);
#else
    mavlink_wing_tm_t packet;
    packet.timestamp = timestamp;
    packet.target_lat = target_lat;
    packet.target_lon = target_lon;
    packet.target_n = target_n;
    packet.target_e = target_e;
    packet.emc_n = emc_n;
    packet.emc_e = emc_e;
    packet.m1_n = m1_n;
    packet.m1_e = m1_e;
    packet.m2_n = m2_n;
    packet.m2_e = m2_e;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WING_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WING_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WING_TM_MIN_LEN, MAVLINK_MSG_ID_WING_TM_LEN, MAVLINK_MSG_ID_WING_TM_CRC);
}

/**
 * @brief Pack a wing_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp in microseconds
 * @param target_lat [deg] Target latitude
 * @param target_lon [deg] Target longitude
 * @param target_n [m] Target N
 * @param target_e [m] Target E
 * @param emc_n [m] EMC N
 * @param emc_e [m] EMC E
 * @param m1_n [m] M1 N
 * @param m1_e [m] M1 E
 * @param m2_n [m] M2 N
 * @param m2_e [m] M2 E
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wing_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float target_lat,float target_lon,float target_n,float target_e,float emc_n,float emc_e,float m1_n,float m1_e,float m2_n,float m2_e)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WING_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, target_lat);
    _mav_put_float(buf, 12, target_lon);
    _mav_put_float(buf, 16, target_n);
    _mav_put_float(buf, 20, target_e);
    _mav_put_float(buf, 24, emc_n);
    _mav_put_float(buf, 28, emc_e);
    _mav_put_float(buf, 32, m1_n);
    _mav_put_float(buf, 36, m1_e);
    _mav_put_float(buf, 40, m2_n);
    _mav_put_float(buf, 44, m2_e);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WING_TM_LEN);
#else
    mavlink_wing_tm_t packet;
    packet.timestamp = timestamp;
    packet.target_lat = target_lat;
    packet.target_lon = target_lon;
    packet.target_n = target_n;
    packet.target_e = target_e;
    packet.emc_n = emc_n;
    packet.emc_e = emc_e;
    packet.m1_n = m1_n;
    packet.m1_e = m1_e;
    packet.m2_n = m2_n;
    packet.m2_e = m2_e;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WING_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WING_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WING_TM_MIN_LEN, MAVLINK_MSG_ID_WING_TM_LEN, MAVLINK_MSG_ID_WING_TM_CRC);
}

/**
 * @brief Encode a wing_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wing_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wing_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wing_tm_t* wing_tm)
{
    return mavlink_msg_wing_tm_pack(system_id, component_id, msg, wing_tm->timestamp, wing_tm->target_lat, wing_tm->target_lon, wing_tm->target_n, wing_tm->target_e, wing_tm->emc_n, wing_tm->emc_e, wing_tm->m1_n, wing_tm->m1_e, wing_tm->m2_n, wing_tm->m2_e);
}

/**
 * @brief Encode a wing_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wing_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wing_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_wing_tm_t* wing_tm)
{
    return mavlink_msg_wing_tm_pack_chan(system_id, component_id, chan, msg, wing_tm->timestamp, wing_tm->target_lat, wing_tm->target_lon, wing_tm->target_n, wing_tm->target_e, wing_tm->emc_n, wing_tm->emc_e, wing_tm->m1_n, wing_tm->m1_e, wing_tm->m2_n, wing_tm->m2_e);
}

/**
 * @brief Send a wing_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param target_lat [deg] Target latitude
 * @param target_lon [deg] Target longitude
 * @param target_n [m] Target N
 * @param target_e [m] Target E
 * @param emc_n [m] EMC N
 * @param emc_e [m] EMC E
 * @param m1_n [m] M1 N
 * @param m1_e [m] M1 E
 * @param m2_n [m] M2 N
 * @param m2_e [m] M2 E
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wing_tm_send(mavlink_channel_t chan, uint64_t timestamp, float target_lat, float target_lon, float target_n, float target_e, float emc_n, float emc_e, float m1_n, float m1_e, float m2_n, float m2_e)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WING_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, target_lat);
    _mav_put_float(buf, 12, target_lon);
    _mav_put_float(buf, 16, target_n);
    _mav_put_float(buf, 20, target_e);
    _mav_put_float(buf, 24, emc_n);
    _mav_put_float(buf, 28, emc_e);
    _mav_put_float(buf, 32, m1_n);
    _mav_put_float(buf, 36, m1_e);
    _mav_put_float(buf, 40, m2_n);
    _mav_put_float(buf, 44, m2_e);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WING_TM, buf, MAVLINK_MSG_ID_WING_TM_MIN_LEN, MAVLINK_MSG_ID_WING_TM_LEN, MAVLINK_MSG_ID_WING_TM_CRC);
#else
    mavlink_wing_tm_t packet;
    packet.timestamp = timestamp;
    packet.target_lat = target_lat;
    packet.target_lon = target_lon;
    packet.target_n = target_n;
    packet.target_e = target_e;
    packet.emc_n = emc_n;
    packet.emc_e = emc_e;
    packet.m1_n = m1_n;
    packet.m1_e = m1_e;
    packet.m2_n = m2_n;
    packet.m2_e = m2_e;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WING_TM, (const char *)&packet, MAVLINK_MSG_ID_WING_TM_MIN_LEN, MAVLINK_MSG_ID_WING_TM_LEN, MAVLINK_MSG_ID_WING_TM_CRC);
#endif
}

/**
 * @brief Send a wing_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_wing_tm_send_struct(mavlink_channel_t chan, const mavlink_wing_tm_t* wing_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_wing_tm_send(chan, wing_tm->timestamp, wing_tm->target_lat, wing_tm->target_lon, wing_tm->target_n, wing_tm->target_e, wing_tm->emc_n, wing_tm->emc_e, wing_tm->m1_n, wing_tm->m1_e, wing_tm->m2_n, wing_tm->m2_e);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WING_TM, (const char *)wing_tm, MAVLINK_MSG_ID_WING_TM_MIN_LEN, MAVLINK_MSG_ID_WING_TM_LEN, MAVLINK_MSG_ID_WING_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_WING_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_wing_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float target_lat, float target_lon, float target_n, float target_e, float emc_n, float emc_e, float m1_n, float m1_e, float m2_n, float m2_e)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, target_lat);
    _mav_put_float(buf, 12, target_lon);
    _mav_put_float(buf, 16, target_n);
    _mav_put_float(buf, 20, target_e);
    _mav_put_float(buf, 24, emc_n);
    _mav_put_float(buf, 28, emc_e);
    _mav_put_float(buf, 32, m1_n);
    _mav_put_float(buf, 36, m1_e);
    _mav_put_float(buf, 40, m2_n);
    _mav_put_float(buf, 44, m2_e);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WING_TM, buf, MAVLINK_MSG_ID_WING_TM_MIN_LEN, MAVLINK_MSG_ID_WING_TM_LEN, MAVLINK_MSG_ID_WING_TM_CRC);
#else
    mavlink_wing_tm_t *packet = (mavlink_wing_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->target_lat = target_lat;
    packet->target_lon = target_lon;
    packet->target_n = target_n;
    packet->target_e = target_e;
    packet->emc_n = emc_n;
    packet->emc_e = emc_e;
    packet->m1_n = m1_n;
    packet->m1_e = m1_e;
    packet->m2_n = m2_n;
    packet->m2_e = m2_e;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WING_TM, (const char *)packet, MAVLINK_MSG_ID_WING_TM_MIN_LEN, MAVLINK_MSG_ID_WING_TM_LEN, MAVLINK_MSG_ID_WING_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE WING_TM UNPACKING


/**
 * @brief Get field timestamp from wing_tm message
 *
 * @return [us] Timestamp in microseconds
 */
static inline uint64_t mavlink_msg_wing_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_lat from wing_tm message
 *
 * @return [deg] Target latitude
 */
static inline float mavlink_msg_wing_tm_get_target_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field target_lon from wing_tm message
 *
 * @return [deg] Target longitude
 */
static inline float mavlink_msg_wing_tm_get_target_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field target_n from wing_tm message
 *
 * @return [m] Target N
 */
static inline float mavlink_msg_wing_tm_get_target_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field target_e from wing_tm message
 *
 * @return [m] Target E
 */
static inline float mavlink_msg_wing_tm_get_target_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field emc_n from wing_tm message
 *
 * @return [m] EMC N
 */
static inline float mavlink_msg_wing_tm_get_emc_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field emc_e from wing_tm message
 *
 * @return [m] EMC E
 */
static inline float mavlink_msg_wing_tm_get_emc_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field m1_n from wing_tm message
 *
 * @return [m] M1 N
 */
static inline float mavlink_msg_wing_tm_get_m1_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field m1_e from wing_tm message
 *
 * @return [m] M1 E
 */
static inline float mavlink_msg_wing_tm_get_m1_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field m2_n from wing_tm message
 *
 * @return [m] M2 N
 */
static inline float mavlink_msg_wing_tm_get_m2_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field m2_e from wing_tm message
 *
 * @return [m] M2 E
 */
static inline float mavlink_msg_wing_tm_get_m2_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a wing_tm message into a struct
 *
 * @param msg The message to decode
 * @param wing_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_wing_tm_decode(const mavlink_message_t* msg, mavlink_wing_tm_t* wing_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    wing_tm->timestamp = mavlink_msg_wing_tm_get_timestamp(msg);
    wing_tm->target_lat = mavlink_msg_wing_tm_get_target_lat(msg);
    wing_tm->target_lon = mavlink_msg_wing_tm_get_target_lon(msg);
    wing_tm->target_n = mavlink_msg_wing_tm_get_target_n(msg);
    wing_tm->target_e = mavlink_msg_wing_tm_get_target_e(msg);
    wing_tm->emc_n = mavlink_msg_wing_tm_get_emc_n(msg);
    wing_tm->emc_e = mavlink_msg_wing_tm_get_emc_e(msg);
    wing_tm->m1_n = mavlink_msg_wing_tm_get_m1_n(msg);
    wing_tm->m1_e = mavlink_msg_wing_tm_get_m1_e(msg);
    wing_tm->m2_n = mavlink_msg_wing_tm_get_m2_n(msg);
    wing_tm->m2_e = mavlink_msg_wing_tm_get_m2_e(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WING_TM_LEN? msg->len : MAVLINK_MSG_ID_WING_TM_LEN;
        memset(wing_tm, 0, MAVLINK_MSG_ID_WING_TM_LEN);
    memcpy(wing_tm, _MAV_PAYLOAD(msg), len);
#endif
}
