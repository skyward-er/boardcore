#pragma once
// MESSAGE WINDTUNNEL_TM PACKING

#define MAVLINK_MSG_ID_WINDTUNNEL_TM 183

MAVPACKED(
typedef struct __mavlink_windtunnel_tm_t {
 uint64_t timestamp; /*< [ms] When was this logged*/
 double pressure_dpl; /*< [Pa] Pressure from deployment vane*/
 float pressure_digital; /*< [Pa] Pressure from digital pressure sensor*/
 float pressure_differential; /*< [Pa] Differential pressure*/
 float pressure_static; /*< [Pa] Pressure from static port*/
 float ab_angle; /*< [deg] Current aerobrake angle*/
 float wind_speed; /*< [m/s] Current tunnel wind speed*/
 float log_status; /*<  Logger status*/
 float log_num; /*<  Logger file number*/
 float last_RSSI; /*< [dBm] Last message RSSI*/
}) mavlink_windtunnel_tm_t;

#define MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN 48
#define MAVLINK_MSG_ID_WINDTUNNEL_TM_MIN_LEN 48
#define MAVLINK_MSG_ID_183_LEN 48
#define MAVLINK_MSG_ID_183_MIN_LEN 48

#define MAVLINK_MSG_ID_WINDTUNNEL_TM_CRC 176
#define MAVLINK_MSG_ID_183_CRC 176



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WINDTUNNEL_TM { \
    183, \
    "WINDTUNNEL_TM", \
    10, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_windtunnel_tm_t, timestamp) }, \
         { "pressure_digital", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_windtunnel_tm_t, pressure_digital) }, \
         { "pressure_differential", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_windtunnel_tm_t, pressure_differential) }, \
         { "pressure_static", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_windtunnel_tm_t, pressure_static) }, \
         { "pressure_dpl", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_windtunnel_tm_t, pressure_dpl) }, \
         { "ab_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_windtunnel_tm_t, ab_angle) }, \
         { "wind_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_windtunnel_tm_t, wind_speed) }, \
         { "log_status", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_windtunnel_tm_t, log_status) }, \
         { "log_num", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_windtunnel_tm_t, log_num) }, \
         { "last_RSSI", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_windtunnel_tm_t, last_RSSI) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WINDTUNNEL_TM { \
    "WINDTUNNEL_TM", \
    10, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_windtunnel_tm_t, timestamp) }, \
         { "pressure_digital", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_windtunnel_tm_t, pressure_digital) }, \
         { "pressure_differential", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_windtunnel_tm_t, pressure_differential) }, \
         { "pressure_static", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_windtunnel_tm_t, pressure_static) }, \
         { "pressure_dpl", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_windtunnel_tm_t, pressure_dpl) }, \
         { "ab_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_windtunnel_tm_t, ab_angle) }, \
         { "wind_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_windtunnel_tm_t, wind_speed) }, \
         { "log_status", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_windtunnel_tm_t, log_status) }, \
         { "log_num", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_windtunnel_tm_t, log_num) }, \
         { "last_RSSI", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_windtunnel_tm_t, last_RSSI) }, \
         } \
}
#endif

/**
 * @brief Pack a windtunnel_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] When was this logged
 * @param pressure_digital [Pa] Pressure from digital pressure sensor
 * @param pressure_differential [Pa] Differential pressure
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane
 * @param ab_angle [deg] Current aerobrake angle
 * @param wind_speed [m/s] Current tunnel wind speed
 * @param log_status  Logger status
 * @param log_num  Logger file number
 * @param last_RSSI [dBm] Last message RSSI
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_windtunnel_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float pressure_digital, float pressure_differential, float pressure_static, double pressure_dpl, float ab_angle, float wind_speed, float log_status, float log_num, float last_RSSI)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_double(buf, 8, pressure_dpl);
    _mav_put_float(buf, 16, pressure_digital);
    _mav_put_float(buf, 20, pressure_differential);
    _mav_put_float(buf, 24, pressure_static);
    _mav_put_float(buf, 28, ab_angle);
    _mav_put_float(buf, 32, wind_speed);
    _mav_put_float(buf, 36, log_status);
    _mav_put_float(buf, 40, log_num);
    _mav_put_float(buf, 44, last_RSSI);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN);
#else
    mavlink_windtunnel_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_dpl = pressure_dpl;
    packet.pressure_digital = pressure_digital;
    packet.pressure_differential = pressure_differential;
    packet.pressure_static = pressure_static;
    packet.ab_angle = ab_angle;
    packet.wind_speed = wind_speed;
    packet.log_status = log_status;
    packet.log_num = log_num;
    packet.last_RSSI = last_RSSI;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WINDTUNNEL_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WINDTUNNEL_TM_MIN_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_CRC);
}

/**
 * @brief Pack a windtunnel_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] When was this logged
 * @param pressure_digital [Pa] Pressure from digital pressure sensor
 * @param pressure_differential [Pa] Differential pressure
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane
 * @param ab_angle [deg] Current aerobrake angle
 * @param wind_speed [m/s] Current tunnel wind speed
 * @param log_status  Logger status
 * @param log_num  Logger file number
 * @param last_RSSI [dBm] Last message RSSI
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_windtunnel_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float pressure_digital,float pressure_differential,float pressure_static,double pressure_dpl,float ab_angle,float wind_speed,float log_status,float log_num,float last_RSSI)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_double(buf, 8, pressure_dpl);
    _mav_put_float(buf, 16, pressure_digital);
    _mav_put_float(buf, 20, pressure_differential);
    _mav_put_float(buf, 24, pressure_static);
    _mav_put_float(buf, 28, ab_angle);
    _mav_put_float(buf, 32, wind_speed);
    _mav_put_float(buf, 36, log_status);
    _mav_put_float(buf, 40, log_num);
    _mav_put_float(buf, 44, last_RSSI);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN);
#else
    mavlink_windtunnel_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_dpl = pressure_dpl;
    packet.pressure_digital = pressure_digital;
    packet.pressure_differential = pressure_differential;
    packet.pressure_static = pressure_static;
    packet.ab_angle = ab_angle;
    packet.wind_speed = wind_speed;
    packet.log_status = log_status;
    packet.log_num = log_num;
    packet.last_RSSI = last_RSSI;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WINDTUNNEL_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WINDTUNNEL_TM_MIN_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_CRC);
}

/**
 * @brief Encode a windtunnel_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param windtunnel_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_windtunnel_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_windtunnel_tm_t* windtunnel_tm)
{
    return mavlink_msg_windtunnel_tm_pack(system_id, component_id, msg, windtunnel_tm->timestamp, windtunnel_tm->pressure_digital, windtunnel_tm->pressure_differential, windtunnel_tm->pressure_static, windtunnel_tm->pressure_dpl, windtunnel_tm->ab_angle, windtunnel_tm->wind_speed, windtunnel_tm->log_status, windtunnel_tm->log_num, windtunnel_tm->last_RSSI);
}

/**
 * @brief Encode a windtunnel_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param windtunnel_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_windtunnel_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_windtunnel_tm_t* windtunnel_tm)
{
    return mavlink_msg_windtunnel_tm_pack_chan(system_id, component_id, chan, msg, windtunnel_tm->timestamp, windtunnel_tm->pressure_digital, windtunnel_tm->pressure_differential, windtunnel_tm->pressure_static, windtunnel_tm->pressure_dpl, windtunnel_tm->ab_angle, windtunnel_tm->wind_speed, windtunnel_tm->log_status, windtunnel_tm->log_num, windtunnel_tm->last_RSSI);
}

/**
 * @brief Send a windtunnel_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] When was this logged
 * @param pressure_digital [Pa] Pressure from digital pressure sensor
 * @param pressure_differential [Pa] Differential pressure
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane
 * @param ab_angle [deg] Current aerobrake angle
 * @param wind_speed [m/s] Current tunnel wind speed
 * @param log_status  Logger status
 * @param log_num  Logger file number
 * @param last_RSSI [dBm] Last message RSSI
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_windtunnel_tm_send(mavlink_channel_t chan, uint64_t timestamp, float pressure_digital, float pressure_differential, float pressure_static, double pressure_dpl, float ab_angle, float wind_speed, float log_status, float log_num, float last_RSSI)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_double(buf, 8, pressure_dpl);
    _mav_put_float(buf, 16, pressure_digital);
    _mav_put_float(buf, 20, pressure_differential);
    _mav_put_float(buf, 24, pressure_static);
    _mav_put_float(buf, 28, ab_angle);
    _mav_put_float(buf, 32, wind_speed);
    _mav_put_float(buf, 36, log_status);
    _mav_put_float(buf, 40, log_num);
    _mav_put_float(buf, 44, last_RSSI);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINDTUNNEL_TM, buf, MAVLINK_MSG_ID_WINDTUNNEL_TM_MIN_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_CRC);
#else
    mavlink_windtunnel_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_dpl = pressure_dpl;
    packet.pressure_digital = pressure_digital;
    packet.pressure_differential = pressure_differential;
    packet.pressure_static = pressure_static;
    packet.ab_angle = ab_angle;
    packet.wind_speed = wind_speed;
    packet.log_status = log_status;
    packet.log_num = log_num;
    packet.last_RSSI = last_RSSI;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINDTUNNEL_TM, (const char *)&packet, MAVLINK_MSG_ID_WINDTUNNEL_TM_MIN_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_CRC);
#endif
}

/**
 * @brief Send a windtunnel_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_windtunnel_tm_send_struct(mavlink_channel_t chan, const mavlink_windtunnel_tm_t* windtunnel_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_windtunnel_tm_send(chan, windtunnel_tm->timestamp, windtunnel_tm->pressure_digital, windtunnel_tm->pressure_differential, windtunnel_tm->pressure_static, windtunnel_tm->pressure_dpl, windtunnel_tm->ab_angle, windtunnel_tm->wind_speed, windtunnel_tm->log_status, windtunnel_tm->log_num, windtunnel_tm->last_RSSI);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINDTUNNEL_TM, (const char *)windtunnel_tm, MAVLINK_MSG_ID_WINDTUNNEL_TM_MIN_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_windtunnel_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float pressure_digital, float pressure_differential, float pressure_static, double pressure_dpl, float ab_angle, float wind_speed, float log_status, float log_num, float last_RSSI)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_double(buf, 8, pressure_dpl);
    _mav_put_float(buf, 16, pressure_digital);
    _mav_put_float(buf, 20, pressure_differential);
    _mav_put_float(buf, 24, pressure_static);
    _mav_put_float(buf, 28, ab_angle);
    _mav_put_float(buf, 32, wind_speed);
    _mav_put_float(buf, 36, log_status);
    _mav_put_float(buf, 40, log_num);
    _mav_put_float(buf, 44, last_RSSI);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINDTUNNEL_TM, buf, MAVLINK_MSG_ID_WINDTUNNEL_TM_MIN_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_CRC);
#else
    mavlink_windtunnel_tm_t *packet = (mavlink_windtunnel_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->pressure_dpl = pressure_dpl;
    packet->pressure_digital = pressure_digital;
    packet->pressure_differential = pressure_differential;
    packet->pressure_static = pressure_static;
    packet->ab_angle = ab_angle;
    packet->wind_speed = wind_speed;
    packet->log_status = log_status;
    packet->log_num = log_num;
    packet->last_RSSI = last_RSSI;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINDTUNNEL_TM, (const char *)packet, MAVLINK_MSG_ID_WINDTUNNEL_TM_MIN_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN, MAVLINK_MSG_ID_WINDTUNNEL_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE WINDTUNNEL_TM UNPACKING


/**
 * @brief Get field timestamp from windtunnel_tm message
 *
 * @return [ms] When was this logged
 */
static inline uint64_t mavlink_msg_windtunnel_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field pressure_digital from windtunnel_tm message
 *
 * @return [Pa] Pressure from digital pressure sensor
 */
static inline float mavlink_msg_windtunnel_tm_get_pressure_digital(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pressure_differential from windtunnel_tm message
 *
 * @return [Pa] Differential pressure
 */
static inline float mavlink_msg_windtunnel_tm_get_pressure_differential(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pressure_static from windtunnel_tm message
 *
 * @return [Pa] Pressure from static port
 */
static inline float mavlink_msg_windtunnel_tm_get_pressure_static(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field pressure_dpl from windtunnel_tm message
 *
 * @return [Pa] Pressure from deployment vane
 */
static inline double mavlink_msg_windtunnel_tm_get_pressure_dpl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field ab_angle from windtunnel_tm message
 *
 * @return [deg] Current aerobrake angle
 */
static inline float mavlink_msg_windtunnel_tm_get_ab_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field wind_speed from windtunnel_tm message
 *
 * @return [m/s] Current tunnel wind speed
 */
static inline float mavlink_msg_windtunnel_tm_get_wind_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field log_status from windtunnel_tm message
 *
 * @return  Logger status
 */
static inline float mavlink_msg_windtunnel_tm_get_log_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field log_num from windtunnel_tm message
 *
 * @return  Logger file number
 */
static inline float mavlink_msg_windtunnel_tm_get_log_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field last_RSSI from windtunnel_tm message
 *
 * @return [dBm] Last message RSSI
 */
static inline float mavlink_msg_windtunnel_tm_get_last_RSSI(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a windtunnel_tm message into a struct
 *
 * @param msg The message to decode
 * @param windtunnel_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_windtunnel_tm_decode(const mavlink_message_t* msg, mavlink_windtunnel_tm_t* windtunnel_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    windtunnel_tm->timestamp = mavlink_msg_windtunnel_tm_get_timestamp(msg);
    windtunnel_tm->pressure_dpl = mavlink_msg_windtunnel_tm_get_pressure_dpl(msg);
    windtunnel_tm->pressure_digital = mavlink_msg_windtunnel_tm_get_pressure_digital(msg);
    windtunnel_tm->pressure_differential = mavlink_msg_windtunnel_tm_get_pressure_differential(msg);
    windtunnel_tm->pressure_static = mavlink_msg_windtunnel_tm_get_pressure_static(msg);
    windtunnel_tm->ab_angle = mavlink_msg_windtunnel_tm_get_ab_angle(msg);
    windtunnel_tm->wind_speed = mavlink_msg_windtunnel_tm_get_wind_speed(msg);
    windtunnel_tm->log_status = mavlink_msg_windtunnel_tm_get_log_status(msg);
    windtunnel_tm->log_num = mavlink_msg_windtunnel_tm_get_log_num(msg);
    windtunnel_tm->last_RSSI = mavlink_msg_windtunnel_tm_get_last_RSSI(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN? msg->len : MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN;
        memset(windtunnel_tm, 0, MAVLINK_MSG_ID_WINDTUNNEL_TM_LEN);
    memcpy(windtunnel_tm, _MAV_PAYLOAD(msg), len);
#endif
}
