#pragma once
// MESSAGE REFERENCE_TM PACKING

#define MAVLINK_MSG_ID_REFERENCE_TM 115


typedef struct __mavlink_reference_tm_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 float ref_altitude; /*< [m] Reference altitude*/
 float ref_pressure; /*< [Pa] Reference atmospheric pressure*/
 float ref_temperature; /*< [degC] Reference temperature*/
 float ref_latitude; /*< [deg] Reference latitude*/
 float ref_longitude; /*< [deg] Reference longitude*/
 float msl_pressure; /*< [Pa] Mean sea level atmospheric pressure*/
 float msl_temperature; /*< [degC] Mean sea level temperature*/
} mavlink_reference_tm_t;

#define MAVLINK_MSG_ID_REFERENCE_TM_LEN 36
#define MAVLINK_MSG_ID_REFERENCE_TM_MIN_LEN 36
#define MAVLINK_MSG_ID_115_LEN 36
#define MAVLINK_MSG_ID_115_MIN_LEN 36

#define MAVLINK_MSG_ID_REFERENCE_TM_CRC 103
#define MAVLINK_MSG_ID_115_CRC 103



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_REFERENCE_TM { \
    115, \
    "REFERENCE_TM", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_reference_tm_t, timestamp) }, \
         { "ref_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_reference_tm_t, ref_altitude) }, \
         { "ref_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_reference_tm_t, ref_pressure) }, \
         { "ref_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_reference_tm_t, ref_temperature) }, \
         { "ref_latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_reference_tm_t, ref_latitude) }, \
         { "ref_longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_reference_tm_t, ref_longitude) }, \
         { "msl_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_reference_tm_t, msl_pressure) }, \
         { "msl_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_reference_tm_t, msl_temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_REFERENCE_TM { \
    "REFERENCE_TM", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_reference_tm_t, timestamp) }, \
         { "ref_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_reference_tm_t, ref_altitude) }, \
         { "ref_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_reference_tm_t, ref_pressure) }, \
         { "ref_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_reference_tm_t, ref_temperature) }, \
         { "ref_latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_reference_tm_t, ref_latitude) }, \
         { "ref_longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_reference_tm_t, ref_longitude) }, \
         { "msl_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_reference_tm_t, msl_pressure) }, \
         { "msl_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_reference_tm_t, msl_temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a reference_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param ref_altitude [m] Reference altitude
 * @param ref_pressure [Pa] Reference atmospheric pressure
 * @param ref_temperature [degC] Reference temperature
 * @param ref_latitude [deg] Reference latitude
 * @param ref_longitude [deg] Reference longitude
 * @param msl_pressure [Pa] Mean sea level atmospheric pressure
 * @param msl_temperature [degC] Mean sea level temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_reference_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float ref_altitude, float ref_pressure, float ref_temperature, float ref_latitude, float ref_longitude, float msl_pressure, float msl_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REFERENCE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, ref_altitude);
    _mav_put_float(buf, 12, ref_pressure);
    _mav_put_float(buf, 16, ref_temperature);
    _mav_put_float(buf, 20, ref_latitude);
    _mav_put_float(buf, 24, ref_longitude);
    _mav_put_float(buf, 28, msl_pressure);
    _mav_put_float(buf, 32, msl_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REFERENCE_TM_LEN);
#else
    mavlink_reference_tm_t packet;
    packet.timestamp = timestamp;
    packet.ref_altitude = ref_altitude;
    packet.ref_pressure = ref_pressure;
    packet.ref_temperature = ref_temperature;
    packet.ref_latitude = ref_latitude;
    packet.ref_longitude = ref_longitude;
    packet.msl_pressure = msl_pressure;
    packet.msl_temperature = msl_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REFERENCE_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REFERENCE_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REFERENCE_TM_MIN_LEN, MAVLINK_MSG_ID_REFERENCE_TM_LEN, MAVLINK_MSG_ID_REFERENCE_TM_CRC);
}

/**
 * @brief Pack a reference_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param ref_altitude [m] Reference altitude
 * @param ref_pressure [Pa] Reference atmospheric pressure
 * @param ref_temperature [degC] Reference temperature
 * @param ref_latitude [deg] Reference latitude
 * @param ref_longitude [deg] Reference longitude
 * @param msl_pressure [Pa] Mean sea level atmospheric pressure
 * @param msl_temperature [degC] Mean sea level temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_reference_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float ref_altitude,float ref_pressure,float ref_temperature,float ref_latitude,float ref_longitude,float msl_pressure,float msl_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REFERENCE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, ref_altitude);
    _mav_put_float(buf, 12, ref_pressure);
    _mav_put_float(buf, 16, ref_temperature);
    _mav_put_float(buf, 20, ref_latitude);
    _mav_put_float(buf, 24, ref_longitude);
    _mav_put_float(buf, 28, msl_pressure);
    _mav_put_float(buf, 32, msl_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REFERENCE_TM_LEN);
#else
    mavlink_reference_tm_t packet;
    packet.timestamp = timestamp;
    packet.ref_altitude = ref_altitude;
    packet.ref_pressure = ref_pressure;
    packet.ref_temperature = ref_temperature;
    packet.ref_latitude = ref_latitude;
    packet.ref_longitude = ref_longitude;
    packet.msl_pressure = msl_pressure;
    packet.msl_temperature = msl_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REFERENCE_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REFERENCE_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REFERENCE_TM_MIN_LEN, MAVLINK_MSG_ID_REFERENCE_TM_LEN, MAVLINK_MSG_ID_REFERENCE_TM_CRC);
}

/**
 * @brief Encode a reference_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param reference_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_reference_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_reference_tm_t* reference_tm)
{
    return mavlink_msg_reference_tm_pack(system_id, component_id, msg, reference_tm->timestamp, reference_tm->ref_altitude, reference_tm->ref_pressure, reference_tm->ref_temperature, reference_tm->ref_latitude, reference_tm->ref_longitude, reference_tm->msl_pressure, reference_tm->msl_temperature);
}

/**
 * @brief Encode a reference_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param reference_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_reference_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_reference_tm_t* reference_tm)
{
    return mavlink_msg_reference_tm_pack_chan(system_id, component_id, chan, msg, reference_tm->timestamp, reference_tm->ref_altitude, reference_tm->ref_pressure, reference_tm->ref_temperature, reference_tm->ref_latitude, reference_tm->ref_longitude, reference_tm->msl_pressure, reference_tm->msl_temperature);
}

/**
 * @brief Send a reference_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param ref_altitude [m] Reference altitude
 * @param ref_pressure [Pa] Reference atmospheric pressure
 * @param ref_temperature [degC] Reference temperature
 * @param ref_latitude [deg] Reference latitude
 * @param ref_longitude [deg] Reference longitude
 * @param msl_pressure [Pa] Mean sea level atmospheric pressure
 * @param msl_temperature [degC] Mean sea level temperature
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_reference_tm_send(mavlink_channel_t chan, uint64_t timestamp, float ref_altitude, float ref_pressure, float ref_temperature, float ref_latitude, float ref_longitude, float msl_pressure, float msl_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REFERENCE_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, ref_altitude);
    _mav_put_float(buf, 12, ref_pressure);
    _mav_put_float(buf, 16, ref_temperature);
    _mav_put_float(buf, 20, ref_latitude);
    _mav_put_float(buf, 24, ref_longitude);
    _mav_put_float(buf, 28, msl_pressure);
    _mav_put_float(buf, 32, msl_temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REFERENCE_TM, buf, MAVLINK_MSG_ID_REFERENCE_TM_MIN_LEN, MAVLINK_MSG_ID_REFERENCE_TM_LEN, MAVLINK_MSG_ID_REFERENCE_TM_CRC);
#else
    mavlink_reference_tm_t packet;
    packet.timestamp = timestamp;
    packet.ref_altitude = ref_altitude;
    packet.ref_pressure = ref_pressure;
    packet.ref_temperature = ref_temperature;
    packet.ref_latitude = ref_latitude;
    packet.ref_longitude = ref_longitude;
    packet.msl_pressure = msl_pressure;
    packet.msl_temperature = msl_temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REFERENCE_TM, (const char *)&packet, MAVLINK_MSG_ID_REFERENCE_TM_MIN_LEN, MAVLINK_MSG_ID_REFERENCE_TM_LEN, MAVLINK_MSG_ID_REFERENCE_TM_CRC);
#endif
}

/**
 * @brief Send a reference_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_reference_tm_send_struct(mavlink_channel_t chan, const mavlink_reference_tm_t* reference_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_reference_tm_send(chan, reference_tm->timestamp, reference_tm->ref_altitude, reference_tm->ref_pressure, reference_tm->ref_temperature, reference_tm->ref_latitude, reference_tm->ref_longitude, reference_tm->msl_pressure, reference_tm->msl_temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REFERENCE_TM, (const char *)reference_tm, MAVLINK_MSG_ID_REFERENCE_TM_MIN_LEN, MAVLINK_MSG_ID_REFERENCE_TM_LEN, MAVLINK_MSG_ID_REFERENCE_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_REFERENCE_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_reference_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float ref_altitude, float ref_pressure, float ref_temperature, float ref_latitude, float ref_longitude, float msl_pressure, float msl_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, ref_altitude);
    _mav_put_float(buf, 12, ref_pressure);
    _mav_put_float(buf, 16, ref_temperature);
    _mav_put_float(buf, 20, ref_latitude);
    _mav_put_float(buf, 24, ref_longitude);
    _mav_put_float(buf, 28, msl_pressure);
    _mav_put_float(buf, 32, msl_temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REFERENCE_TM, buf, MAVLINK_MSG_ID_REFERENCE_TM_MIN_LEN, MAVLINK_MSG_ID_REFERENCE_TM_LEN, MAVLINK_MSG_ID_REFERENCE_TM_CRC);
#else
    mavlink_reference_tm_t *packet = (mavlink_reference_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->ref_altitude = ref_altitude;
    packet->ref_pressure = ref_pressure;
    packet->ref_temperature = ref_temperature;
    packet->ref_latitude = ref_latitude;
    packet->ref_longitude = ref_longitude;
    packet->msl_pressure = msl_pressure;
    packet->msl_temperature = msl_temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REFERENCE_TM, (const char *)packet, MAVLINK_MSG_ID_REFERENCE_TM_MIN_LEN, MAVLINK_MSG_ID_REFERENCE_TM_LEN, MAVLINK_MSG_ID_REFERENCE_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE REFERENCE_TM UNPACKING


/**
 * @brief Get field timestamp from reference_tm message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_reference_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field ref_altitude from reference_tm message
 *
 * @return [m] Reference altitude
 */
static inline float mavlink_msg_reference_tm_get_ref_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field ref_pressure from reference_tm message
 *
 * @return [Pa] Reference atmospheric pressure
 */
static inline float mavlink_msg_reference_tm_get_ref_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field ref_temperature from reference_tm message
 *
 * @return [degC] Reference temperature
 */
static inline float mavlink_msg_reference_tm_get_ref_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field ref_latitude from reference_tm message
 *
 * @return [deg] Reference latitude
 */
static inline float mavlink_msg_reference_tm_get_ref_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ref_longitude from reference_tm message
 *
 * @return [deg] Reference longitude
 */
static inline float mavlink_msg_reference_tm_get_ref_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field msl_pressure from reference_tm message
 *
 * @return [Pa] Mean sea level atmospheric pressure
 */
static inline float mavlink_msg_reference_tm_get_msl_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field msl_temperature from reference_tm message
 *
 * @return [degC] Mean sea level temperature
 */
static inline float mavlink_msg_reference_tm_get_msl_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a reference_tm message into a struct
 *
 * @param msg The message to decode
 * @param reference_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_reference_tm_decode(const mavlink_message_t* msg, mavlink_reference_tm_t* reference_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    reference_tm->timestamp = mavlink_msg_reference_tm_get_timestamp(msg);
    reference_tm->ref_altitude = mavlink_msg_reference_tm_get_ref_altitude(msg);
    reference_tm->ref_pressure = mavlink_msg_reference_tm_get_ref_pressure(msg);
    reference_tm->ref_temperature = mavlink_msg_reference_tm_get_ref_temperature(msg);
    reference_tm->ref_latitude = mavlink_msg_reference_tm_get_ref_latitude(msg);
    reference_tm->ref_longitude = mavlink_msg_reference_tm_get_ref_longitude(msg);
    reference_tm->msl_pressure = mavlink_msg_reference_tm_get_msl_pressure(msg);
    reference_tm->msl_temperature = mavlink_msg_reference_tm_get_msl_temperature(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_REFERENCE_TM_LEN? msg->len : MAVLINK_MSG_ID_REFERENCE_TM_LEN;
        memset(reference_tm, 0, MAVLINK_MSG_ID_REFERENCE_TM_LEN);
    memcpy(reference_tm, _MAV_PAYLOAD(msg), len);
#endif
}
