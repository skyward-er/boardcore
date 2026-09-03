#pragma once
// MESSAGE MS5803_TM PACKING

#define MAVLINK_MSG_ID_MS5803_TM 172

MAVPACKED(
typedef struct __mavlink_ms5803_tm_t {
 uint64_t timestamp; /*< [ms] When was this logged*/
 float pressure; /*< [Pa] Pressure of the digital barometer*/
 float temperature; /*< [deg] Temperature of the digital barometer*/
}) mavlink_ms5803_tm_t;

#define MAVLINK_MSG_ID_MS5803_TM_LEN 16
#define MAVLINK_MSG_ID_MS5803_TM_MIN_LEN 16
#define MAVLINK_MSG_ID_172_LEN 16
#define MAVLINK_MSG_ID_172_MIN_LEN 16

#define MAVLINK_MSG_ID_MS5803_TM_CRC 56
#define MAVLINK_MSG_ID_172_CRC 56



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MS5803_TM { \
    172, \
    "MS5803_TM", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ms5803_tm_t, timestamp) }, \
         { "pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ms5803_tm_t, pressure) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ms5803_tm_t, temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MS5803_TM { \
    "MS5803_TM", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ms5803_tm_t, timestamp) }, \
         { "pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ms5803_tm_t, pressure) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ms5803_tm_t, temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a ms5803_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] When was this logged
 * @param pressure [Pa] Pressure of the digital barometer
 * @param temperature [deg] Temperature of the digital barometer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ms5803_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float pressure, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MS5803_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure);
    _mav_put_float(buf, 12, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MS5803_TM_LEN);
#else
    mavlink_ms5803_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure = pressure;
    packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MS5803_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MS5803_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MS5803_TM_MIN_LEN, MAVLINK_MSG_ID_MS5803_TM_LEN, MAVLINK_MSG_ID_MS5803_TM_CRC);
}

/**
 * @brief Pack a ms5803_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] When was this logged
 * @param pressure [Pa] Pressure of the digital barometer
 * @param temperature [deg] Temperature of the digital barometer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ms5803_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float pressure,float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MS5803_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure);
    _mav_put_float(buf, 12, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MS5803_TM_LEN);
#else
    mavlink_ms5803_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure = pressure;
    packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MS5803_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MS5803_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MS5803_TM_MIN_LEN, MAVLINK_MSG_ID_MS5803_TM_LEN, MAVLINK_MSG_ID_MS5803_TM_CRC);
}

/**
 * @brief Encode a ms5803_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ms5803_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ms5803_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ms5803_tm_t* ms5803_tm)
{
    return mavlink_msg_ms5803_tm_pack(system_id, component_id, msg, ms5803_tm->timestamp, ms5803_tm->pressure, ms5803_tm->temperature);
}

/**
 * @brief Encode a ms5803_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ms5803_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ms5803_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ms5803_tm_t* ms5803_tm)
{
    return mavlink_msg_ms5803_tm_pack_chan(system_id, component_id, chan, msg, ms5803_tm->timestamp, ms5803_tm->pressure, ms5803_tm->temperature);
}

/**
 * @brief Send a ms5803_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] When was this logged
 * @param pressure [Pa] Pressure of the digital barometer
 * @param temperature [deg] Temperature of the digital barometer
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ms5803_tm_send(mavlink_channel_t chan, uint64_t timestamp, float pressure, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MS5803_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure);
    _mav_put_float(buf, 12, temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MS5803_TM, buf, MAVLINK_MSG_ID_MS5803_TM_MIN_LEN, MAVLINK_MSG_ID_MS5803_TM_LEN, MAVLINK_MSG_ID_MS5803_TM_CRC);
#else
    mavlink_ms5803_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure = pressure;
    packet.temperature = temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MS5803_TM, (const char *)&packet, MAVLINK_MSG_ID_MS5803_TM_MIN_LEN, MAVLINK_MSG_ID_MS5803_TM_LEN, MAVLINK_MSG_ID_MS5803_TM_CRC);
#endif
}

/**
 * @brief Send a ms5803_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ms5803_tm_send_struct(mavlink_channel_t chan, const mavlink_ms5803_tm_t* ms5803_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ms5803_tm_send(chan, ms5803_tm->timestamp, ms5803_tm->pressure, ms5803_tm->temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MS5803_TM, (const char *)ms5803_tm, MAVLINK_MSG_ID_MS5803_TM_MIN_LEN, MAVLINK_MSG_ID_MS5803_TM_LEN, MAVLINK_MSG_ID_MS5803_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_MS5803_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ms5803_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float pressure, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure);
    _mav_put_float(buf, 12, temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MS5803_TM, buf, MAVLINK_MSG_ID_MS5803_TM_MIN_LEN, MAVLINK_MSG_ID_MS5803_TM_LEN, MAVLINK_MSG_ID_MS5803_TM_CRC);
#else
    mavlink_ms5803_tm_t *packet = (mavlink_ms5803_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->pressure = pressure;
    packet->temperature = temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MS5803_TM, (const char *)packet, MAVLINK_MSG_ID_MS5803_TM_MIN_LEN, MAVLINK_MSG_ID_MS5803_TM_LEN, MAVLINK_MSG_ID_MS5803_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE MS5803_TM UNPACKING


/**
 * @brief Get field timestamp from ms5803_tm message
 *
 * @return [ms] When was this logged
 */
static inline uint64_t mavlink_msg_ms5803_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field pressure from ms5803_tm message
 *
 * @return [Pa] Pressure of the digital barometer
 */
static inline float mavlink_msg_ms5803_tm_get_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field temperature from ms5803_tm message
 *
 * @return [deg] Temperature of the digital barometer
 */
static inline float mavlink_msg_ms5803_tm_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a ms5803_tm message into a struct
 *
 * @param msg The message to decode
 * @param ms5803_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_ms5803_tm_decode(const mavlink_message_t* msg, mavlink_ms5803_tm_t* ms5803_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ms5803_tm->timestamp = mavlink_msg_ms5803_tm_get_timestamp(msg);
    ms5803_tm->pressure = mavlink_msg_ms5803_tm_get_pressure(msg);
    ms5803_tm->temperature = mavlink_msg_ms5803_tm_get_temperature(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MS5803_TM_LEN? msg->len : MAVLINK_MSG_ID_MS5803_TM_LEN;
        memset(ms5803_tm, 0, MAVLINK_MSG_ID_MS5803_TM_LEN);
    memcpy(ms5803_tm, _MAV_PAYLOAD(msg), len);
#endif
}
