#pragma once
// MESSAGE ADC_TM PACKING

#define MAVLINK_MSG_ID_ADC_TM 171

MAVPACKED(
typedef struct __mavlink_adc_tm_t {
 uint64_t timestamp; /*< [ms] When was this logged*/
 float pitot_pressure; /*< [Pa] Pitot tube dynamic pressure*/
 float dpl_pressure; /*< [Pa] Deployment vane internal pressure*/
 float static_pressure; /*< [Pa] Static ports pressure*/
 float bat_voltage; /*< [V] LiPo battery voltage*/
 float bat_voltage_5v; /*< [V] 5V power supply*/
}) mavlink_adc_tm_t;

#define MAVLINK_MSG_ID_ADC_TM_LEN 28
#define MAVLINK_MSG_ID_ADC_TM_MIN_LEN 28
#define MAVLINK_MSG_ID_171_LEN 28
#define MAVLINK_MSG_ID_171_MIN_LEN 28

#define MAVLINK_MSG_ID_ADC_TM_CRC 135
#define MAVLINK_MSG_ID_171_CRC 135



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADC_TM { \
    171, \
    "ADC_TM", \
    6, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_adc_tm_t, timestamp) }, \
         { "pitot_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_adc_tm_t, pitot_pressure) }, \
         { "dpl_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_adc_tm_t, dpl_pressure) }, \
         { "static_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_adc_tm_t, static_pressure) }, \
         { "bat_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_adc_tm_t, bat_voltage) }, \
         { "bat_voltage_5v", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_adc_tm_t, bat_voltage_5v) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADC_TM { \
    "ADC_TM", \
    6, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_adc_tm_t, timestamp) }, \
         { "pitot_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_adc_tm_t, pitot_pressure) }, \
         { "dpl_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_adc_tm_t, dpl_pressure) }, \
         { "static_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_adc_tm_t, static_pressure) }, \
         { "bat_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_adc_tm_t, bat_voltage) }, \
         { "bat_voltage_5v", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_adc_tm_t, bat_voltage_5v) }, \
         } \
}
#endif

/**
 * @brief Pack a adc_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] When was this logged
 * @param pitot_pressure [Pa] Pitot tube dynamic pressure
 * @param dpl_pressure [Pa] Deployment vane internal pressure
 * @param static_pressure [Pa] Static ports pressure
 * @param bat_voltage [V] LiPo battery voltage
 * @param bat_voltage_5v [V] 5V power supply
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adc_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float pitot_pressure, float dpl_pressure, float static_pressure, float bat_voltage, float bat_voltage_5v)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADC_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pitot_pressure);
    _mav_put_float(buf, 12, dpl_pressure);
    _mav_put_float(buf, 16, static_pressure);
    _mav_put_float(buf, 20, bat_voltage);
    _mav_put_float(buf, 24, bat_voltage_5v);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADC_TM_LEN);
#else
    mavlink_adc_tm_t packet;
    packet.timestamp = timestamp;
    packet.pitot_pressure = pitot_pressure;
    packet.dpl_pressure = dpl_pressure;
    packet.static_pressure = static_pressure;
    packet.bat_voltage = bat_voltage;
    packet.bat_voltage_5v = bat_voltage_5v;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADC_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADC_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
}

/**
 * @brief Pack a adc_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] When was this logged
 * @param pitot_pressure [Pa] Pitot tube dynamic pressure
 * @param dpl_pressure [Pa] Deployment vane internal pressure
 * @param static_pressure [Pa] Static ports pressure
 * @param bat_voltage [V] LiPo battery voltage
 * @param bat_voltage_5v [V] 5V power supply
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adc_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float pitot_pressure,float dpl_pressure,float static_pressure,float bat_voltage,float bat_voltage_5v)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADC_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pitot_pressure);
    _mav_put_float(buf, 12, dpl_pressure);
    _mav_put_float(buf, 16, static_pressure);
    _mav_put_float(buf, 20, bat_voltage);
    _mav_put_float(buf, 24, bat_voltage_5v);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADC_TM_LEN);
#else
    mavlink_adc_tm_t packet;
    packet.timestamp = timestamp;
    packet.pitot_pressure = pitot_pressure;
    packet.dpl_pressure = dpl_pressure;
    packet.static_pressure = static_pressure;
    packet.bat_voltage = bat_voltage;
    packet.bat_voltage_5v = bat_voltage_5v;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADC_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADC_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
}

/**
 * @brief Encode a adc_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adc_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adc_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adc_tm_t* adc_tm)
{
    return mavlink_msg_adc_tm_pack(system_id, component_id, msg, adc_tm->timestamp, adc_tm->pitot_pressure, adc_tm->dpl_pressure, adc_tm->static_pressure, adc_tm->bat_voltage, adc_tm->bat_voltage_5v);
}

/**
 * @brief Encode a adc_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adc_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adc_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adc_tm_t* adc_tm)
{
    return mavlink_msg_adc_tm_pack_chan(system_id, component_id, chan, msg, adc_tm->timestamp, adc_tm->pitot_pressure, adc_tm->dpl_pressure, adc_tm->static_pressure, adc_tm->bat_voltage, adc_tm->bat_voltage_5v);
}

/**
 * @brief Send a adc_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] When was this logged
 * @param pitot_pressure [Pa] Pitot tube dynamic pressure
 * @param dpl_pressure [Pa] Deployment vane internal pressure
 * @param static_pressure [Pa] Static ports pressure
 * @param bat_voltage [V] LiPo battery voltage
 * @param bat_voltage_5v [V] 5V power supply
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adc_tm_send(mavlink_channel_t chan, uint64_t timestamp, float pitot_pressure, float dpl_pressure, float static_pressure, float bat_voltage, float bat_voltage_5v)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADC_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pitot_pressure);
    _mav_put_float(buf, 12, dpl_pressure);
    _mav_put_float(buf, 16, static_pressure);
    _mav_put_float(buf, 20, bat_voltage);
    _mav_put_float(buf, 24, bat_voltage_5v);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_TM, buf, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
#else
    mavlink_adc_tm_t packet;
    packet.timestamp = timestamp;
    packet.pitot_pressure = pitot_pressure;
    packet.dpl_pressure = dpl_pressure;
    packet.static_pressure = static_pressure;
    packet.bat_voltage = bat_voltage;
    packet.bat_voltage_5v = bat_voltage_5v;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_TM, (const char *)&packet, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
#endif
}

/**
 * @brief Send a adc_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_adc_tm_send_struct(mavlink_channel_t chan, const mavlink_adc_tm_t* adc_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adc_tm_send(chan, adc_tm->timestamp, adc_tm->pitot_pressure, adc_tm->dpl_pressure, adc_tm->static_pressure, adc_tm->bat_voltage, adc_tm->bat_voltage_5v);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_TM, (const char *)adc_tm, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADC_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adc_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float pitot_pressure, float dpl_pressure, float static_pressure, float bat_voltage, float bat_voltage_5v)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pitot_pressure);
    _mav_put_float(buf, 12, dpl_pressure);
    _mav_put_float(buf, 16, static_pressure);
    _mav_put_float(buf, 20, bat_voltage);
    _mav_put_float(buf, 24, bat_voltage_5v);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_TM, buf, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
#else
    mavlink_adc_tm_t *packet = (mavlink_adc_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->pitot_pressure = pitot_pressure;
    packet->dpl_pressure = dpl_pressure;
    packet->static_pressure = static_pressure;
    packet->bat_voltage = bat_voltage;
    packet->bat_voltage_5v = bat_voltage_5v;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_TM, (const char *)packet, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ADC_TM UNPACKING


/**
 * @brief Get field timestamp from adc_tm message
 *
 * @return [ms] When was this logged
 */
static inline uint64_t mavlink_msg_adc_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field pitot_pressure from adc_tm message
 *
 * @return [Pa] Pitot tube dynamic pressure
 */
static inline float mavlink_msg_adc_tm_get_pitot_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field dpl_pressure from adc_tm message
 *
 * @return [Pa] Deployment vane internal pressure
 */
static inline float mavlink_msg_adc_tm_get_dpl_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field static_pressure from adc_tm message
 *
 * @return [Pa] Static ports pressure
 */
static inline float mavlink_msg_adc_tm_get_static_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field bat_voltage from adc_tm message
 *
 * @return [V] LiPo battery voltage
 */
static inline float mavlink_msg_adc_tm_get_bat_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field bat_voltage_5v from adc_tm message
 *
 * @return [V] 5V power supply
 */
static inline float mavlink_msg_adc_tm_get_bat_voltage_5v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a adc_tm message into a struct
 *
 * @param msg The message to decode
 * @param adc_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_adc_tm_decode(const mavlink_message_t* msg, mavlink_adc_tm_t* adc_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    adc_tm->timestamp = mavlink_msg_adc_tm_get_timestamp(msg);
    adc_tm->pitot_pressure = mavlink_msg_adc_tm_get_pitot_pressure(msg);
    adc_tm->dpl_pressure = mavlink_msg_adc_tm_get_dpl_pressure(msg);
    adc_tm->static_pressure = mavlink_msg_adc_tm_get_static_pressure(msg);
    adc_tm->bat_voltage = mavlink_msg_adc_tm_get_bat_voltage(msg);
    adc_tm->bat_voltage_5v = mavlink_msg_adc_tm_get_bat_voltage_5v(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADC_TM_LEN? msg->len : MAVLINK_MSG_ID_ADC_TM_LEN;
        memset(adc_tm, 0, MAVLINK_MSG_ID_ADC_TM_LEN);
    memcpy(adc_tm, _MAV_PAYLOAD(msg), len);
#endif
}
