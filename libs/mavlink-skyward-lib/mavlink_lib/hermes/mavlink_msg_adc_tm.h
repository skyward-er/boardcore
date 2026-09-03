#pragma once
// MESSAGE ADC_TM PACKING

#define MAVLINK_MSG_ID_ADC_TM 169

MAVPACKED(
typedef struct __mavlink_adc_tm_t {
 uint64_t timestamp; /*<  When was this logged*/
 float hw_baro_volt; /*<  Honeywell barometer voltage*/
 float nxp_baro_volt; /*<  NXP barometer battery_voltage*/
 float hw_baro_pressure; /*<  Honeywell barometer measured pressure*/
 float nxp_baro_pressure; /*<  NXP barometer measured pressure*/
 float battery_voltage; /*<  Battery voltage*/
 float current_sense_1; /*<  Current Sense 1*/
 float current_sense_2; /*<  Current Sense 2*/
 uint8_t hw_baro_flag; /*<  Honeywell barometer flag*/
 uint8_t nxp_baro_flag; /*<  NXP barometer flag*/
}) mavlink_adc_tm_t;

#define MAVLINK_MSG_ID_ADC_TM_LEN 38
#define MAVLINK_MSG_ID_ADC_TM_MIN_LEN 38
#define MAVLINK_MSG_ID_169_LEN 38
#define MAVLINK_MSG_ID_169_MIN_LEN 38

#define MAVLINK_MSG_ID_ADC_TM_CRC 230
#define MAVLINK_MSG_ID_169_CRC 230



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADC_TM { \
    169, \
    "ADC_TM", \
    10, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_adc_tm_t, timestamp) }, \
         { "hw_baro_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_adc_tm_t, hw_baro_volt) }, \
         { "nxp_baro_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_adc_tm_t, nxp_baro_volt) }, \
         { "hw_baro_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_adc_tm_t, hw_baro_flag) }, \
         { "nxp_baro_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_adc_tm_t, nxp_baro_flag) }, \
         { "hw_baro_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_adc_tm_t, hw_baro_pressure) }, \
         { "nxp_baro_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_adc_tm_t, nxp_baro_pressure) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_adc_tm_t, battery_voltage) }, \
         { "current_sense_1", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_adc_tm_t, current_sense_1) }, \
         { "current_sense_2", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_adc_tm_t, current_sense_2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADC_TM { \
    "ADC_TM", \
    10, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_adc_tm_t, timestamp) }, \
         { "hw_baro_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_adc_tm_t, hw_baro_volt) }, \
         { "nxp_baro_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_adc_tm_t, nxp_baro_volt) }, \
         { "hw_baro_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_adc_tm_t, hw_baro_flag) }, \
         { "nxp_baro_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_adc_tm_t, nxp_baro_flag) }, \
         { "hw_baro_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_adc_tm_t, hw_baro_pressure) }, \
         { "nxp_baro_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_adc_tm_t, nxp_baro_pressure) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_adc_tm_t, battery_voltage) }, \
         { "current_sense_1", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_adc_tm_t, current_sense_1) }, \
         { "current_sense_2", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_adc_tm_t, current_sense_2) }, \
         } \
}
#endif

/**
 * @brief Pack a adc_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  When was this logged
 * @param hw_baro_volt  Honeywell barometer voltage
 * @param nxp_baro_volt  NXP barometer battery_voltage
 * @param hw_baro_flag  Honeywell barometer flag
 * @param nxp_baro_flag  NXP barometer flag
 * @param hw_baro_pressure  Honeywell barometer measured pressure
 * @param nxp_baro_pressure  NXP barometer measured pressure
 * @param battery_voltage  Battery voltage
 * @param current_sense_1  Current Sense 1
 * @param current_sense_2  Current Sense 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adc_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float hw_baro_volt, float nxp_baro_volt, uint8_t hw_baro_flag, uint8_t nxp_baro_flag, float hw_baro_pressure, float nxp_baro_pressure, float battery_voltage, float current_sense_1, float current_sense_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADC_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, hw_baro_volt);
    _mav_put_float(buf, 12, nxp_baro_volt);
    _mav_put_float(buf, 16, hw_baro_pressure);
    _mav_put_float(buf, 20, nxp_baro_pressure);
    _mav_put_float(buf, 24, battery_voltage);
    _mav_put_float(buf, 28, current_sense_1);
    _mav_put_float(buf, 32, current_sense_2);
    _mav_put_uint8_t(buf, 36, hw_baro_flag);
    _mav_put_uint8_t(buf, 37, nxp_baro_flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADC_TM_LEN);
#else
    mavlink_adc_tm_t packet;
    packet.timestamp = timestamp;
    packet.hw_baro_volt = hw_baro_volt;
    packet.nxp_baro_volt = nxp_baro_volt;
    packet.hw_baro_pressure = hw_baro_pressure;
    packet.nxp_baro_pressure = nxp_baro_pressure;
    packet.battery_voltage = battery_voltage;
    packet.current_sense_1 = current_sense_1;
    packet.current_sense_2 = current_sense_2;
    packet.hw_baro_flag = hw_baro_flag;
    packet.nxp_baro_flag = nxp_baro_flag;

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
 * @param timestamp  When was this logged
 * @param hw_baro_volt  Honeywell barometer voltage
 * @param nxp_baro_volt  NXP barometer battery_voltage
 * @param hw_baro_flag  Honeywell barometer flag
 * @param nxp_baro_flag  NXP barometer flag
 * @param hw_baro_pressure  Honeywell barometer measured pressure
 * @param nxp_baro_pressure  NXP barometer measured pressure
 * @param battery_voltage  Battery voltage
 * @param current_sense_1  Current Sense 1
 * @param current_sense_2  Current Sense 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adc_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float hw_baro_volt,float nxp_baro_volt,uint8_t hw_baro_flag,uint8_t nxp_baro_flag,float hw_baro_pressure,float nxp_baro_pressure,float battery_voltage,float current_sense_1,float current_sense_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADC_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, hw_baro_volt);
    _mav_put_float(buf, 12, nxp_baro_volt);
    _mav_put_float(buf, 16, hw_baro_pressure);
    _mav_put_float(buf, 20, nxp_baro_pressure);
    _mav_put_float(buf, 24, battery_voltage);
    _mav_put_float(buf, 28, current_sense_1);
    _mav_put_float(buf, 32, current_sense_2);
    _mav_put_uint8_t(buf, 36, hw_baro_flag);
    _mav_put_uint8_t(buf, 37, nxp_baro_flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADC_TM_LEN);
#else
    mavlink_adc_tm_t packet;
    packet.timestamp = timestamp;
    packet.hw_baro_volt = hw_baro_volt;
    packet.nxp_baro_volt = nxp_baro_volt;
    packet.hw_baro_pressure = hw_baro_pressure;
    packet.nxp_baro_pressure = nxp_baro_pressure;
    packet.battery_voltage = battery_voltage;
    packet.current_sense_1 = current_sense_1;
    packet.current_sense_2 = current_sense_2;
    packet.hw_baro_flag = hw_baro_flag;
    packet.nxp_baro_flag = nxp_baro_flag;

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
    return mavlink_msg_adc_tm_pack(system_id, component_id, msg, adc_tm->timestamp, adc_tm->hw_baro_volt, adc_tm->nxp_baro_volt, adc_tm->hw_baro_flag, adc_tm->nxp_baro_flag, adc_tm->hw_baro_pressure, adc_tm->nxp_baro_pressure, adc_tm->battery_voltage, adc_tm->current_sense_1, adc_tm->current_sense_2);
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
    return mavlink_msg_adc_tm_pack_chan(system_id, component_id, chan, msg, adc_tm->timestamp, adc_tm->hw_baro_volt, adc_tm->nxp_baro_volt, adc_tm->hw_baro_flag, adc_tm->nxp_baro_flag, adc_tm->hw_baro_pressure, adc_tm->nxp_baro_pressure, adc_tm->battery_voltage, adc_tm->current_sense_1, adc_tm->current_sense_2);
}

/**
 * @brief Send a adc_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  When was this logged
 * @param hw_baro_volt  Honeywell barometer voltage
 * @param nxp_baro_volt  NXP barometer battery_voltage
 * @param hw_baro_flag  Honeywell barometer flag
 * @param nxp_baro_flag  NXP barometer flag
 * @param hw_baro_pressure  Honeywell barometer measured pressure
 * @param nxp_baro_pressure  NXP barometer measured pressure
 * @param battery_voltage  Battery voltage
 * @param current_sense_1  Current Sense 1
 * @param current_sense_2  Current Sense 2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adc_tm_send(mavlink_channel_t chan, uint64_t timestamp, float hw_baro_volt, float nxp_baro_volt, uint8_t hw_baro_flag, uint8_t nxp_baro_flag, float hw_baro_pressure, float nxp_baro_pressure, float battery_voltage, float current_sense_1, float current_sense_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADC_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, hw_baro_volt);
    _mav_put_float(buf, 12, nxp_baro_volt);
    _mav_put_float(buf, 16, hw_baro_pressure);
    _mav_put_float(buf, 20, nxp_baro_pressure);
    _mav_put_float(buf, 24, battery_voltage);
    _mav_put_float(buf, 28, current_sense_1);
    _mav_put_float(buf, 32, current_sense_2);
    _mav_put_uint8_t(buf, 36, hw_baro_flag);
    _mav_put_uint8_t(buf, 37, nxp_baro_flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_TM, buf, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
#else
    mavlink_adc_tm_t packet;
    packet.timestamp = timestamp;
    packet.hw_baro_volt = hw_baro_volt;
    packet.nxp_baro_volt = nxp_baro_volt;
    packet.hw_baro_pressure = hw_baro_pressure;
    packet.nxp_baro_pressure = nxp_baro_pressure;
    packet.battery_voltage = battery_voltage;
    packet.current_sense_1 = current_sense_1;
    packet.current_sense_2 = current_sense_2;
    packet.hw_baro_flag = hw_baro_flag;
    packet.nxp_baro_flag = nxp_baro_flag;

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
    mavlink_msg_adc_tm_send(chan, adc_tm->timestamp, adc_tm->hw_baro_volt, adc_tm->nxp_baro_volt, adc_tm->hw_baro_flag, adc_tm->nxp_baro_flag, adc_tm->hw_baro_pressure, adc_tm->nxp_baro_pressure, adc_tm->battery_voltage, adc_tm->current_sense_1, adc_tm->current_sense_2);
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
static inline void mavlink_msg_adc_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float hw_baro_volt, float nxp_baro_volt, uint8_t hw_baro_flag, uint8_t nxp_baro_flag, float hw_baro_pressure, float nxp_baro_pressure, float battery_voltage, float current_sense_1, float current_sense_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, hw_baro_volt);
    _mav_put_float(buf, 12, nxp_baro_volt);
    _mav_put_float(buf, 16, hw_baro_pressure);
    _mav_put_float(buf, 20, nxp_baro_pressure);
    _mav_put_float(buf, 24, battery_voltage);
    _mav_put_float(buf, 28, current_sense_1);
    _mav_put_float(buf, 32, current_sense_2);
    _mav_put_uint8_t(buf, 36, hw_baro_flag);
    _mav_put_uint8_t(buf, 37, nxp_baro_flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_TM, buf, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
#else
    mavlink_adc_tm_t *packet = (mavlink_adc_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->hw_baro_volt = hw_baro_volt;
    packet->nxp_baro_volt = nxp_baro_volt;
    packet->hw_baro_pressure = hw_baro_pressure;
    packet->nxp_baro_pressure = nxp_baro_pressure;
    packet->battery_voltage = battery_voltage;
    packet->current_sense_1 = current_sense_1;
    packet->current_sense_2 = current_sense_2;
    packet->hw_baro_flag = hw_baro_flag;
    packet->nxp_baro_flag = nxp_baro_flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_TM, (const char *)packet, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ADC_TM UNPACKING


/**
 * @brief Get field timestamp from adc_tm message
 *
 * @return  When was this logged
 */
static inline uint64_t mavlink_msg_adc_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field hw_baro_volt from adc_tm message
 *
 * @return  Honeywell barometer voltage
 */
static inline float mavlink_msg_adc_tm_get_hw_baro_volt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field nxp_baro_volt from adc_tm message
 *
 * @return  NXP barometer battery_voltage
 */
static inline float mavlink_msg_adc_tm_get_nxp_baro_volt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field hw_baro_flag from adc_tm message
 *
 * @return  Honeywell barometer flag
 */
static inline uint8_t mavlink_msg_adc_tm_get_hw_baro_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field nxp_baro_flag from adc_tm message
 *
 * @return  NXP barometer flag
 */
static inline uint8_t mavlink_msg_adc_tm_get_nxp_baro_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field hw_baro_pressure from adc_tm message
 *
 * @return  Honeywell barometer measured pressure
 */
static inline float mavlink_msg_adc_tm_get_hw_baro_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field nxp_baro_pressure from adc_tm message
 *
 * @return  NXP barometer measured pressure
 */
static inline float mavlink_msg_adc_tm_get_nxp_baro_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field battery_voltage from adc_tm message
 *
 * @return  Battery voltage
 */
static inline float mavlink_msg_adc_tm_get_battery_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field current_sense_1 from adc_tm message
 *
 * @return  Current Sense 1
 */
static inline float mavlink_msg_adc_tm_get_current_sense_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field current_sense_2 from adc_tm message
 *
 * @return  Current Sense 2
 */
static inline float mavlink_msg_adc_tm_get_current_sense_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
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
    adc_tm->hw_baro_volt = mavlink_msg_adc_tm_get_hw_baro_volt(msg);
    adc_tm->nxp_baro_volt = mavlink_msg_adc_tm_get_nxp_baro_volt(msg);
    adc_tm->hw_baro_pressure = mavlink_msg_adc_tm_get_hw_baro_pressure(msg);
    adc_tm->nxp_baro_pressure = mavlink_msg_adc_tm_get_nxp_baro_pressure(msg);
    adc_tm->battery_voltage = mavlink_msg_adc_tm_get_battery_voltage(msg);
    adc_tm->current_sense_1 = mavlink_msg_adc_tm_get_current_sense_1(msg);
    adc_tm->current_sense_2 = mavlink_msg_adc_tm_get_current_sense_2(msg);
    adc_tm->hw_baro_flag = mavlink_msg_adc_tm_get_hw_baro_flag(msg);
    adc_tm->nxp_baro_flag = mavlink_msg_adc_tm_get_nxp_baro_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADC_TM_LEN? msg->len : MAVLINK_MSG_ID_ADC_TM_LEN;
        memset(adc_tm, 0, MAVLINK_MSG_ID_ADC_TM_LEN);
    memcpy(adc_tm, _MAV_PAYLOAD(msg), len);
#endif
}
