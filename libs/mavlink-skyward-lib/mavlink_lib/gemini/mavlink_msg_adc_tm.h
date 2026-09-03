#pragma once
// MESSAGE ADC_TM PACKING

#define MAVLINK_MSG_ID_ADC_TM 105


typedef struct __mavlink_adc_tm_t {
 uint64_t timestamp; /*< [us] When was this logged*/
 float channel_0; /*< [V] ADC voltage measured on channel 0*/
 float channel_1; /*< [V] ADC voltage measured on channel 1*/
 float channel_2; /*< [V] ADC voltage measured on channel 2*/
 float channel_3; /*< [V] ADC voltage measured on channel 3*/
 float channel_4; /*< [V] ADC voltage measured on channel 4*/
 float channel_5; /*< [V] ADC voltage measured on channel 5*/
 float channel_6; /*< [V] ADC voltage measured on channel 6*/
 float channel_7; /*< [V] ADC voltage measured on channel 7*/
 char sensor_name[20]; /*<  Sensor name*/
} mavlink_adc_tm_t;

#define MAVLINK_MSG_ID_ADC_TM_LEN 60
#define MAVLINK_MSG_ID_ADC_TM_MIN_LEN 60
#define MAVLINK_MSG_ID_105_LEN 60
#define MAVLINK_MSG_ID_105_MIN_LEN 60

#define MAVLINK_MSG_ID_ADC_TM_CRC 229
#define MAVLINK_MSG_ID_105_CRC 229

#define MAVLINK_MSG_ADC_TM_FIELD_SENSOR_NAME_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADC_TM { \
    105, \
    "ADC_TM", \
    10, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_adc_tm_t, timestamp) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 20, 40, offsetof(mavlink_adc_tm_t, sensor_name) }, \
         { "channel_0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_adc_tm_t, channel_0) }, \
         { "channel_1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_adc_tm_t, channel_1) }, \
         { "channel_2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_adc_tm_t, channel_2) }, \
         { "channel_3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_adc_tm_t, channel_3) }, \
         { "channel_4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_adc_tm_t, channel_4) }, \
         { "channel_5", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_adc_tm_t, channel_5) }, \
         { "channel_6", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_adc_tm_t, channel_6) }, \
         { "channel_7", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_adc_tm_t, channel_7) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADC_TM { \
    "ADC_TM", \
    10, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_adc_tm_t, timestamp) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 20, 40, offsetof(mavlink_adc_tm_t, sensor_name) }, \
         { "channel_0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_adc_tm_t, channel_0) }, \
         { "channel_1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_adc_tm_t, channel_1) }, \
         { "channel_2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_adc_tm_t, channel_2) }, \
         { "channel_3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_adc_tm_t, channel_3) }, \
         { "channel_4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_adc_tm_t, channel_4) }, \
         { "channel_5", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_adc_tm_t, channel_5) }, \
         { "channel_6", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_adc_tm_t, channel_6) }, \
         { "channel_7", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_adc_tm_t, channel_7) }, \
         } \
}
#endif

/**
 * @brief Pack a adc_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] When was this logged
 * @param sensor_name  Sensor name
 * @param channel_0 [V] ADC voltage measured on channel 0
 * @param channel_1 [V] ADC voltage measured on channel 1
 * @param channel_2 [V] ADC voltage measured on channel 2
 * @param channel_3 [V] ADC voltage measured on channel 3
 * @param channel_4 [V] ADC voltage measured on channel 4
 * @param channel_5 [V] ADC voltage measured on channel 5
 * @param channel_6 [V] ADC voltage measured on channel 6
 * @param channel_7 [V] ADC voltage measured on channel 7
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adc_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, const char *sensor_name, float channel_0, float channel_1, float channel_2, float channel_3, float channel_4, float channel_5, float channel_6, float channel_7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADC_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, channel_0);
    _mav_put_float(buf, 12, channel_1);
    _mav_put_float(buf, 16, channel_2);
    _mav_put_float(buf, 20, channel_3);
    _mav_put_float(buf, 24, channel_4);
    _mav_put_float(buf, 28, channel_5);
    _mav_put_float(buf, 32, channel_6);
    _mav_put_float(buf, 36, channel_7);
    _mav_put_char_array(buf, 40, sensor_name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADC_TM_LEN);
#else
    mavlink_adc_tm_t packet;
    packet.timestamp = timestamp;
    packet.channel_0 = channel_0;
    packet.channel_1 = channel_1;
    packet.channel_2 = channel_2;
    packet.channel_3 = channel_3;
    packet.channel_4 = channel_4;
    packet.channel_5 = channel_5;
    packet.channel_6 = channel_6;
    packet.channel_7 = channel_7;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
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
 * @param timestamp [us] When was this logged
 * @param sensor_name  Sensor name
 * @param channel_0 [V] ADC voltage measured on channel 0
 * @param channel_1 [V] ADC voltage measured on channel 1
 * @param channel_2 [V] ADC voltage measured on channel 2
 * @param channel_3 [V] ADC voltage measured on channel 3
 * @param channel_4 [V] ADC voltage measured on channel 4
 * @param channel_5 [V] ADC voltage measured on channel 5
 * @param channel_6 [V] ADC voltage measured on channel 6
 * @param channel_7 [V] ADC voltage measured on channel 7
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adc_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,const char *sensor_name,float channel_0,float channel_1,float channel_2,float channel_3,float channel_4,float channel_5,float channel_6,float channel_7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADC_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, channel_0);
    _mav_put_float(buf, 12, channel_1);
    _mav_put_float(buf, 16, channel_2);
    _mav_put_float(buf, 20, channel_3);
    _mav_put_float(buf, 24, channel_4);
    _mav_put_float(buf, 28, channel_5);
    _mav_put_float(buf, 32, channel_6);
    _mav_put_float(buf, 36, channel_7);
    _mav_put_char_array(buf, 40, sensor_name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADC_TM_LEN);
#else
    mavlink_adc_tm_t packet;
    packet.timestamp = timestamp;
    packet.channel_0 = channel_0;
    packet.channel_1 = channel_1;
    packet.channel_2 = channel_2;
    packet.channel_3 = channel_3;
    packet.channel_4 = channel_4;
    packet.channel_5 = channel_5;
    packet.channel_6 = channel_6;
    packet.channel_7 = channel_7;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
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
    return mavlink_msg_adc_tm_pack(system_id, component_id, msg, adc_tm->timestamp, adc_tm->sensor_name, adc_tm->channel_0, adc_tm->channel_1, adc_tm->channel_2, adc_tm->channel_3, adc_tm->channel_4, adc_tm->channel_5, adc_tm->channel_6, adc_tm->channel_7);
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
    return mavlink_msg_adc_tm_pack_chan(system_id, component_id, chan, msg, adc_tm->timestamp, adc_tm->sensor_name, adc_tm->channel_0, adc_tm->channel_1, adc_tm->channel_2, adc_tm->channel_3, adc_tm->channel_4, adc_tm->channel_5, adc_tm->channel_6, adc_tm->channel_7);
}

/**
 * @brief Send a adc_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged
 * @param sensor_name  Sensor name
 * @param channel_0 [V] ADC voltage measured on channel 0
 * @param channel_1 [V] ADC voltage measured on channel 1
 * @param channel_2 [V] ADC voltage measured on channel 2
 * @param channel_3 [V] ADC voltage measured on channel 3
 * @param channel_4 [V] ADC voltage measured on channel 4
 * @param channel_5 [V] ADC voltage measured on channel 5
 * @param channel_6 [V] ADC voltage measured on channel 6
 * @param channel_7 [V] ADC voltage measured on channel 7
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adc_tm_send(mavlink_channel_t chan, uint64_t timestamp, const char *sensor_name, float channel_0, float channel_1, float channel_2, float channel_3, float channel_4, float channel_5, float channel_6, float channel_7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADC_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, channel_0);
    _mav_put_float(buf, 12, channel_1);
    _mav_put_float(buf, 16, channel_2);
    _mav_put_float(buf, 20, channel_3);
    _mav_put_float(buf, 24, channel_4);
    _mav_put_float(buf, 28, channel_5);
    _mav_put_float(buf, 32, channel_6);
    _mav_put_float(buf, 36, channel_7);
    _mav_put_char_array(buf, 40, sensor_name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_TM, buf, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
#else
    mavlink_adc_tm_t packet;
    packet.timestamp = timestamp;
    packet.channel_0 = channel_0;
    packet.channel_1 = channel_1;
    packet.channel_2 = channel_2;
    packet.channel_3 = channel_3;
    packet.channel_4 = channel_4;
    packet.channel_5 = channel_5;
    packet.channel_6 = channel_6;
    packet.channel_7 = channel_7;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
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
    mavlink_msg_adc_tm_send(chan, adc_tm->timestamp, adc_tm->sensor_name, adc_tm->channel_0, adc_tm->channel_1, adc_tm->channel_2, adc_tm->channel_3, adc_tm->channel_4, adc_tm->channel_5, adc_tm->channel_6, adc_tm->channel_7);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_TM, (const char *)adc_tm, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADC_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adc_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, const char *sensor_name, float channel_0, float channel_1, float channel_2, float channel_3, float channel_4, float channel_5, float channel_6, float channel_7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, channel_0);
    _mav_put_float(buf, 12, channel_1);
    _mav_put_float(buf, 16, channel_2);
    _mav_put_float(buf, 20, channel_3);
    _mav_put_float(buf, 24, channel_4);
    _mav_put_float(buf, 28, channel_5);
    _mav_put_float(buf, 32, channel_6);
    _mav_put_float(buf, 36, channel_7);
    _mav_put_char_array(buf, 40, sensor_name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_TM, buf, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
#else
    mavlink_adc_tm_t *packet = (mavlink_adc_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->channel_0 = channel_0;
    packet->channel_1 = channel_1;
    packet->channel_2 = channel_2;
    packet->channel_3 = channel_3;
    packet->channel_4 = channel_4;
    packet->channel_5 = channel_5;
    packet->channel_6 = channel_6;
    packet->channel_7 = channel_7;
    mav_array_memcpy(packet->sensor_name, sensor_name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_TM, (const char *)packet, MAVLINK_MSG_ID_ADC_TM_MIN_LEN, MAVLINK_MSG_ID_ADC_TM_LEN, MAVLINK_MSG_ID_ADC_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ADC_TM UNPACKING


/**
 * @brief Get field timestamp from adc_tm message
 *
 * @return [us] When was this logged
 */
static inline uint64_t mavlink_msg_adc_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sensor_name from adc_tm message
 *
 * @return  Sensor name
 */
static inline uint16_t mavlink_msg_adc_tm_get_sensor_name(const mavlink_message_t* msg, char *sensor_name)
{
    return _MAV_RETURN_char_array(msg, sensor_name, 20,  40);
}

/**
 * @brief Get field channel_0 from adc_tm message
 *
 * @return [V] ADC voltage measured on channel 0
 */
static inline float mavlink_msg_adc_tm_get_channel_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field channel_1 from adc_tm message
 *
 * @return [V] ADC voltage measured on channel 1
 */
static inline float mavlink_msg_adc_tm_get_channel_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field channel_2 from adc_tm message
 *
 * @return [V] ADC voltage measured on channel 2
 */
static inline float mavlink_msg_adc_tm_get_channel_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field channel_3 from adc_tm message
 *
 * @return [V] ADC voltage measured on channel 3
 */
static inline float mavlink_msg_adc_tm_get_channel_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field channel_4 from adc_tm message
 *
 * @return [V] ADC voltage measured on channel 4
 */
static inline float mavlink_msg_adc_tm_get_channel_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field channel_5 from adc_tm message
 *
 * @return [V] ADC voltage measured on channel 5
 */
static inline float mavlink_msg_adc_tm_get_channel_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field channel_6 from adc_tm message
 *
 * @return [V] ADC voltage measured on channel 6
 */
static inline float mavlink_msg_adc_tm_get_channel_6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field channel_7 from adc_tm message
 *
 * @return [V] ADC voltage measured on channel 7
 */
static inline float mavlink_msg_adc_tm_get_channel_7(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
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
    adc_tm->channel_0 = mavlink_msg_adc_tm_get_channel_0(msg);
    adc_tm->channel_1 = mavlink_msg_adc_tm_get_channel_1(msg);
    adc_tm->channel_2 = mavlink_msg_adc_tm_get_channel_2(msg);
    adc_tm->channel_3 = mavlink_msg_adc_tm_get_channel_3(msg);
    adc_tm->channel_4 = mavlink_msg_adc_tm_get_channel_4(msg);
    adc_tm->channel_5 = mavlink_msg_adc_tm_get_channel_5(msg);
    adc_tm->channel_6 = mavlink_msg_adc_tm_get_channel_6(msg);
    adc_tm->channel_7 = mavlink_msg_adc_tm_get_channel_7(msg);
    mavlink_msg_adc_tm_get_sensor_name(msg, adc_tm->sensor_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADC_TM_LEN? msg->len : MAVLINK_MSG_ID_ADC_TM_LEN;
        memset(adc_tm, 0, MAVLINK_MSG_ID_ADC_TM_LEN);
    memcpy(adc_tm, _MAV_PAYLOAD(msg), len);
#endif
}
