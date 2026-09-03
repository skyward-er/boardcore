#pragma once
// MESSAGE TEST_TM PACKING

#define MAVLINK_MSG_ID_TEST_TM 182

MAVPACKED(
typedef struct __mavlink_test_tm_t {
 uint64_t timestamp; /*< [ms] When was this logged (mission clock)*/
 float pressure_hw; /*<  Raw pressure from the HoneyWell barometer*/
 float temp_analog; /*<  Analog board temperature*/
 float temp_imu; /*<  IMU board temperature*/
 float battery_volt; /*<  Battery voltage*/
 uint8_t gps_nsats; /*<  GPS num sats*/
}) mavlink_test_tm_t;

#define MAVLINK_MSG_ID_TEST_TM_LEN 25
#define MAVLINK_MSG_ID_TEST_TM_MIN_LEN 25
#define MAVLINK_MSG_ID_182_LEN 25
#define MAVLINK_MSG_ID_182_MIN_LEN 25

#define MAVLINK_MSG_ID_TEST_TM_CRC 177
#define MAVLINK_MSG_ID_182_CRC 177



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TEST_TM { \
    182, \
    "TEST_TM", \
    6, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_test_tm_t, timestamp) }, \
         { "pressure_hw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_test_tm_t, pressure_hw) }, \
         { "gps_nsats", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_test_tm_t, gps_nsats) }, \
         { "temp_analog", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_test_tm_t, temp_analog) }, \
         { "temp_imu", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_test_tm_t, temp_imu) }, \
         { "battery_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_test_tm_t, battery_volt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TEST_TM { \
    "TEST_TM", \
    6, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_test_tm_t, timestamp) }, \
         { "pressure_hw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_test_tm_t, pressure_hw) }, \
         { "gps_nsats", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_test_tm_t, gps_nsats) }, \
         { "temp_analog", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_test_tm_t, temp_analog) }, \
         { "temp_imu", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_test_tm_t, temp_imu) }, \
         { "battery_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_test_tm_t, battery_volt) }, \
         } \
}
#endif

/**
 * @brief Pack a test_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] When was this logged (mission clock)
 * @param pressure_hw  Raw pressure from the HoneyWell barometer
 * @param gps_nsats  GPS num sats
 * @param temp_analog  Analog board temperature
 * @param temp_imu  IMU board temperature
 * @param battery_volt  Battery voltage
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_test_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float pressure_hw, uint8_t gps_nsats, float temp_analog, float temp_imu, float battery_volt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TEST_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_hw);
    _mav_put_float(buf, 12, temp_analog);
    _mav_put_float(buf, 16, temp_imu);
    _mav_put_float(buf, 20, battery_volt);
    _mav_put_uint8_t(buf, 24, gps_nsats);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TEST_TM_LEN);
#else
    mavlink_test_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_hw = pressure_hw;
    packet.temp_analog = temp_analog;
    packet.temp_imu = temp_imu;
    packet.battery_volt = battery_volt;
    packet.gps_nsats = gps_nsats;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TEST_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TEST_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TEST_TM_MIN_LEN, MAVLINK_MSG_ID_TEST_TM_LEN, MAVLINK_MSG_ID_TEST_TM_CRC);
}

/**
 * @brief Pack a test_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] When was this logged (mission clock)
 * @param pressure_hw  Raw pressure from the HoneyWell barometer
 * @param gps_nsats  GPS num sats
 * @param temp_analog  Analog board temperature
 * @param temp_imu  IMU board temperature
 * @param battery_volt  Battery voltage
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_test_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float pressure_hw,uint8_t gps_nsats,float temp_analog,float temp_imu,float battery_volt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TEST_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_hw);
    _mav_put_float(buf, 12, temp_analog);
    _mav_put_float(buf, 16, temp_imu);
    _mav_put_float(buf, 20, battery_volt);
    _mav_put_uint8_t(buf, 24, gps_nsats);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TEST_TM_LEN);
#else
    mavlink_test_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_hw = pressure_hw;
    packet.temp_analog = temp_analog;
    packet.temp_imu = temp_imu;
    packet.battery_volt = battery_volt;
    packet.gps_nsats = gps_nsats;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TEST_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TEST_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TEST_TM_MIN_LEN, MAVLINK_MSG_ID_TEST_TM_LEN, MAVLINK_MSG_ID_TEST_TM_CRC);
}

/**
 * @brief Encode a test_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param test_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_test_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_test_tm_t* test_tm)
{
    return mavlink_msg_test_tm_pack(system_id, component_id, msg, test_tm->timestamp, test_tm->pressure_hw, test_tm->gps_nsats, test_tm->temp_analog, test_tm->temp_imu, test_tm->battery_volt);
}

/**
 * @brief Encode a test_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param test_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_test_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_test_tm_t* test_tm)
{
    return mavlink_msg_test_tm_pack_chan(system_id, component_id, chan, msg, test_tm->timestamp, test_tm->pressure_hw, test_tm->gps_nsats, test_tm->temp_analog, test_tm->temp_imu, test_tm->battery_volt);
}

/**
 * @brief Send a test_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] When was this logged (mission clock)
 * @param pressure_hw  Raw pressure from the HoneyWell barometer
 * @param gps_nsats  GPS num sats
 * @param temp_analog  Analog board temperature
 * @param temp_imu  IMU board temperature
 * @param battery_volt  Battery voltage
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_test_tm_send(mavlink_channel_t chan, uint64_t timestamp, float pressure_hw, uint8_t gps_nsats, float temp_analog, float temp_imu, float battery_volt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TEST_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_hw);
    _mav_put_float(buf, 12, temp_analog);
    _mav_put_float(buf, 16, temp_imu);
    _mav_put_float(buf, 20, battery_volt);
    _mav_put_uint8_t(buf, 24, gps_nsats);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_TM, buf, MAVLINK_MSG_ID_TEST_TM_MIN_LEN, MAVLINK_MSG_ID_TEST_TM_LEN, MAVLINK_MSG_ID_TEST_TM_CRC);
#else
    mavlink_test_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_hw = pressure_hw;
    packet.temp_analog = temp_analog;
    packet.temp_imu = temp_imu;
    packet.battery_volt = battery_volt;
    packet.gps_nsats = gps_nsats;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_TM, (const char *)&packet, MAVLINK_MSG_ID_TEST_TM_MIN_LEN, MAVLINK_MSG_ID_TEST_TM_LEN, MAVLINK_MSG_ID_TEST_TM_CRC);
#endif
}

/**
 * @brief Send a test_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_test_tm_send_struct(mavlink_channel_t chan, const mavlink_test_tm_t* test_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_test_tm_send(chan, test_tm->timestamp, test_tm->pressure_hw, test_tm->gps_nsats, test_tm->temp_analog, test_tm->temp_imu, test_tm->battery_volt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_TM, (const char *)test_tm, MAVLINK_MSG_ID_TEST_TM_MIN_LEN, MAVLINK_MSG_ID_TEST_TM_LEN, MAVLINK_MSG_ID_TEST_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_TEST_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_test_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float pressure_hw, uint8_t gps_nsats, float temp_analog, float temp_imu, float battery_volt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_hw);
    _mav_put_float(buf, 12, temp_analog);
    _mav_put_float(buf, 16, temp_imu);
    _mav_put_float(buf, 20, battery_volt);
    _mav_put_uint8_t(buf, 24, gps_nsats);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_TM, buf, MAVLINK_MSG_ID_TEST_TM_MIN_LEN, MAVLINK_MSG_ID_TEST_TM_LEN, MAVLINK_MSG_ID_TEST_TM_CRC);
#else
    mavlink_test_tm_t *packet = (mavlink_test_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->pressure_hw = pressure_hw;
    packet->temp_analog = temp_analog;
    packet->temp_imu = temp_imu;
    packet->battery_volt = battery_volt;
    packet->gps_nsats = gps_nsats;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_TM, (const char *)packet, MAVLINK_MSG_ID_TEST_TM_MIN_LEN, MAVLINK_MSG_ID_TEST_TM_LEN, MAVLINK_MSG_ID_TEST_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE TEST_TM UNPACKING


/**
 * @brief Get field timestamp from test_tm message
 *
 * @return [ms] When was this logged (mission clock)
 */
static inline uint64_t mavlink_msg_test_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field pressure_hw from test_tm message
 *
 * @return  Raw pressure from the HoneyWell barometer
 */
static inline float mavlink_msg_test_tm_get_pressure_hw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field gps_nsats from test_tm message
 *
 * @return  GPS num sats
 */
static inline uint8_t mavlink_msg_test_tm_get_gps_nsats(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field temp_analog from test_tm message
 *
 * @return  Analog board temperature
 */
static inline float mavlink_msg_test_tm_get_temp_analog(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field temp_imu from test_tm message
 *
 * @return  IMU board temperature
 */
static inline float mavlink_msg_test_tm_get_temp_imu(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field battery_volt from test_tm message
 *
 * @return  Battery voltage
 */
static inline float mavlink_msg_test_tm_get_battery_volt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a test_tm message into a struct
 *
 * @param msg The message to decode
 * @param test_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_test_tm_decode(const mavlink_message_t* msg, mavlink_test_tm_t* test_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    test_tm->timestamp = mavlink_msg_test_tm_get_timestamp(msg);
    test_tm->pressure_hw = mavlink_msg_test_tm_get_pressure_hw(msg);
    test_tm->temp_analog = mavlink_msg_test_tm_get_temp_analog(msg);
    test_tm->temp_imu = mavlink_msg_test_tm_get_temp_imu(msg);
    test_tm->battery_volt = mavlink_msg_test_tm_get_battery_volt(msg);
    test_tm->gps_nsats = mavlink_msg_test_tm_get_gps_nsats(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TEST_TM_LEN? msg->len : MAVLINK_MSG_ID_TEST_TM_LEN;
        memset(test_tm, 0, MAVLINK_MSG_ID_TEST_TM_LEN);
    memcpy(test_tm, _MAV_PAYLOAD(msg), len);
#endif
}
