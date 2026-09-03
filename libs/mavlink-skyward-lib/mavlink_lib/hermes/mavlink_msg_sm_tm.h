#pragma once
// MESSAGE SM_TM PACKING

#define MAVLINK_MSG_ID_SM_TM 164

MAVPACKED(
typedef struct __mavlink_sm_tm_t {
 uint64_t timestamp; /*<  When was this logged */
 float task_10hz_min; /*<  10 Hz task min period*/
 float task_10hz_max; /*<  10 Hz task max period*/
 float task_10hz_mean; /*<  10 Hz task mean period*/
 float task_10hz_stddev; /*<  10 Hz task period std deviation*/
 float task_20hz_min; /*<  20 Hz task min period*/
 float task_20hz_max; /*<  20 Hz task max period*/
 float task_20hz_mean; /*<  20 Hz task mean period*/
 float task_20hz_stddev; /*<  20 Hz task period std deviation*/
 float task_250hz_min; /*<  250 Hz task min period*/
 float task_250hz_max; /*<  250 Hz task max period*/
 float task_250hz_mean; /*<  250 Hz task mean period*/
 float task_250hz_stddev; /*<  250 Hz task period std deviation*/
 uint16_t sensor_state; /*<  Sensors init + self check result*/
 uint8_t state; /*<  Sensor manager state*/
}) mavlink_sm_tm_t;

#define MAVLINK_MSG_ID_SM_TM_LEN 59
#define MAVLINK_MSG_ID_SM_TM_MIN_LEN 59
#define MAVLINK_MSG_ID_164_LEN 59
#define MAVLINK_MSG_ID_164_MIN_LEN 59

#define MAVLINK_MSG_ID_SM_TM_CRC 154
#define MAVLINK_MSG_ID_164_CRC 154



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SM_TM { \
    164, \
    "SM_TM", \
    15, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sm_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 58, offsetof(mavlink_sm_tm_t, state) }, \
         { "sensor_state", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_sm_tm_t, sensor_state) }, \
         { "task_10hz_min", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sm_tm_t, task_10hz_min) }, \
         { "task_10hz_max", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sm_tm_t, task_10hz_max) }, \
         { "task_10hz_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sm_tm_t, task_10hz_mean) }, \
         { "task_10hz_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sm_tm_t, task_10hz_stddev) }, \
         { "task_20hz_min", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sm_tm_t, task_20hz_min) }, \
         { "task_20hz_max", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sm_tm_t, task_20hz_max) }, \
         { "task_20hz_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sm_tm_t, task_20hz_mean) }, \
         { "task_20hz_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sm_tm_t, task_20hz_stddev) }, \
         { "task_250hz_min", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sm_tm_t, task_250hz_min) }, \
         { "task_250hz_max", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_sm_tm_t, task_250hz_max) }, \
         { "task_250hz_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_sm_tm_t, task_250hz_mean) }, \
         { "task_250hz_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_sm_tm_t, task_250hz_stddev) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SM_TM { \
    "SM_TM", \
    15, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sm_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 58, offsetof(mavlink_sm_tm_t, state) }, \
         { "sensor_state", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_sm_tm_t, sensor_state) }, \
         { "task_10hz_min", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sm_tm_t, task_10hz_min) }, \
         { "task_10hz_max", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sm_tm_t, task_10hz_max) }, \
         { "task_10hz_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sm_tm_t, task_10hz_mean) }, \
         { "task_10hz_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sm_tm_t, task_10hz_stddev) }, \
         { "task_20hz_min", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sm_tm_t, task_20hz_min) }, \
         { "task_20hz_max", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sm_tm_t, task_20hz_max) }, \
         { "task_20hz_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sm_tm_t, task_20hz_mean) }, \
         { "task_20hz_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sm_tm_t, task_20hz_stddev) }, \
         { "task_250hz_min", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sm_tm_t, task_250hz_min) }, \
         { "task_250hz_max", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_sm_tm_t, task_250hz_max) }, \
         { "task_250hz_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_sm_tm_t, task_250hz_mean) }, \
         { "task_250hz_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_sm_tm_t, task_250hz_stddev) }, \
         } \
}
#endif

/**
 * @brief Pack a sm_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  When was this logged 
 * @param state  Sensor manager state
 * @param sensor_state  Sensors init + self check result
 * @param task_10hz_min  10 Hz task min period
 * @param task_10hz_max  10 Hz task max period
 * @param task_10hz_mean  10 Hz task mean period
 * @param task_10hz_stddev  10 Hz task period std deviation
 * @param task_20hz_min  20 Hz task min period
 * @param task_20hz_max  20 Hz task max period
 * @param task_20hz_mean  20 Hz task mean period
 * @param task_20hz_stddev  20 Hz task period std deviation
 * @param task_250hz_min  250 Hz task min period
 * @param task_250hz_max  250 Hz task max period
 * @param task_250hz_mean  250 Hz task mean period
 * @param task_250hz_stddev  250 Hz task period std deviation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sm_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t state, uint16_t sensor_state, float task_10hz_min, float task_10hz_max, float task_10hz_mean, float task_10hz_stddev, float task_20hz_min, float task_20hz_max, float task_20hz_mean, float task_20hz_stddev, float task_250hz_min, float task_250hz_max, float task_250hz_mean, float task_250hz_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SM_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, task_10hz_min);
    _mav_put_float(buf, 12, task_10hz_max);
    _mav_put_float(buf, 16, task_10hz_mean);
    _mav_put_float(buf, 20, task_10hz_stddev);
    _mav_put_float(buf, 24, task_20hz_min);
    _mav_put_float(buf, 28, task_20hz_max);
    _mav_put_float(buf, 32, task_20hz_mean);
    _mav_put_float(buf, 36, task_20hz_stddev);
    _mav_put_float(buf, 40, task_250hz_min);
    _mav_put_float(buf, 44, task_250hz_max);
    _mav_put_float(buf, 48, task_250hz_mean);
    _mav_put_float(buf, 52, task_250hz_stddev);
    _mav_put_uint16_t(buf, 56, sensor_state);
    _mav_put_uint8_t(buf, 58, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SM_TM_LEN);
#else
    mavlink_sm_tm_t packet;
    packet.timestamp = timestamp;
    packet.task_10hz_min = task_10hz_min;
    packet.task_10hz_max = task_10hz_max;
    packet.task_10hz_mean = task_10hz_mean;
    packet.task_10hz_stddev = task_10hz_stddev;
    packet.task_20hz_min = task_20hz_min;
    packet.task_20hz_max = task_20hz_max;
    packet.task_20hz_mean = task_20hz_mean;
    packet.task_20hz_stddev = task_20hz_stddev;
    packet.task_250hz_min = task_250hz_min;
    packet.task_250hz_max = task_250hz_max;
    packet.task_250hz_mean = task_250hz_mean;
    packet.task_250hz_stddev = task_250hz_stddev;
    packet.sensor_state = sensor_state;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SM_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SM_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SM_TM_MIN_LEN, MAVLINK_MSG_ID_SM_TM_LEN, MAVLINK_MSG_ID_SM_TM_CRC);
}

/**
 * @brief Pack a sm_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  When was this logged 
 * @param state  Sensor manager state
 * @param sensor_state  Sensors init + self check result
 * @param task_10hz_min  10 Hz task min period
 * @param task_10hz_max  10 Hz task max period
 * @param task_10hz_mean  10 Hz task mean period
 * @param task_10hz_stddev  10 Hz task period std deviation
 * @param task_20hz_min  20 Hz task min period
 * @param task_20hz_max  20 Hz task max period
 * @param task_20hz_mean  20 Hz task mean period
 * @param task_20hz_stddev  20 Hz task period std deviation
 * @param task_250hz_min  250 Hz task min period
 * @param task_250hz_max  250 Hz task max period
 * @param task_250hz_mean  250 Hz task mean period
 * @param task_250hz_stddev  250 Hz task period std deviation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sm_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t state,uint16_t sensor_state,float task_10hz_min,float task_10hz_max,float task_10hz_mean,float task_10hz_stddev,float task_20hz_min,float task_20hz_max,float task_20hz_mean,float task_20hz_stddev,float task_250hz_min,float task_250hz_max,float task_250hz_mean,float task_250hz_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SM_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, task_10hz_min);
    _mav_put_float(buf, 12, task_10hz_max);
    _mav_put_float(buf, 16, task_10hz_mean);
    _mav_put_float(buf, 20, task_10hz_stddev);
    _mav_put_float(buf, 24, task_20hz_min);
    _mav_put_float(buf, 28, task_20hz_max);
    _mav_put_float(buf, 32, task_20hz_mean);
    _mav_put_float(buf, 36, task_20hz_stddev);
    _mav_put_float(buf, 40, task_250hz_min);
    _mav_put_float(buf, 44, task_250hz_max);
    _mav_put_float(buf, 48, task_250hz_mean);
    _mav_put_float(buf, 52, task_250hz_stddev);
    _mav_put_uint16_t(buf, 56, sensor_state);
    _mav_put_uint8_t(buf, 58, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SM_TM_LEN);
#else
    mavlink_sm_tm_t packet;
    packet.timestamp = timestamp;
    packet.task_10hz_min = task_10hz_min;
    packet.task_10hz_max = task_10hz_max;
    packet.task_10hz_mean = task_10hz_mean;
    packet.task_10hz_stddev = task_10hz_stddev;
    packet.task_20hz_min = task_20hz_min;
    packet.task_20hz_max = task_20hz_max;
    packet.task_20hz_mean = task_20hz_mean;
    packet.task_20hz_stddev = task_20hz_stddev;
    packet.task_250hz_min = task_250hz_min;
    packet.task_250hz_max = task_250hz_max;
    packet.task_250hz_mean = task_250hz_mean;
    packet.task_250hz_stddev = task_250hz_stddev;
    packet.sensor_state = sensor_state;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SM_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SM_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SM_TM_MIN_LEN, MAVLINK_MSG_ID_SM_TM_LEN, MAVLINK_MSG_ID_SM_TM_CRC);
}

/**
 * @brief Encode a sm_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sm_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sm_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sm_tm_t* sm_tm)
{
    return mavlink_msg_sm_tm_pack(system_id, component_id, msg, sm_tm->timestamp, sm_tm->state, sm_tm->sensor_state, sm_tm->task_10hz_min, sm_tm->task_10hz_max, sm_tm->task_10hz_mean, sm_tm->task_10hz_stddev, sm_tm->task_20hz_min, sm_tm->task_20hz_max, sm_tm->task_20hz_mean, sm_tm->task_20hz_stddev, sm_tm->task_250hz_min, sm_tm->task_250hz_max, sm_tm->task_250hz_mean, sm_tm->task_250hz_stddev);
}

/**
 * @brief Encode a sm_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sm_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sm_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sm_tm_t* sm_tm)
{
    return mavlink_msg_sm_tm_pack_chan(system_id, component_id, chan, msg, sm_tm->timestamp, sm_tm->state, sm_tm->sensor_state, sm_tm->task_10hz_min, sm_tm->task_10hz_max, sm_tm->task_10hz_mean, sm_tm->task_10hz_stddev, sm_tm->task_20hz_min, sm_tm->task_20hz_max, sm_tm->task_20hz_mean, sm_tm->task_20hz_stddev, sm_tm->task_250hz_min, sm_tm->task_250hz_max, sm_tm->task_250hz_mean, sm_tm->task_250hz_stddev);
}

/**
 * @brief Send a sm_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  When was this logged 
 * @param state  Sensor manager state
 * @param sensor_state  Sensors init + self check result
 * @param task_10hz_min  10 Hz task min period
 * @param task_10hz_max  10 Hz task max period
 * @param task_10hz_mean  10 Hz task mean period
 * @param task_10hz_stddev  10 Hz task period std deviation
 * @param task_20hz_min  20 Hz task min period
 * @param task_20hz_max  20 Hz task max period
 * @param task_20hz_mean  20 Hz task mean period
 * @param task_20hz_stddev  20 Hz task period std deviation
 * @param task_250hz_min  250 Hz task min period
 * @param task_250hz_max  250 Hz task max period
 * @param task_250hz_mean  250 Hz task mean period
 * @param task_250hz_stddev  250 Hz task period std deviation
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sm_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t state, uint16_t sensor_state, float task_10hz_min, float task_10hz_max, float task_10hz_mean, float task_10hz_stddev, float task_20hz_min, float task_20hz_max, float task_20hz_mean, float task_20hz_stddev, float task_250hz_min, float task_250hz_max, float task_250hz_mean, float task_250hz_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SM_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, task_10hz_min);
    _mav_put_float(buf, 12, task_10hz_max);
    _mav_put_float(buf, 16, task_10hz_mean);
    _mav_put_float(buf, 20, task_10hz_stddev);
    _mav_put_float(buf, 24, task_20hz_min);
    _mav_put_float(buf, 28, task_20hz_max);
    _mav_put_float(buf, 32, task_20hz_mean);
    _mav_put_float(buf, 36, task_20hz_stddev);
    _mav_put_float(buf, 40, task_250hz_min);
    _mav_put_float(buf, 44, task_250hz_max);
    _mav_put_float(buf, 48, task_250hz_mean);
    _mav_put_float(buf, 52, task_250hz_stddev);
    _mav_put_uint16_t(buf, 56, sensor_state);
    _mav_put_uint8_t(buf, 58, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SM_TM, buf, MAVLINK_MSG_ID_SM_TM_MIN_LEN, MAVLINK_MSG_ID_SM_TM_LEN, MAVLINK_MSG_ID_SM_TM_CRC);
#else
    mavlink_sm_tm_t packet;
    packet.timestamp = timestamp;
    packet.task_10hz_min = task_10hz_min;
    packet.task_10hz_max = task_10hz_max;
    packet.task_10hz_mean = task_10hz_mean;
    packet.task_10hz_stddev = task_10hz_stddev;
    packet.task_20hz_min = task_20hz_min;
    packet.task_20hz_max = task_20hz_max;
    packet.task_20hz_mean = task_20hz_mean;
    packet.task_20hz_stddev = task_20hz_stddev;
    packet.task_250hz_min = task_250hz_min;
    packet.task_250hz_max = task_250hz_max;
    packet.task_250hz_mean = task_250hz_mean;
    packet.task_250hz_stddev = task_250hz_stddev;
    packet.sensor_state = sensor_state;
    packet.state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SM_TM, (const char *)&packet, MAVLINK_MSG_ID_SM_TM_MIN_LEN, MAVLINK_MSG_ID_SM_TM_LEN, MAVLINK_MSG_ID_SM_TM_CRC);
#endif
}

/**
 * @brief Send a sm_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sm_tm_send_struct(mavlink_channel_t chan, const mavlink_sm_tm_t* sm_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sm_tm_send(chan, sm_tm->timestamp, sm_tm->state, sm_tm->sensor_state, sm_tm->task_10hz_min, sm_tm->task_10hz_max, sm_tm->task_10hz_mean, sm_tm->task_10hz_stddev, sm_tm->task_20hz_min, sm_tm->task_20hz_max, sm_tm->task_20hz_mean, sm_tm->task_20hz_stddev, sm_tm->task_250hz_min, sm_tm->task_250hz_max, sm_tm->task_250hz_mean, sm_tm->task_250hz_stddev);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SM_TM, (const char *)sm_tm, MAVLINK_MSG_ID_SM_TM_MIN_LEN, MAVLINK_MSG_ID_SM_TM_LEN, MAVLINK_MSG_ID_SM_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_SM_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sm_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t state, uint16_t sensor_state, float task_10hz_min, float task_10hz_max, float task_10hz_mean, float task_10hz_stddev, float task_20hz_min, float task_20hz_max, float task_20hz_mean, float task_20hz_stddev, float task_250hz_min, float task_250hz_max, float task_250hz_mean, float task_250hz_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, task_10hz_min);
    _mav_put_float(buf, 12, task_10hz_max);
    _mav_put_float(buf, 16, task_10hz_mean);
    _mav_put_float(buf, 20, task_10hz_stddev);
    _mav_put_float(buf, 24, task_20hz_min);
    _mav_put_float(buf, 28, task_20hz_max);
    _mav_put_float(buf, 32, task_20hz_mean);
    _mav_put_float(buf, 36, task_20hz_stddev);
    _mav_put_float(buf, 40, task_250hz_min);
    _mav_put_float(buf, 44, task_250hz_max);
    _mav_put_float(buf, 48, task_250hz_mean);
    _mav_put_float(buf, 52, task_250hz_stddev);
    _mav_put_uint16_t(buf, 56, sensor_state);
    _mav_put_uint8_t(buf, 58, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SM_TM, buf, MAVLINK_MSG_ID_SM_TM_MIN_LEN, MAVLINK_MSG_ID_SM_TM_LEN, MAVLINK_MSG_ID_SM_TM_CRC);
#else
    mavlink_sm_tm_t *packet = (mavlink_sm_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->task_10hz_min = task_10hz_min;
    packet->task_10hz_max = task_10hz_max;
    packet->task_10hz_mean = task_10hz_mean;
    packet->task_10hz_stddev = task_10hz_stddev;
    packet->task_20hz_min = task_20hz_min;
    packet->task_20hz_max = task_20hz_max;
    packet->task_20hz_mean = task_20hz_mean;
    packet->task_20hz_stddev = task_20hz_stddev;
    packet->task_250hz_min = task_250hz_min;
    packet->task_250hz_max = task_250hz_max;
    packet->task_250hz_mean = task_250hz_mean;
    packet->task_250hz_stddev = task_250hz_stddev;
    packet->sensor_state = sensor_state;
    packet->state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SM_TM, (const char *)packet, MAVLINK_MSG_ID_SM_TM_MIN_LEN, MAVLINK_MSG_ID_SM_TM_LEN, MAVLINK_MSG_ID_SM_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE SM_TM UNPACKING


/**
 * @brief Get field timestamp from sm_tm message
 *
 * @return  When was this logged 
 */
static inline uint64_t mavlink_msg_sm_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field state from sm_tm message
 *
 * @return  Sensor manager state
 */
static inline uint8_t mavlink_msg_sm_tm_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  58);
}

/**
 * @brief Get field sensor_state from sm_tm message
 *
 * @return  Sensors init + self check result
 */
static inline uint16_t mavlink_msg_sm_tm_get_sensor_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  56);
}

/**
 * @brief Get field task_10hz_min from sm_tm message
 *
 * @return  10 Hz task min period
 */
static inline float mavlink_msg_sm_tm_get_task_10hz_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field task_10hz_max from sm_tm message
 *
 * @return  10 Hz task max period
 */
static inline float mavlink_msg_sm_tm_get_task_10hz_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field task_10hz_mean from sm_tm message
 *
 * @return  10 Hz task mean period
 */
static inline float mavlink_msg_sm_tm_get_task_10hz_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field task_10hz_stddev from sm_tm message
 *
 * @return  10 Hz task period std deviation
 */
static inline float mavlink_msg_sm_tm_get_task_10hz_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field task_20hz_min from sm_tm message
 *
 * @return  20 Hz task min period
 */
static inline float mavlink_msg_sm_tm_get_task_20hz_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field task_20hz_max from sm_tm message
 *
 * @return  20 Hz task max period
 */
static inline float mavlink_msg_sm_tm_get_task_20hz_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field task_20hz_mean from sm_tm message
 *
 * @return  20 Hz task mean period
 */
static inline float mavlink_msg_sm_tm_get_task_20hz_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field task_20hz_stddev from sm_tm message
 *
 * @return  20 Hz task period std deviation
 */
static inline float mavlink_msg_sm_tm_get_task_20hz_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field task_250hz_min from sm_tm message
 *
 * @return  250 Hz task min period
 */
static inline float mavlink_msg_sm_tm_get_task_250hz_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field task_250hz_max from sm_tm message
 *
 * @return  250 Hz task max period
 */
static inline float mavlink_msg_sm_tm_get_task_250hz_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field task_250hz_mean from sm_tm message
 *
 * @return  250 Hz task mean period
 */
static inline float mavlink_msg_sm_tm_get_task_250hz_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field task_250hz_stddev from sm_tm message
 *
 * @return  250 Hz task period std deviation
 */
static inline float mavlink_msg_sm_tm_get_task_250hz_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Decode a sm_tm message into a struct
 *
 * @param msg The message to decode
 * @param sm_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_sm_tm_decode(const mavlink_message_t* msg, mavlink_sm_tm_t* sm_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sm_tm->timestamp = mavlink_msg_sm_tm_get_timestamp(msg);
    sm_tm->task_10hz_min = mavlink_msg_sm_tm_get_task_10hz_min(msg);
    sm_tm->task_10hz_max = mavlink_msg_sm_tm_get_task_10hz_max(msg);
    sm_tm->task_10hz_mean = mavlink_msg_sm_tm_get_task_10hz_mean(msg);
    sm_tm->task_10hz_stddev = mavlink_msg_sm_tm_get_task_10hz_stddev(msg);
    sm_tm->task_20hz_min = mavlink_msg_sm_tm_get_task_20hz_min(msg);
    sm_tm->task_20hz_max = mavlink_msg_sm_tm_get_task_20hz_max(msg);
    sm_tm->task_20hz_mean = mavlink_msg_sm_tm_get_task_20hz_mean(msg);
    sm_tm->task_20hz_stddev = mavlink_msg_sm_tm_get_task_20hz_stddev(msg);
    sm_tm->task_250hz_min = mavlink_msg_sm_tm_get_task_250hz_min(msg);
    sm_tm->task_250hz_max = mavlink_msg_sm_tm_get_task_250hz_max(msg);
    sm_tm->task_250hz_mean = mavlink_msg_sm_tm_get_task_250hz_mean(msg);
    sm_tm->task_250hz_stddev = mavlink_msg_sm_tm_get_task_250hz_stddev(msg);
    sm_tm->sensor_state = mavlink_msg_sm_tm_get_sensor_state(msg);
    sm_tm->state = mavlink_msg_sm_tm_get_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SM_TM_LEN? msg->len : MAVLINK_MSG_ID_SM_TM_LEN;
        memset(sm_tm, 0, MAVLINK_MSG_ID_SM_TM_LEN);
    memcpy(sm_tm, _MAV_PAYLOAD(msg), len);
#endif
}
