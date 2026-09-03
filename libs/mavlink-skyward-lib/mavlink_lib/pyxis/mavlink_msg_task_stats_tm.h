#pragma once
// MESSAGE TASK_STATS_TM PACKING

#define MAVLINK_MSG_ID_TASK_STATS_TM 204


typedef struct __mavlink_task_stats_tm_t {
 uint64_t timestamp; /*< [us] When was this logged */
 float task_min; /*<  Task min period*/
 float task_max; /*<  Task max period*/
 float task_mean; /*<  Task mean period*/
 float task_stddev; /*<  Task period std deviation*/
 uint16_t task_period; /*< [ms] Period of the task*/
 uint8_t task_id; /*<  Task ID*/
} mavlink_task_stats_tm_t;

#define MAVLINK_MSG_ID_TASK_STATS_TM_LEN 27
#define MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN 27
#define MAVLINK_MSG_ID_204_LEN 27
#define MAVLINK_MSG_ID_204_MIN_LEN 27

#define MAVLINK_MSG_ID_TASK_STATS_TM_CRC 133
#define MAVLINK_MSG_ID_204_CRC 133



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TASK_STATS_TM { \
    204, \
    "TASK_STATS_TM", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_task_stats_tm_t, timestamp) }, \
         { "task_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_task_stats_tm_t, task_id) }, \
         { "task_period", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_task_stats_tm_t, task_period) }, \
         { "task_min", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_task_stats_tm_t, task_min) }, \
         { "task_max", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_task_stats_tm_t, task_max) }, \
         { "task_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_task_stats_tm_t, task_mean) }, \
         { "task_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_task_stats_tm_t, task_stddev) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TASK_STATS_TM { \
    "TASK_STATS_TM", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_task_stats_tm_t, timestamp) }, \
         { "task_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_task_stats_tm_t, task_id) }, \
         { "task_period", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_task_stats_tm_t, task_period) }, \
         { "task_min", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_task_stats_tm_t, task_min) }, \
         { "task_max", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_task_stats_tm_t, task_max) }, \
         { "task_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_task_stats_tm_t, task_mean) }, \
         { "task_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_task_stats_tm_t, task_stddev) }, \
         } \
}
#endif

/**
 * @brief Pack a task_stats_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] When was this logged 
 * @param task_id  Task ID
 * @param task_period [ms] Period of the task
 * @param task_min  Task min period
 * @param task_max  Task max period
 * @param task_mean  Task mean period
 * @param task_stddev  Task period std deviation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_stats_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t task_id, uint16_t task_period, float task_min, float task_max, float task_mean, float task_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASK_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, task_min);
    _mav_put_float(buf, 12, task_max);
    _mav_put_float(buf, 16, task_mean);
    _mav_put_float(buf, 20, task_stddev);
    _mav_put_uint16_t(buf, 24, task_period);
    _mav_put_uint8_t(buf, 26, task_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
#else
    mavlink_task_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.task_min = task_min;
    packet.task_max = task_max;
    packet.task_mean = task_mean;
    packet.task_stddev = task_stddev;
    packet.task_period = task_period;
    packet.task_id = task_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TASK_STATS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
}

/**
 * @brief Pack a task_stats_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] When was this logged 
 * @param task_id  Task ID
 * @param task_period [ms] Period of the task
 * @param task_min  Task min period
 * @param task_max  Task max period
 * @param task_mean  Task mean period
 * @param task_stddev  Task period std deviation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_stats_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t task_id,uint16_t task_period,float task_min,float task_max,float task_mean,float task_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASK_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, task_min);
    _mav_put_float(buf, 12, task_max);
    _mav_put_float(buf, 16, task_mean);
    _mav_put_float(buf, 20, task_stddev);
    _mav_put_uint16_t(buf, 24, task_period);
    _mav_put_uint8_t(buf, 26, task_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
#else
    mavlink_task_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.task_min = task_min;
    packet.task_max = task_max;
    packet.task_mean = task_mean;
    packet.task_stddev = task_stddev;
    packet.task_period = task_period;
    packet.task_id = task_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TASK_STATS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
}

/**
 * @brief Encode a task_stats_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param task_stats_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_task_stats_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_task_stats_tm_t* task_stats_tm)
{
    return mavlink_msg_task_stats_tm_pack(system_id, component_id, msg, task_stats_tm->timestamp, task_stats_tm->task_id, task_stats_tm->task_period, task_stats_tm->task_min, task_stats_tm->task_max, task_stats_tm->task_mean, task_stats_tm->task_stddev);
}

/**
 * @brief Encode a task_stats_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param task_stats_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_task_stats_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_task_stats_tm_t* task_stats_tm)
{
    return mavlink_msg_task_stats_tm_pack_chan(system_id, component_id, chan, msg, task_stats_tm->timestamp, task_stats_tm->task_id, task_stats_tm->task_period, task_stats_tm->task_min, task_stats_tm->task_max, task_stats_tm->task_mean, task_stats_tm->task_stddev);
}

/**
 * @brief Send a task_stats_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged 
 * @param task_id  Task ID
 * @param task_period [ms] Period of the task
 * @param task_min  Task min period
 * @param task_max  Task max period
 * @param task_mean  Task mean period
 * @param task_stddev  Task period std deviation
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_task_stats_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t task_id, uint16_t task_period, float task_min, float task_max, float task_mean, float task_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASK_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, task_min);
    _mav_put_float(buf, 12, task_max);
    _mav_put_float(buf, 16, task_mean);
    _mav_put_float(buf, 20, task_stddev);
    _mav_put_uint16_t(buf, 24, task_period);
    _mav_put_uint8_t(buf, 26, task_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATS_TM, buf, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
#else
    mavlink_task_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.task_min = task_min;
    packet.task_max = task_max;
    packet.task_mean = task_mean;
    packet.task_stddev = task_stddev;
    packet.task_period = task_period;
    packet.task_id = task_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATS_TM, (const char *)&packet, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
#endif
}

/**
 * @brief Send a task_stats_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_task_stats_tm_send_struct(mavlink_channel_t chan, const mavlink_task_stats_tm_t* task_stats_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_task_stats_tm_send(chan, task_stats_tm->timestamp, task_stats_tm->task_id, task_stats_tm->task_period, task_stats_tm->task_min, task_stats_tm->task_max, task_stats_tm->task_mean, task_stats_tm->task_stddev);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATS_TM, (const char *)task_stats_tm, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_TASK_STATS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_task_stats_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t task_id, uint16_t task_period, float task_min, float task_max, float task_mean, float task_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, task_min);
    _mav_put_float(buf, 12, task_max);
    _mav_put_float(buf, 16, task_mean);
    _mav_put_float(buf, 20, task_stddev);
    _mav_put_uint16_t(buf, 24, task_period);
    _mav_put_uint8_t(buf, 26, task_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATS_TM, buf, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
#else
    mavlink_task_stats_tm_t *packet = (mavlink_task_stats_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->task_min = task_min;
    packet->task_max = task_max;
    packet->task_mean = task_mean;
    packet->task_stddev = task_stddev;
    packet->task_period = task_period;
    packet->task_id = task_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATS_TM, (const char *)packet, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE TASK_STATS_TM UNPACKING


/**
 * @brief Get field timestamp from task_stats_tm message
 *
 * @return [us] When was this logged 
 */
static inline uint64_t mavlink_msg_task_stats_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field task_id from task_stats_tm message
 *
 * @return  Task ID
 */
static inline uint8_t mavlink_msg_task_stats_tm_get_task_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field task_period from task_stats_tm message
 *
 * @return [ms] Period of the task
 */
static inline uint16_t mavlink_msg_task_stats_tm_get_task_period(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field task_min from task_stats_tm message
 *
 * @return  Task min period
 */
static inline float mavlink_msg_task_stats_tm_get_task_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field task_max from task_stats_tm message
 *
 * @return  Task max period
 */
static inline float mavlink_msg_task_stats_tm_get_task_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field task_mean from task_stats_tm message
 *
 * @return  Task mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field task_stddev from task_stats_tm message
 *
 * @return  Task period std deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a task_stats_tm message into a struct
 *
 * @param msg The message to decode
 * @param task_stats_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_task_stats_tm_decode(const mavlink_message_t* msg, mavlink_task_stats_tm_t* task_stats_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    task_stats_tm->timestamp = mavlink_msg_task_stats_tm_get_timestamp(msg);
    task_stats_tm->task_min = mavlink_msg_task_stats_tm_get_task_min(msg);
    task_stats_tm->task_max = mavlink_msg_task_stats_tm_get_task_max(msg);
    task_stats_tm->task_mean = mavlink_msg_task_stats_tm_get_task_mean(msg);
    task_stats_tm->task_stddev = mavlink_msg_task_stats_tm_get_task_stddev(msg);
    task_stats_tm->task_period = mavlink_msg_task_stats_tm_get_task_period(msg);
    task_stats_tm->task_id = mavlink_msg_task_stats_tm_get_task_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TASK_STATS_TM_LEN? msg->len : MAVLINK_MSG_ID_TASK_STATS_TM_LEN;
        memset(task_stats_tm, 0, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
    memcpy(task_stats_tm, _MAV_PAYLOAD(msg), len);
#endif
}
