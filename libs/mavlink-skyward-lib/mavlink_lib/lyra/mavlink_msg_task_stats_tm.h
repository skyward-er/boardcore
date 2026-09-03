#pragma once
// MESSAGE TASK_STATS_TM PACKING

#define MAVLINK_MSG_ID_TASK_STATS_TM 204


typedef struct __mavlink_task_stats_tm_t {
 uint64_t timestamp; /*< [us] When was this logged */
 uint32_t task_missed_events; /*<  Number of missed events*/
 uint32_t task_failed_events; /*<  Number of missed events*/
 float task_activation_mean; /*<  Task activation mean period*/
 float task_activation_stddev; /*<  Task activation mean standard deviation*/
 float task_period_mean; /*<  Task period mean period*/
 float task_period_stddev; /*<  Task period mean standard deviation*/
 float task_workload_mean; /*<  Task workload mean period*/
 float task_workload_stddev; /*<  Task workload mean standard deviation*/
 uint16_t task_id; /*<  Task ID*/
 uint16_t task_period; /*< [ns] Period of the task*/
} mavlink_task_stats_tm_t;

#define MAVLINK_MSG_ID_TASK_STATS_TM_LEN 44
#define MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN 44
#define MAVLINK_MSG_ID_204_LEN 44
#define MAVLINK_MSG_ID_204_MIN_LEN 44

#define MAVLINK_MSG_ID_TASK_STATS_TM_CRC 19
#define MAVLINK_MSG_ID_204_CRC 19



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TASK_STATS_TM { \
    204, \
    "TASK_STATS_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_task_stats_tm_t, timestamp) }, \
         { "task_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_task_stats_tm_t, task_id) }, \
         { "task_period", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_task_stats_tm_t, task_period) }, \
         { "task_missed_events", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_task_stats_tm_t, task_missed_events) }, \
         { "task_failed_events", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_task_stats_tm_t, task_failed_events) }, \
         { "task_activation_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_task_stats_tm_t, task_activation_mean) }, \
         { "task_activation_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_task_stats_tm_t, task_activation_stddev) }, \
         { "task_period_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_task_stats_tm_t, task_period_mean) }, \
         { "task_period_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_task_stats_tm_t, task_period_stddev) }, \
         { "task_workload_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_task_stats_tm_t, task_workload_mean) }, \
         { "task_workload_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_task_stats_tm_t, task_workload_stddev) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TASK_STATS_TM { \
    "TASK_STATS_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_task_stats_tm_t, timestamp) }, \
         { "task_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_task_stats_tm_t, task_id) }, \
         { "task_period", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_task_stats_tm_t, task_period) }, \
         { "task_missed_events", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_task_stats_tm_t, task_missed_events) }, \
         { "task_failed_events", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_task_stats_tm_t, task_failed_events) }, \
         { "task_activation_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_task_stats_tm_t, task_activation_mean) }, \
         { "task_activation_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_task_stats_tm_t, task_activation_stddev) }, \
         { "task_period_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_task_stats_tm_t, task_period_mean) }, \
         { "task_period_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_task_stats_tm_t, task_period_stddev) }, \
         { "task_workload_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_task_stats_tm_t, task_workload_mean) }, \
         { "task_workload_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_task_stats_tm_t, task_workload_stddev) }, \
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
 * @param task_period [ns] Period of the task
 * @param task_missed_events  Number of missed events
 * @param task_failed_events  Number of missed events
 * @param task_activation_mean  Task activation mean period
 * @param task_activation_stddev  Task activation mean standard deviation
 * @param task_period_mean  Task period mean period
 * @param task_period_stddev  Task period mean standard deviation
 * @param task_workload_mean  Task workload mean period
 * @param task_workload_stddev  Task workload mean standard deviation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_stats_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint16_t task_id, uint16_t task_period, uint32_t task_missed_events, uint32_t task_failed_events, float task_activation_mean, float task_activation_stddev, float task_period_mean, float task_period_stddev, float task_workload_mean, float task_workload_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASK_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, task_missed_events);
    _mav_put_uint32_t(buf, 12, task_failed_events);
    _mav_put_float(buf, 16, task_activation_mean);
    _mav_put_float(buf, 20, task_activation_stddev);
    _mav_put_float(buf, 24, task_period_mean);
    _mav_put_float(buf, 28, task_period_stddev);
    _mav_put_float(buf, 32, task_workload_mean);
    _mav_put_float(buf, 36, task_workload_stddev);
    _mav_put_uint16_t(buf, 40, task_id);
    _mav_put_uint16_t(buf, 42, task_period);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
#else
    mavlink_task_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.task_missed_events = task_missed_events;
    packet.task_failed_events = task_failed_events;
    packet.task_activation_mean = task_activation_mean;
    packet.task_activation_stddev = task_activation_stddev;
    packet.task_period_mean = task_period_mean;
    packet.task_period_stddev = task_period_stddev;
    packet.task_workload_mean = task_workload_mean;
    packet.task_workload_stddev = task_workload_stddev;
    packet.task_id = task_id;
    packet.task_period = task_period;

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
 * @param task_period [ns] Period of the task
 * @param task_missed_events  Number of missed events
 * @param task_failed_events  Number of missed events
 * @param task_activation_mean  Task activation mean period
 * @param task_activation_stddev  Task activation mean standard deviation
 * @param task_period_mean  Task period mean period
 * @param task_period_stddev  Task period mean standard deviation
 * @param task_workload_mean  Task workload mean period
 * @param task_workload_stddev  Task workload mean standard deviation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_stats_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint16_t task_id,uint16_t task_period,uint32_t task_missed_events,uint32_t task_failed_events,float task_activation_mean,float task_activation_stddev,float task_period_mean,float task_period_stddev,float task_workload_mean,float task_workload_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASK_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, task_missed_events);
    _mav_put_uint32_t(buf, 12, task_failed_events);
    _mav_put_float(buf, 16, task_activation_mean);
    _mav_put_float(buf, 20, task_activation_stddev);
    _mav_put_float(buf, 24, task_period_mean);
    _mav_put_float(buf, 28, task_period_stddev);
    _mav_put_float(buf, 32, task_workload_mean);
    _mav_put_float(buf, 36, task_workload_stddev);
    _mav_put_uint16_t(buf, 40, task_id);
    _mav_put_uint16_t(buf, 42, task_period);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
#else
    mavlink_task_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.task_missed_events = task_missed_events;
    packet.task_failed_events = task_failed_events;
    packet.task_activation_mean = task_activation_mean;
    packet.task_activation_stddev = task_activation_stddev;
    packet.task_period_mean = task_period_mean;
    packet.task_period_stddev = task_period_stddev;
    packet.task_workload_mean = task_workload_mean;
    packet.task_workload_stddev = task_workload_stddev;
    packet.task_id = task_id;
    packet.task_period = task_period;

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
    return mavlink_msg_task_stats_tm_pack(system_id, component_id, msg, task_stats_tm->timestamp, task_stats_tm->task_id, task_stats_tm->task_period, task_stats_tm->task_missed_events, task_stats_tm->task_failed_events, task_stats_tm->task_activation_mean, task_stats_tm->task_activation_stddev, task_stats_tm->task_period_mean, task_stats_tm->task_period_stddev, task_stats_tm->task_workload_mean, task_stats_tm->task_workload_stddev);
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
    return mavlink_msg_task_stats_tm_pack_chan(system_id, component_id, chan, msg, task_stats_tm->timestamp, task_stats_tm->task_id, task_stats_tm->task_period, task_stats_tm->task_missed_events, task_stats_tm->task_failed_events, task_stats_tm->task_activation_mean, task_stats_tm->task_activation_stddev, task_stats_tm->task_period_mean, task_stats_tm->task_period_stddev, task_stats_tm->task_workload_mean, task_stats_tm->task_workload_stddev);
}

/**
 * @brief Send a task_stats_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged 
 * @param task_id  Task ID
 * @param task_period [ns] Period of the task
 * @param task_missed_events  Number of missed events
 * @param task_failed_events  Number of missed events
 * @param task_activation_mean  Task activation mean period
 * @param task_activation_stddev  Task activation mean standard deviation
 * @param task_period_mean  Task period mean period
 * @param task_period_stddev  Task period mean standard deviation
 * @param task_workload_mean  Task workload mean period
 * @param task_workload_stddev  Task workload mean standard deviation
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_task_stats_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint16_t task_id, uint16_t task_period, uint32_t task_missed_events, uint32_t task_failed_events, float task_activation_mean, float task_activation_stddev, float task_period_mean, float task_period_stddev, float task_workload_mean, float task_workload_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASK_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, task_missed_events);
    _mav_put_uint32_t(buf, 12, task_failed_events);
    _mav_put_float(buf, 16, task_activation_mean);
    _mav_put_float(buf, 20, task_activation_stddev);
    _mav_put_float(buf, 24, task_period_mean);
    _mav_put_float(buf, 28, task_period_stddev);
    _mav_put_float(buf, 32, task_workload_mean);
    _mav_put_float(buf, 36, task_workload_stddev);
    _mav_put_uint16_t(buf, 40, task_id);
    _mav_put_uint16_t(buf, 42, task_period);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATS_TM, buf, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
#else
    mavlink_task_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.task_missed_events = task_missed_events;
    packet.task_failed_events = task_failed_events;
    packet.task_activation_mean = task_activation_mean;
    packet.task_activation_stddev = task_activation_stddev;
    packet.task_period_mean = task_period_mean;
    packet.task_period_stddev = task_period_stddev;
    packet.task_workload_mean = task_workload_mean;
    packet.task_workload_stddev = task_workload_stddev;
    packet.task_id = task_id;
    packet.task_period = task_period;

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
    mavlink_msg_task_stats_tm_send(chan, task_stats_tm->timestamp, task_stats_tm->task_id, task_stats_tm->task_period, task_stats_tm->task_missed_events, task_stats_tm->task_failed_events, task_stats_tm->task_activation_mean, task_stats_tm->task_activation_stddev, task_stats_tm->task_period_mean, task_stats_tm->task_period_stddev, task_stats_tm->task_workload_mean, task_stats_tm->task_workload_stddev);
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
static inline void mavlink_msg_task_stats_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint16_t task_id, uint16_t task_period, uint32_t task_missed_events, uint32_t task_failed_events, float task_activation_mean, float task_activation_stddev, float task_period_mean, float task_period_stddev, float task_workload_mean, float task_workload_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, task_missed_events);
    _mav_put_uint32_t(buf, 12, task_failed_events);
    _mav_put_float(buf, 16, task_activation_mean);
    _mav_put_float(buf, 20, task_activation_stddev);
    _mav_put_float(buf, 24, task_period_mean);
    _mav_put_float(buf, 28, task_period_stddev);
    _mav_put_float(buf, 32, task_workload_mean);
    _mav_put_float(buf, 36, task_workload_stddev);
    _mav_put_uint16_t(buf, 40, task_id);
    _mav_put_uint16_t(buf, 42, task_period);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATS_TM, buf, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
#else
    mavlink_task_stats_tm_t *packet = (mavlink_task_stats_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->task_missed_events = task_missed_events;
    packet->task_failed_events = task_failed_events;
    packet->task_activation_mean = task_activation_mean;
    packet->task_activation_stddev = task_activation_stddev;
    packet->task_period_mean = task_period_mean;
    packet->task_period_stddev = task_period_stddev;
    packet->task_workload_mean = task_workload_mean;
    packet->task_workload_stddev = task_workload_stddev;
    packet->task_id = task_id;
    packet->task_period = task_period;

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
static inline uint16_t mavlink_msg_task_stats_tm_get_task_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field task_period from task_stats_tm message
 *
 * @return [ns] Period of the task
 */
static inline uint16_t mavlink_msg_task_stats_tm_get_task_period(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  42);
}

/**
 * @brief Get field task_missed_events from task_stats_tm message
 *
 * @return  Number of missed events
 */
static inline uint32_t mavlink_msg_task_stats_tm_get_task_missed_events(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field task_failed_events from task_stats_tm message
 *
 * @return  Number of missed events
 */
static inline uint32_t mavlink_msg_task_stats_tm_get_task_failed_events(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field task_activation_mean from task_stats_tm message
 *
 * @return  Task activation mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_activation_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field task_activation_stddev from task_stats_tm message
 *
 * @return  Task activation mean standard deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_activation_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field task_period_mean from task_stats_tm message
 *
 * @return  Task period mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_period_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field task_period_stddev from task_stats_tm message
 *
 * @return  Task period mean standard deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_period_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field task_workload_mean from task_stats_tm message
 *
 * @return  Task workload mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_workload_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field task_workload_stddev from task_stats_tm message
 *
 * @return  Task workload mean standard deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_workload_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
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
    task_stats_tm->task_missed_events = mavlink_msg_task_stats_tm_get_task_missed_events(msg);
    task_stats_tm->task_failed_events = mavlink_msg_task_stats_tm_get_task_failed_events(msg);
    task_stats_tm->task_activation_mean = mavlink_msg_task_stats_tm_get_task_activation_mean(msg);
    task_stats_tm->task_activation_stddev = mavlink_msg_task_stats_tm_get_task_activation_stddev(msg);
    task_stats_tm->task_period_mean = mavlink_msg_task_stats_tm_get_task_period_mean(msg);
    task_stats_tm->task_period_stddev = mavlink_msg_task_stats_tm_get_task_period_stddev(msg);
    task_stats_tm->task_workload_mean = mavlink_msg_task_stats_tm_get_task_workload_mean(msg);
    task_stats_tm->task_workload_stddev = mavlink_msg_task_stats_tm_get_task_workload_stddev(msg);
    task_stats_tm->task_id = mavlink_msg_task_stats_tm_get_task_id(msg);
    task_stats_tm->task_period = mavlink_msg_task_stats_tm_get_task_period(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TASK_STATS_TM_LEN? msg->len : MAVLINK_MSG_ID_TASK_STATS_TM_LEN;
        memset(task_stats_tm, 0, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
    memcpy(task_stats_tm, _MAV_PAYLOAD(msg), len);
#endif
}
