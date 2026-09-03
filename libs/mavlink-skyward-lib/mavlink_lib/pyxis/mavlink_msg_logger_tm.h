#pragma once
// MESSAGE LOGGER_TM PACKING

#define MAVLINK_MSG_ID_LOGGER_TM 202


typedef struct __mavlink_logger_tm_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 int32_t too_large_samples; /*<  Number of dropped samples because too large*/
 int32_t dropped_samples; /*<  Number of dropped samples due to fifo full*/
 int32_t queued_samples; /*<  Number of samples written to buffer*/
 int32_t buffers_filled; /*<  Number of buffers filled*/
 int32_t buffers_written; /*<  Number of buffers written to disk*/
 int32_t writes_failed; /*<  Number of fwrite() that failed*/
 int32_t last_write_error; /*<  Error of the last fwrite() that failed*/
 int32_t average_write_time; /*<  Average time to perform an fwrite() of a buffer*/
 int32_t max_write_time; /*<  Max time to perform an fwrite() of a buffer*/
 int16_t log_number; /*<  Currently active log file, -1 if the logger is inactive*/
} mavlink_logger_tm_t;

#define MAVLINK_MSG_ID_LOGGER_TM_LEN 46
#define MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN 46
#define MAVLINK_MSG_ID_202_LEN 46
#define MAVLINK_MSG_ID_202_MIN_LEN 46

#define MAVLINK_MSG_ID_LOGGER_TM_CRC 142
#define MAVLINK_MSG_ID_202_CRC 142



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LOGGER_TM { \
    202, \
    "LOGGER_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_logger_tm_t, timestamp) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_logger_tm_t, log_number) }, \
         { "too_large_samples", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_logger_tm_t, too_large_samples) }, \
         { "dropped_samples", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_logger_tm_t, dropped_samples) }, \
         { "queued_samples", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_logger_tm_t, queued_samples) }, \
         { "buffers_filled", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_logger_tm_t, buffers_filled) }, \
         { "buffers_written", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_logger_tm_t, buffers_written) }, \
         { "writes_failed", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_logger_tm_t, writes_failed) }, \
         { "last_write_error", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_logger_tm_t, last_write_error) }, \
         { "average_write_time", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_logger_tm_t, average_write_time) }, \
         { "max_write_time", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_logger_tm_t, max_write_time) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LOGGER_TM { \
    "LOGGER_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_logger_tm_t, timestamp) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_logger_tm_t, log_number) }, \
         { "too_large_samples", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_logger_tm_t, too_large_samples) }, \
         { "dropped_samples", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_logger_tm_t, dropped_samples) }, \
         { "queued_samples", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_logger_tm_t, queued_samples) }, \
         { "buffers_filled", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_logger_tm_t, buffers_filled) }, \
         { "buffers_written", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_logger_tm_t, buffers_written) }, \
         { "writes_failed", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_logger_tm_t, writes_failed) }, \
         { "last_write_error", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_logger_tm_t, last_write_error) }, \
         { "average_write_time", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_logger_tm_t, average_write_time) }, \
         { "max_write_time", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_logger_tm_t, max_write_time) }, \
         } \
}
#endif

/**
 * @brief Pack a logger_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param too_large_samples  Number of dropped samples because too large
 * @param dropped_samples  Number of dropped samples due to fifo full
 * @param queued_samples  Number of samples written to buffer
 * @param buffers_filled  Number of buffers filled
 * @param buffers_written  Number of buffers written to disk
 * @param writes_failed  Number of fwrite() that failed
 * @param last_write_error  Error of the last fwrite() that failed
 * @param average_write_time  Average time to perform an fwrite() of a buffer
 * @param max_write_time  Max time to perform an fwrite() of a buffer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_logger_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, int16_t log_number, int32_t too_large_samples, int32_t dropped_samples, int32_t queued_samples, int32_t buffers_filled, int32_t buffers_written, int32_t writes_failed, int32_t last_write_error, int32_t average_write_time, int32_t max_write_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOGGER_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, too_large_samples);
    _mav_put_int32_t(buf, 12, dropped_samples);
    _mav_put_int32_t(buf, 16, queued_samples);
    _mav_put_int32_t(buf, 20, buffers_filled);
    _mav_put_int32_t(buf, 24, buffers_written);
    _mav_put_int32_t(buf, 28, writes_failed);
    _mav_put_int32_t(buf, 32, last_write_error);
    _mav_put_int32_t(buf, 36, average_write_time);
    _mav_put_int32_t(buf, 40, max_write_time);
    _mav_put_int16_t(buf, 44, log_number);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOGGER_TM_LEN);
#else
    mavlink_logger_tm_t packet;
    packet.timestamp = timestamp;
    packet.too_large_samples = too_large_samples;
    packet.dropped_samples = dropped_samples;
    packet.queued_samples = queued_samples;
    packet.buffers_filled = buffers_filled;
    packet.buffers_written = buffers_written;
    packet.writes_failed = writes_failed;
    packet.last_write_error = last_write_error;
    packet.average_write_time = average_write_time;
    packet.max_write_time = max_write_time;
    packet.log_number = log_number;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOGGER_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOGGER_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN, MAVLINK_MSG_ID_LOGGER_TM_LEN, MAVLINK_MSG_ID_LOGGER_TM_CRC);
}

/**
 * @brief Pack a logger_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param too_large_samples  Number of dropped samples because too large
 * @param dropped_samples  Number of dropped samples due to fifo full
 * @param queued_samples  Number of samples written to buffer
 * @param buffers_filled  Number of buffers filled
 * @param buffers_written  Number of buffers written to disk
 * @param writes_failed  Number of fwrite() that failed
 * @param last_write_error  Error of the last fwrite() that failed
 * @param average_write_time  Average time to perform an fwrite() of a buffer
 * @param max_write_time  Max time to perform an fwrite() of a buffer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_logger_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,int16_t log_number,int32_t too_large_samples,int32_t dropped_samples,int32_t queued_samples,int32_t buffers_filled,int32_t buffers_written,int32_t writes_failed,int32_t last_write_error,int32_t average_write_time,int32_t max_write_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOGGER_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, too_large_samples);
    _mav_put_int32_t(buf, 12, dropped_samples);
    _mav_put_int32_t(buf, 16, queued_samples);
    _mav_put_int32_t(buf, 20, buffers_filled);
    _mav_put_int32_t(buf, 24, buffers_written);
    _mav_put_int32_t(buf, 28, writes_failed);
    _mav_put_int32_t(buf, 32, last_write_error);
    _mav_put_int32_t(buf, 36, average_write_time);
    _mav_put_int32_t(buf, 40, max_write_time);
    _mav_put_int16_t(buf, 44, log_number);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOGGER_TM_LEN);
#else
    mavlink_logger_tm_t packet;
    packet.timestamp = timestamp;
    packet.too_large_samples = too_large_samples;
    packet.dropped_samples = dropped_samples;
    packet.queued_samples = queued_samples;
    packet.buffers_filled = buffers_filled;
    packet.buffers_written = buffers_written;
    packet.writes_failed = writes_failed;
    packet.last_write_error = last_write_error;
    packet.average_write_time = average_write_time;
    packet.max_write_time = max_write_time;
    packet.log_number = log_number;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOGGER_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOGGER_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN, MAVLINK_MSG_ID_LOGGER_TM_LEN, MAVLINK_MSG_ID_LOGGER_TM_CRC);
}

/**
 * @brief Encode a logger_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param logger_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_logger_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_logger_tm_t* logger_tm)
{
    return mavlink_msg_logger_tm_pack(system_id, component_id, msg, logger_tm->timestamp, logger_tm->log_number, logger_tm->too_large_samples, logger_tm->dropped_samples, logger_tm->queued_samples, logger_tm->buffers_filled, logger_tm->buffers_written, logger_tm->writes_failed, logger_tm->last_write_error, logger_tm->average_write_time, logger_tm->max_write_time);
}

/**
 * @brief Encode a logger_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param logger_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_logger_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_logger_tm_t* logger_tm)
{
    return mavlink_msg_logger_tm_pack_chan(system_id, component_id, chan, msg, logger_tm->timestamp, logger_tm->log_number, logger_tm->too_large_samples, logger_tm->dropped_samples, logger_tm->queued_samples, logger_tm->buffers_filled, logger_tm->buffers_written, logger_tm->writes_failed, logger_tm->last_write_error, logger_tm->average_write_time, logger_tm->max_write_time);
}

/**
 * @brief Send a logger_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @param too_large_samples  Number of dropped samples because too large
 * @param dropped_samples  Number of dropped samples due to fifo full
 * @param queued_samples  Number of samples written to buffer
 * @param buffers_filled  Number of buffers filled
 * @param buffers_written  Number of buffers written to disk
 * @param writes_failed  Number of fwrite() that failed
 * @param last_write_error  Error of the last fwrite() that failed
 * @param average_write_time  Average time to perform an fwrite() of a buffer
 * @param max_write_time  Max time to perform an fwrite() of a buffer
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_logger_tm_send(mavlink_channel_t chan, uint64_t timestamp, int16_t log_number, int32_t too_large_samples, int32_t dropped_samples, int32_t queued_samples, int32_t buffers_filled, int32_t buffers_written, int32_t writes_failed, int32_t last_write_error, int32_t average_write_time, int32_t max_write_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOGGER_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, too_large_samples);
    _mav_put_int32_t(buf, 12, dropped_samples);
    _mav_put_int32_t(buf, 16, queued_samples);
    _mav_put_int32_t(buf, 20, buffers_filled);
    _mav_put_int32_t(buf, 24, buffers_written);
    _mav_put_int32_t(buf, 28, writes_failed);
    _mav_put_int32_t(buf, 32, last_write_error);
    _mav_put_int32_t(buf, 36, average_write_time);
    _mav_put_int32_t(buf, 40, max_write_time);
    _mav_put_int16_t(buf, 44, log_number);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGER_TM, buf, MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN, MAVLINK_MSG_ID_LOGGER_TM_LEN, MAVLINK_MSG_ID_LOGGER_TM_CRC);
#else
    mavlink_logger_tm_t packet;
    packet.timestamp = timestamp;
    packet.too_large_samples = too_large_samples;
    packet.dropped_samples = dropped_samples;
    packet.queued_samples = queued_samples;
    packet.buffers_filled = buffers_filled;
    packet.buffers_written = buffers_written;
    packet.writes_failed = writes_failed;
    packet.last_write_error = last_write_error;
    packet.average_write_time = average_write_time;
    packet.max_write_time = max_write_time;
    packet.log_number = log_number;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGER_TM, (const char *)&packet, MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN, MAVLINK_MSG_ID_LOGGER_TM_LEN, MAVLINK_MSG_ID_LOGGER_TM_CRC);
#endif
}

/**
 * @brief Send a logger_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_logger_tm_send_struct(mavlink_channel_t chan, const mavlink_logger_tm_t* logger_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_logger_tm_send(chan, logger_tm->timestamp, logger_tm->log_number, logger_tm->too_large_samples, logger_tm->dropped_samples, logger_tm->queued_samples, logger_tm->buffers_filled, logger_tm->buffers_written, logger_tm->writes_failed, logger_tm->last_write_error, logger_tm->average_write_time, logger_tm->max_write_time);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGER_TM, (const char *)logger_tm, MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN, MAVLINK_MSG_ID_LOGGER_TM_LEN, MAVLINK_MSG_ID_LOGGER_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_LOGGER_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_logger_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, int16_t log_number, int32_t too_large_samples, int32_t dropped_samples, int32_t queued_samples, int32_t buffers_filled, int32_t buffers_written, int32_t writes_failed, int32_t last_write_error, int32_t average_write_time, int32_t max_write_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, too_large_samples);
    _mav_put_int32_t(buf, 12, dropped_samples);
    _mav_put_int32_t(buf, 16, queued_samples);
    _mav_put_int32_t(buf, 20, buffers_filled);
    _mav_put_int32_t(buf, 24, buffers_written);
    _mav_put_int32_t(buf, 28, writes_failed);
    _mav_put_int32_t(buf, 32, last_write_error);
    _mav_put_int32_t(buf, 36, average_write_time);
    _mav_put_int32_t(buf, 40, max_write_time);
    _mav_put_int16_t(buf, 44, log_number);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGER_TM, buf, MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN, MAVLINK_MSG_ID_LOGGER_TM_LEN, MAVLINK_MSG_ID_LOGGER_TM_CRC);
#else
    mavlink_logger_tm_t *packet = (mavlink_logger_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->too_large_samples = too_large_samples;
    packet->dropped_samples = dropped_samples;
    packet->queued_samples = queued_samples;
    packet->buffers_filled = buffers_filled;
    packet->buffers_written = buffers_written;
    packet->writes_failed = writes_failed;
    packet->last_write_error = last_write_error;
    packet->average_write_time = average_write_time;
    packet->max_write_time = max_write_time;
    packet->log_number = log_number;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGER_TM, (const char *)packet, MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN, MAVLINK_MSG_ID_LOGGER_TM_LEN, MAVLINK_MSG_ID_LOGGER_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE LOGGER_TM UNPACKING


/**
 * @brief Get field timestamp from logger_tm message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_logger_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field log_number from logger_tm message
 *
 * @return  Currently active log file, -1 if the logger is inactive
 */
static inline int16_t mavlink_msg_logger_tm_get_log_number(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  44);
}

/**
 * @brief Get field too_large_samples from logger_tm message
 *
 * @return  Number of dropped samples because too large
 */
static inline int32_t mavlink_msg_logger_tm_get_too_large_samples(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field dropped_samples from logger_tm message
 *
 * @return  Number of dropped samples due to fifo full
 */
static inline int32_t mavlink_msg_logger_tm_get_dropped_samples(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field queued_samples from logger_tm message
 *
 * @return  Number of samples written to buffer
 */
static inline int32_t mavlink_msg_logger_tm_get_queued_samples(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field buffers_filled from logger_tm message
 *
 * @return  Number of buffers filled
 */
static inline int32_t mavlink_msg_logger_tm_get_buffers_filled(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field buffers_written from logger_tm message
 *
 * @return  Number of buffers written to disk
 */
static inline int32_t mavlink_msg_logger_tm_get_buffers_written(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field writes_failed from logger_tm message
 *
 * @return  Number of fwrite() that failed
 */
static inline int32_t mavlink_msg_logger_tm_get_writes_failed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field last_write_error from logger_tm message
 *
 * @return  Error of the last fwrite() that failed
 */
static inline int32_t mavlink_msg_logger_tm_get_last_write_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field average_write_time from logger_tm message
 *
 * @return  Average time to perform an fwrite() of a buffer
 */
static inline int32_t mavlink_msg_logger_tm_get_average_write_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field max_write_time from logger_tm message
 *
 * @return  Max time to perform an fwrite() of a buffer
 */
static inline int32_t mavlink_msg_logger_tm_get_max_write_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  40);
}

/**
 * @brief Decode a logger_tm message into a struct
 *
 * @param msg The message to decode
 * @param logger_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_logger_tm_decode(const mavlink_message_t* msg, mavlink_logger_tm_t* logger_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    logger_tm->timestamp = mavlink_msg_logger_tm_get_timestamp(msg);
    logger_tm->too_large_samples = mavlink_msg_logger_tm_get_too_large_samples(msg);
    logger_tm->dropped_samples = mavlink_msg_logger_tm_get_dropped_samples(msg);
    logger_tm->queued_samples = mavlink_msg_logger_tm_get_queued_samples(msg);
    logger_tm->buffers_filled = mavlink_msg_logger_tm_get_buffers_filled(msg);
    logger_tm->buffers_written = mavlink_msg_logger_tm_get_buffers_written(msg);
    logger_tm->writes_failed = mavlink_msg_logger_tm_get_writes_failed(msg);
    logger_tm->last_write_error = mavlink_msg_logger_tm_get_last_write_error(msg);
    logger_tm->average_write_time = mavlink_msg_logger_tm_get_average_write_time(msg);
    logger_tm->max_write_time = mavlink_msg_logger_tm_get_max_write_time(msg);
    logger_tm->log_number = mavlink_msg_logger_tm_get_log_number(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LOGGER_TM_LEN? msg->len : MAVLINK_MSG_ID_LOGGER_TM_LEN;
        memset(logger_tm, 0, MAVLINK_MSG_ID_LOGGER_TM_LEN);
    memcpy(logger_tm, _MAV_PAYLOAD(msg), len);
#endif
}
