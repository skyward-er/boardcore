#pragma once
// MESSAGE LOGGER_TM PACKING

#define MAVLINK_MSG_ID_LOGGER_TM 162

MAVPACKED(
typedef struct __mavlink_logger_tm_t {
 uint64_t timestamp; /*<  Timestamp*/
 int32_t statTooLargeSamples; /*<  Number of dropped samples because too large*/
 int32_t statDroppedSamples; /*<  Number of dropped samples due to fifo full*/
 int32_t statQueuedSamples; /*<  Number of samples written to buffer*/
 int32_t statBufferFilled; /*<  Number of buffers filled*/
 int32_t statBufferWritten; /*<  Number of buffers written to disk*/
 int32_t statWriteFailed; /*<  Number of fwrite() that failed*/
 int32_t statWriteError; /*<  Error of the last fwrite() that failed*/
 int32_t statWriteTime; /*<  Time to perform an fwrite() of a buffer*/
 int32_t statMaxWriteTime; /*<  Max time to perform an fwrite() of a buffer*/
 uint16_t statLogNumber; /*<  Currently active log file*/
}) mavlink_logger_tm_t;

#define MAVLINK_MSG_ID_LOGGER_TM_LEN 46
#define MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN 46
#define MAVLINK_MSG_ID_162_LEN 46
#define MAVLINK_MSG_ID_162_MIN_LEN 46

#define MAVLINK_MSG_ID_LOGGER_TM_CRC 163
#define MAVLINK_MSG_ID_162_CRC 163



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LOGGER_TM { \
    162, \
    "LOGGER_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_logger_tm_t, timestamp) }, \
         { "statLogNumber", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_logger_tm_t, statLogNumber) }, \
         { "statTooLargeSamples", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_logger_tm_t, statTooLargeSamples) }, \
         { "statDroppedSamples", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_logger_tm_t, statDroppedSamples) }, \
         { "statQueuedSamples", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_logger_tm_t, statQueuedSamples) }, \
         { "statBufferFilled", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_logger_tm_t, statBufferFilled) }, \
         { "statBufferWritten", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_logger_tm_t, statBufferWritten) }, \
         { "statWriteFailed", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_logger_tm_t, statWriteFailed) }, \
         { "statWriteError", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_logger_tm_t, statWriteError) }, \
         { "statWriteTime", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_logger_tm_t, statWriteTime) }, \
         { "statMaxWriteTime", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_logger_tm_t, statMaxWriteTime) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LOGGER_TM { \
    "LOGGER_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_logger_tm_t, timestamp) }, \
         { "statLogNumber", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_logger_tm_t, statLogNumber) }, \
         { "statTooLargeSamples", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_logger_tm_t, statTooLargeSamples) }, \
         { "statDroppedSamples", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_logger_tm_t, statDroppedSamples) }, \
         { "statQueuedSamples", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_logger_tm_t, statQueuedSamples) }, \
         { "statBufferFilled", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_logger_tm_t, statBufferFilled) }, \
         { "statBufferWritten", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_logger_tm_t, statBufferWritten) }, \
         { "statWriteFailed", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_logger_tm_t, statWriteFailed) }, \
         { "statWriteError", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_logger_tm_t, statWriteError) }, \
         { "statWriteTime", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_logger_tm_t, statWriteTime) }, \
         { "statMaxWriteTime", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_logger_tm_t, statMaxWriteTime) }, \
         } \
}
#endif

/**
 * @brief Pack a logger_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Timestamp
 * @param statLogNumber  Currently active log file
 * @param statTooLargeSamples  Number of dropped samples because too large
 * @param statDroppedSamples  Number of dropped samples due to fifo full
 * @param statQueuedSamples  Number of samples written to buffer
 * @param statBufferFilled  Number of buffers filled
 * @param statBufferWritten  Number of buffers written to disk
 * @param statWriteFailed  Number of fwrite() that failed
 * @param statWriteError  Error of the last fwrite() that failed
 * @param statWriteTime  Time to perform an fwrite() of a buffer
 * @param statMaxWriteTime  Max time to perform an fwrite() of a buffer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_logger_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint16_t statLogNumber, int32_t statTooLargeSamples, int32_t statDroppedSamples, int32_t statQueuedSamples, int32_t statBufferFilled, int32_t statBufferWritten, int32_t statWriteFailed, int32_t statWriteError, int32_t statWriteTime, int32_t statMaxWriteTime)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOGGER_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, statTooLargeSamples);
    _mav_put_int32_t(buf, 12, statDroppedSamples);
    _mav_put_int32_t(buf, 16, statQueuedSamples);
    _mav_put_int32_t(buf, 20, statBufferFilled);
    _mav_put_int32_t(buf, 24, statBufferWritten);
    _mav_put_int32_t(buf, 28, statWriteFailed);
    _mav_put_int32_t(buf, 32, statWriteError);
    _mav_put_int32_t(buf, 36, statWriteTime);
    _mav_put_int32_t(buf, 40, statMaxWriteTime);
    _mav_put_uint16_t(buf, 44, statLogNumber);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOGGER_TM_LEN);
#else
    mavlink_logger_tm_t packet;
    packet.timestamp = timestamp;
    packet.statTooLargeSamples = statTooLargeSamples;
    packet.statDroppedSamples = statDroppedSamples;
    packet.statQueuedSamples = statQueuedSamples;
    packet.statBufferFilled = statBufferFilled;
    packet.statBufferWritten = statBufferWritten;
    packet.statWriteFailed = statWriteFailed;
    packet.statWriteError = statWriteError;
    packet.statWriteTime = statWriteTime;
    packet.statMaxWriteTime = statMaxWriteTime;
    packet.statLogNumber = statLogNumber;

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
 * @param timestamp  Timestamp
 * @param statLogNumber  Currently active log file
 * @param statTooLargeSamples  Number of dropped samples because too large
 * @param statDroppedSamples  Number of dropped samples due to fifo full
 * @param statQueuedSamples  Number of samples written to buffer
 * @param statBufferFilled  Number of buffers filled
 * @param statBufferWritten  Number of buffers written to disk
 * @param statWriteFailed  Number of fwrite() that failed
 * @param statWriteError  Error of the last fwrite() that failed
 * @param statWriteTime  Time to perform an fwrite() of a buffer
 * @param statMaxWriteTime  Max time to perform an fwrite() of a buffer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_logger_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint16_t statLogNumber,int32_t statTooLargeSamples,int32_t statDroppedSamples,int32_t statQueuedSamples,int32_t statBufferFilled,int32_t statBufferWritten,int32_t statWriteFailed,int32_t statWriteError,int32_t statWriteTime,int32_t statMaxWriteTime)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOGGER_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, statTooLargeSamples);
    _mav_put_int32_t(buf, 12, statDroppedSamples);
    _mav_put_int32_t(buf, 16, statQueuedSamples);
    _mav_put_int32_t(buf, 20, statBufferFilled);
    _mav_put_int32_t(buf, 24, statBufferWritten);
    _mav_put_int32_t(buf, 28, statWriteFailed);
    _mav_put_int32_t(buf, 32, statWriteError);
    _mav_put_int32_t(buf, 36, statWriteTime);
    _mav_put_int32_t(buf, 40, statMaxWriteTime);
    _mav_put_uint16_t(buf, 44, statLogNumber);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOGGER_TM_LEN);
#else
    mavlink_logger_tm_t packet;
    packet.timestamp = timestamp;
    packet.statTooLargeSamples = statTooLargeSamples;
    packet.statDroppedSamples = statDroppedSamples;
    packet.statQueuedSamples = statQueuedSamples;
    packet.statBufferFilled = statBufferFilled;
    packet.statBufferWritten = statBufferWritten;
    packet.statWriteFailed = statWriteFailed;
    packet.statWriteError = statWriteError;
    packet.statWriteTime = statWriteTime;
    packet.statMaxWriteTime = statMaxWriteTime;
    packet.statLogNumber = statLogNumber;

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
    return mavlink_msg_logger_tm_pack(system_id, component_id, msg, logger_tm->timestamp, logger_tm->statLogNumber, logger_tm->statTooLargeSamples, logger_tm->statDroppedSamples, logger_tm->statQueuedSamples, logger_tm->statBufferFilled, logger_tm->statBufferWritten, logger_tm->statWriteFailed, logger_tm->statWriteError, logger_tm->statWriteTime, logger_tm->statMaxWriteTime);
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
    return mavlink_msg_logger_tm_pack_chan(system_id, component_id, chan, msg, logger_tm->timestamp, logger_tm->statLogNumber, logger_tm->statTooLargeSamples, logger_tm->statDroppedSamples, logger_tm->statQueuedSamples, logger_tm->statBufferFilled, logger_tm->statBufferWritten, logger_tm->statWriteFailed, logger_tm->statWriteError, logger_tm->statWriteTime, logger_tm->statMaxWriteTime);
}

/**
 * @brief Send a logger_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Timestamp
 * @param statLogNumber  Currently active log file
 * @param statTooLargeSamples  Number of dropped samples because too large
 * @param statDroppedSamples  Number of dropped samples due to fifo full
 * @param statQueuedSamples  Number of samples written to buffer
 * @param statBufferFilled  Number of buffers filled
 * @param statBufferWritten  Number of buffers written to disk
 * @param statWriteFailed  Number of fwrite() that failed
 * @param statWriteError  Error of the last fwrite() that failed
 * @param statWriteTime  Time to perform an fwrite() of a buffer
 * @param statMaxWriteTime  Max time to perform an fwrite() of a buffer
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_logger_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint16_t statLogNumber, int32_t statTooLargeSamples, int32_t statDroppedSamples, int32_t statQueuedSamples, int32_t statBufferFilled, int32_t statBufferWritten, int32_t statWriteFailed, int32_t statWriteError, int32_t statWriteTime, int32_t statMaxWriteTime)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOGGER_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, statTooLargeSamples);
    _mav_put_int32_t(buf, 12, statDroppedSamples);
    _mav_put_int32_t(buf, 16, statQueuedSamples);
    _mav_put_int32_t(buf, 20, statBufferFilled);
    _mav_put_int32_t(buf, 24, statBufferWritten);
    _mav_put_int32_t(buf, 28, statWriteFailed);
    _mav_put_int32_t(buf, 32, statWriteError);
    _mav_put_int32_t(buf, 36, statWriteTime);
    _mav_put_int32_t(buf, 40, statMaxWriteTime);
    _mav_put_uint16_t(buf, 44, statLogNumber);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGER_TM, buf, MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN, MAVLINK_MSG_ID_LOGGER_TM_LEN, MAVLINK_MSG_ID_LOGGER_TM_CRC);
#else
    mavlink_logger_tm_t packet;
    packet.timestamp = timestamp;
    packet.statTooLargeSamples = statTooLargeSamples;
    packet.statDroppedSamples = statDroppedSamples;
    packet.statQueuedSamples = statQueuedSamples;
    packet.statBufferFilled = statBufferFilled;
    packet.statBufferWritten = statBufferWritten;
    packet.statWriteFailed = statWriteFailed;
    packet.statWriteError = statWriteError;
    packet.statWriteTime = statWriteTime;
    packet.statMaxWriteTime = statMaxWriteTime;
    packet.statLogNumber = statLogNumber;

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
    mavlink_msg_logger_tm_send(chan, logger_tm->timestamp, logger_tm->statLogNumber, logger_tm->statTooLargeSamples, logger_tm->statDroppedSamples, logger_tm->statQueuedSamples, logger_tm->statBufferFilled, logger_tm->statBufferWritten, logger_tm->statWriteFailed, logger_tm->statWriteError, logger_tm->statWriteTime, logger_tm->statMaxWriteTime);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGER_TM, (const char *)logger_tm, MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN, MAVLINK_MSG_ID_LOGGER_TM_LEN, MAVLINK_MSG_ID_LOGGER_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_LOGGER_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_logger_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint16_t statLogNumber, int32_t statTooLargeSamples, int32_t statDroppedSamples, int32_t statQueuedSamples, int32_t statBufferFilled, int32_t statBufferWritten, int32_t statWriteFailed, int32_t statWriteError, int32_t statWriteTime, int32_t statMaxWriteTime)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, statTooLargeSamples);
    _mav_put_int32_t(buf, 12, statDroppedSamples);
    _mav_put_int32_t(buf, 16, statQueuedSamples);
    _mav_put_int32_t(buf, 20, statBufferFilled);
    _mav_put_int32_t(buf, 24, statBufferWritten);
    _mav_put_int32_t(buf, 28, statWriteFailed);
    _mav_put_int32_t(buf, 32, statWriteError);
    _mav_put_int32_t(buf, 36, statWriteTime);
    _mav_put_int32_t(buf, 40, statMaxWriteTime);
    _mav_put_uint16_t(buf, 44, statLogNumber);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGER_TM, buf, MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN, MAVLINK_MSG_ID_LOGGER_TM_LEN, MAVLINK_MSG_ID_LOGGER_TM_CRC);
#else
    mavlink_logger_tm_t *packet = (mavlink_logger_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->statTooLargeSamples = statTooLargeSamples;
    packet->statDroppedSamples = statDroppedSamples;
    packet->statQueuedSamples = statQueuedSamples;
    packet->statBufferFilled = statBufferFilled;
    packet->statBufferWritten = statBufferWritten;
    packet->statWriteFailed = statWriteFailed;
    packet->statWriteError = statWriteError;
    packet->statWriteTime = statWriteTime;
    packet->statMaxWriteTime = statMaxWriteTime;
    packet->statLogNumber = statLogNumber;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGER_TM, (const char *)packet, MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN, MAVLINK_MSG_ID_LOGGER_TM_LEN, MAVLINK_MSG_ID_LOGGER_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE LOGGER_TM UNPACKING


/**
 * @brief Get field timestamp from logger_tm message
 *
 * @return  Timestamp
 */
static inline uint64_t mavlink_msg_logger_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field statLogNumber from logger_tm message
 *
 * @return  Currently active log file
 */
static inline uint16_t mavlink_msg_logger_tm_get_statLogNumber(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  44);
}

/**
 * @brief Get field statTooLargeSamples from logger_tm message
 *
 * @return  Number of dropped samples because too large
 */
static inline int32_t mavlink_msg_logger_tm_get_statTooLargeSamples(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field statDroppedSamples from logger_tm message
 *
 * @return  Number of dropped samples due to fifo full
 */
static inline int32_t mavlink_msg_logger_tm_get_statDroppedSamples(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field statQueuedSamples from logger_tm message
 *
 * @return  Number of samples written to buffer
 */
static inline int32_t mavlink_msg_logger_tm_get_statQueuedSamples(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field statBufferFilled from logger_tm message
 *
 * @return  Number of buffers filled
 */
static inline int32_t mavlink_msg_logger_tm_get_statBufferFilled(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field statBufferWritten from logger_tm message
 *
 * @return  Number of buffers written to disk
 */
static inline int32_t mavlink_msg_logger_tm_get_statBufferWritten(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field statWriteFailed from logger_tm message
 *
 * @return  Number of fwrite() that failed
 */
static inline int32_t mavlink_msg_logger_tm_get_statWriteFailed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field statWriteError from logger_tm message
 *
 * @return  Error of the last fwrite() that failed
 */
static inline int32_t mavlink_msg_logger_tm_get_statWriteError(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field statWriteTime from logger_tm message
 *
 * @return  Time to perform an fwrite() of a buffer
 */
static inline int32_t mavlink_msg_logger_tm_get_statWriteTime(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field statMaxWriteTime from logger_tm message
 *
 * @return  Max time to perform an fwrite() of a buffer
 */
static inline int32_t mavlink_msg_logger_tm_get_statMaxWriteTime(const mavlink_message_t* msg)
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
    logger_tm->statTooLargeSamples = mavlink_msg_logger_tm_get_statTooLargeSamples(msg);
    logger_tm->statDroppedSamples = mavlink_msg_logger_tm_get_statDroppedSamples(msg);
    logger_tm->statQueuedSamples = mavlink_msg_logger_tm_get_statQueuedSamples(msg);
    logger_tm->statBufferFilled = mavlink_msg_logger_tm_get_statBufferFilled(msg);
    logger_tm->statBufferWritten = mavlink_msg_logger_tm_get_statBufferWritten(msg);
    logger_tm->statWriteFailed = mavlink_msg_logger_tm_get_statWriteFailed(msg);
    logger_tm->statWriteError = mavlink_msg_logger_tm_get_statWriteError(msg);
    logger_tm->statWriteTime = mavlink_msg_logger_tm_get_statWriteTime(msg);
    logger_tm->statMaxWriteTime = mavlink_msg_logger_tm_get_statMaxWriteTime(msg);
    logger_tm->statLogNumber = mavlink_msg_logger_tm_get_statLogNumber(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LOGGER_TM_LEN? msg->len : MAVLINK_MSG_ID_LOGGER_TM_LEN;
        memset(logger_tm, 0, MAVLINK_MSG_ID_LOGGER_TM_LEN);
    memcpy(logger_tm, _MAV_PAYLOAD(msg), len);
#endif
}
