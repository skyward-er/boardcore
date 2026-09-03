#pragma once
// MESSAGE SYS_TM PACKING

#define MAVLINK_MSG_ID_SYS_TM 200


typedef struct __mavlink_sys_tm_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 uint8_t logger; /*<  True if the logger started correctly*/
 uint8_t event_broker; /*<  True if the event broker started correctly*/
 uint8_t radio; /*<  True if the radio started correctly*/
 uint8_t pin_observer; /*<  True if the pin observer started correctly*/
 uint8_t sensors; /*<  True if the sensors started correctly*/
 uint8_t board_scheduler; /*<  True if the board scheduler is running*/
} mavlink_sys_tm_t;

#define MAVLINK_MSG_ID_SYS_TM_LEN 14
#define MAVLINK_MSG_ID_SYS_TM_MIN_LEN 14
#define MAVLINK_MSG_ID_200_LEN 14
#define MAVLINK_MSG_ID_200_MIN_LEN 14

#define MAVLINK_MSG_ID_SYS_TM_CRC 183
#define MAVLINK_MSG_ID_200_CRC 183



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SYS_TM { \
    200, \
    "SYS_TM", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sys_tm_t, timestamp) }, \
         { "logger", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_sys_tm_t, logger) }, \
         { "event_broker", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_sys_tm_t, event_broker) }, \
         { "radio", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_sys_tm_t, radio) }, \
         { "pin_observer", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_sys_tm_t, pin_observer) }, \
         { "sensors", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_sys_tm_t, sensors) }, \
         { "board_scheduler", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_sys_tm_t, board_scheduler) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SYS_TM { \
    "SYS_TM", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sys_tm_t, timestamp) }, \
         { "logger", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_sys_tm_t, logger) }, \
         { "event_broker", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_sys_tm_t, event_broker) }, \
         { "radio", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_sys_tm_t, radio) }, \
         { "pin_observer", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_sys_tm_t, pin_observer) }, \
         { "sensors", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_sys_tm_t, sensors) }, \
         { "board_scheduler", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_sys_tm_t, board_scheduler) }, \
         } \
}
#endif

/**
 * @brief Pack a sys_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param logger  True if the logger started correctly
 * @param event_broker  True if the event broker started correctly
 * @param radio  True if the radio started correctly
 * @param pin_observer  True if the pin observer started correctly
 * @param sensors  True if the sensors started correctly
 * @param board_scheduler  True if the board scheduler is running
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t logger, uint8_t event_broker, uint8_t radio, uint8_t pin_observer, uint8_t sensors, uint8_t board_scheduler)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, logger);
    _mav_put_uint8_t(buf, 9, event_broker);
    _mav_put_uint8_t(buf, 10, radio);
    _mav_put_uint8_t(buf, 11, pin_observer);
    _mav_put_uint8_t(buf, 12, sensors);
    _mav_put_uint8_t(buf, 13, board_scheduler);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_TM_LEN);
#else
    mavlink_sys_tm_t packet;
    packet.timestamp = timestamp;
    packet.logger = logger;
    packet.event_broker = event_broker;
    packet.radio = radio;
    packet.pin_observer = pin_observer;
    packet.sensors = sensors;
    packet.board_scheduler = board_scheduler;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
}

/**
 * @brief Pack a sys_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param logger  True if the logger started correctly
 * @param event_broker  True if the event broker started correctly
 * @param radio  True if the radio started correctly
 * @param pin_observer  True if the pin observer started correctly
 * @param sensors  True if the sensors started correctly
 * @param board_scheduler  True if the board scheduler is running
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t logger,uint8_t event_broker,uint8_t radio,uint8_t pin_observer,uint8_t sensors,uint8_t board_scheduler)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, logger);
    _mav_put_uint8_t(buf, 9, event_broker);
    _mav_put_uint8_t(buf, 10, radio);
    _mav_put_uint8_t(buf, 11, pin_observer);
    _mav_put_uint8_t(buf, 12, sensors);
    _mav_put_uint8_t(buf, 13, board_scheduler);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_TM_LEN);
#else
    mavlink_sys_tm_t packet;
    packet.timestamp = timestamp;
    packet.logger = logger;
    packet.event_broker = event_broker;
    packet.radio = radio;
    packet.pin_observer = pin_observer;
    packet.sensors = sensors;
    packet.board_scheduler = board_scheduler;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
}

/**
 * @brief Encode a sys_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sys_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sys_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sys_tm_t* sys_tm)
{
    return mavlink_msg_sys_tm_pack(system_id, component_id, msg, sys_tm->timestamp, sys_tm->logger, sys_tm->event_broker, sys_tm->radio, sys_tm->pin_observer, sys_tm->sensors, sys_tm->board_scheduler);
}

/**
 * @brief Encode a sys_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sys_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sys_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sys_tm_t* sys_tm)
{
    return mavlink_msg_sys_tm_pack_chan(system_id, component_id, chan, msg, sys_tm->timestamp, sys_tm->logger, sys_tm->event_broker, sys_tm->radio, sys_tm->pin_observer, sys_tm->sensors, sys_tm->board_scheduler);
}

/**
 * @brief Send a sys_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param logger  True if the logger started correctly
 * @param event_broker  True if the event broker started correctly
 * @param radio  True if the radio started correctly
 * @param pin_observer  True if the pin observer started correctly
 * @param sensors  True if the sensors started correctly
 * @param board_scheduler  True if the board scheduler is running
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sys_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t logger, uint8_t event_broker, uint8_t radio, uint8_t pin_observer, uint8_t sensors, uint8_t board_scheduler)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, logger);
    _mav_put_uint8_t(buf, 9, event_broker);
    _mav_put_uint8_t(buf, 10, radio);
    _mav_put_uint8_t(buf, 11, pin_observer);
    _mav_put_uint8_t(buf, 12, sensors);
    _mav_put_uint8_t(buf, 13, board_scheduler);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_TM, buf, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
#else
    mavlink_sys_tm_t packet;
    packet.timestamp = timestamp;
    packet.logger = logger;
    packet.event_broker = event_broker;
    packet.radio = radio;
    packet.pin_observer = pin_observer;
    packet.sensors = sensors;
    packet.board_scheduler = board_scheduler;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_TM, (const char *)&packet, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
#endif
}

/**
 * @brief Send a sys_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sys_tm_send_struct(mavlink_channel_t chan, const mavlink_sys_tm_t* sys_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sys_tm_send(chan, sys_tm->timestamp, sys_tm->logger, sys_tm->event_broker, sys_tm->radio, sys_tm->pin_observer, sys_tm->sensors, sys_tm->board_scheduler);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_TM, (const char *)sys_tm, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_SYS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sys_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t logger, uint8_t event_broker, uint8_t radio, uint8_t pin_observer, uint8_t sensors, uint8_t board_scheduler)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, logger);
    _mav_put_uint8_t(buf, 9, event_broker);
    _mav_put_uint8_t(buf, 10, radio);
    _mav_put_uint8_t(buf, 11, pin_observer);
    _mav_put_uint8_t(buf, 12, sensors);
    _mav_put_uint8_t(buf, 13, board_scheduler);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_TM, buf, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
#else
    mavlink_sys_tm_t *packet = (mavlink_sys_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->logger = logger;
    packet->event_broker = event_broker;
    packet->radio = radio;
    packet->pin_observer = pin_observer;
    packet->sensors = sensors;
    packet->board_scheduler = board_scheduler;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_TM, (const char *)packet, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE SYS_TM UNPACKING


/**
 * @brief Get field timestamp from sys_tm message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_sys_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field logger from sys_tm message
 *
 * @return  True if the logger started correctly
 */
static inline uint8_t mavlink_msg_sys_tm_get_logger(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field event_broker from sys_tm message
 *
 * @return  True if the event broker started correctly
 */
static inline uint8_t mavlink_msg_sys_tm_get_event_broker(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field radio from sys_tm message
 *
 * @return  True if the radio started correctly
 */
static inline uint8_t mavlink_msg_sys_tm_get_radio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field pin_observer from sys_tm message
 *
 * @return  True if the pin observer started correctly
 */
static inline uint8_t mavlink_msg_sys_tm_get_pin_observer(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field sensors from sys_tm message
 *
 * @return  True if the sensors started correctly
 */
static inline uint8_t mavlink_msg_sys_tm_get_sensors(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field board_scheduler from sys_tm message
 *
 * @return  True if the board scheduler is running
 */
static inline uint8_t mavlink_msg_sys_tm_get_board_scheduler(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a sys_tm message into a struct
 *
 * @param msg The message to decode
 * @param sys_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_sys_tm_decode(const mavlink_message_t* msg, mavlink_sys_tm_t* sys_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sys_tm->timestamp = mavlink_msg_sys_tm_get_timestamp(msg);
    sys_tm->logger = mavlink_msg_sys_tm_get_logger(msg);
    sys_tm->event_broker = mavlink_msg_sys_tm_get_event_broker(msg);
    sys_tm->radio = mavlink_msg_sys_tm_get_radio(msg);
    sys_tm->pin_observer = mavlink_msg_sys_tm_get_pin_observer(msg);
    sys_tm->sensors = mavlink_msg_sys_tm_get_sensors(msg);
    sys_tm->board_scheduler = mavlink_msg_sys_tm_get_board_scheduler(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SYS_TM_LEN? msg->len : MAVLINK_MSG_ID_SYS_TM_LEN;
        memset(sys_tm, 0, MAVLINK_MSG_ID_SYS_TM_LEN);
    memcpy(sys_tm, _MAV_PAYLOAD(msg), len);
#endif
}
