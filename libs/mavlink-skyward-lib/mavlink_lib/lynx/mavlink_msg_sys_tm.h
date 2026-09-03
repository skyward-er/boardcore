#pragma once
// MESSAGE SYS_TM PACKING

#define MAVLINK_MSG_ID_SYS_TM 160

MAVPACKED(
typedef struct __mavlink_sys_tm_t {
 uint64_t timestamp; /*< [ms] Timestamp*/
 uint8_t death_stack; /*<  */
 uint8_t logger; /*<  */
 uint8_t ev_broker; /*<  */
 uint8_t pin_obs; /*<  */
 uint8_t radio; /*<  */
 uint8_t state_machines; /*<  */
 uint8_t sensors; /*<  */
 uint8_t bmx160_status; /*<  */
 uint8_t ms5803_status; /*<  */
 uint8_t lis3mdl_status; /*<  */
 uint8_t gps_status; /*<  */
 uint8_t internal_adc_status; /*<  */
 uint8_t ads1118_status; /*<  */
}) mavlink_sys_tm_t;

#define MAVLINK_MSG_ID_SYS_TM_LEN 21
#define MAVLINK_MSG_ID_SYS_TM_MIN_LEN 21
#define MAVLINK_MSG_ID_160_LEN 21
#define MAVLINK_MSG_ID_160_MIN_LEN 21

#define MAVLINK_MSG_ID_SYS_TM_CRC 230
#define MAVLINK_MSG_ID_160_CRC 230



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SYS_TM { \
    160, \
    "SYS_TM", \
    14, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sys_tm_t, timestamp) }, \
         { "death_stack", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_sys_tm_t, death_stack) }, \
         { "logger", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_sys_tm_t, logger) }, \
         { "ev_broker", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_sys_tm_t, ev_broker) }, \
         { "pin_obs", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_sys_tm_t, pin_obs) }, \
         { "radio", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_sys_tm_t, radio) }, \
         { "state_machines", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_sys_tm_t, state_machines) }, \
         { "sensors", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_sys_tm_t, sensors) }, \
         { "bmx160_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_sys_tm_t, bmx160_status) }, \
         { "ms5803_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_sys_tm_t, ms5803_status) }, \
         { "lis3mdl_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_sys_tm_t, lis3mdl_status) }, \
         { "gps_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_sys_tm_t, gps_status) }, \
         { "internal_adc_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_sys_tm_t, internal_adc_status) }, \
         { "ads1118_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_sys_tm_t, ads1118_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SYS_TM { \
    "SYS_TM", \
    14, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sys_tm_t, timestamp) }, \
         { "death_stack", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_sys_tm_t, death_stack) }, \
         { "logger", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_sys_tm_t, logger) }, \
         { "ev_broker", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_sys_tm_t, ev_broker) }, \
         { "pin_obs", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_sys_tm_t, pin_obs) }, \
         { "radio", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_sys_tm_t, radio) }, \
         { "state_machines", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_sys_tm_t, state_machines) }, \
         { "sensors", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_sys_tm_t, sensors) }, \
         { "bmx160_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_sys_tm_t, bmx160_status) }, \
         { "ms5803_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_sys_tm_t, ms5803_status) }, \
         { "lis3mdl_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_sys_tm_t, lis3mdl_status) }, \
         { "gps_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_sys_tm_t, gps_status) }, \
         { "internal_adc_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_sys_tm_t, internal_adc_status) }, \
         { "ads1118_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_sys_tm_t, ads1118_status) }, \
         } \
}
#endif

/**
 * @brief Pack a sys_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] Timestamp
 * @param death_stack  
 * @param logger  
 * @param ev_broker  
 * @param pin_obs  
 * @param radio  
 * @param state_machines  
 * @param sensors  
 * @param bmx160_status  
 * @param ms5803_status  
 * @param lis3mdl_status  
 * @param gps_status  
 * @param internal_adc_status  
 * @param ads1118_status  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t death_stack, uint8_t logger, uint8_t ev_broker, uint8_t pin_obs, uint8_t radio, uint8_t state_machines, uint8_t sensors, uint8_t bmx160_status, uint8_t ms5803_status, uint8_t lis3mdl_status, uint8_t gps_status, uint8_t internal_adc_status, uint8_t ads1118_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, death_stack);
    _mav_put_uint8_t(buf, 9, logger);
    _mav_put_uint8_t(buf, 10, ev_broker);
    _mav_put_uint8_t(buf, 11, pin_obs);
    _mav_put_uint8_t(buf, 12, radio);
    _mav_put_uint8_t(buf, 13, state_machines);
    _mav_put_uint8_t(buf, 14, sensors);
    _mav_put_uint8_t(buf, 15, bmx160_status);
    _mav_put_uint8_t(buf, 16, ms5803_status);
    _mav_put_uint8_t(buf, 17, lis3mdl_status);
    _mav_put_uint8_t(buf, 18, gps_status);
    _mav_put_uint8_t(buf, 19, internal_adc_status);
    _mav_put_uint8_t(buf, 20, ads1118_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_TM_LEN);
#else
    mavlink_sys_tm_t packet;
    packet.timestamp = timestamp;
    packet.death_stack = death_stack;
    packet.logger = logger;
    packet.ev_broker = ev_broker;
    packet.pin_obs = pin_obs;
    packet.radio = radio;
    packet.state_machines = state_machines;
    packet.sensors = sensors;
    packet.bmx160_status = bmx160_status;
    packet.ms5803_status = ms5803_status;
    packet.lis3mdl_status = lis3mdl_status;
    packet.gps_status = gps_status;
    packet.internal_adc_status = internal_adc_status;
    packet.ads1118_status = ads1118_status;

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
 * @param timestamp [ms] Timestamp
 * @param death_stack  
 * @param logger  
 * @param ev_broker  
 * @param pin_obs  
 * @param radio  
 * @param state_machines  
 * @param sensors  
 * @param bmx160_status  
 * @param ms5803_status  
 * @param lis3mdl_status  
 * @param gps_status  
 * @param internal_adc_status  
 * @param ads1118_status  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t death_stack,uint8_t logger,uint8_t ev_broker,uint8_t pin_obs,uint8_t radio,uint8_t state_machines,uint8_t sensors,uint8_t bmx160_status,uint8_t ms5803_status,uint8_t lis3mdl_status,uint8_t gps_status,uint8_t internal_adc_status,uint8_t ads1118_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, death_stack);
    _mav_put_uint8_t(buf, 9, logger);
    _mav_put_uint8_t(buf, 10, ev_broker);
    _mav_put_uint8_t(buf, 11, pin_obs);
    _mav_put_uint8_t(buf, 12, radio);
    _mav_put_uint8_t(buf, 13, state_machines);
    _mav_put_uint8_t(buf, 14, sensors);
    _mav_put_uint8_t(buf, 15, bmx160_status);
    _mav_put_uint8_t(buf, 16, ms5803_status);
    _mav_put_uint8_t(buf, 17, lis3mdl_status);
    _mav_put_uint8_t(buf, 18, gps_status);
    _mav_put_uint8_t(buf, 19, internal_adc_status);
    _mav_put_uint8_t(buf, 20, ads1118_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_TM_LEN);
#else
    mavlink_sys_tm_t packet;
    packet.timestamp = timestamp;
    packet.death_stack = death_stack;
    packet.logger = logger;
    packet.ev_broker = ev_broker;
    packet.pin_obs = pin_obs;
    packet.radio = radio;
    packet.state_machines = state_machines;
    packet.sensors = sensors;
    packet.bmx160_status = bmx160_status;
    packet.ms5803_status = ms5803_status;
    packet.lis3mdl_status = lis3mdl_status;
    packet.gps_status = gps_status;
    packet.internal_adc_status = internal_adc_status;
    packet.ads1118_status = ads1118_status;

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
    return mavlink_msg_sys_tm_pack(system_id, component_id, msg, sys_tm->timestamp, sys_tm->death_stack, sys_tm->logger, sys_tm->ev_broker, sys_tm->pin_obs, sys_tm->radio, sys_tm->state_machines, sys_tm->sensors, sys_tm->bmx160_status, sys_tm->ms5803_status, sys_tm->lis3mdl_status, sys_tm->gps_status, sys_tm->internal_adc_status, sys_tm->ads1118_status);
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
    return mavlink_msg_sys_tm_pack_chan(system_id, component_id, chan, msg, sys_tm->timestamp, sys_tm->death_stack, sys_tm->logger, sys_tm->ev_broker, sys_tm->pin_obs, sys_tm->radio, sys_tm->state_machines, sys_tm->sensors, sys_tm->bmx160_status, sys_tm->ms5803_status, sys_tm->lis3mdl_status, sys_tm->gps_status, sys_tm->internal_adc_status, sys_tm->ads1118_status);
}

/**
 * @brief Send a sys_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] Timestamp
 * @param death_stack  
 * @param logger  
 * @param ev_broker  
 * @param pin_obs  
 * @param radio  
 * @param state_machines  
 * @param sensors  
 * @param bmx160_status  
 * @param ms5803_status  
 * @param lis3mdl_status  
 * @param gps_status  
 * @param internal_adc_status  
 * @param ads1118_status  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sys_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t death_stack, uint8_t logger, uint8_t ev_broker, uint8_t pin_obs, uint8_t radio, uint8_t state_machines, uint8_t sensors, uint8_t bmx160_status, uint8_t ms5803_status, uint8_t lis3mdl_status, uint8_t gps_status, uint8_t internal_adc_status, uint8_t ads1118_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, death_stack);
    _mav_put_uint8_t(buf, 9, logger);
    _mav_put_uint8_t(buf, 10, ev_broker);
    _mav_put_uint8_t(buf, 11, pin_obs);
    _mav_put_uint8_t(buf, 12, radio);
    _mav_put_uint8_t(buf, 13, state_machines);
    _mav_put_uint8_t(buf, 14, sensors);
    _mav_put_uint8_t(buf, 15, bmx160_status);
    _mav_put_uint8_t(buf, 16, ms5803_status);
    _mav_put_uint8_t(buf, 17, lis3mdl_status);
    _mav_put_uint8_t(buf, 18, gps_status);
    _mav_put_uint8_t(buf, 19, internal_adc_status);
    _mav_put_uint8_t(buf, 20, ads1118_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_TM, buf, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
#else
    mavlink_sys_tm_t packet;
    packet.timestamp = timestamp;
    packet.death_stack = death_stack;
    packet.logger = logger;
    packet.ev_broker = ev_broker;
    packet.pin_obs = pin_obs;
    packet.radio = radio;
    packet.state_machines = state_machines;
    packet.sensors = sensors;
    packet.bmx160_status = bmx160_status;
    packet.ms5803_status = ms5803_status;
    packet.lis3mdl_status = lis3mdl_status;
    packet.gps_status = gps_status;
    packet.internal_adc_status = internal_adc_status;
    packet.ads1118_status = ads1118_status;

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
    mavlink_msg_sys_tm_send(chan, sys_tm->timestamp, sys_tm->death_stack, sys_tm->logger, sys_tm->ev_broker, sys_tm->pin_obs, sys_tm->radio, sys_tm->state_machines, sys_tm->sensors, sys_tm->bmx160_status, sys_tm->ms5803_status, sys_tm->lis3mdl_status, sys_tm->gps_status, sys_tm->internal_adc_status, sys_tm->ads1118_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_TM, (const char *)sys_tm, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_SYS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sys_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t death_stack, uint8_t logger, uint8_t ev_broker, uint8_t pin_obs, uint8_t radio, uint8_t state_machines, uint8_t sensors, uint8_t bmx160_status, uint8_t ms5803_status, uint8_t lis3mdl_status, uint8_t gps_status, uint8_t internal_adc_status, uint8_t ads1118_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, death_stack);
    _mav_put_uint8_t(buf, 9, logger);
    _mav_put_uint8_t(buf, 10, ev_broker);
    _mav_put_uint8_t(buf, 11, pin_obs);
    _mav_put_uint8_t(buf, 12, radio);
    _mav_put_uint8_t(buf, 13, state_machines);
    _mav_put_uint8_t(buf, 14, sensors);
    _mav_put_uint8_t(buf, 15, bmx160_status);
    _mav_put_uint8_t(buf, 16, ms5803_status);
    _mav_put_uint8_t(buf, 17, lis3mdl_status);
    _mav_put_uint8_t(buf, 18, gps_status);
    _mav_put_uint8_t(buf, 19, internal_adc_status);
    _mav_put_uint8_t(buf, 20, ads1118_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_TM, buf, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
#else
    mavlink_sys_tm_t *packet = (mavlink_sys_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->death_stack = death_stack;
    packet->logger = logger;
    packet->ev_broker = ev_broker;
    packet->pin_obs = pin_obs;
    packet->radio = radio;
    packet->state_machines = state_machines;
    packet->sensors = sensors;
    packet->bmx160_status = bmx160_status;
    packet->ms5803_status = ms5803_status;
    packet->lis3mdl_status = lis3mdl_status;
    packet->gps_status = gps_status;
    packet->internal_adc_status = internal_adc_status;
    packet->ads1118_status = ads1118_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_TM, (const char *)packet, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE SYS_TM UNPACKING


/**
 * @brief Get field timestamp from sys_tm message
 *
 * @return [ms] Timestamp
 */
static inline uint64_t mavlink_msg_sys_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field death_stack from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_death_stack(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field logger from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_logger(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field ev_broker from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_ev_broker(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field pin_obs from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_pin_obs(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field radio from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_radio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field state_machines from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_state_machines(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field sensors from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_sensors(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field bmx160_status from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_bmx160_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field ms5803_status from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_ms5803_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field lis3mdl_status from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_lis3mdl_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field gps_status from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_gps_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field internal_adc_status from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_internal_adc_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field ads1118_status from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_ads1118_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
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
    sys_tm->death_stack = mavlink_msg_sys_tm_get_death_stack(msg);
    sys_tm->logger = mavlink_msg_sys_tm_get_logger(msg);
    sys_tm->ev_broker = mavlink_msg_sys_tm_get_ev_broker(msg);
    sys_tm->pin_obs = mavlink_msg_sys_tm_get_pin_obs(msg);
    sys_tm->radio = mavlink_msg_sys_tm_get_radio(msg);
    sys_tm->state_machines = mavlink_msg_sys_tm_get_state_machines(msg);
    sys_tm->sensors = mavlink_msg_sys_tm_get_sensors(msg);
    sys_tm->bmx160_status = mavlink_msg_sys_tm_get_bmx160_status(msg);
    sys_tm->ms5803_status = mavlink_msg_sys_tm_get_ms5803_status(msg);
    sys_tm->lis3mdl_status = mavlink_msg_sys_tm_get_lis3mdl_status(msg);
    sys_tm->gps_status = mavlink_msg_sys_tm_get_gps_status(msg);
    sys_tm->internal_adc_status = mavlink_msg_sys_tm_get_internal_adc_status(msg);
    sys_tm->ads1118_status = mavlink_msg_sys_tm_get_ads1118_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SYS_TM_LEN? msg->len : MAVLINK_MSG_ID_SYS_TM_LEN;
        memset(sys_tm, 0, MAVLINK_MSG_ID_SYS_TM_LEN);
    memcpy(sys_tm, _MAV_PAYLOAD(msg), len);
#endif
}
