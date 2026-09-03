#pragma once
// MESSAGE SYS_TM PACKING

#define MAVLINK_MSG_ID_SYS_TM 160

MAVPACKED(
typedef struct __mavlink_sys_tm_t {
 uint64_t timestamp; /*<  Timestamp*/
 uint8_t death_stack; /*<  */
 uint8_t logger; /*<  */
 uint8_t ev_broker; /*<  */
 uint8_t pin_obs; /*<  */
 uint8_t fmm; /*<  */
 uint8_t sensor_manager; /*<  */
 uint8_t ada; /*<  */
 uint8_t tmtc; /*<  */
 uint8_t ign; /*<  */
 uint8_t dpl; /*<  */
}) mavlink_sys_tm_t;

#define MAVLINK_MSG_ID_SYS_TM_LEN 18
#define MAVLINK_MSG_ID_SYS_TM_MIN_LEN 18
#define MAVLINK_MSG_ID_160_LEN 18
#define MAVLINK_MSG_ID_160_MIN_LEN 18

#define MAVLINK_MSG_ID_SYS_TM_CRC 191
#define MAVLINK_MSG_ID_160_CRC 191



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SYS_TM { \
    160, \
    "SYS_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sys_tm_t, timestamp) }, \
         { "death_stack", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_sys_tm_t, death_stack) }, \
         { "logger", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_sys_tm_t, logger) }, \
         { "ev_broker", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_sys_tm_t, ev_broker) }, \
         { "pin_obs", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_sys_tm_t, pin_obs) }, \
         { "fmm", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_sys_tm_t, fmm) }, \
         { "sensor_manager", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_sys_tm_t, sensor_manager) }, \
         { "ada", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_sys_tm_t, ada) }, \
         { "tmtc", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_sys_tm_t, tmtc) }, \
         { "ign", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_sys_tm_t, ign) }, \
         { "dpl", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_sys_tm_t, dpl) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SYS_TM { \
    "SYS_TM", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sys_tm_t, timestamp) }, \
         { "death_stack", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_sys_tm_t, death_stack) }, \
         { "logger", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_sys_tm_t, logger) }, \
         { "ev_broker", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_sys_tm_t, ev_broker) }, \
         { "pin_obs", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_sys_tm_t, pin_obs) }, \
         { "fmm", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_sys_tm_t, fmm) }, \
         { "sensor_manager", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_sys_tm_t, sensor_manager) }, \
         { "ada", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_sys_tm_t, ada) }, \
         { "tmtc", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_sys_tm_t, tmtc) }, \
         { "ign", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_sys_tm_t, ign) }, \
         { "dpl", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_sys_tm_t, dpl) }, \
         } \
}
#endif

/**
 * @brief Pack a sys_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Timestamp
 * @param death_stack  
 * @param logger  
 * @param ev_broker  
 * @param pin_obs  
 * @param fmm  
 * @param sensor_manager  
 * @param ada  
 * @param tmtc  
 * @param ign  
 * @param dpl  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t death_stack, uint8_t logger, uint8_t ev_broker, uint8_t pin_obs, uint8_t fmm, uint8_t sensor_manager, uint8_t ada, uint8_t tmtc, uint8_t ign, uint8_t dpl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, death_stack);
    _mav_put_uint8_t(buf, 9, logger);
    _mav_put_uint8_t(buf, 10, ev_broker);
    _mav_put_uint8_t(buf, 11, pin_obs);
    _mav_put_uint8_t(buf, 12, fmm);
    _mav_put_uint8_t(buf, 13, sensor_manager);
    _mav_put_uint8_t(buf, 14, ada);
    _mav_put_uint8_t(buf, 15, tmtc);
    _mav_put_uint8_t(buf, 16, ign);
    _mav_put_uint8_t(buf, 17, dpl);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_TM_LEN);
#else
    mavlink_sys_tm_t packet;
    packet.timestamp = timestamp;
    packet.death_stack = death_stack;
    packet.logger = logger;
    packet.ev_broker = ev_broker;
    packet.pin_obs = pin_obs;
    packet.fmm = fmm;
    packet.sensor_manager = sensor_manager;
    packet.ada = ada;
    packet.tmtc = tmtc;
    packet.ign = ign;
    packet.dpl = dpl;

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
 * @param timestamp  Timestamp
 * @param death_stack  
 * @param logger  
 * @param ev_broker  
 * @param pin_obs  
 * @param fmm  
 * @param sensor_manager  
 * @param ada  
 * @param tmtc  
 * @param ign  
 * @param dpl  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t death_stack,uint8_t logger,uint8_t ev_broker,uint8_t pin_obs,uint8_t fmm,uint8_t sensor_manager,uint8_t ada,uint8_t tmtc,uint8_t ign,uint8_t dpl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, death_stack);
    _mav_put_uint8_t(buf, 9, logger);
    _mav_put_uint8_t(buf, 10, ev_broker);
    _mav_put_uint8_t(buf, 11, pin_obs);
    _mav_put_uint8_t(buf, 12, fmm);
    _mav_put_uint8_t(buf, 13, sensor_manager);
    _mav_put_uint8_t(buf, 14, ada);
    _mav_put_uint8_t(buf, 15, tmtc);
    _mav_put_uint8_t(buf, 16, ign);
    _mav_put_uint8_t(buf, 17, dpl);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_TM_LEN);
#else
    mavlink_sys_tm_t packet;
    packet.timestamp = timestamp;
    packet.death_stack = death_stack;
    packet.logger = logger;
    packet.ev_broker = ev_broker;
    packet.pin_obs = pin_obs;
    packet.fmm = fmm;
    packet.sensor_manager = sensor_manager;
    packet.ada = ada;
    packet.tmtc = tmtc;
    packet.ign = ign;
    packet.dpl = dpl;

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
    return mavlink_msg_sys_tm_pack(system_id, component_id, msg, sys_tm->timestamp, sys_tm->death_stack, sys_tm->logger, sys_tm->ev_broker, sys_tm->pin_obs, sys_tm->fmm, sys_tm->sensor_manager, sys_tm->ada, sys_tm->tmtc, sys_tm->ign, sys_tm->dpl);
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
    return mavlink_msg_sys_tm_pack_chan(system_id, component_id, chan, msg, sys_tm->timestamp, sys_tm->death_stack, sys_tm->logger, sys_tm->ev_broker, sys_tm->pin_obs, sys_tm->fmm, sys_tm->sensor_manager, sys_tm->ada, sys_tm->tmtc, sys_tm->ign, sys_tm->dpl);
}

/**
 * @brief Send a sys_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Timestamp
 * @param death_stack  
 * @param logger  
 * @param ev_broker  
 * @param pin_obs  
 * @param fmm  
 * @param sensor_manager  
 * @param ada  
 * @param tmtc  
 * @param ign  
 * @param dpl  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sys_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t death_stack, uint8_t logger, uint8_t ev_broker, uint8_t pin_obs, uint8_t fmm, uint8_t sensor_manager, uint8_t ada, uint8_t tmtc, uint8_t ign, uint8_t dpl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, death_stack);
    _mav_put_uint8_t(buf, 9, logger);
    _mav_put_uint8_t(buf, 10, ev_broker);
    _mav_put_uint8_t(buf, 11, pin_obs);
    _mav_put_uint8_t(buf, 12, fmm);
    _mav_put_uint8_t(buf, 13, sensor_manager);
    _mav_put_uint8_t(buf, 14, ada);
    _mav_put_uint8_t(buf, 15, tmtc);
    _mav_put_uint8_t(buf, 16, ign);
    _mav_put_uint8_t(buf, 17, dpl);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_TM, buf, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
#else
    mavlink_sys_tm_t packet;
    packet.timestamp = timestamp;
    packet.death_stack = death_stack;
    packet.logger = logger;
    packet.ev_broker = ev_broker;
    packet.pin_obs = pin_obs;
    packet.fmm = fmm;
    packet.sensor_manager = sensor_manager;
    packet.ada = ada;
    packet.tmtc = tmtc;
    packet.ign = ign;
    packet.dpl = dpl;

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
    mavlink_msg_sys_tm_send(chan, sys_tm->timestamp, sys_tm->death_stack, sys_tm->logger, sys_tm->ev_broker, sys_tm->pin_obs, sys_tm->fmm, sys_tm->sensor_manager, sys_tm->ada, sys_tm->tmtc, sys_tm->ign, sys_tm->dpl);
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
static inline void mavlink_msg_sys_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t death_stack, uint8_t logger, uint8_t ev_broker, uint8_t pin_obs, uint8_t fmm, uint8_t sensor_manager, uint8_t ada, uint8_t tmtc, uint8_t ign, uint8_t dpl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, death_stack);
    _mav_put_uint8_t(buf, 9, logger);
    _mav_put_uint8_t(buf, 10, ev_broker);
    _mav_put_uint8_t(buf, 11, pin_obs);
    _mav_put_uint8_t(buf, 12, fmm);
    _mav_put_uint8_t(buf, 13, sensor_manager);
    _mav_put_uint8_t(buf, 14, ada);
    _mav_put_uint8_t(buf, 15, tmtc);
    _mav_put_uint8_t(buf, 16, ign);
    _mav_put_uint8_t(buf, 17, dpl);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_TM, buf, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
#else
    mavlink_sys_tm_t *packet = (mavlink_sys_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->death_stack = death_stack;
    packet->logger = logger;
    packet->ev_broker = ev_broker;
    packet->pin_obs = pin_obs;
    packet->fmm = fmm;
    packet->sensor_manager = sensor_manager;
    packet->ada = ada;
    packet->tmtc = tmtc;
    packet->ign = ign;
    packet->dpl = dpl;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_TM, (const char *)packet, MAVLINK_MSG_ID_SYS_TM_MIN_LEN, MAVLINK_MSG_ID_SYS_TM_LEN, MAVLINK_MSG_ID_SYS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE SYS_TM UNPACKING


/**
 * @brief Get field timestamp from sys_tm message
 *
 * @return  Timestamp
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
 * @brief Get field fmm from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_fmm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field sensor_manager from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_sensor_manager(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field ada from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_ada(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field tmtc from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_tmtc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field ign from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_ign(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field dpl from sys_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sys_tm_get_dpl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
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
    sys_tm->fmm = mavlink_msg_sys_tm_get_fmm(msg);
    sys_tm->sensor_manager = mavlink_msg_sys_tm_get_sensor_manager(msg);
    sys_tm->ada = mavlink_msg_sys_tm_get_ada(msg);
    sys_tm->tmtc = mavlink_msg_sys_tm_get_tmtc(msg);
    sys_tm->ign = mavlink_msg_sys_tm_get_ign(msg);
    sys_tm->dpl = mavlink_msg_sys_tm_get_dpl(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SYS_TM_LEN? msg->len : MAVLINK_MSG_ID_SYS_TM_LEN;
        memset(sys_tm, 0, MAVLINK_MSG_ID_SYS_TM_LEN);
    memcpy(sys_tm, _MAV_PAYLOAD(msg), len);
#endif
}
