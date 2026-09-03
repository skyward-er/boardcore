#pragma once
// MESSAGE FSM_TM PACKING

#define MAVLINK_MSG_ID_FSM_TM 201


typedef struct __mavlink_fsm_tm_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 uint8_t ada_state; /*<  Apogee Detection Algorithm state*/
 uint8_t abk_state; /*<  Air Brakes state*/
 uint8_t dpl_state; /*<  Deployment state*/
 uint8_t fmm_state; /*<  Flight Mode Manager state*/
 uint8_t nas_state; /*<  Navigation and Attitude System state*/
 uint8_t wes_state; /*<  Wind Estimation System state*/
} mavlink_fsm_tm_t;

#define MAVLINK_MSG_ID_FSM_TM_LEN 14
#define MAVLINK_MSG_ID_FSM_TM_MIN_LEN 14
#define MAVLINK_MSG_ID_201_LEN 14
#define MAVLINK_MSG_ID_201_MIN_LEN 14

#define MAVLINK_MSG_ID_FSM_TM_CRC 242
#define MAVLINK_MSG_ID_201_CRC 242



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FSM_TM { \
    201, \
    "FSM_TM", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_fsm_tm_t, timestamp) }, \
         { "ada_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_fsm_tm_t, ada_state) }, \
         { "abk_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_fsm_tm_t, abk_state) }, \
         { "dpl_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_fsm_tm_t, dpl_state) }, \
         { "fmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_fsm_tm_t, fmm_state) }, \
         { "nas_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_fsm_tm_t, nas_state) }, \
         { "wes_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_fsm_tm_t, wes_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FSM_TM { \
    "FSM_TM", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_fsm_tm_t, timestamp) }, \
         { "ada_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_fsm_tm_t, ada_state) }, \
         { "abk_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_fsm_tm_t, abk_state) }, \
         { "dpl_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_fsm_tm_t, dpl_state) }, \
         { "fmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_fsm_tm_t, fmm_state) }, \
         { "nas_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_fsm_tm_t, nas_state) }, \
         { "wes_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_fsm_tm_t, wes_state) }, \
         } \
}
#endif

/**
 * @brief Pack a fsm_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param ada_state  Apogee Detection Algorithm state
 * @param abk_state  Air Brakes state
 * @param dpl_state  Deployment state
 * @param fmm_state  Flight Mode Manager state
 * @param nas_state  Navigation and Attitude System state
 * @param wes_state  Wind Estimation System state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fsm_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t ada_state, uint8_t abk_state, uint8_t dpl_state, uint8_t fmm_state, uint8_t nas_state, uint8_t wes_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FSM_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, ada_state);
    _mav_put_uint8_t(buf, 9, abk_state);
    _mav_put_uint8_t(buf, 10, dpl_state);
    _mav_put_uint8_t(buf, 11, fmm_state);
    _mav_put_uint8_t(buf, 12, nas_state);
    _mav_put_uint8_t(buf, 13, wes_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FSM_TM_LEN);
#else
    mavlink_fsm_tm_t packet;
    packet.timestamp = timestamp;
    packet.ada_state = ada_state;
    packet.abk_state = abk_state;
    packet.dpl_state = dpl_state;
    packet.fmm_state = fmm_state;
    packet.nas_state = nas_state;
    packet.wes_state = wes_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FSM_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FSM_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FSM_TM_MIN_LEN, MAVLINK_MSG_ID_FSM_TM_LEN, MAVLINK_MSG_ID_FSM_TM_CRC);
}

/**
 * @brief Pack a fsm_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param ada_state  Apogee Detection Algorithm state
 * @param abk_state  Air Brakes state
 * @param dpl_state  Deployment state
 * @param fmm_state  Flight Mode Manager state
 * @param nas_state  Navigation and Attitude System state
 * @param wes_state  Wind Estimation System state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fsm_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t ada_state,uint8_t abk_state,uint8_t dpl_state,uint8_t fmm_state,uint8_t nas_state,uint8_t wes_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FSM_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, ada_state);
    _mav_put_uint8_t(buf, 9, abk_state);
    _mav_put_uint8_t(buf, 10, dpl_state);
    _mav_put_uint8_t(buf, 11, fmm_state);
    _mav_put_uint8_t(buf, 12, nas_state);
    _mav_put_uint8_t(buf, 13, wes_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FSM_TM_LEN);
#else
    mavlink_fsm_tm_t packet;
    packet.timestamp = timestamp;
    packet.ada_state = ada_state;
    packet.abk_state = abk_state;
    packet.dpl_state = dpl_state;
    packet.fmm_state = fmm_state;
    packet.nas_state = nas_state;
    packet.wes_state = wes_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FSM_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FSM_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FSM_TM_MIN_LEN, MAVLINK_MSG_ID_FSM_TM_LEN, MAVLINK_MSG_ID_FSM_TM_CRC);
}

/**
 * @brief Encode a fsm_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fsm_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fsm_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fsm_tm_t* fsm_tm)
{
    return mavlink_msg_fsm_tm_pack(system_id, component_id, msg, fsm_tm->timestamp, fsm_tm->ada_state, fsm_tm->abk_state, fsm_tm->dpl_state, fsm_tm->fmm_state, fsm_tm->nas_state, fsm_tm->wes_state);
}

/**
 * @brief Encode a fsm_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fsm_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fsm_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fsm_tm_t* fsm_tm)
{
    return mavlink_msg_fsm_tm_pack_chan(system_id, component_id, chan, msg, fsm_tm->timestamp, fsm_tm->ada_state, fsm_tm->abk_state, fsm_tm->dpl_state, fsm_tm->fmm_state, fsm_tm->nas_state, fsm_tm->wes_state);
}

/**
 * @brief Send a fsm_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param ada_state  Apogee Detection Algorithm state
 * @param abk_state  Air Brakes state
 * @param dpl_state  Deployment state
 * @param fmm_state  Flight Mode Manager state
 * @param nas_state  Navigation and Attitude System state
 * @param wes_state  Wind Estimation System state
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fsm_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t ada_state, uint8_t abk_state, uint8_t dpl_state, uint8_t fmm_state, uint8_t nas_state, uint8_t wes_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FSM_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, ada_state);
    _mav_put_uint8_t(buf, 9, abk_state);
    _mav_put_uint8_t(buf, 10, dpl_state);
    _mav_put_uint8_t(buf, 11, fmm_state);
    _mav_put_uint8_t(buf, 12, nas_state);
    _mav_put_uint8_t(buf, 13, wes_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FSM_TM, buf, MAVLINK_MSG_ID_FSM_TM_MIN_LEN, MAVLINK_MSG_ID_FSM_TM_LEN, MAVLINK_MSG_ID_FSM_TM_CRC);
#else
    mavlink_fsm_tm_t packet;
    packet.timestamp = timestamp;
    packet.ada_state = ada_state;
    packet.abk_state = abk_state;
    packet.dpl_state = dpl_state;
    packet.fmm_state = fmm_state;
    packet.nas_state = nas_state;
    packet.wes_state = wes_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FSM_TM, (const char *)&packet, MAVLINK_MSG_ID_FSM_TM_MIN_LEN, MAVLINK_MSG_ID_FSM_TM_LEN, MAVLINK_MSG_ID_FSM_TM_CRC);
#endif
}

/**
 * @brief Send a fsm_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_fsm_tm_send_struct(mavlink_channel_t chan, const mavlink_fsm_tm_t* fsm_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_fsm_tm_send(chan, fsm_tm->timestamp, fsm_tm->ada_state, fsm_tm->abk_state, fsm_tm->dpl_state, fsm_tm->fmm_state, fsm_tm->nas_state, fsm_tm->wes_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FSM_TM, (const char *)fsm_tm, MAVLINK_MSG_ID_FSM_TM_MIN_LEN, MAVLINK_MSG_ID_FSM_TM_LEN, MAVLINK_MSG_ID_FSM_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_FSM_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fsm_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t ada_state, uint8_t abk_state, uint8_t dpl_state, uint8_t fmm_state, uint8_t nas_state, uint8_t wes_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, ada_state);
    _mav_put_uint8_t(buf, 9, abk_state);
    _mav_put_uint8_t(buf, 10, dpl_state);
    _mav_put_uint8_t(buf, 11, fmm_state);
    _mav_put_uint8_t(buf, 12, nas_state);
    _mav_put_uint8_t(buf, 13, wes_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FSM_TM, buf, MAVLINK_MSG_ID_FSM_TM_MIN_LEN, MAVLINK_MSG_ID_FSM_TM_LEN, MAVLINK_MSG_ID_FSM_TM_CRC);
#else
    mavlink_fsm_tm_t *packet = (mavlink_fsm_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->ada_state = ada_state;
    packet->abk_state = abk_state;
    packet->dpl_state = dpl_state;
    packet->fmm_state = fmm_state;
    packet->nas_state = nas_state;
    packet->wes_state = wes_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FSM_TM, (const char *)packet, MAVLINK_MSG_ID_FSM_TM_MIN_LEN, MAVLINK_MSG_ID_FSM_TM_LEN, MAVLINK_MSG_ID_FSM_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE FSM_TM UNPACKING


/**
 * @brief Get field timestamp from fsm_tm message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_fsm_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field ada_state from fsm_tm message
 *
 * @return  Apogee Detection Algorithm state
 */
static inline uint8_t mavlink_msg_fsm_tm_get_ada_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field abk_state from fsm_tm message
 *
 * @return  Air Brakes state
 */
static inline uint8_t mavlink_msg_fsm_tm_get_abk_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field dpl_state from fsm_tm message
 *
 * @return  Deployment state
 */
static inline uint8_t mavlink_msg_fsm_tm_get_dpl_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field fmm_state from fsm_tm message
 *
 * @return  Flight Mode Manager state
 */
static inline uint8_t mavlink_msg_fsm_tm_get_fmm_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field nas_state from fsm_tm message
 *
 * @return  Navigation and Attitude System state
 */
static inline uint8_t mavlink_msg_fsm_tm_get_nas_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field wes_state from fsm_tm message
 *
 * @return  Wind Estimation System state
 */
static inline uint8_t mavlink_msg_fsm_tm_get_wes_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a fsm_tm message into a struct
 *
 * @param msg The message to decode
 * @param fsm_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_fsm_tm_decode(const mavlink_message_t* msg, mavlink_fsm_tm_t* fsm_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    fsm_tm->timestamp = mavlink_msg_fsm_tm_get_timestamp(msg);
    fsm_tm->ada_state = mavlink_msg_fsm_tm_get_ada_state(msg);
    fsm_tm->abk_state = mavlink_msg_fsm_tm_get_abk_state(msg);
    fsm_tm->dpl_state = mavlink_msg_fsm_tm_get_dpl_state(msg);
    fsm_tm->fmm_state = mavlink_msg_fsm_tm_get_fmm_state(msg);
    fsm_tm->nas_state = mavlink_msg_fsm_tm_get_nas_state(msg);
    fsm_tm->wes_state = mavlink_msg_fsm_tm_get_wes_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FSM_TM_LEN? msg->len : MAVLINK_MSG_ID_FSM_TM_LEN;
        memset(fsm_tm, 0, MAVLINK_MSG_ID_FSM_TM_LEN);
    memcpy(fsm_tm, _MAV_PAYLOAD(msg), len);
#endif
}
