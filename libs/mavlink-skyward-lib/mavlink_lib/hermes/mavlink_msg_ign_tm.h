#pragma once
// MESSAGE IGN_TM PACKING

#define MAVLINK_MSG_ID_IGN_TM 165

MAVPACKED(
typedef struct __mavlink_ign_tm_t {
 uint64_t timestamp; /*<  When was this logged*/
 uint16_t n_sent_messages; /*<  Number of sent messages on the Canbus*/
 uint16_t n_rcv_message; /*<  Number of messages received over the Canbus*/
 uint8_t fsm_state; /*<  State of the Controller's FSM*/
 uint8_t last_event; /*<  Last event received*/
 uint8_t cmd_bitfield; /*<  (LSB)launch_sent, abort_sent, abort_rcv(MSB)*/
 uint8_t stm32_bitfield; /*<  STM32 micro status bitfield*/
 uint8_t avr_bitfield; /*<  AVR micro status bitfield*/
}) mavlink_ign_tm_t;

#define MAVLINK_MSG_ID_IGN_TM_LEN 17
#define MAVLINK_MSG_ID_IGN_TM_MIN_LEN 17
#define MAVLINK_MSG_ID_165_LEN 17
#define MAVLINK_MSG_ID_165_MIN_LEN 17

#define MAVLINK_MSG_ID_IGN_TM_CRC 196
#define MAVLINK_MSG_ID_165_CRC 196



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IGN_TM { \
    165, \
    "IGN_TM", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ign_tm_t, timestamp) }, \
         { "fsm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_ign_tm_t, fsm_state) }, \
         { "last_event", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_ign_tm_t, last_event) }, \
         { "n_sent_messages", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_ign_tm_t, n_sent_messages) }, \
         { "n_rcv_message", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_ign_tm_t, n_rcv_message) }, \
         { "cmd_bitfield", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_ign_tm_t, cmd_bitfield) }, \
         { "stm32_bitfield", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_ign_tm_t, stm32_bitfield) }, \
         { "avr_bitfield", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_ign_tm_t, avr_bitfield) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IGN_TM { \
    "IGN_TM", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ign_tm_t, timestamp) }, \
         { "fsm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_ign_tm_t, fsm_state) }, \
         { "last_event", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_ign_tm_t, last_event) }, \
         { "n_sent_messages", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_ign_tm_t, n_sent_messages) }, \
         { "n_rcv_message", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_ign_tm_t, n_rcv_message) }, \
         { "cmd_bitfield", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_ign_tm_t, cmd_bitfield) }, \
         { "stm32_bitfield", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_ign_tm_t, stm32_bitfield) }, \
         { "avr_bitfield", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_ign_tm_t, avr_bitfield) }, \
         } \
}
#endif

/**
 * @brief Pack a ign_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  When was this logged
 * @param fsm_state  State of the Controller's FSM
 * @param last_event  Last event received
 * @param n_sent_messages  Number of sent messages on the Canbus
 * @param n_rcv_message  Number of messages received over the Canbus
 * @param cmd_bitfield  (LSB)launch_sent, abort_sent, abort_rcv(MSB)
 * @param stm32_bitfield  STM32 micro status bitfield
 * @param avr_bitfield  AVR micro status bitfield
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ign_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t fsm_state, uint8_t last_event, uint16_t n_sent_messages, uint16_t n_rcv_message, uint8_t cmd_bitfield, uint8_t stm32_bitfield, uint8_t avr_bitfield)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IGN_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 8, n_sent_messages);
    _mav_put_uint16_t(buf, 10, n_rcv_message);
    _mav_put_uint8_t(buf, 12, fsm_state);
    _mav_put_uint8_t(buf, 13, last_event);
    _mav_put_uint8_t(buf, 14, cmd_bitfield);
    _mav_put_uint8_t(buf, 15, stm32_bitfield);
    _mav_put_uint8_t(buf, 16, avr_bitfield);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IGN_TM_LEN);
#else
    mavlink_ign_tm_t packet;
    packet.timestamp = timestamp;
    packet.n_sent_messages = n_sent_messages;
    packet.n_rcv_message = n_rcv_message;
    packet.fsm_state = fsm_state;
    packet.last_event = last_event;
    packet.cmd_bitfield = cmd_bitfield;
    packet.stm32_bitfield = stm32_bitfield;
    packet.avr_bitfield = avr_bitfield;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IGN_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IGN_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IGN_TM_MIN_LEN, MAVLINK_MSG_ID_IGN_TM_LEN, MAVLINK_MSG_ID_IGN_TM_CRC);
}

/**
 * @brief Pack a ign_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  When was this logged
 * @param fsm_state  State of the Controller's FSM
 * @param last_event  Last event received
 * @param n_sent_messages  Number of sent messages on the Canbus
 * @param n_rcv_message  Number of messages received over the Canbus
 * @param cmd_bitfield  (LSB)launch_sent, abort_sent, abort_rcv(MSB)
 * @param stm32_bitfield  STM32 micro status bitfield
 * @param avr_bitfield  AVR micro status bitfield
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ign_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t fsm_state,uint8_t last_event,uint16_t n_sent_messages,uint16_t n_rcv_message,uint8_t cmd_bitfield,uint8_t stm32_bitfield,uint8_t avr_bitfield)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IGN_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 8, n_sent_messages);
    _mav_put_uint16_t(buf, 10, n_rcv_message);
    _mav_put_uint8_t(buf, 12, fsm_state);
    _mav_put_uint8_t(buf, 13, last_event);
    _mav_put_uint8_t(buf, 14, cmd_bitfield);
    _mav_put_uint8_t(buf, 15, stm32_bitfield);
    _mav_put_uint8_t(buf, 16, avr_bitfield);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IGN_TM_LEN);
#else
    mavlink_ign_tm_t packet;
    packet.timestamp = timestamp;
    packet.n_sent_messages = n_sent_messages;
    packet.n_rcv_message = n_rcv_message;
    packet.fsm_state = fsm_state;
    packet.last_event = last_event;
    packet.cmd_bitfield = cmd_bitfield;
    packet.stm32_bitfield = stm32_bitfield;
    packet.avr_bitfield = avr_bitfield;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IGN_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IGN_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IGN_TM_MIN_LEN, MAVLINK_MSG_ID_IGN_TM_LEN, MAVLINK_MSG_ID_IGN_TM_CRC);
}

/**
 * @brief Encode a ign_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ign_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ign_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ign_tm_t* ign_tm)
{
    return mavlink_msg_ign_tm_pack(system_id, component_id, msg, ign_tm->timestamp, ign_tm->fsm_state, ign_tm->last_event, ign_tm->n_sent_messages, ign_tm->n_rcv_message, ign_tm->cmd_bitfield, ign_tm->stm32_bitfield, ign_tm->avr_bitfield);
}

/**
 * @brief Encode a ign_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ign_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ign_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ign_tm_t* ign_tm)
{
    return mavlink_msg_ign_tm_pack_chan(system_id, component_id, chan, msg, ign_tm->timestamp, ign_tm->fsm_state, ign_tm->last_event, ign_tm->n_sent_messages, ign_tm->n_rcv_message, ign_tm->cmd_bitfield, ign_tm->stm32_bitfield, ign_tm->avr_bitfield);
}

/**
 * @brief Send a ign_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  When was this logged
 * @param fsm_state  State of the Controller's FSM
 * @param last_event  Last event received
 * @param n_sent_messages  Number of sent messages on the Canbus
 * @param n_rcv_message  Number of messages received over the Canbus
 * @param cmd_bitfield  (LSB)launch_sent, abort_sent, abort_rcv(MSB)
 * @param stm32_bitfield  STM32 micro status bitfield
 * @param avr_bitfield  AVR micro status bitfield
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ign_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t fsm_state, uint8_t last_event, uint16_t n_sent_messages, uint16_t n_rcv_message, uint8_t cmd_bitfield, uint8_t stm32_bitfield, uint8_t avr_bitfield)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IGN_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 8, n_sent_messages);
    _mav_put_uint16_t(buf, 10, n_rcv_message);
    _mav_put_uint8_t(buf, 12, fsm_state);
    _mav_put_uint8_t(buf, 13, last_event);
    _mav_put_uint8_t(buf, 14, cmd_bitfield);
    _mav_put_uint8_t(buf, 15, stm32_bitfield);
    _mav_put_uint8_t(buf, 16, avr_bitfield);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IGN_TM, buf, MAVLINK_MSG_ID_IGN_TM_MIN_LEN, MAVLINK_MSG_ID_IGN_TM_LEN, MAVLINK_MSG_ID_IGN_TM_CRC);
#else
    mavlink_ign_tm_t packet;
    packet.timestamp = timestamp;
    packet.n_sent_messages = n_sent_messages;
    packet.n_rcv_message = n_rcv_message;
    packet.fsm_state = fsm_state;
    packet.last_event = last_event;
    packet.cmd_bitfield = cmd_bitfield;
    packet.stm32_bitfield = stm32_bitfield;
    packet.avr_bitfield = avr_bitfield;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IGN_TM, (const char *)&packet, MAVLINK_MSG_ID_IGN_TM_MIN_LEN, MAVLINK_MSG_ID_IGN_TM_LEN, MAVLINK_MSG_ID_IGN_TM_CRC);
#endif
}

/**
 * @brief Send a ign_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ign_tm_send_struct(mavlink_channel_t chan, const mavlink_ign_tm_t* ign_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ign_tm_send(chan, ign_tm->timestamp, ign_tm->fsm_state, ign_tm->last_event, ign_tm->n_sent_messages, ign_tm->n_rcv_message, ign_tm->cmd_bitfield, ign_tm->stm32_bitfield, ign_tm->avr_bitfield);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IGN_TM, (const char *)ign_tm, MAVLINK_MSG_ID_IGN_TM_MIN_LEN, MAVLINK_MSG_ID_IGN_TM_LEN, MAVLINK_MSG_ID_IGN_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_IGN_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ign_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t fsm_state, uint8_t last_event, uint16_t n_sent_messages, uint16_t n_rcv_message, uint8_t cmd_bitfield, uint8_t stm32_bitfield, uint8_t avr_bitfield)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 8, n_sent_messages);
    _mav_put_uint16_t(buf, 10, n_rcv_message);
    _mav_put_uint8_t(buf, 12, fsm_state);
    _mav_put_uint8_t(buf, 13, last_event);
    _mav_put_uint8_t(buf, 14, cmd_bitfield);
    _mav_put_uint8_t(buf, 15, stm32_bitfield);
    _mav_put_uint8_t(buf, 16, avr_bitfield);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IGN_TM, buf, MAVLINK_MSG_ID_IGN_TM_MIN_LEN, MAVLINK_MSG_ID_IGN_TM_LEN, MAVLINK_MSG_ID_IGN_TM_CRC);
#else
    mavlink_ign_tm_t *packet = (mavlink_ign_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->n_sent_messages = n_sent_messages;
    packet->n_rcv_message = n_rcv_message;
    packet->fsm_state = fsm_state;
    packet->last_event = last_event;
    packet->cmd_bitfield = cmd_bitfield;
    packet->stm32_bitfield = stm32_bitfield;
    packet->avr_bitfield = avr_bitfield;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IGN_TM, (const char *)packet, MAVLINK_MSG_ID_IGN_TM_MIN_LEN, MAVLINK_MSG_ID_IGN_TM_LEN, MAVLINK_MSG_ID_IGN_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE IGN_TM UNPACKING


/**
 * @brief Get field timestamp from ign_tm message
 *
 * @return  When was this logged
 */
static inline uint64_t mavlink_msg_ign_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field fsm_state from ign_tm message
 *
 * @return  State of the Controller's FSM
 */
static inline uint8_t mavlink_msg_ign_tm_get_fsm_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field last_event from ign_tm message
 *
 * @return  Last event received
 */
static inline uint8_t mavlink_msg_ign_tm_get_last_event(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field n_sent_messages from ign_tm message
 *
 * @return  Number of sent messages on the Canbus
 */
static inline uint16_t mavlink_msg_ign_tm_get_n_sent_messages(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field n_rcv_message from ign_tm message
 *
 * @return  Number of messages received over the Canbus
 */
static inline uint16_t mavlink_msg_ign_tm_get_n_rcv_message(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field cmd_bitfield from ign_tm message
 *
 * @return  (LSB)launch_sent, abort_sent, abort_rcv(MSB)
 */
static inline uint8_t mavlink_msg_ign_tm_get_cmd_bitfield(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field stm32_bitfield from ign_tm message
 *
 * @return  STM32 micro status bitfield
 */
static inline uint8_t mavlink_msg_ign_tm_get_stm32_bitfield(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field avr_bitfield from ign_tm message
 *
 * @return  AVR micro status bitfield
 */
static inline uint8_t mavlink_msg_ign_tm_get_avr_bitfield(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Decode a ign_tm message into a struct
 *
 * @param msg The message to decode
 * @param ign_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_ign_tm_decode(const mavlink_message_t* msg, mavlink_ign_tm_t* ign_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ign_tm->timestamp = mavlink_msg_ign_tm_get_timestamp(msg);
    ign_tm->n_sent_messages = mavlink_msg_ign_tm_get_n_sent_messages(msg);
    ign_tm->n_rcv_message = mavlink_msg_ign_tm_get_n_rcv_message(msg);
    ign_tm->fsm_state = mavlink_msg_ign_tm_get_fsm_state(msg);
    ign_tm->last_event = mavlink_msg_ign_tm_get_last_event(msg);
    ign_tm->cmd_bitfield = mavlink_msg_ign_tm_get_cmd_bitfield(msg);
    ign_tm->stm32_bitfield = mavlink_msg_ign_tm_get_stm32_bitfield(msg);
    ign_tm->avr_bitfield = mavlink_msg_ign_tm_get_avr_bitfield(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IGN_TM_LEN? msg->len : MAVLINK_MSG_ID_IGN_TM_LEN;
        memset(ign_tm, 0, MAVLINK_MSG_ID_IGN_TM_LEN);
    memcpy(ign_tm, _MAV_PAYLOAD(msg), len);
#endif
}
