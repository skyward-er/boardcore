#pragma once
// MESSAGE TMTC_TM PACKING

#define MAVLINK_MSG_ID_TMTC_TM 163

MAVPACKED(
typedef struct __mavlink_tmtc_tm_t {
 uint64_t timestamp; /*<  When was this logged */
 uint32_t parse_state; /*<   Parsing state machine*/
 uint16_t n_send_queue; /*<  Current len of the occupied portion of the queue*/
 uint16_t max_send_queue; /*<  Max occupied len of the queue */
 uint16_t n_send_errors; /*<  Number of packet not sent correctly by the TMTC*/
 uint16_t packet_rx_success_count; /*<   Received packets*/
 uint16_t packet_rx_drop_count; /*<    Number of packet drops   */
 uint8_t msg_received; /*<   Number of received messages*/
 uint8_t buffer_overrun; /*<   Number of buffer overruns*/
 uint8_t parse_error; /*<   Number of parse errors*/
 uint8_t packet_idx; /*<   Index in current packet*/
 uint8_t current_rx_seq; /*<   Sequence number of last packet received*/
 uint8_t current_tx_seq; /*<   Sequence number of last packet sent  */
}) mavlink_tmtc_tm_t;

#define MAVLINK_MSG_ID_TMTC_TM_LEN 28
#define MAVLINK_MSG_ID_TMTC_TM_MIN_LEN 28
#define MAVLINK_MSG_ID_163_LEN 28
#define MAVLINK_MSG_ID_163_MIN_LEN 28

#define MAVLINK_MSG_ID_TMTC_TM_CRC 164
#define MAVLINK_MSG_ID_163_CRC 164



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TMTC_TM { \
    163, \
    "TMTC_TM", \
    13, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tmtc_tm_t, timestamp) }, \
         { "n_send_queue", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_tmtc_tm_t, n_send_queue) }, \
         { "max_send_queue", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_tmtc_tm_t, max_send_queue) }, \
         { "n_send_errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_tmtc_tm_t, n_send_errors) }, \
         { "msg_received", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_tmtc_tm_t, msg_received) }, \
         { "buffer_overrun", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_tmtc_tm_t, buffer_overrun) }, \
         { "parse_error", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_tmtc_tm_t, parse_error) }, \
         { "parse_state", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_tmtc_tm_t, parse_state) }, \
         { "packet_idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_tmtc_tm_t, packet_idx) }, \
         { "current_rx_seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_tmtc_tm_t, current_rx_seq) }, \
         { "current_tx_seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_tmtc_tm_t, current_tx_seq) }, \
         { "packet_rx_success_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_tmtc_tm_t, packet_rx_success_count) }, \
         { "packet_rx_drop_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_tmtc_tm_t, packet_rx_drop_count) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TMTC_TM { \
    "TMTC_TM", \
    13, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tmtc_tm_t, timestamp) }, \
         { "n_send_queue", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_tmtc_tm_t, n_send_queue) }, \
         { "max_send_queue", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_tmtc_tm_t, max_send_queue) }, \
         { "n_send_errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_tmtc_tm_t, n_send_errors) }, \
         { "msg_received", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_tmtc_tm_t, msg_received) }, \
         { "buffer_overrun", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_tmtc_tm_t, buffer_overrun) }, \
         { "parse_error", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_tmtc_tm_t, parse_error) }, \
         { "parse_state", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_tmtc_tm_t, parse_state) }, \
         { "packet_idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_tmtc_tm_t, packet_idx) }, \
         { "current_rx_seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_tmtc_tm_t, current_rx_seq) }, \
         { "current_tx_seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_tmtc_tm_t, current_tx_seq) }, \
         { "packet_rx_success_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_tmtc_tm_t, packet_rx_success_count) }, \
         { "packet_rx_drop_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_tmtc_tm_t, packet_rx_drop_count) }, \
         } \
}
#endif

/**
 * @brief Pack a tmtc_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  When was this logged 
 * @param n_send_queue  Current len of the occupied portion of the queue
 * @param max_send_queue  Max occupied len of the queue 
 * @param n_send_errors  Number of packet not sent correctly by the TMTC
 * @param msg_received   Number of received messages
 * @param buffer_overrun   Number of buffer overruns
 * @param parse_error   Number of parse errors
 * @param parse_state   Parsing state machine
 * @param packet_idx   Index in current packet
 * @param current_rx_seq   Sequence number of last packet received
 * @param current_tx_seq   Sequence number of last packet sent  
 * @param packet_rx_success_count   Received packets
 * @param packet_rx_drop_count    Number of packet drops   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tmtc_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint16_t n_send_queue, uint16_t max_send_queue, uint16_t n_send_errors, uint8_t msg_received, uint8_t buffer_overrun, uint8_t parse_error, uint32_t parse_state, uint8_t packet_idx, uint8_t current_rx_seq, uint8_t current_tx_seq, uint16_t packet_rx_success_count, uint16_t packet_rx_drop_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TMTC_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, parse_state);
    _mav_put_uint16_t(buf, 12, n_send_queue);
    _mav_put_uint16_t(buf, 14, max_send_queue);
    _mav_put_uint16_t(buf, 16, n_send_errors);
    _mav_put_uint16_t(buf, 18, packet_rx_success_count);
    _mav_put_uint16_t(buf, 20, packet_rx_drop_count);
    _mav_put_uint8_t(buf, 22, msg_received);
    _mav_put_uint8_t(buf, 23, buffer_overrun);
    _mav_put_uint8_t(buf, 24, parse_error);
    _mav_put_uint8_t(buf, 25, packet_idx);
    _mav_put_uint8_t(buf, 26, current_rx_seq);
    _mav_put_uint8_t(buf, 27, current_tx_seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TMTC_TM_LEN);
#else
    mavlink_tmtc_tm_t packet;
    packet.timestamp = timestamp;
    packet.parse_state = parse_state;
    packet.n_send_queue = n_send_queue;
    packet.max_send_queue = max_send_queue;
    packet.n_send_errors = n_send_errors;
    packet.packet_rx_success_count = packet_rx_success_count;
    packet.packet_rx_drop_count = packet_rx_drop_count;
    packet.msg_received = msg_received;
    packet.buffer_overrun = buffer_overrun;
    packet.parse_error = parse_error;
    packet.packet_idx = packet_idx;
    packet.current_rx_seq = current_rx_seq;
    packet.current_tx_seq = current_tx_seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TMTC_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TMTC_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TMTC_TM_MIN_LEN, MAVLINK_MSG_ID_TMTC_TM_LEN, MAVLINK_MSG_ID_TMTC_TM_CRC);
}

/**
 * @brief Pack a tmtc_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  When was this logged 
 * @param n_send_queue  Current len of the occupied portion of the queue
 * @param max_send_queue  Max occupied len of the queue 
 * @param n_send_errors  Number of packet not sent correctly by the TMTC
 * @param msg_received   Number of received messages
 * @param buffer_overrun   Number of buffer overruns
 * @param parse_error   Number of parse errors
 * @param parse_state   Parsing state machine
 * @param packet_idx   Index in current packet
 * @param current_rx_seq   Sequence number of last packet received
 * @param current_tx_seq   Sequence number of last packet sent  
 * @param packet_rx_success_count   Received packets
 * @param packet_rx_drop_count    Number of packet drops   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tmtc_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint16_t n_send_queue,uint16_t max_send_queue,uint16_t n_send_errors,uint8_t msg_received,uint8_t buffer_overrun,uint8_t parse_error,uint32_t parse_state,uint8_t packet_idx,uint8_t current_rx_seq,uint8_t current_tx_seq,uint16_t packet_rx_success_count,uint16_t packet_rx_drop_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TMTC_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, parse_state);
    _mav_put_uint16_t(buf, 12, n_send_queue);
    _mav_put_uint16_t(buf, 14, max_send_queue);
    _mav_put_uint16_t(buf, 16, n_send_errors);
    _mav_put_uint16_t(buf, 18, packet_rx_success_count);
    _mav_put_uint16_t(buf, 20, packet_rx_drop_count);
    _mav_put_uint8_t(buf, 22, msg_received);
    _mav_put_uint8_t(buf, 23, buffer_overrun);
    _mav_put_uint8_t(buf, 24, parse_error);
    _mav_put_uint8_t(buf, 25, packet_idx);
    _mav_put_uint8_t(buf, 26, current_rx_seq);
    _mav_put_uint8_t(buf, 27, current_tx_seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TMTC_TM_LEN);
#else
    mavlink_tmtc_tm_t packet;
    packet.timestamp = timestamp;
    packet.parse_state = parse_state;
    packet.n_send_queue = n_send_queue;
    packet.max_send_queue = max_send_queue;
    packet.n_send_errors = n_send_errors;
    packet.packet_rx_success_count = packet_rx_success_count;
    packet.packet_rx_drop_count = packet_rx_drop_count;
    packet.msg_received = msg_received;
    packet.buffer_overrun = buffer_overrun;
    packet.parse_error = parse_error;
    packet.packet_idx = packet_idx;
    packet.current_rx_seq = current_rx_seq;
    packet.current_tx_seq = current_tx_seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TMTC_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TMTC_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TMTC_TM_MIN_LEN, MAVLINK_MSG_ID_TMTC_TM_LEN, MAVLINK_MSG_ID_TMTC_TM_CRC);
}

/**
 * @brief Encode a tmtc_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tmtc_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tmtc_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tmtc_tm_t* tmtc_tm)
{
    return mavlink_msg_tmtc_tm_pack(system_id, component_id, msg, tmtc_tm->timestamp, tmtc_tm->n_send_queue, tmtc_tm->max_send_queue, tmtc_tm->n_send_errors, tmtc_tm->msg_received, tmtc_tm->buffer_overrun, tmtc_tm->parse_error, tmtc_tm->parse_state, tmtc_tm->packet_idx, tmtc_tm->current_rx_seq, tmtc_tm->current_tx_seq, tmtc_tm->packet_rx_success_count, tmtc_tm->packet_rx_drop_count);
}

/**
 * @brief Encode a tmtc_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tmtc_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tmtc_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tmtc_tm_t* tmtc_tm)
{
    return mavlink_msg_tmtc_tm_pack_chan(system_id, component_id, chan, msg, tmtc_tm->timestamp, tmtc_tm->n_send_queue, tmtc_tm->max_send_queue, tmtc_tm->n_send_errors, tmtc_tm->msg_received, tmtc_tm->buffer_overrun, tmtc_tm->parse_error, tmtc_tm->parse_state, tmtc_tm->packet_idx, tmtc_tm->current_rx_seq, tmtc_tm->current_tx_seq, tmtc_tm->packet_rx_success_count, tmtc_tm->packet_rx_drop_count);
}

/**
 * @brief Send a tmtc_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  When was this logged 
 * @param n_send_queue  Current len of the occupied portion of the queue
 * @param max_send_queue  Max occupied len of the queue 
 * @param n_send_errors  Number of packet not sent correctly by the TMTC
 * @param msg_received   Number of received messages
 * @param buffer_overrun   Number of buffer overruns
 * @param parse_error   Number of parse errors
 * @param parse_state   Parsing state machine
 * @param packet_idx   Index in current packet
 * @param current_rx_seq   Sequence number of last packet received
 * @param current_tx_seq   Sequence number of last packet sent  
 * @param packet_rx_success_count   Received packets
 * @param packet_rx_drop_count    Number of packet drops   
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tmtc_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint16_t n_send_queue, uint16_t max_send_queue, uint16_t n_send_errors, uint8_t msg_received, uint8_t buffer_overrun, uint8_t parse_error, uint32_t parse_state, uint8_t packet_idx, uint8_t current_rx_seq, uint8_t current_tx_seq, uint16_t packet_rx_success_count, uint16_t packet_rx_drop_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TMTC_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, parse_state);
    _mav_put_uint16_t(buf, 12, n_send_queue);
    _mav_put_uint16_t(buf, 14, max_send_queue);
    _mav_put_uint16_t(buf, 16, n_send_errors);
    _mav_put_uint16_t(buf, 18, packet_rx_success_count);
    _mav_put_uint16_t(buf, 20, packet_rx_drop_count);
    _mav_put_uint8_t(buf, 22, msg_received);
    _mav_put_uint8_t(buf, 23, buffer_overrun);
    _mav_put_uint8_t(buf, 24, parse_error);
    _mav_put_uint8_t(buf, 25, packet_idx);
    _mav_put_uint8_t(buf, 26, current_rx_seq);
    _mav_put_uint8_t(buf, 27, current_tx_seq);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TMTC_TM, buf, MAVLINK_MSG_ID_TMTC_TM_MIN_LEN, MAVLINK_MSG_ID_TMTC_TM_LEN, MAVLINK_MSG_ID_TMTC_TM_CRC);
#else
    mavlink_tmtc_tm_t packet;
    packet.timestamp = timestamp;
    packet.parse_state = parse_state;
    packet.n_send_queue = n_send_queue;
    packet.max_send_queue = max_send_queue;
    packet.n_send_errors = n_send_errors;
    packet.packet_rx_success_count = packet_rx_success_count;
    packet.packet_rx_drop_count = packet_rx_drop_count;
    packet.msg_received = msg_received;
    packet.buffer_overrun = buffer_overrun;
    packet.parse_error = parse_error;
    packet.packet_idx = packet_idx;
    packet.current_rx_seq = current_rx_seq;
    packet.current_tx_seq = current_tx_seq;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TMTC_TM, (const char *)&packet, MAVLINK_MSG_ID_TMTC_TM_MIN_LEN, MAVLINK_MSG_ID_TMTC_TM_LEN, MAVLINK_MSG_ID_TMTC_TM_CRC);
#endif
}

/**
 * @brief Send a tmtc_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tmtc_tm_send_struct(mavlink_channel_t chan, const mavlink_tmtc_tm_t* tmtc_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tmtc_tm_send(chan, tmtc_tm->timestamp, tmtc_tm->n_send_queue, tmtc_tm->max_send_queue, tmtc_tm->n_send_errors, tmtc_tm->msg_received, tmtc_tm->buffer_overrun, tmtc_tm->parse_error, tmtc_tm->parse_state, tmtc_tm->packet_idx, tmtc_tm->current_rx_seq, tmtc_tm->current_tx_seq, tmtc_tm->packet_rx_success_count, tmtc_tm->packet_rx_drop_count);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TMTC_TM, (const char *)tmtc_tm, MAVLINK_MSG_ID_TMTC_TM_MIN_LEN, MAVLINK_MSG_ID_TMTC_TM_LEN, MAVLINK_MSG_ID_TMTC_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_TMTC_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tmtc_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint16_t n_send_queue, uint16_t max_send_queue, uint16_t n_send_errors, uint8_t msg_received, uint8_t buffer_overrun, uint8_t parse_error, uint32_t parse_state, uint8_t packet_idx, uint8_t current_rx_seq, uint8_t current_tx_seq, uint16_t packet_rx_success_count, uint16_t packet_rx_drop_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, parse_state);
    _mav_put_uint16_t(buf, 12, n_send_queue);
    _mav_put_uint16_t(buf, 14, max_send_queue);
    _mav_put_uint16_t(buf, 16, n_send_errors);
    _mav_put_uint16_t(buf, 18, packet_rx_success_count);
    _mav_put_uint16_t(buf, 20, packet_rx_drop_count);
    _mav_put_uint8_t(buf, 22, msg_received);
    _mav_put_uint8_t(buf, 23, buffer_overrun);
    _mav_put_uint8_t(buf, 24, parse_error);
    _mav_put_uint8_t(buf, 25, packet_idx);
    _mav_put_uint8_t(buf, 26, current_rx_seq);
    _mav_put_uint8_t(buf, 27, current_tx_seq);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TMTC_TM, buf, MAVLINK_MSG_ID_TMTC_TM_MIN_LEN, MAVLINK_MSG_ID_TMTC_TM_LEN, MAVLINK_MSG_ID_TMTC_TM_CRC);
#else
    mavlink_tmtc_tm_t *packet = (mavlink_tmtc_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->parse_state = parse_state;
    packet->n_send_queue = n_send_queue;
    packet->max_send_queue = max_send_queue;
    packet->n_send_errors = n_send_errors;
    packet->packet_rx_success_count = packet_rx_success_count;
    packet->packet_rx_drop_count = packet_rx_drop_count;
    packet->msg_received = msg_received;
    packet->buffer_overrun = buffer_overrun;
    packet->parse_error = parse_error;
    packet->packet_idx = packet_idx;
    packet->current_rx_seq = current_rx_seq;
    packet->current_tx_seq = current_tx_seq;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TMTC_TM, (const char *)packet, MAVLINK_MSG_ID_TMTC_TM_MIN_LEN, MAVLINK_MSG_ID_TMTC_TM_LEN, MAVLINK_MSG_ID_TMTC_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE TMTC_TM UNPACKING


/**
 * @brief Get field timestamp from tmtc_tm message
 *
 * @return  When was this logged 
 */
static inline uint64_t mavlink_msg_tmtc_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field n_send_queue from tmtc_tm message
 *
 * @return  Current len of the occupied portion of the queue
 */
static inline uint16_t mavlink_msg_tmtc_tm_get_n_send_queue(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field max_send_queue from tmtc_tm message
 *
 * @return  Max occupied len of the queue 
 */
static inline uint16_t mavlink_msg_tmtc_tm_get_max_send_queue(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field n_send_errors from tmtc_tm message
 *
 * @return  Number of packet not sent correctly by the TMTC
 */
static inline uint16_t mavlink_msg_tmtc_tm_get_n_send_errors(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field msg_received from tmtc_tm message
 *
 * @return   Number of received messages
 */
static inline uint8_t mavlink_msg_tmtc_tm_get_msg_received(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field buffer_overrun from tmtc_tm message
 *
 * @return   Number of buffer overruns
 */
static inline uint8_t mavlink_msg_tmtc_tm_get_buffer_overrun(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field parse_error from tmtc_tm message
 *
 * @return   Number of parse errors
 */
static inline uint8_t mavlink_msg_tmtc_tm_get_parse_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field parse_state from tmtc_tm message
 *
 * @return   Parsing state machine
 */
static inline uint32_t mavlink_msg_tmtc_tm_get_parse_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field packet_idx from tmtc_tm message
 *
 * @return   Index in current packet
 */
static inline uint8_t mavlink_msg_tmtc_tm_get_packet_idx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field current_rx_seq from tmtc_tm message
 *
 * @return   Sequence number of last packet received
 */
static inline uint8_t mavlink_msg_tmtc_tm_get_current_rx_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field current_tx_seq from tmtc_tm message
 *
 * @return   Sequence number of last packet sent  
 */
static inline uint8_t mavlink_msg_tmtc_tm_get_current_tx_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field packet_rx_success_count from tmtc_tm message
 *
 * @return   Received packets
 */
static inline uint16_t mavlink_msg_tmtc_tm_get_packet_rx_success_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field packet_rx_drop_count from tmtc_tm message
 *
 * @return    Number of packet drops   
 */
static inline uint16_t mavlink_msg_tmtc_tm_get_packet_rx_drop_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Decode a tmtc_tm message into a struct
 *
 * @param msg The message to decode
 * @param tmtc_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_tmtc_tm_decode(const mavlink_message_t* msg, mavlink_tmtc_tm_t* tmtc_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tmtc_tm->timestamp = mavlink_msg_tmtc_tm_get_timestamp(msg);
    tmtc_tm->parse_state = mavlink_msg_tmtc_tm_get_parse_state(msg);
    tmtc_tm->n_send_queue = mavlink_msg_tmtc_tm_get_n_send_queue(msg);
    tmtc_tm->max_send_queue = mavlink_msg_tmtc_tm_get_max_send_queue(msg);
    tmtc_tm->n_send_errors = mavlink_msg_tmtc_tm_get_n_send_errors(msg);
    tmtc_tm->packet_rx_success_count = mavlink_msg_tmtc_tm_get_packet_rx_success_count(msg);
    tmtc_tm->packet_rx_drop_count = mavlink_msg_tmtc_tm_get_packet_rx_drop_count(msg);
    tmtc_tm->msg_received = mavlink_msg_tmtc_tm_get_msg_received(msg);
    tmtc_tm->buffer_overrun = mavlink_msg_tmtc_tm_get_buffer_overrun(msg);
    tmtc_tm->parse_error = mavlink_msg_tmtc_tm_get_parse_error(msg);
    tmtc_tm->packet_idx = mavlink_msg_tmtc_tm_get_packet_idx(msg);
    tmtc_tm->current_rx_seq = mavlink_msg_tmtc_tm_get_current_rx_seq(msg);
    tmtc_tm->current_tx_seq = mavlink_msg_tmtc_tm_get_current_tx_seq(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TMTC_TM_LEN? msg->len : MAVLINK_MSG_ID_TMTC_TM_LEN;
        memset(tmtc_tm, 0, MAVLINK_MSG_ID_TMTC_TM_LEN);
    memcpy(tmtc_tm, _MAV_PAYLOAD(msg), len);
#endif
}
