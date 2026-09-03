#pragma once
// MESSAGE HOMEONE_STATUS_TM PACKING

#define MAVLINK_MSG_ID_HOMEONE_STATUS_TM 140

MAVPACKED(
typedef struct __mavlink_homeone_status_tm_t {
 uint8_t sampling_status; /*<  True=Sampling enabled, False=Sampling disabled.*/
 uint8_t FMM_current_state; /*<  Value of FMM_STATE_ENUM.*/
 uint8_t log_status; /*<  True=Log active, False=Log not active.*/
 char umbilical_connection_status; /*<  True=Umbilical connected, False=Umbilical not connected,*/
 uint8_t fault_counter_array[15]; /*<  Array of all the fault counters in the system. //TODO set the correct size*/
}) mavlink_homeone_status_tm_t;

#define MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN 19
#define MAVLINK_MSG_ID_HOMEONE_STATUS_TM_MIN_LEN 19
#define MAVLINK_MSG_ID_140_LEN 19
#define MAVLINK_MSG_ID_140_MIN_LEN 19

#define MAVLINK_MSG_ID_HOMEONE_STATUS_TM_CRC 119
#define MAVLINK_MSG_ID_140_CRC 119

#define MAVLINK_MSG_HOMEONE_STATUS_TM_FIELD_FAULT_COUNTER_ARRAY_LEN 15

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HOMEONE_STATUS_TM { \
    140, \
    "HOMEONE_STATUS_TM", \
    5, \
    {  { "sampling_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_homeone_status_tm_t, sampling_status) }, \
         { "FMM_current_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_homeone_status_tm_t, FMM_current_state) }, \
         { "log_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_homeone_status_tm_t, log_status) }, \
         { "umbilical_connection_status", NULL, MAVLINK_TYPE_CHAR, 0, 3, offsetof(mavlink_homeone_status_tm_t, umbilical_connection_status) }, \
         { "fault_counter_array", NULL, MAVLINK_TYPE_UINT8_T, 15, 4, offsetof(mavlink_homeone_status_tm_t, fault_counter_array) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HOMEONE_STATUS_TM { \
    "HOMEONE_STATUS_TM", \
    5, \
    {  { "sampling_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_homeone_status_tm_t, sampling_status) }, \
         { "FMM_current_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_homeone_status_tm_t, FMM_current_state) }, \
         { "log_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_homeone_status_tm_t, log_status) }, \
         { "umbilical_connection_status", NULL, MAVLINK_TYPE_CHAR, 0, 3, offsetof(mavlink_homeone_status_tm_t, umbilical_connection_status) }, \
         { "fault_counter_array", NULL, MAVLINK_TYPE_UINT8_T, 15, 4, offsetof(mavlink_homeone_status_tm_t, fault_counter_array) }, \
         } \
}
#endif

/**
 * @brief Pack a homeone_status_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sampling_status  True=Sampling enabled, False=Sampling disabled.
 * @param FMM_current_state  Value of FMM_STATE_ENUM.
 * @param log_status  True=Log active, False=Log not active.
 * @param umbilical_connection_status  True=Umbilical connected, False=Umbilical not connected,
 * @param fault_counter_array  Array of all the fault counters in the system. //TODO set the correct size
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_homeone_status_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t sampling_status, uint8_t FMM_current_state, uint8_t log_status, char umbilical_connection_status, const uint8_t *fault_counter_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN];
    _mav_put_uint8_t(buf, 0, sampling_status);
    _mav_put_uint8_t(buf, 1, FMM_current_state);
    _mav_put_uint8_t(buf, 2, log_status);
    _mav_put_char(buf, 3, umbilical_connection_status);
    _mav_put_uint8_t_array(buf, 4, fault_counter_array, 15);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN);
#else
    mavlink_homeone_status_tm_t packet;
    packet.sampling_status = sampling_status;
    packet.FMM_current_state = FMM_current_state;
    packet.log_status = log_status;
    packet.umbilical_connection_status = umbilical_connection_status;
    mav_array_memcpy(packet.fault_counter_array, fault_counter_array, sizeof(uint8_t)*15);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HOMEONE_STATUS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_CRC);
}

/**
 * @brief Pack a homeone_status_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sampling_status  True=Sampling enabled, False=Sampling disabled.
 * @param FMM_current_state  Value of FMM_STATE_ENUM.
 * @param log_status  True=Log active, False=Log not active.
 * @param umbilical_connection_status  True=Umbilical connected, False=Umbilical not connected,
 * @param fault_counter_array  Array of all the fault counters in the system. //TODO set the correct size
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_homeone_status_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t sampling_status,uint8_t FMM_current_state,uint8_t log_status,char umbilical_connection_status,const uint8_t *fault_counter_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN];
    _mav_put_uint8_t(buf, 0, sampling_status);
    _mav_put_uint8_t(buf, 1, FMM_current_state);
    _mav_put_uint8_t(buf, 2, log_status);
    _mav_put_char(buf, 3, umbilical_connection_status);
    _mav_put_uint8_t_array(buf, 4, fault_counter_array, 15);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN);
#else
    mavlink_homeone_status_tm_t packet;
    packet.sampling_status = sampling_status;
    packet.FMM_current_state = FMM_current_state;
    packet.log_status = log_status;
    packet.umbilical_connection_status = umbilical_connection_status;
    mav_array_memcpy(packet.fault_counter_array, fault_counter_array, sizeof(uint8_t)*15);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HOMEONE_STATUS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_CRC);
}

/**
 * @brief Encode a homeone_status_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param homeone_status_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_homeone_status_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_homeone_status_tm_t* homeone_status_tm)
{
    return mavlink_msg_homeone_status_tm_pack(system_id, component_id, msg, homeone_status_tm->sampling_status, homeone_status_tm->FMM_current_state, homeone_status_tm->log_status, homeone_status_tm->umbilical_connection_status, homeone_status_tm->fault_counter_array);
}

/**
 * @brief Encode a homeone_status_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param homeone_status_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_homeone_status_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_homeone_status_tm_t* homeone_status_tm)
{
    return mavlink_msg_homeone_status_tm_pack_chan(system_id, component_id, chan, msg, homeone_status_tm->sampling_status, homeone_status_tm->FMM_current_state, homeone_status_tm->log_status, homeone_status_tm->umbilical_connection_status, homeone_status_tm->fault_counter_array);
}

/**
 * @brief Send a homeone_status_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param sampling_status  True=Sampling enabled, False=Sampling disabled.
 * @param FMM_current_state  Value of FMM_STATE_ENUM.
 * @param log_status  True=Log active, False=Log not active.
 * @param umbilical_connection_status  True=Umbilical connected, False=Umbilical not connected,
 * @param fault_counter_array  Array of all the fault counters in the system. //TODO set the correct size
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_homeone_status_tm_send(mavlink_channel_t chan, uint8_t sampling_status, uint8_t FMM_current_state, uint8_t log_status, char umbilical_connection_status, const uint8_t *fault_counter_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN];
    _mav_put_uint8_t(buf, 0, sampling_status);
    _mav_put_uint8_t(buf, 1, FMM_current_state);
    _mav_put_uint8_t(buf, 2, log_status);
    _mav_put_char(buf, 3, umbilical_connection_status);
    _mav_put_uint8_t_array(buf, 4, fault_counter_array, 15);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOMEONE_STATUS_TM, buf, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_CRC);
#else
    mavlink_homeone_status_tm_t packet;
    packet.sampling_status = sampling_status;
    packet.FMM_current_state = FMM_current_state;
    packet.log_status = log_status;
    packet.umbilical_connection_status = umbilical_connection_status;
    mav_array_memcpy(packet.fault_counter_array, fault_counter_array, sizeof(uint8_t)*15);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOMEONE_STATUS_TM, (const char *)&packet, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_CRC);
#endif
}

/**
 * @brief Send a homeone_status_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_homeone_status_tm_send_struct(mavlink_channel_t chan, const mavlink_homeone_status_tm_t* homeone_status_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_homeone_status_tm_send(chan, homeone_status_tm->sampling_status, homeone_status_tm->FMM_current_state, homeone_status_tm->log_status, homeone_status_tm->umbilical_connection_status, homeone_status_tm->fault_counter_array);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOMEONE_STATUS_TM, (const char *)homeone_status_tm, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_homeone_status_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t sampling_status, uint8_t FMM_current_state, uint8_t log_status, char umbilical_connection_status, const uint8_t *fault_counter_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, sampling_status);
    _mav_put_uint8_t(buf, 1, FMM_current_state);
    _mav_put_uint8_t(buf, 2, log_status);
    _mav_put_char(buf, 3, umbilical_connection_status);
    _mav_put_uint8_t_array(buf, 4, fault_counter_array, 15);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOMEONE_STATUS_TM, buf, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_CRC);
#else
    mavlink_homeone_status_tm_t *packet = (mavlink_homeone_status_tm_t *)msgbuf;
    packet->sampling_status = sampling_status;
    packet->FMM_current_state = FMM_current_state;
    packet->log_status = log_status;
    packet->umbilical_connection_status = umbilical_connection_status;
    mav_array_memcpy(packet->fault_counter_array, fault_counter_array, sizeof(uint8_t)*15);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOMEONE_STATUS_TM, (const char *)packet, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_MIN_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE HOMEONE_STATUS_TM UNPACKING


/**
 * @brief Get field sampling_status from homeone_status_tm message
 *
 * @return  True=Sampling enabled, False=Sampling disabled.
 */
static inline uint8_t mavlink_msg_homeone_status_tm_get_sampling_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field FMM_current_state from homeone_status_tm message
 *
 * @return  Value of FMM_STATE_ENUM.
 */
static inline uint8_t mavlink_msg_homeone_status_tm_get_FMM_current_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field log_status from homeone_status_tm message
 *
 * @return  True=Log active, False=Log not active.
 */
static inline uint8_t mavlink_msg_homeone_status_tm_get_log_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field umbilical_connection_status from homeone_status_tm message
 *
 * @return  True=Umbilical connected, False=Umbilical not connected,
 */
static inline char mavlink_msg_homeone_status_tm_get_umbilical_connection_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_char(msg,  3);
}

/**
 * @brief Get field fault_counter_array from homeone_status_tm message
 *
 * @return  Array of all the fault counters in the system. //TODO set the correct size
 */
static inline uint16_t mavlink_msg_homeone_status_tm_get_fault_counter_array(const mavlink_message_t* msg, uint8_t *fault_counter_array)
{
    return _MAV_RETURN_uint8_t_array(msg, fault_counter_array, 15,  4);
}

/**
 * @brief Decode a homeone_status_tm message into a struct
 *
 * @param msg The message to decode
 * @param homeone_status_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_homeone_status_tm_decode(const mavlink_message_t* msg, mavlink_homeone_status_tm_t* homeone_status_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    homeone_status_tm->sampling_status = mavlink_msg_homeone_status_tm_get_sampling_status(msg);
    homeone_status_tm->FMM_current_state = mavlink_msg_homeone_status_tm_get_FMM_current_state(msg);
    homeone_status_tm->log_status = mavlink_msg_homeone_status_tm_get_log_status(msg);
    homeone_status_tm->umbilical_connection_status = mavlink_msg_homeone_status_tm_get_umbilical_connection_status(msg);
    mavlink_msg_homeone_status_tm_get_fault_counter_array(msg, homeone_status_tm->fault_counter_array);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN? msg->len : MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN;
        memset(homeone_status_tm, 0, MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN);
    memcpy(homeone_status_tm, _MAV_PAYLOAD(msg), len);
#endif
}
