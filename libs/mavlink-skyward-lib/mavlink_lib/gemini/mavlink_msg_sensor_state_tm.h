#pragma once
// MESSAGE SENSOR_STATE_TM PACKING

#define MAVLINK_MSG_ID_SENSOR_STATE_TM 111


typedef struct __mavlink_sensor_state_tm_t {
 char sensor_name[20]; /*<  Sensor name*/
 uint8_t state; /*<  Boolean that represents the init state*/
} mavlink_sensor_state_tm_t;

#define MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN 21
#define MAVLINK_MSG_ID_SENSOR_STATE_TM_MIN_LEN 21
#define MAVLINK_MSG_ID_111_LEN 21
#define MAVLINK_MSG_ID_111_MIN_LEN 21

#define MAVLINK_MSG_ID_SENSOR_STATE_TM_CRC 155
#define MAVLINK_MSG_ID_111_CRC 155

#define MAVLINK_MSG_SENSOR_STATE_TM_FIELD_SENSOR_NAME_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENSOR_STATE_TM { \
    111, \
    "SENSOR_STATE_TM", \
    2, \
    {  { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 20, 0, offsetof(mavlink_sensor_state_tm_t, sensor_name) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_sensor_state_tm_t, state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENSOR_STATE_TM { \
    "SENSOR_STATE_TM", \
    2, \
    {  { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 20, 0, offsetof(mavlink_sensor_state_tm_t, sensor_name) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_sensor_state_tm_t, state) }, \
         } \
}
#endif

/**
 * @brief Pack a sensor_state_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sensor_name  Sensor name
 * @param state  Boolean that represents the init state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_state_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *sensor_name, uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN];
    _mav_put_uint8_t(buf, 20, state);
    _mav_put_char_array(buf, 0, sensor_name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN);
#else
    mavlink_sensor_state_tm_t packet;
    packet.state = state;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_STATE_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSOR_STATE_TM_MIN_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_CRC);
}

/**
 * @brief Pack a sensor_state_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensor_name  Sensor name
 * @param state  Boolean that represents the init state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_state_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *sensor_name,uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN];
    _mav_put_uint8_t(buf, 20, state);
    _mav_put_char_array(buf, 0, sensor_name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN);
#else
    mavlink_sensor_state_tm_t packet;
    packet.state = state;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_STATE_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSOR_STATE_TM_MIN_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_CRC);
}

/**
 * @brief Encode a sensor_state_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor_state_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_state_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_state_tm_t* sensor_state_tm)
{
    return mavlink_msg_sensor_state_tm_pack(system_id, component_id, msg, sensor_state_tm->sensor_name, sensor_state_tm->state);
}

/**
 * @brief Encode a sensor_state_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensor_state_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_state_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensor_state_tm_t* sensor_state_tm)
{
    return mavlink_msg_sensor_state_tm_pack_chan(system_id, component_id, chan, msg, sensor_state_tm->sensor_name, sensor_state_tm->state);
}

/**
 * @brief Send a sensor_state_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param sensor_name  Sensor name
 * @param state  Boolean that represents the init state
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_state_tm_send(mavlink_channel_t chan, const char *sensor_name, uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN];
    _mav_put_uint8_t(buf, 20, state);
    _mav_put_char_array(buf, 0, sensor_name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_STATE_TM, buf, MAVLINK_MSG_ID_SENSOR_STATE_TM_MIN_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_CRC);
#else
    mavlink_sensor_state_tm_t packet;
    packet.state = state;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_STATE_TM, (const char *)&packet, MAVLINK_MSG_ID_SENSOR_STATE_TM_MIN_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_CRC);
#endif
}

/**
 * @brief Send a sensor_state_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sensor_state_tm_send_struct(mavlink_channel_t chan, const mavlink_sensor_state_tm_t* sensor_state_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sensor_state_tm_send(chan, sensor_state_tm->sensor_name, sensor_state_tm->state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_STATE_TM, (const char *)sensor_state_tm, MAVLINK_MSG_ID_SENSOR_STATE_TM_MIN_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sensor_state_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *sensor_name, uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 20, state);
    _mav_put_char_array(buf, 0, sensor_name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_STATE_TM, buf, MAVLINK_MSG_ID_SENSOR_STATE_TM_MIN_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_CRC);
#else
    mavlink_sensor_state_tm_t *packet = (mavlink_sensor_state_tm_t *)msgbuf;
    packet->state = state;
    mav_array_memcpy(packet->sensor_name, sensor_name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_STATE_TM, (const char *)packet, MAVLINK_MSG_ID_SENSOR_STATE_TM_MIN_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN, MAVLINK_MSG_ID_SENSOR_STATE_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE SENSOR_STATE_TM UNPACKING


/**
 * @brief Get field sensor_name from sensor_state_tm message
 *
 * @return  Sensor name
 */
static inline uint16_t mavlink_msg_sensor_state_tm_get_sensor_name(const mavlink_message_t* msg, char *sensor_name)
{
    return _MAV_RETURN_char_array(msg, sensor_name, 20,  0);
}

/**
 * @brief Get field state from sensor_state_tm message
 *
 * @return  Boolean that represents the init state
 */
static inline uint8_t mavlink_msg_sensor_state_tm_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a sensor_state_tm message into a struct
 *
 * @param msg The message to decode
 * @param sensor_state_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensor_state_tm_decode(const mavlink_message_t* msg, mavlink_sensor_state_tm_t* sensor_state_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sensor_state_tm_get_sensor_name(msg, sensor_state_tm->sensor_name);
    sensor_state_tm->state = mavlink_msg_sensor_state_tm_get_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN? msg->len : MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN;
        memset(sensor_state_tm, 0, MAVLINK_MSG_ID_SENSOR_STATE_TM_LEN);
    memcpy(sensor_state_tm, _MAV_PAYLOAD(msg), len);
#endif
}
