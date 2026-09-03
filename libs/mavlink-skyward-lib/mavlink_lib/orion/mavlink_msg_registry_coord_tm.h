#pragma once
// MESSAGE REGISTRY_COORD_TM PACKING

#define MAVLINK_MSG_ID_REGISTRY_COORD_TM 118


typedef struct __mavlink_registry_coord_tm_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 uint32_t key_id; /*<  Id of this configuration entry*/
 float latitude; /*< [deg] Latitude in this configuration*/
 float longitude; /*< [deg] Latitude in this configuration*/
 char key_name[20]; /*<  Name of this configuration entry*/
} mavlink_registry_coord_tm_t;

#define MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN 40
#define MAVLINK_MSG_ID_REGISTRY_COORD_TM_MIN_LEN 40
#define MAVLINK_MSG_ID_118_LEN 40
#define MAVLINK_MSG_ID_118_MIN_LEN 40

#define MAVLINK_MSG_ID_REGISTRY_COORD_TM_CRC 234
#define MAVLINK_MSG_ID_118_CRC 234

#define MAVLINK_MSG_REGISTRY_COORD_TM_FIELD_KEY_NAME_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_REGISTRY_COORD_TM { \
    118, \
    "REGISTRY_COORD_TM", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_registry_coord_tm_t, timestamp) }, \
         { "key_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_registry_coord_tm_t, key_id) }, \
         { "key_name", NULL, MAVLINK_TYPE_CHAR, 20, 20, offsetof(mavlink_registry_coord_tm_t, key_name) }, \
         { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_registry_coord_tm_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_registry_coord_tm_t, longitude) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_REGISTRY_COORD_TM { \
    "REGISTRY_COORD_TM", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_registry_coord_tm_t, timestamp) }, \
         { "key_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_registry_coord_tm_t, key_id) }, \
         { "key_name", NULL, MAVLINK_TYPE_CHAR, 20, 20, offsetof(mavlink_registry_coord_tm_t, key_name) }, \
         { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_registry_coord_tm_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_registry_coord_tm_t, longitude) }, \
         } \
}
#endif

/**
 * @brief Pack a registry_coord_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param key_id  Id of this configuration entry
 * @param key_name  Name of this configuration entry
 * @param latitude [deg] Latitude in this configuration
 * @param longitude [deg] Latitude in this configuration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_registry_coord_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t key_id, const char *key_name, float latitude, float longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, key_id);
    _mav_put_float(buf, 12, latitude);
    _mav_put_float(buf, 16, longitude);
    _mav_put_char_array(buf, 20, key_name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN);
#else
    mavlink_registry_coord_tm_t packet;
    packet.timestamp = timestamp;
    packet.key_id = key_id;
    packet.latitude = latitude;
    packet.longitude = longitude;
    mav_array_memcpy(packet.key_name, key_name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REGISTRY_COORD_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REGISTRY_COORD_TM_MIN_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_CRC);
}

/**
 * @brief Pack a registry_coord_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param key_id  Id of this configuration entry
 * @param key_name  Name of this configuration entry
 * @param latitude [deg] Latitude in this configuration
 * @param longitude [deg] Latitude in this configuration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_registry_coord_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t key_id,const char *key_name,float latitude,float longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, key_id);
    _mav_put_float(buf, 12, latitude);
    _mav_put_float(buf, 16, longitude);
    _mav_put_char_array(buf, 20, key_name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN);
#else
    mavlink_registry_coord_tm_t packet;
    packet.timestamp = timestamp;
    packet.key_id = key_id;
    packet.latitude = latitude;
    packet.longitude = longitude;
    mav_array_memcpy(packet.key_name, key_name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REGISTRY_COORD_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REGISTRY_COORD_TM_MIN_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_CRC);
}

/**
 * @brief Encode a registry_coord_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param registry_coord_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_registry_coord_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_registry_coord_tm_t* registry_coord_tm)
{
    return mavlink_msg_registry_coord_tm_pack(system_id, component_id, msg, registry_coord_tm->timestamp, registry_coord_tm->key_id, registry_coord_tm->key_name, registry_coord_tm->latitude, registry_coord_tm->longitude);
}

/**
 * @brief Encode a registry_coord_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param registry_coord_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_registry_coord_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_registry_coord_tm_t* registry_coord_tm)
{
    return mavlink_msg_registry_coord_tm_pack_chan(system_id, component_id, chan, msg, registry_coord_tm->timestamp, registry_coord_tm->key_id, registry_coord_tm->key_name, registry_coord_tm->latitude, registry_coord_tm->longitude);
}

/**
 * @brief Send a registry_coord_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param key_id  Id of this configuration entry
 * @param key_name  Name of this configuration entry
 * @param latitude [deg] Latitude in this configuration
 * @param longitude [deg] Latitude in this configuration
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_registry_coord_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t key_id, const char *key_name, float latitude, float longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, key_id);
    _mav_put_float(buf, 12, latitude);
    _mav_put_float(buf, 16, longitude);
    _mav_put_char_array(buf, 20, key_name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REGISTRY_COORD_TM, buf, MAVLINK_MSG_ID_REGISTRY_COORD_TM_MIN_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_CRC);
#else
    mavlink_registry_coord_tm_t packet;
    packet.timestamp = timestamp;
    packet.key_id = key_id;
    packet.latitude = latitude;
    packet.longitude = longitude;
    mav_array_memcpy(packet.key_name, key_name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REGISTRY_COORD_TM, (const char *)&packet, MAVLINK_MSG_ID_REGISTRY_COORD_TM_MIN_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_CRC);
#endif
}

/**
 * @brief Send a registry_coord_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_registry_coord_tm_send_struct(mavlink_channel_t chan, const mavlink_registry_coord_tm_t* registry_coord_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_registry_coord_tm_send(chan, registry_coord_tm->timestamp, registry_coord_tm->key_id, registry_coord_tm->key_name, registry_coord_tm->latitude, registry_coord_tm->longitude);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REGISTRY_COORD_TM, (const char *)registry_coord_tm, MAVLINK_MSG_ID_REGISTRY_COORD_TM_MIN_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_registry_coord_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t key_id, const char *key_name, float latitude, float longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, key_id);
    _mav_put_float(buf, 12, latitude);
    _mav_put_float(buf, 16, longitude);
    _mav_put_char_array(buf, 20, key_name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REGISTRY_COORD_TM, buf, MAVLINK_MSG_ID_REGISTRY_COORD_TM_MIN_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_CRC);
#else
    mavlink_registry_coord_tm_t *packet = (mavlink_registry_coord_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->key_id = key_id;
    packet->latitude = latitude;
    packet->longitude = longitude;
    mav_array_memcpy(packet->key_name, key_name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REGISTRY_COORD_TM, (const char *)packet, MAVLINK_MSG_ID_REGISTRY_COORD_TM_MIN_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN, MAVLINK_MSG_ID_REGISTRY_COORD_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE REGISTRY_COORD_TM UNPACKING


/**
 * @brief Get field timestamp from registry_coord_tm message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_registry_coord_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field key_id from registry_coord_tm message
 *
 * @return  Id of this configuration entry
 */
static inline uint32_t mavlink_msg_registry_coord_tm_get_key_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field key_name from registry_coord_tm message
 *
 * @return  Name of this configuration entry
 */
static inline uint16_t mavlink_msg_registry_coord_tm_get_key_name(const mavlink_message_t* msg, char *key_name)
{
    return _MAV_RETURN_char_array(msg, key_name, 20,  20);
}

/**
 * @brief Get field latitude from registry_coord_tm message
 *
 * @return [deg] Latitude in this configuration
 */
static inline float mavlink_msg_registry_coord_tm_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field longitude from registry_coord_tm message
 *
 * @return [deg] Latitude in this configuration
 */
static inline float mavlink_msg_registry_coord_tm_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a registry_coord_tm message into a struct
 *
 * @param msg The message to decode
 * @param registry_coord_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_registry_coord_tm_decode(const mavlink_message_t* msg, mavlink_registry_coord_tm_t* registry_coord_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    registry_coord_tm->timestamp = mavlink_msg_registry_coord_tm_get_timestamp(msg);
    registry_coord_tm->key_id = mavlink_msg_registry_coord_tm_get_key_id(msg);
    registry_coord_tm->latitude = mavlink_msg_registry_coord_tm_get_latitude(msg);
    registry_coord_tm->longitude = mavlink_msg_registry_coord_tm_get_longitude(msg);
    mavlink_msg_registry_coord_tm_get_key_name(msg, registry_coord_tm->key_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN? msg->len : MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN;
        memset(registry_coord_tm, 0, MAVLINK_MSG_ID_REGISTRY_COORD_TM_LEN);
    memcpy(registry_coord_tm, _MAV_PAYLOAD(msg), len);
#endif
}
