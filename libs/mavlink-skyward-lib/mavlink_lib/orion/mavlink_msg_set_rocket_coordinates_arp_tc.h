#pragma once
// MESSAGE SET_ROCKET_COORDINATES_ARP_TC PACKING

#define MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC 64


typedef struct __mavlink_set_rocket_coordinates_arp_tc_t {
 float latitude; /*< [deg] Latitude*/
 float longitude; /*< [deg] Longitude*/
 float height; /*< [m] Height*/
} mavlink_set_rocket_coordinates_arp_tc_t;

#define MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN 12
#define MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_MIN_LEN 12
#define MAVLINK_MSG_ID_64_LEN 12
#define MAVLINK_MSG_ID_64_MIN_LEN 12

#define MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_CRC 220
#define MAVLINK_MSG_ID_64_CRC 220



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_ROCKET_COORDINATES_ARP_TC { \
    64, \
    "SET_ROCKET_COORDINATES_ARP_TC", \
    3, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_rocket_coordinates_arp_tc_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_rocket_coordinates_arp_tc_t, longitude) }, \
         { "height", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_rocket_coordinates_arp_tc_t, height) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_ROCKET_COORDINATES_ARP_TC { \
    "SET_ROCKET_COORDINATES_ARP_TC", \
    3, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_rocket_coordinates_arp_tc_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_rocket_coordinates_arp_tc_t, longitude) }, \
         { "height", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_rocket_coordinates_arp_tc_t, height) }, \
         } \
}
#endif

/**
 * @brief Pack a set_rocket_coordinates_arp_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude [deg] Latitude
 * @param longitude [deg] Longitude
 * @param height [m] Height
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_rocket_coordinates_arp_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float latitude, float longitude, float height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, height);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN);
#else
    mavlink_set_rocket_coordinates_arp_tc_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.height = height;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_CRC);
}

/**
 * @brief Pack a set_rocket_coordinates_arp_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude [deg] Latitude
 * @param longitude [deg] Longitude
 * @param height [m] Height
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_rocket_coordinates_arp_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float latitude,float longitude,float height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, height);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN);
#else
    mavlink_set_rocket_coordinates_arp_tc_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.height = height;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_CRC);
}

/**
 * @brief Encode a set_rocket_coordinates_arp_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_rocket_coordinates_arp_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_rocket_coordinates_arp_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_rocket_coordinates_arp_tc_t* set_rocket_coordinates_arp_tc)
{
    return mavlink_msg_set_rocket_coordinates_arp_tc_pack(system_id, component_id, msg, set_rocket_coordinates_arp_tc->latitude, set_rocket_coordinates_arp_tc->longitude, set_rocket_coordinates_arp_tc->height);
}

/**
 * @brief Encode a set_rocket_coordinates_arp_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_rocket_coordinates_arp_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_rocket_coordinates_arp_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_rocket_coordinates_arp_tc_t* set_rocket_coordinates_arp_tc)
{
    return mavlink_msg_set_rocket_coordinates_arp_tc_pack_chan(system_id, component_id, chan, msg, set_rocket_coordinates_arp_tc->latitude, set_rocket_coordinates_arp_tc->longitude, set_rocket_coordinates_arp_tc->height);
}

/**
 * @brief Send a set_rocket_coordinates_arp_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude [deg] Latitude
 * @param longitude [deg] Longitude
 * @param height [m] Height
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_rocket_coordinates_arp_tc_send(mavlink_channel_t chan, float latitude, float longitude, float height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, height);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC, buf, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_CRC);
#else
    mavlink_set_rocket_coordinates_arp_tc_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.height = height;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC, (const char *)&packet, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_CRC);
#endif
}

/**
 * @brief Send a set_rocket_coordinates_arp_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_rocket_coordinates_arp_tc_send_struct(mavlink_channel_t chan, const mavlink_set_rocket_coordinates_arp_tc_t* set_rocket_coordinates_arp_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_rocket_coordinates_arp_tc_send(chan, set_rocket_coordinates_arp_tc->latitude, set_rocket_coordinates_arp_tc->longitude, set_rocket_coordinates_arp_tc->height);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC, (const char *)set_rocket_coordinates_arp_tc, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_rocket_coordinates_arp_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float latitude, float longitude, float height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, height);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC, buf, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_CRC);
#else
    mavlink_set_rocket_coordinates_arp_tc_t *packet = (mavlink_set_rocket_coordinates_arp_tc_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->height = height;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC, (const char *)packet, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_MIN_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_ROCKET_COORDINATES_ARP_TC UNPACKING


/**
 * @brief Get field latitude from set_rocket_coordinates_arp_tc message
 *
 * @return [deg] Latitude
 */
static inline float mavlink_msg_set_rocket_coordinates_arp_tc_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field longitude from set_rocket_coordinates_arp_tc message
 *
 * @return [deg] Longitude
 */
static inline float mavlink_msg_set_rocket_coordinates_arp_tc_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field height from set_rocket_coordinates_arp_tc message
 *
 * @return [m] Height
 */
static inline float mavlink_msg_set_rocket_coordinates_arp_tc_get_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a set_rocket_coordinates_arp_tc message into a struct
 *
 * @param msg The message to decode
 * @param set_rocket_coordinates_arp_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_rocket_coordinates_arp_tc_decode(const mavlink_message_t* msg, mavlink_set_rocket_coordinates_arp_tc_t* set_rocket_coordinates_arp_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_rocket_coordinates_arp_tc->latitude = mavlink_msg_set_rocket_coordinates_arp_tc_get_latitude(msg);
    set_rocket_coordinates_arp_tc->longitude = mavlink_msg_set_rocket_coordinates_arp_tc_get_longitude(msg);
    set_rocket_coordinates_arp_tc->height = mavlink_msg_set_rocket_coordinates_arp_tc_get_height(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN? msg->len : MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN;
        memset(set_rocket_coordinates_arp_tc, 0, MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_LEN);
    memcpy(set_rocket_coordinates_arp_tc, _MAV_PAYLOAD(msg), len);
#endif
}
