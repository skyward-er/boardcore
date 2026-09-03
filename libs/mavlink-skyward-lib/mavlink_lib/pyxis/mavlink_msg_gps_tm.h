#pragma once
// MESSAGE GPS_TM PACKING

#define MAVLINK_MSG_ID_GPS_TM 102


typedef struct __mavlink_gps_tm_t {
 uint64_t timestamp; /*< [us] When was this logged*/
 double latitude; /*< [deg] Latitude*/
 double longitude; /*< [deg] Longitude*/
 double height; /*< [m] Altitude*/
 float vel_north; /*< [m/s] Velocity in NED frame (north)*/
 float vel_east; /*< [m/s] Velocity in NED frame (east)*/
 float vel_down; /*< [m/s] Velocity in NED frame (down)*/
 float speed; /*< [m/s] Speed*/
 float track; /*< [deg] Track*/
 char sensor_id[20]; /*<  Sensor name*/
 uint8_t fix; /*<  Wether the GPS has a FIX*/
 uint8_t n_satellites; /*<  Number of connected satellites*/
} mavlink_gps_tm_t;

#define MAVLINK_MSG_ID_GPS_TM_LEN 74
#define MAVLINK_MSG_ID_GPS_TM_MIN_LEN 74
#define MAVLINK_MSG_ID_102_LEN 74
#define MAVLINK_MSG_ID_102_MIN_LEN 74

#define MAVLINK_MSG_ID_GPS_TM_CRC 39
#define MAVLINK_MSG_ID_102_CRC 39

#define MAVLINK_MSG_GPS_TM_FIELD_SENSOR_ID_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS_TM { \
    102, \
    "GPS_TM", \
    12, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_tm_t, timestamp) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_CHAR, 20, 52, offsetof(mavlink_gps_tm_t, sensor_id) }, \
         { "fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_gps_tm_t, fix) }, \
         { "latitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_gps_tm_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_gps_tm_t, longitude) }, \
         { "height", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_gps_tm_t, height) }, \
         { "vel_north", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gps_tm_t, vel_north) }, \
         { "vel_east", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gps_tm_t, vel_east) }, \
         { "vel_down", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gps_tm_t, vel_down) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_gps_tm_t, speed) }, \
         { "track", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_gps_tm_t, track) }, \
         { "n_satellites", NULL, MAVLINK_TYPE_UINT8_T, 0, 73, offsetof(mavlink_gps_tm_t, n_satellites) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS_TM { \
    "GPS_TM", \
    12, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_tm_t, timestamp) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_CHAR, 20, 52, offsetof(mavlink_gps_tm_t, sensor_id) }, \
         { "fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_gps_tm_t, fix) }, \
         { "latitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_gps_tm_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_gps_tm_t, longitude) }, \
         { "height", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_gps_tm_t, height) }, \
         { "vel_north", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gps_tm_t, vel_north) }, \
         { "vel_east", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gps_tm_t, vel_east) }, \
         { "vel_down", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gps_tm_t, vel_down) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_gps_tm_t, speed) }, \
         { "track", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_gps_tm_t, track) }, \
         { "n_satellites", NULL, MAVLINK_TYPE_UINT8_T, 0, 73, offsetof(mavlink_gps_tm_t, n_satellites) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] When was this logged
 * @param sensor_id  Sensor name
 * @param fix  Wether the GPS has a FIX
 * @param latitude [deg] Latitude
 * @param longitude [deg] Longitude
 * @param height [m] Altitude
 * @param vel_north [m/s] Velocity in NED frame (north)
 * @param vel_east [m/s] Velocity in NED frame (east)
 * @param vel_down [m/s] Velocity in NED frame (down)
 * @param speed [m/s] Speed
 * @param track [deg] Track
 * @param n_satellites  Number of connected satellites
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, const char *sensor_id, uint8_t fix, double latitude, double longitude, double height, float vel_north, float vel_east, float vel_down, float speed, float track, uint8_t n_satellites)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_double(buf, 8, latitude);
    _mav_put_double(buf, 16, longitude);
    _mav_put_double(buf, 24, height);
    _mav_put_float(buf, 32, vel_north);
    _mav_put_float(buf, 36, vel_east);
    _mav_put_float(buf, 40, vel_down);
    _mav_put_float(buf, 44, speed);
    _mav_put_float(buf, 48, track);
    _mav_put_uint8_t(buf, 72, fix);
    _mav_put_uint8_t(buf, 73, n_satellites);
    _mav_put_char_array(buf, 52, sensor_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_TM_LEN);
#else
    mavlink_gps_tm_t packet;
    packet.timestamp = timestamp;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.height = height;
    packet.vel_north = vel_north;
    packet.vel_east = vel_east;
    packet.vel_down = vel_down;
    packet.speed = speed;
    packet.track = track;
    packet.fix = fix;
    packet.n_satellites = n_satellites;
    mav_array_memcpy(packet.sensor_id, sensor_id, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_TM_MIN_LEN, MAVLINK_MSG_ID_GPS_TM_LEN, MAVLINK_MSG_ID_GPS_TM_CRC);
}

/**
 * @brief Pack a gps_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] When was this logged
 * @param sensor_id  Sensor name
 * @param fix  Wether the GPS has a FIX
 * @param latitude [deg] Latitude
 * @param longitude [deg] Longitude
 * @param height [m] Altitude
 * @param vel_north [m/s] Velocity in NED frame (north)
 * @param vel_east [m/s] Velocity in NED frame (east)
 * @param vel_down [m/s] Velocity in NED frame (down)
 * @param speed [m/s] Speed
 * @param track [deg] Track
 * @param n_satellites  Number of connected satellites
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,const char *sensor_id,uint8_t fix,double latitude,double longitude,double height,float vel_north,float vel_east,float vel_down,float speed,float track,uint8_t n_satellites)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_double(buf, 8, latitude);
    _mav_put_double(buf, 16, longitude);
    _mav_put_double(buf, 24, height);
    _mav_put_float(buf, 32, vel_north);
    _mav_put_float(buf, 36, vel_east);
    _mav_put_float(buf, 40, vel_down);
    _mav_put_float(buf, 44, speed);
    _mav_put_float(buf, 48, track);
    _mav_put_uint8_t(buf, 72, fix);
    _mav_put_uint8_t(buf, 73, n_satellites);
    _mav_put_char_array(buf, 52, sensor_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_TM_LEN);
#else
    mavlink_gps_tm_t packet;
    packet.timestamp = timestamp;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.height = height;
    packet.vel_north = vel_north;
    packet.vel_east = vel_east;
    packet.vel_down = vel_down;
    packet.speed = speed;
    packet.track = track;
    packet.fix = fix;
    packet.n_satellites = n_satellites;
    mav_array_memcpy(packet.sensor_id, sensor_id, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_TM_MIN_LEN, MAVLINK_MSG_ID_GPS_TM_LEN, MAVLINK_MSG_ID_GPS_TM_CRC);
}

/**
 * @brief Encode a gps_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_tm_t* gps_tm)
{
    return mavlink_msg_gps_tm_pack(system_id, component_id, msg, gps_tm->timestamp, gps_tm->sensor_id, gps_tm->fix, gps_tm->latitude, gps_tm->longitude, gps_tm->height, gps_tm->vel_north, gps_tm->vel_east, gps_tm->vel_down, gps_tm->speed, gps_tm->track, gps_tm->n_satellites);
}

/**
 * @brief Encode a gps_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_tm_t* gps_tm)
{
    return mavlink_msg_gps_tm_pack_chan(system_id, component_id, chan, msg, gps_tm->timestamp, gps_tm->sensor_id, gps_tm->fix, gps_tm->latitude, gps_tm->longitude, gps_tm->height, gps_tm->vel_north, gps_tm->vel_east, gps_tm->vel_down, gps_tm->speed, gps_tm->track, gps_tm->n_satellites);
}

/**
 * @brief Send a gps_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged
 * @param sensor_id  Sensor name
 * @param fix  Wether the GPS has a FIX
 * @param latitude [deg] Latitude
 * @param longitude [deg] Longitude
 * @param height [m] Altitude
 * @param vel_north [m/s] Velocity in NED frame (north)
 * @param vel_east [m/s] Velocity in NED frame (east)
 * @param vel_down [m/s] Velocity in NED frame (down)
 * @param speed [m/s] Speed
 * @param track [deg] Track
 * @param n_satellites  Number of connected satellites
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_tm_send(mavlink_channel_t chan, uint64_t timestamp, const char *sensor_id, uint8_t fix, double latitude, double longitude, double height, float vel_north, float vel_east, float vel_down, float speed, float track, uint8_t n_satellites)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_double(buf, 8, latitude);
    _mav_put_double(buf, 16, longitude);
    _mav_put_double(buf, 24, height);
    _mav_put_float(buf, 32, vel_north);
    _mav_put_float(buf, 36, vel_east);
    _mav_put_float(buf, 40, vel_down);
    _mav_put_float(buf, 44, speed);
    _mav_put_float(buf, 48, track);
    _mav_put_uint8_t(buf, 72, fix);
    _mav_put_uint8_t(buf, 73, n_satellites);
    _mav_put_char_array(buf, 52, sensor_id, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_TM, buf, MAVLINK_MSG_ID_GPS_TM_MIN_LEN, MAVLINK_MSG_ID_GPS_TM_LEN, MAVLINK_MSG_ID_GPS_TM_CRC);
#else
    mavlink_gps_tm_t packet;
    packet.timestamp = timestamp;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.height = height;
    packet.vel_north = vel_north;
    packet.vel_east = vel_east;
    packet.vel_down = vel_down;
    packet.speed = speed;
    packet.track = track;
    packet.fix = fix;
    packet.n_satellites = n_satellites;
    mav_array_memcpy(packet.sensor_id, sensor_id, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_TM, (const char *)&packet, MAVLINK_MSG_ID_GPS_TM_MIN_LEN, MAVLINK_MSG_ID_GPS_TM_LEN, MAVLINK_MSG_ID_GPS_TM_CRC);
#endif
}

/**
 * @brief Send a gps_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gps_tm_send_struct(mavlink_channel_t chan, const mavlink_gps_tm_t* gps_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gps_tm_send(chan, gps_tm->timestamp, gps_tm->sensor_id, gps_tm->fix, gps_tm->latitude, gps_tm->longitude, gps_tm->height, gps_tm->vel_north, gps_tm->vel_east, gps_tm->vel_down, gps_tm->speed, gps_tm->track, gps_tm->n_satellites);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_TM, (const char *)gps_tm, MAVLINK_MSG_ID_GPS_TM_MIN_LEN, MAVLINK_MSG_ID_GPS_TM_LEN, MAVLINK_MSG_ID_GPS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, const char *sensor_id, uint8_t fix, double latitude, double longitude, double height, float vel_north, float vel_east, float vel_down, float speed, float track, uint8_t n_satellites)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_double(buf, 8, latitude);
    _mav_put_double(buf, 16, longitude);
    _mav_put_double(buf, 24, height);
    _mav_put_float(buf, 32, vel_north);
    _mav_put_float(buf, 36, vel_east);
    _mav_put_float(buf, 40, vel_down);
    _mav_put_float(buf, 44, speed);
    _mav_put_float(buf, 48, track);
    _mav_put_uint8_t(buf, 72, fix);
    _mav_put_uint8_t(buf, 73, n_satellites);
    _mav_put_char_array(buf, 52, sensor_id, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_TM, buf, MAVLINK_MSG_ID_GPS_TM_MIN_LEN, MAVLINK_MSG_ID_GPS_TM_LEN, MAVLINK_MSG_ID_GPS_TM_CRC);
#else
    mavlink_gps_tm_t *packet = (mavlink_gps_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->height = height;
    packet->vel_north = vel_north;
    packet->vel_east = vel_east;
    packet->vel_down = vel_down;
    packet->speed = speed;
    packet->track = track;
    packet->fix = fix;
    packet->n_satellites = n_satellites;
    mav_array_memcpy(packet->sensor_id, sensor_id, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_TM, (const char *)packet, MAVLINK_MSG_ID_GPS_TM_MIN_LEN, MAVLINK_MSG_ID_GPS_TM_LEN, MAVLINK_MSG_ID_GPS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_TM UNPACKING


/**
 * @brief Get field timestamp from gps_tm message
 *
 * @return [us] When was this logged
 */
static inline uint64_t mavlink_msg_gps_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sensor_id from gps_tm message
 *
 * @return  Sensor name
 */
static inline uint16_t mavlink_msg_gps_tm_get_sensor_id(const mavlink_message_t* msg, char *sensor_id)
{
    return _MAV_RETURN_char_array(msg, sensor_id, 20,  52);
}

/**
 * @brief Get field fix from gps_tm message
 *
 * @return  Wether the GPS has a FIX
 */
static inline uint8_t mavlink_msg_gps_tm_get_fix(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  72);
}

/**
 * @brief Get field latitude from gps_tm message
 *
 * @return [deg] Latitude
 */
static inline double mavlink_msg_gps_tm_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field longitude from gps_tm message
 *
 * @return [deg] Longitude
 */
static inline double mavlink_msg_gps_tm_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field height from gps_tm message
 *
 * @return [m] Altitude
 */
static inline double mavlink_msg_gps_tm_get_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  24);
}

/**
 * @brief Get field vel_north from gps_tm message
 *
 * @return [m/s] Velocity in NED frame (north)
 */
static inline float mavlink_msg_gps_tm_get_vel_north(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field vel_east from gps_tm message
 *
 * @return [m/s] Velocity in NED frame (east)
 */
static inline float mavlink_msg_gps_tm_get_vel_east(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field vel_down from gps_tm message
 *
 * @return [m/s] Velocity in NED frame (down)
 */
static inline float mavlink_msg_gps_tm_get_vel_down(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field speed from gps_tm message
 *
 * @return [m/s] Speed
 */
static inline float mavlink_msg_gps_tm_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field track from gps_tm message
 *
 * @return [deg] Track
 */
static inline float mavlink_msg_gps_tm_get_track(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field n_satellites from gps_tm message
 *
 * @return  Number of connected satellites
 */
static inline uint8_t mavlink_msg_gps_tm_get_n_satellites(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  73);
}

/**
 * @brief Decode a gps_tm message into a struct
 *
 * @param msg The message to decode
 * @param gps_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_tm_decode(const mavlink_message_t* msg, mavlink_gps_tm_t* gps_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gps_tm->timestamp = mavlink_msg_gps_tm_get_timestamp(msg);
    gps_tm->latitude = mavlink_msg_gps_tm_get_latitude(msg);
    gps_tm->longitude = mavlink_msg_gps_tm_get_longitude(msg);
    gps_tm->height = mavlink_msg_gps_tm_get_height(msg);
    gps_tm->vel_north = mavlink_msg_gps_tm_get_vel_north(msg);
    gps_tm->vel_east = mavlink_msg_gps_tm_get_vel_east(msg);
    gps_tm->vel_down = mavlink_msg_gps_tm_get_vel_down(msg);
    gps_tm->speed = mavlink_msg_gps_tm_get_speed(msg);
    gps_tm->track = mavlink_msg_gps_tm_get_track(msg);
    mavlink_msg_gps_tm_get_sensor_id(msg, gps_tm->sensor_id);
    gps_tm->fix = mavlink_msg_gps_tm_get_fix(msg);
    gps_tm->n_satellites = mavlink_msg_gps_tm_get_n_satellites(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS_TM_LEN? msg->len : MAVLINK_MSG_ID_GPS_TM_LEN;
        memset(gps_tm, 0, MAVLINK_MSG_ID_GPS_TM_LEN);
    memcpy(gps_tm, _MAV_PAYLOAD(msg), len);
#endif
}
