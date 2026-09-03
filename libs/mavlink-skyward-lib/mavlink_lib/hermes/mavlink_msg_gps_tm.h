#pragma once
// MESSAGE GPS_TM PACKING

#define MAVLINK_MSG_ID_GPS_TM 172

MAVPACKED(
typedef struct __mavlink_gps_tm_t {
 uint64_t timestamp; /*<  When was this logged*/
 double lat; /*<  Latitude*/
 double lon; /*<  Longitude*/
 double altitude; /*<  Altitude*/
 float vel_north; /*<  Velocity in NED frame (north)*/
 float vel_east; /*<  Velocity in NED frame (east)*/
 float vel_down; /*<  Velocity in NED frame (down)*/
 float vel_mag; /*<  Speed*/
 int32_t n_satellites; /*<  Number of connected satellites*/
 uint8_t fix; /*<  Wether the GPS has a FIX*/
}) mavlink_gps_tm_t;

#define MAVLINK_MSG_ID_GPS_TM_LEN 53
#define MAVLINK_MSG_ID_GPS_TM_MIN_LEN 53
#define MAVLINK_MSG_ID_172_LEN 53
#define MAVLINK_MSG_ID_172_MIN_LEN 53

#define MAVLINK_MSG_ID_GPS_TM_CRC 204
#define MAVLINK_MSG_ID_172_CRC 204



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS_TM { \
    172, \
    "GPS_TM", \
    10, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_tm_t, timestamp) }, \
         { "fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_gps_tm_t, fix) }, \
         { "lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_gps_tm_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_gps_tm_t, lon) }, \
         { "altitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_gps_tm_t, altitude) }, \
         { "vel_north", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gps_tm_t, vel_north) }, \
         { "vel_east", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gps_tm_t, vel_east) }, \
         { "vel_down", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gps_tm_t, vel_down) }, \
         { "vel_mag", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_gps_tm_t, vel_mag) }, \
         { "n_satellites", NULL, MAVLINK_TYPE_INT32_T, 0, 48, offsetof(mavlink_gps_tm_t, n_satellites) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS_TM { \
    "GPS_TM", \
    10, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_tm_t, timestamp) }, \
         { "fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_gps_tm_t, fix) }, \
         { "lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_gps_tm_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_gps_tm_t, lon) }, \
         { "altitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_gps_tm_t, altitude) }, \
         { "vel_north", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gps_tm_t, vel_north) }, \
         { "vel_east", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gps_tm_t, vel_east) }, \
         { "vel_down", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gps_tm_t, vel_down) }, \
         { "vel_mag", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_gps_tm_t, vel_mag) }, \
         { "n_satellites", NULL, MAVLINK_TYPE_INT32_T, 0, 48, offsetof(mavlink_gps_tm_t, n_satellites) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  When was this logged
 * @param fix  Wether the GPS has a FIX
 * @param lat  Latitude
 * @param lon  Longitude
 * @param altitude  Altitude
 * @param vel_north  Velocity in NED frame (north)
 * @param vel_east  Velocity in NED frame (east)
 * @param vel_down  Velocity in NED frame (down)
 * @param vel_mag  Speed
 * @param n_satellites  Number of connected satellites
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t fix, double lat, double lon, double altitude, float vel_north, float vel_east, float vel_down, float vel_mag, int32_t n_satellites)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_double(buf, 8, lat);
    _mav_put_double(buf, 16, lon);
    _mav_put_double(buf, 24, altitude);
    _mav_put_float(buf, 32, vel_north);
    _mav_put_float(buf, 36, vel_east);
    _mav_put_float(buf, 40, vel_down);
    _mav_put_float(buf, 44, vel_mag);
    _mav_put_int32_t(buf, 48, n_satellites);
    _mav_put_uint8_t(buf, 52, fix);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_TM_LEN);
#else
    mavlink_gps_tm_t packet;
    packet.timestamp = timestamp;
    packet.lat = lat;
    packet.lon = lon;
    packet.altitude = altitude;
    packet.vel_north = vel_north;
    packet.vel_east = vel_east;
    packet.vel_down = vel_down;
    packet.vel_mag = vel_mag;
    packet.n_satellites = n_satellites;
    packet.fix = fix;

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
 * @param timestamp  When was this logged
 * @param fix  Wether the GPS has a FIX
 * @param lat  Latitude
 * @param lon  Longitude
 * @param altitude  Altitude
 * @param vel_north  Velocity in NED frame (north)
 * @param vel_east  Velocity in NED frame (east)
 * @param vel_down  Velocity in NED frame (down)
 * @param vel_mag  Speed
 * @param n_satellites  Number of connected satellites
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t fix,double lat,double lon,double altitude,float vel_north,float vel_east,float vel_down,float vel_mag,int32_t n_satellites)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_double(buf, 8, lat);
    _mav_put_double(buf, 16, lon);
    _mav_put_double(buf, 24, altitude);
    _mav_put_float(buf, 32, vel_north);
    _mav_put_float(buf, 36, vel_east);
    _mav_put_float(buf, 40, vel_down);
    _mav_put_float(buf, 44, vel_mag);
    _mav_put_int32_t(buf, 48, n_satellites);
    _mav_put_uint8_t(buf, 52, fix);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_TM_LEN);
#else
    mavlink_gps_tm_t packet;
    packet.timestamp = timestamp;
    packet.lat = lat;
    packet.lon = lon;
    packet.altitude = altitude;
    packet.vel_north = vel_north;
    packet.vel_east = vel_east;
    packet.vel_down = vel_down;
    packet.vel_mag = vel_mag;
    packet.n_satellites = n_satellites;
    packet.fix = fix;

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
    return mavlink_msg_gps_tm_pack(system_id, component_id, msg, gps_tm->timestamp, gps_tm->fix, gps_tm->lat, gps_tm->lon, gps_tm->altitude, gps_tm->vel_north, gps_tm->vel_east, gps_tm->vel_down, gps_tm->vel_mag, gps_tm->n_satellites);
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
    return mavlink_msg_gps_tm_pack_chan(system_id, component_id, chan, msg, gps_tm->timestamp, gps_tm->fix, gps_tm->lat, gps_tm->lon, gps_tm->altitude, gps_tm->vel_north, gps_tm->vel_east, gps_tm->vel_down, gps_tm->vel_mag, gps_tm->n_satellites);
}

/**
 * @brief Send a gps_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  When was this logged
 * @param fix  Wether the GPS has a FIX
 * @param lat  Latitude
 * @param lon  Longitude
 * @param altitude  Altitude
 * @param vel_north  Velocity in NED frame (north)
 * @param vel_east  Velocity in NED frame (east)
 * @param vel_down  Velocity in NED frame (down)
 * @param vel_mag  Speed
 * @param n_satellites  Number of connected satellites
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t fix, double lat, double lon, double altitude, float vel_north, float vel_east, float vel_down, float vel_mag, int32_t n_satellites)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_double(buf, 8, lat);
    _mav_put_double(buf, 16, lon);
    _mav_put_double(buf, 24, altitude);
    _mav_put_float(buf, 32, vel_north);
    _mav_put_float(buf, 36, vel_east);
    _mav_put_float(buf, 40, vel_down);
    _mav_put_float(buf, 44, vel_mag);
    _mav_put_int32_t(buf, 48, n_satellites);
    _mav_put_uint8_t(buf, 52, fix);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_TM, buf, MAVLINK_MSG_ID_GPS_TM_MIN_LEN, MAVLINK_MSG_ID_GPS_TM_LEN, MAVLINK_MSG_ID_GPS_TM_CRC);
#else
    mavlink_gps_tm_t packet;
    packet.timestamp = timestamp;
    packet.lat = lat;
    packet.lon = lon;
    packet.altitude = altitude;
    packet.vel_north = vel_north;
    packet.vel_east = vel_east;
    packet.vel_down = vel_down;
    packet.vel_mag = vel_mag;
    packet.n_satellites = n_satellites;
    packet.fix = fix;

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
    mavlink_msg_gps_tm_send(chan, gps_tm->timestamp, gps_tm->fix, gps_tm->lat, gps_tm->lon, gps_tm->altitude, gps_tm->vel_north, gps_tm->vel_east, gps_tm->vel_down, gps_tm->vel_mag, gps_tm->n_satellites);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_TM, (const char *)gps_tm, MAVLINK_MSG_ID_GPS_TM_MIN_LEN, MAVLINK_MSG_ID_GPS_TM_LEN, MAVLINK_MSG_ID_GPS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t fix, double lat, double lon, double altitude, float vel_north, float vel_east, float vel_down, float vel_mag, int32_t n_satellites)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_double(buf, 8, lat);
    _mav_put_double(buf, 16, lon);
    _mav_put_double(buf, 24, altitude);
    _mav_put_float(buf, 32, vel_north);
    _mav_put_float(buf, 36, vel_east);
    _mav_put_float(buf, 40, vel_down);
    _mav_put_float(buf, 44, vel_mag);
    _mav_put_int32_t(buf, 48, n_satellites);
    _mav_put_uint8_t(buf, 52, fix);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_TM, buf, MAVLINK_MSG_ID_GPS_TM_MIN_LEN, MAVLINK_MSG_ID_GPS_TM_LEN, MAVLINK_MSG_ID_GPS_TM_CRC);
#else
    mavlink_gps_tm_t *packet = (mavlink_gps_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->lat = lat;
    packet->lon = lon;
    packet->altitude = altitude;
    packet->vel_north = vel_north;
    packet->vel_east = vel_east;
    packet->vel_down = vel_down;
    packet->vel_mag = vel_mag;
    packet->n_satellites = n_satellites;
    packet->fix = fix;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_TM, (const char *)packet, MAVLINK_MSG_ID_GPS_TM_MIN_LEN, MAVLINK_MSG_ID_GPS_TM_LEN, MAVLINK_MSG_ID_GPS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_TM UNPACKING


/**
 * @brief Get field timestamp from gps_tm message
 *
 * @return  When was this logged
 */
static inline uint64_t mavlink_msg_gps_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field fix from gps_tm message
 *
 * @return  Wether the GPS has a FIX
 */
static inline uint8_t mavlink_msg_gps_tm_get_fix(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field lat from gps_tm message
 *
 * @return  Latitude
 */
static inline double mavlink_msg_gps_tm_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field lon from gps_tm message
 *
 * @return  Longitude
 */
static inline double mavlink_msg_gps_tm_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field altitude from gps_tm message
 *
 * @return  Altitude
 */
static inline double mavlink_msg_gps_tm_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  24);
}

/**
 * @brief Get field vel_north from gps_tm message
 *
 * @return  Velocity in NED frame (north)
 */
static inline float mavlink_msg_gps_tm_get_vel_north(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field vel_east from gps_tm message
 *
 * @return  Velocity in NED frame (east)
 */
static inline float mavlink_msg_gps_tm_get_vel_east(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field vel_down from gps_tm message
 *
 * @return  Velocity in NED frame (down)
 */
static inline float mavlink_msg_gps_tm_get_vel_down(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field vel_mag from gps_tm message
 *
 * @return  Speed
 */
static inline float mavlink_msg_gps_tm_get_vel_mag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field n_satellites from gps_tm message
 *
 * @return  Number of connected satellites
 */
static inline int32_t mavlink_msg_gps_tm_get_n_satellites(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  48);
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
    gps_tm->lat = mavlink_msg_gps_tm_get_lat(msg);
    gps_tm->lon = mavlink_msg_gps_tm_get_lon(msg);
    gps_tm->altitude = mavlink_msg_gps_tm_get_altitude(msg);
    gps_tm->vel_north = mavlink_msg_gps_tm_get_vel_north(msg);
    gps_tm->vel_east = mavlink_msg_gps_tm_get_vel_east(msg);
    gps_tm->vel_down = mavlink_msg_gps_tm_get_vel_down(msg);
    gps_tm->vel_mag = mavlink_msg_gps_tm_get_vel_mag(msg);
    gps_tm->n_satellites = mavlink_msg_gps_tm_get_n_satellites(msg);
    gps_tm->fix = mavlink_msg_gps_tm_get_fix(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS_TM_LEN? msg->len : MAVLINK_MSG_ID_GPS_TM_LEN;
        memset(gps_tm, 0, MAVLINK_MSG_ID_GPS_TM_LEN);
    memcpy(gps_tm, _MAV_PAYLOAD(msg), len);
#endif
}
