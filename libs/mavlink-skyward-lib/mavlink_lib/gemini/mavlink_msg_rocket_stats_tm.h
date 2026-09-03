#pragma once
// MESSAGE ROCKET_STATS_TM PACKING

#define MAVLINK_MSG_ID_ROCKET_STATS_TM 210


typedef struct __mavlink_rocket_stats_tm_t {
 uint64_t liftoff_ts; /*< [us] System clock at liftoff*/
 uint64_t liftoff_max_acc_ts; /*< [us] System clock at the maximum liftoff acceleration*/
 uint64_t dpl_ts; /*< [us] System clock at drouge deployment*/
 uint64_t max_z_speed_ts; /*< [us] System clock at ADA max vertical speed*/
 uint64_t apogee_ts; /*< [us] System clock at apogee*/
 float liftoff_max_acc; /*< [m/s2] Maximum liftoff acceleration*/
 float dpl_max_acc; /*< [m/s2] Max acceleration during drouge deployment*/
 float max_z_speed; /*< [m/s] Max measured vertical speed - ADA*/
 float max_airspeed_pitot; /*< [m/s] Max speed read by the pitot tube*/
 float max_speed_altitude; /*< [m] Altitude at max speed*/
 float apogee_lat; /*< [deg] Apogee latitude*/
 float apogee_lon; /*< [deg] Apogee longitude*/
 float apogee_alt; /*< [m] Apogee altitude*/
 float min_pressure; /*< [Pa] Apogee pressure - Digital barometer*/
 float ada_min_pressure; /*< [Pa] Apogee pressure - ADA filtered*/
 float dpl_vane_max_pressure; /*< [Pa] Max pressure in the deployment bay during drogue deployment*/
 float cpu_load; /*<  CPU load in percentage*/
 uint32_t free_heap; /*<  Amount of available heap in memory*/
} mavlink_rocket_stats_tm_t;

#define MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN 92
#define MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN 92
#define MAVLINK_MSG_ID_210_LEN 92
#define MAVLINK_MSG_ID_210_MIN_LEN 92

#define MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC 245
#define MAVLINK_MSG_ID_210_CRC 245



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROCKET_STATS_TM { \
    210, \
    "ROCKET_STATS_TM", \
    18, \
    {  { "liftoff_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rocket_stats_tm_t, liftoff_ts) }, \
         { "liftoff_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_rocket_stats_tm_t, liftoff_max_acc_ts) }, \
         { "liftoff_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_rocket_stats_tm_t, liftoff_max_acc) }, \
         { "dpl_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_rocket_stats_tm_t, dpl_ts) }, \
         { "dpl_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_rocket_stats_tm_t, dpl_max_acc) }, \
         { "max_z_speed_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 24, offsetof(mavlink_rocket_stats_tm_t, max_z_speed_ts) }, \
         { "max_z_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_rocket_stats_tm_t, max_z_speed) }, \
         { "max_airspeed_pitot", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_rocket_stats_tm_t, max_airspeed_pitot) }, \
         { "max_speed_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_rocket_stats_tm_t, max_speed_altitude) }, \
         { "apogee_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 32, offsetof(mavlink_rocket_stats_tm_t, apogee_ts) }, \
         { "apogee_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_rocket_stats_tm_t, apogee_lat) }, \
         { "apogee_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_rocket_stats_tm_t, apogee_lon) }, \
         { "apogee_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_rocket_stats_tm_t, apogee_alt) }, \
         { "min_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_rocket_stats_tm_t, min_pressure) }, \
         { "ada_min_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_rocket_stats_tm_t, ada_min_pressure) }, \
         { "dpl_vane_max_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_rocket_stats_tm_t, dpl_vane_max_pressure) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_rocket_stats_tm_t, cpu_load) }, \
         { "free_heap", NULL, MAVLINK_TYPE_UINT32_T, 0, 88, offsetof(mavlink_rocket_stats_tm_t, free_heap) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROCKET_STATS_TM { \
    "ROCKET_STATS_TM", \
    18, \
    {  { "liftoff_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rocket_stats_tm_t, liftoff_ts) }, \
         { "liftoff_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_rocket_stats_tm_t, liftoff_max_acc_ts) }, \
         { "liftoff_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_rocket_stats_tm_t, liftoff_max_acc) }, \
         { "dpl_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_rocket_stats_tm_t, dpl_ts) }, \
         { "dpl_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_rocket_stats_tm_t, dpl_max_acc) }, \
         { "max_z_speed_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 24, offsetof(mavlink_rocket_stats_tm_t, max_z_speed_ts) }, \
         { "max_z_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_rocket_stats_tm_t, max_z_speed) }, \
         { "max_airspeed_pitot", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_rocket_stats_tm_t, max_airspeed_pitot) }, \
         { "max_speed_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_rocket_stats_tm_t, max_speed_altitude) }, \
         { "apogee_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 32, offsetof(mavlink_rocket_stats_tm_t, apogee_ts) }, \
         { "apogee_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_rocket_stats_tm_t, apogee_lat) }, \
         { "apogee_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_rocket_stats_tm_t, apogee_lon) }, \
         { "apogee_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_rocket_stats_tm_t, apogee_alt) }, \
         { "min_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_rocket_stats_tm_t, min_pressure) }, \
         { "ada_min_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_rocket_stats_tm_t, ada_min_pressure) }, \
         { "dpl_vane_max_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_rocket_stats_tm_t, dpl_vane_max_pressure) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_rocket_stats_tm_t, cpu_load) }, \
         { "free_heap", NULL, MAVLINK_TYPE_UINT32_T, 0, 88, offsetof(mavlink_rocket_stats_tm_t, free_heap) }, \
         } \
}
#endif

/**
 * @brief Pack a rocket_stats_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param liftoff_ts [us] System clock at liftoff
 * @param liftoff_max_acc_ts [us] System clock at the maximum liftoff acceleration
 * @param liftoff_max_acc [m/s2] Maximum liftoff acceleration
 * @param dpl_ts [us] System clock at drouge deployment
 * @param dpl_max_acc [m/s2] Max acceleration during drouge deployment
 * @param max_z_speed_ts [us] System clock at ADA max vertical speed
 * @param max_z_speed [m/s] Max measured vertical speed - ADA
 * @param max_airspeed_pitot [m/s] Max speed read by the pitot tube
 * @param max_speed_altitude [m] Altitude at max speed
 * @param apogee_ts [us] System clock at apogee
 * @param apogee_lat [deg] Apogee latitude
 * @param apogee_lon [deg] Apogee longitude
 * @param apogee_alt [m] Apogee altitude
 * @param min_pressure [Pa] Apogee pressure - Digital barometer
 * @param ada_min_pressure [Pa] Apogee pressure - ADA filtered
 * @param dpl_vane_max_pressure [Pa] Max pressure in the deployment bay during drogue deployment
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rocket_stats_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t liftoff_ts, uint64_t liftoff_max_acc_ts, float liftoff_max_acc, uint64_t dpl_ts, float dpl_max_acc, uint64_t max_z_speed_ts, float max_z_speed, float max_airspeed_pitot, float max_speed_altitude, uint64_t apogee_ts, float apogee_lat, float apogee_lon, float apogee_alt, float min_pressure, float ada_min_pressure, float dpl_vane_max_pressure, float cpu_load, uint32_t free_heap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, liftoff_ts);
    _mav_put_uint64_t(buf, 8, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 16, dpl_ts);
    _mav_put_uint64_t(buf, 24, max_z_speed_ts);
    _mav_put_uint64_t(buf, 32, apogee_ts);
    _mav_put_float(buf, 40, liftoff_max_acc);
    _mav_put_float(buf, 44, dpl_max_acc);
    _mav_put_float(buf, 48, max_z_speed);
    _mav_put_float(buf, 52, max_airspeed_pitot);
    _mav_put_float(buf, 56, max_speed_altitude);
    _mav_put_float(buf, 60, apogee_lat);
    _mav_put_float(buf, 64, apogee_lon);
    _mav_put_float(buf, 68, apogee_alt);
    _mav_put_float(buf, 72, min_pressure);
    _mav_put_float(buf, 76, ada_min_pressure);
    _mav_put_float(buf, 80, dpl_vane_max_pressure);
    _mav_put_float(buf, 84, cpu_load);
    _mav_put_uint32_t(buf, 88, free_heap);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN);
#else
    mavlink_rocket_stats_tm_t packet;
    packet.liftoff_ts = liftoff_ts;
    packet.liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet.dpl_ts = dpl_ts;
    packet.max_z_speed_ts = max_z_speed_ts;
    packet.apogee_ts = apogee_ts;
    packet.liftoff_max_acc = liftoff_max_acc;
    packet.dpl_max_acc = dpl_max_acc;
    packet.max_z_speed = max_z_speed;
    packet.max_airspeed_pitot = max_airspeed_pitot;
    packet.max_speed_altitude = max_speed_altitude;
    packet.apogee_lat = apogee_lat;
    packet.apogee_lon = apogee_lon;
    packet.apogee_alt = apogee_alt;
    packet.min_pressure = min_pressure;
    packet.ada_min_pressure = ada_min_pressure;
    packet.dpl_vane_max_pressure = dpl_vane_max_pressure;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROCKET_STATS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
}

/**
 * @brief Pack a rocket_stats_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param liftoff_ts [us] System clock at liftoff
 * @param liftoff_max_acc_ts [us] System clock at the maximum liftoff acceleration
 * @param liftoff_max_acc [m/s2] Maximum liftoff acceleration
 * @param dpl_ts [us] System clock at drouge deployment
 * @param dpl_max_acc [m/s2] Max acceleration during drouge deployment
 * @param max_z_speed_ts [us] System clock at ADA max vertical speed
 * @param max_z_speed [m/s] Max measured vertical speed - ADA
 * @param max_airspeed_pitot [m/s] Max speed read by the pitot tube
 * @param max_speed_altitude [m] Altitude at max speed
 * @param apogee_ts [us] System clock at apogee
 * @param apogee_lat [deg] Apogee latitude
 * @param apogee_lon [deg] Apogee longitude
 * @param apogee_alt [m] Apogee altitude
 * @param min_pressure [Pa] Apogee pressure - Digital barometer
 * @param ada_min_pressure [Pa] Apogee pressure - ADA filtered
 * @param dpl_vane_max_pressure [Pa] Max pressure in the deployment bay during drogue deployment
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rocket_stats_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t liftoff_ts,uint64_t liftoff_max_acc_ts,float liftoff_max_acc,uint64_t dpl_ts,float dpl_max_acc,uint64_t max_z_speed_ts,float max_z_speed,float max_airspeed_pitot,float max_speed_altitude,uint64_t apogee_ts,float apogee_lat,float apogee_lon,float apogee_alt,float min_pressure,float ada_min_pressure,float dpl_vane_max_pressure,float cpu_load,uint32_t free_heap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, liftoff_ts);
    _mav_put_uint64_t(buf, 8, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 16, dpl_ts);
    _mav_put_uint64_t(buf, 24, max_z_speed_ts);
    _mav_put_uint64_t(buf, 32, apogee_ts);
    _mav_put_float(buf, 40, liftoff_max_acc);
    _mav_put_float(buf, 44, dpl_max_acc);
    _mav_put_float(buf, 48, max_z_speed);
    _mav_put_float(buf, 52, max_airspeed_pitot);
    _mav_put_float(buf, 56, max_speed_altitude);
    _mav_put_float(buf, 60, apogee_lat);
    _mav_put_float(buf, 64, apogee_lon);
    _mav_put_float(buf, 68, apogee_alt);
    _mav_put_float(buf, 72, min_pressure);
    _mav_put_float(buf, 76, ada_min_pressure);
    _mav_put_float(buf, 80, dpl_vane_max_pressure);
    _mav_put_float(buf, 84, cpu_load);
    _mav_put_uint32_t(buf, 88, free_heap);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN);
#else
    mavlink_rocket_stats_tm_t packet;
    packet.liftoff_ts = liftoff_ts;
    packet.liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet.dpl_ts = dpl_ts;
    packet.max_z_speed_ts = max_z_speed_ts;
    packet.apogee_ts = apogee_ts;
    packet.liftoff_max_acc = liftoff_max_acc;
    packet.dpl_max_acc = dpl_max_acc;
    packet.max_z_speed = max_z_speed;
    packet.max_airspeed_pitot = max_airspeed_pitot;
    packet.max_speed_altitude = max_speed_altitude;
    packet.apogee_lat = apogee_lat;
    packet.apogee_lon = apogee_lon;
    packet.apogee_alt = apogee_alt;
    packet.min_pressure = min_pressure;
    packet.ada_min_pressure = ada_min_pressure;
    packet.dpl_vane_max_pressure = dpl_vane_max_pressure;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROCKET_STATS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
}

/**
 * @brief Encode a rocket_stats_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rocket_stats_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rocket_stats_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rocket_stats_tm_t* rocket_stats_tm)
{
    return mavlink_msg_rocket_stats_tm_pack(system_id, component_id, msg, rocket_stats_tm->liftoff_ts, rocket_stats_tm->liftoff_max_acc_ts, rocket_stats_tm->liftoff_max_acc, rocket_stats_tm->dpl_ts, rocket_stats_tm->dpl_max_acc, rocket_stats_tm->max_z_speed_ts, rocket_stats_tm->max_z_speed, rocket_stats_tm->max_airspeed_pitot, rocket_stats_tm->max_speed_altitude, rocket_stats_tm->apogee_ts, rocket_stats_tm->apogee_lat, rocket_stats_tm->apogee_lon, rocket_stats_tm->apogee_alt, rocket_stats_tm->min_pressure, rocket_stats_tm->ada_min_pressure, rocket_stats_tm->dpl_vane_max_pressure, rocket_stats_tm->cpu_load, rocket_stats_tm->free_heap);
}

/**
 * @brief Encode a rocket_stats_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rocket_stats_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rocket_stats_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rocket_stats_tm_t* rocket_stats_tm)
{
    return mavlink_msg_rocket_stats_tm_pack_chan(system_id, component_id, chan, msg, rocket_stats_tm->liftoff_ts, rocket_stats_tm->liftoff_max_acc_ts, rocket_stats_tm->liftoff_max_acc, rocket_stats_tm->dpl_ts, rocket_stats_tm->dpl_max_acc, rocket_stats_tm->max_z_speed_ts, rocket_stats_tm->max_z_speed, rocket_stats_tm->max_airspeed_pitot, rocket_stats_tm->max_speed_altitude, rocket_stats_tm->apogee_ts, rocket_stats_tm->apogee_lat, rocket_stats_tm->apogee_lon, rocket_stats_tm->apogee_alt, rocket_stats_tm->min_pressure, rocket_stats_tm->ada_min_pressure, rocket_stats_tm->dpl_vane_max_pressure, rocket_stats_tm->cpu_load, rocket_stats_tm->free_heap);
}

/**
 * @brief Send a rocket_stats_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param liftoff_ts [us] System clock at liftoff
 * @param liftoff_max_acc_ts [us] System clock at the maximum liftoff acceleration
 * @param liftoff_max_acc [m/s2] Maximum liftoff acceleration
 * @param dpl_ts [us] System clock at drouge deployment
 * @param dpl_max_acc [m/s2] Max acceleration during drouge deployment
 * @param max_z_speed_ts [us] System clock at ADA max vertical speed
 * @param max_z_speed [m/s] Max measured vertical speed - ADA
 * @param max_airspeed_pitot [m/s] Max speed read by the pitot tube
 * @param max_speed_altitude [m] Altitude at max speed
 * @param apogee_ts [us] System clock at apogee
 * @param apogee_lat [deg] Apogee latitude
 * @param apogee_lon [deg] Apogee longitude
 * @param apogee_alt [m] Apogee altitude
 * @param min_pressure [Pa] Apogee pressure - Digital barometer
 * @param ada_min_pressure [Pa] Apogee pressure - ADA filtered
 * @param dpl_vane_max_pressure [Pa] Max pressure in the deployment bay during drogue deployment
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rocket_stats_tm_send(mavlink_channel_t chan, uint64_t liftoff_ts, uint64_t liftoff_max_acc_ts, float liftoff_max_acc, uint64_t dpl_ts, float dpl_max_acc, uint64_t max_z_speed_ts, float max_z_speed, float max_airspeed_pitot, float max_speed_altitude, uint64_t apogee_ts, float apogee_lat, float apogee_lon, float apogee_alt, float min_pressure, float ada_min_pressure, float dpl_vane_max_pressure, float cpu_load, uint32_t free_heap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, liftoff_ts);
    _mav_put_uint64_t(buf, 8, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 16, dpl_ts);
    _mav_put_uint64_t(buf, 24, max_z_speed_ts);
    _mav_put_uint64_t(buf, 32, apogee_ts);
    _mav_put_float(buf, 40, liftoff_max_acc);
    _mav_put_float(buf, 44, dpl_max_acc);
    _mav_put_float(buf, 48, max_z_speed);
    _mav_put_float(buf, 52, max_airspeed_pitot);
    _mav_put_float(buf, 56, max_speed_altitude);
    _mav_put_float(buf, 60, apogee_lat);
    _mav_put_float(buf, 64, apogee_lon);
    _mav_put_float(buf, 68, apogee_alt);
    _mav_put_float(buf, 72, min_pressure);
    _mav_put_float(buf, 76, ada_min_pressure);
    _mav_put_float(buf, 80, dpl_vane_max_pressure);
    _mav_put_float(buf, 84, cpu_load);
    _mav_put_uint32_t(buf, 88, free_heap);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_STATS_TM, buf, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
#else
    mavlink_rocket_stats_tm_t packet;
    packet.liftoff_ts = liftoff_ts;
    packet.liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet.dpl_ts = dpl_ts;
    packet.max_z_speed_ts = max_z_speed_ts;
    packet.apogee_ts = apogee_ts;
    packet.liftoff_max_acc = liftoff_max_acc;
    packet.dpl_max_acc = dpl_max_acc;
    packet.max_z_speed = max_z_speed;
    packet.max_airspeed_pitot = max_airspeed_pitot;
    packet.max_speed_altitude = max_speed_altitude;
    packet.apogee_lat = apogee_lat;
    packet.apogee_lon = apogee_lon;
    packet.apogee_alt = apogee_alt;
    packet.min_pressure = min_pressure;
    packet.ada_min_pressure = ada_min_pressure;
    packet.dpl_vane_max_pressure = dpl_vane_max_pressure;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_STATS_TM, (const char *)&packet, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
#endif
}

/**
 * @brief Send a rocket_stats_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rocket_stats_tm_send_struct(mavlink_channel_t chan, const mavlink_rocket_stats_tm_t* rocket_stats_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rocket_stats_tm_send(chan, rocket_stats_tm->liftoff_ts, rocket_stats_tm->liftoff_max_acc_ts, rocket_stats_tm->liftoff_max_acc, rocket_stats_tm->dpl_ts, rocket_stats_tm->dpl_max_acc, rocket_stats_tm->max_z_speed_ts, rocket_stats_tm->max_z_speed, rocket_stats_tm->max_airspeed_pitot, rocket_stats_tm->max_speed_altitude, rocket_stats_tm->apogee_ts, rocket_stats_tm->apogee_lat, rocket_stats_tm->apogee_lon, rocket_stats_tm->apogee_alt, rocket_stats_tm->min_pressure, rocket_stats_tm->ada_min_pressure, rocket_stats_tm->dpl_vane_max_pressure, rocket_stats_tm->cpu_load, rocket_stats_tm->free_heap);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_STATS_TM, (const char *)rocket_stats_tm, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rocket_stats_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t liftoff_ts, uint64_t liftoff_max_acc_ts, float liftoff_max_acc, uint64_t dpl_ts, float dpl_max_acc, uint64_t max_z_speed_ts, float max_z_speed, float max_airspeed_pitot, float max_speed_altitude, uint64_t apogee_ts, float apogee_lat, float apogee_lon, float apogee_alt, float min_pressure, float ada_min_pressure, float dpl_vane_max_pressure, float cpu_load, uint32_t free_heap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, liftoff_ts);
    _mav_put_uint64_t(buf, 8, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 16, dpl_ts);
    _mav_put_uint64_t(buf, 24, max_z_speed_ts);
    _mav_put_uint64_t(buf, 32, apogee_ts);
    _mav_put_float(buf, 40, liftoff_max_acc);
    _mav_put_float(buf, 44, dpl_max_acc);
    _mav_put_float(buf, 48, max_z_speed);
    _mav_put_float(buf, 52, max_airspeed_pitot);
    _mav_put_float(buf, 56, max_speed_altitude);
    _mav_put_float(buf, 60, apogee_lat);
    _mav_put_float(buf, 64, apogee_lon);
    _mav_put_float(buf, 68, apogee_alt);
    _mav_put_float(buf, 72, min_pressure);
    _mav_put_float(buf, 76, ada_min_pressure);
    _mav_put_float(buf, 80, dpl_vane_max_pressure);
    _mav_put_float(buf, 84, cpu_load);
    _mav_put_uint32_t(buf, 88, free_heap);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_STATS_TM, buf, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
#else
    mavlink_rocket_stats_tm_t *packet = (mavlink_rocket_stats_tm_t *)msgbuf;
    packet->liftoff_ts = liftoff_ts;
    packet->liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet->dpl_ts = dpl_ts;
    packet->max_z_speed_ts = max_z_speed_ts;
    packet->apogee_ts = apogee_ts;
    packet->liftoff_max_acc = liftoff_max_acc;
    packet->dpl_max_acc = dpl_max_acc;
    packet->max_z_speed = max_z_speed;
    packet->max_airspeed_pitot = max_airspeed_pitot;
    packet->max_speed_altitude = max_speed_altitude;
    packet->apogee_lat = apogee_lat;
    packet->apogee_lon = apogee_lon;
    packet->apogee_alt = apogee_alt;
    packet->min_pressure = min_pressure;
    packet->ada_min_pressure = ada_min_pressure;
    packet->dpl_vane_max_pressure = dpl_vane_max_pressure;
    packet->cpu_load = cpu_load;
    packet->free_heap = free_heap;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_STATS_TM, (const char *)packet, MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN, MAVLINK_MSG_ID_ROCKET_STATS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ROCKET_STATS_TM UNPACKING


/**
 * @brief Get field liftoff_ts from rocket_stats_tm message
 *
 * @return [us] System clock at liftoff
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_liftoff_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field liftoff_max_acc_ts from rocket_stats_tm message
 *
 * @return [us] System clock at the maximum liftoff acceleration
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_liftoff_max_acc_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field liftoff_max_acc from rocket_stats_tm message
 *
 * @return [m/s2] Maximum liftoff acceleration
 */
static inline float mavlink_msg_rocket_stats_tm_get_liftoff_max_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field dpl_ts from rocket_stats_tm message
 *
 * @return [us] System clock at drouge deployment
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_dpl_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Get field dpl_max_acc from rocket_stats_tm message
 *
 * @return [m/s2] Max acceleration during drouge deployment
 */
static inline float mavlink_msg_rocket_stats_tm_get_dpl_max_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field max_z_speed_ts from rocket_stats_tm message
 *
 * @return [us] System clock at ADA max vertical speed
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_max_z_speed_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  24);
}

/**
 * @brief Get field max_z_speed from rocket_stats_tm message
 *
 * @return [m/s] Max measured vertical speed - ADA
 */
static inline float mavlink_msg_rocket_stats_tm_get_max_z_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field max_airspeed_pitot from rocket_stats_tm message
 *
 * @return [m/s] Max speed read by the pitot tube
 */
static inline float mavlink_msg_rocket_stats_tm_get_max_airspeed_pitot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field max_speed_altitude from rocket_stats_tm message
 *
 * @return [m] Altitude at max speed
 */
static inline float mavlink_msg_rocket_stats_tm_get_max_speed_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field apogee_ts from rocket_stats_tm message
 *
 * @return [us] System clock at apogee
 */
static inline uint64_t mavlink_msg_rocket_stats_tm_get_apogee_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  32);
}

/**
 * @brief Get field apogee_lat from rocket_stats_tm message
 *
 * @return [deg] Apogee latitude
 */
static inline float mavlink_msg_rocket_stats_tm_get_apogee_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field apogee_lon from rocket_stats_tm message
 *
 * @return [deg] Apogee longitude
 */
static inline float mavlink_msg_rocket_stats_tm_get_apogee_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field apogee_alt from rocket_stats_tm message
 *
 * @return [m] Apogee altitude
 */
static inline float mavlink_msg_rocket_stats_tm_get_apogee_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field min_pressure from rocket_stats_tm message
 *
 * @return [Pa] Apogee pressure - Digital barometer
 */
static inline float mavlink_msg_rocket_stats_tm_get_min_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field ada_min_pressure from rocket_stats_tm message
 *
 * @return [Pa] Apogee pressure - ADA filtered
 */
static inline float mavlink_msg_rocket_stats_tm_get_ada_min_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field dpl_vane_max_pressure from rocket_stats_tm message
 *
 * @return [Pa] Max pressure in the deployment bay during drogue deployment
 */
static inline float mavlink_msg_rocket_stats_tm_get_dpl_vane_max_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field cpu_load from rocket_stats_tm message
 *
 * @return  CPU load in percentage
 */
static inline float mavlink_msg_rocket_stats_tm_get_cpu_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field free_heap from rocket_stats_tm message
 *
 * @return  Amount of available heap in memory
 */
static inline uint32_t mavlink_msg_rocket_stats_tm_get_free_heap(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  88);
}

/**
 * @brief Decode a rocket_stats_tm message into a struct
 *
 * @param msg The message to decode
 * @param rocket_stats_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_rocket_stats_tm_decode(const mavlink_message_t* msg, mavlink_rocket_stats_tm_t* rocket_stats_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rocket_stats_tm->liftoff_ts = mavlink_msg_rocket_stats_tm_get_liftoff_ts(msg);
    rocket_stats_tm->liftoff_max_acc_ts = mavlink_msg_rocket_stats_tm_get_liftoff_max_acc_ts(msg);
    rocket_stats_tm->dpl_ts = mavlink_msg_rocket_stats_tm_get_dpl_ts(msg);
    rocket_stats_tm->max_z_speed_ts = mavlink_msg_rocket_stats_tm_get_max_z_speed_ts(msg);
    rocket_stats_tm->apogee_ts = mavlink_msg_rocket_stats_tm_get_apogee_ts(msg);
    rocket_stats_tm->liftoff_max_acc = mavlink_msg_rocket_stats_tm_get_liftoff_max_acc(msg);
    rocket_stats_tm->dpl_max_acc = mavlink_msg_rocket_stats_tm_get_dpl_max_acc(msg);
    rocket_stats_tm->max_z_speed = mavlink_msg_rocket_stats_tm_get_max_z_speed(msg);
    rocket_stats_tm->max_airspeed_pitot = mavlink_msg_rocket_stats_tm_get_max_airspeed_pitot(msg);
    rocket_stats_tm->max_speed_altitude = mavlink_msg_rocket_stats_tm_get_max_speed_altitude(msg);
    rocket_stats_tm->apogee_lat = mavlink_msg_rocket_stats_tm_get_apogee_lat(msg);
    rocket_stats_tm->apogee_lon = mavlink_msg_rocket_stats_tm_get_apogee_lon(msg);
    rocket_stats_tm->apogee_alt = mavlink_msg_rocket_stats_tm_get_apogee_alt(msg);
    rocket_stats_tm->min_pressure = mavlink_msg_rocket_stats_tm_get_min_pressure(msg);
    rocket_stats_tm->ada_min_pressure = mavlink_msg_rocket_stats_tm_get_ada_min_pressure(msg);
    rocket_stats_tm->dpl_vane_max_pressure = mavlink_msg_rocket_stats_tm_get_dpl_vane_max_pressure(msg);
    rocket_stats_tm->cpu_load = mavlink_msg_rocket_stats_tm_get_cpu_load(msg);
    rocket_stats_tm->free_heap = mavlink_msg_rocket_stats_tm_get_free_heap(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN? msg->len : MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN;
        memset(rocket_stats_tm, 0, MAVLINK_MSG_ID_ROCKET_STATS_TM_LEN);
    memcpy(rocket_stats_tm, _MAV_PAYLOAD(msg), len);
#endif
}
