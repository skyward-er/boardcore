#pragma once
// MESSAGE LR_TM PACKING

#define MAVLINK_MSG_ID_LR_TM 181

MAVPACKED(
typedef struct __mavlink_lr_tm_t {
 uint64_t liftoff_ts; /*< [ms] System clock at liftoff*/
 uint64_t liftoff_max_acc_ts; /*< [ms] System clock at the maximum liftoff acceleration*/
 uint64_t max_z_speed_ts; /*< [ms] System clock at ADA max vertical speed*/
 uint64_t apogee_ts; /*< [ms] System clock at apogee*/
 uint64_t drogue_dpl_ts; /*< [ms] System clock at drouge deployment*/
 uint64_t main_dpl_altitude_ts; /*< [ms] System clock at main chute deployment*/
 float liftoff_max_acc; /*< [m/s2] Maximum liftoff acceleration*/
 float max_z_speed; /*< [m/s] Max measured vertical speed - ADA*/
 float max_airspeed_pitot; /*< [m/s] Max speed read by the pitot tube*/
 float max_speed_altitude; /*< [m] Altitude at max speed*/
 float apogee_lat; /*< [deg] Apogee latitude*/
 float apogee_lon; /*< [deg] Apogee longitude*/
 float static_min_pressure; /*< [Pa] Apogee pressure - Static ports*/
 float digital_min_pressure; /*< [Pa] Apogee pressure - Digital barometer*/
 float ada_min_pressure; /*< [Pa] Apogee pressure - ADA filtered*/
 float baro_max_altitutde; /*< [m] Apogee altitude - Digital barometer*/
 float gps_max_altitude; /*< [m] Apogee altitude - GPS*/
 float drogue_dpl_max_acc; /*< [m/s2] Max acceleration during drouge deployment*/
 float dpl_vane_max_pressure; /*< [Pa] Max pressure in the deployment bay during drogue deployment*/
 float main_dpl_altitude; /*< [m] Altittude at main deployment (m.s.l)*/
 float main_dpl_zspeed; /*< [m/s] Vertical speed at main deployment (sensor z axis)*/
 float main_dpl_acc; /*< [m/s2] Max measured vertical acceleration (sensor z axis) after main parachute deployment*/
 float cpu_load; /*<  CPU load in percentage*/
 uint32_t free_heap; /*<  Amount of available heap in memory*/
}) mavlink_lr_tm_t;

#define MAVLINK_MSG_ID_LR_TM_LEN 120
#define MAVLINK_MSG_ID_LR_TM_MIN_LEN 120
#define MAVLINK_MSG_ID_181_LEN 120
#define MAVLINK_MSG_ID_181_MIN_LEN 120

#define MAVLINK_MSG_ID_LR_TM_CRC 11
#define MAVLINK_MSG_ID_181_CRC 11



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LR_TM { \
    181, \
    "LR_TM", \
    24, \
    {  { "liftoff_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lr_tm_t, liftoff_ts) }, \
         { "liftoff_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_lr_tm_t, liftoff_max_acc_ts) }, \
         { "liftoff_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_lr_tm_t, liftoff_max_acc) }, \
         { "max_z_speed_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_lr_tm_t, max_z_speed_ts) }, \
         { "max_z_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_lr_tm_t, max_z_speed) }, \
         { "max_airspeed_pitot", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_lr_tm_t, max_airspeed_pitot) }, \
         { "max_speed_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_lr_tm_t, max_speed_altitude) }, \
         { "apogee_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 24, offsetof(mavlink_lr_tm_t, apogee_ts) }, \
         { "apogee_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_lr_tm_t, apogee_lat) }, \
         { "apogee_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_lr_tm_t, apogee_lon) }, \
         { "static_min_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_lr_tm_t, static_min_pressure) }, \
         { "digital_min_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_lr_tm_t, digital_min_pressure) }, \
         { "ada_min_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_lr_tm_t, ada_min_pressure) }, \
         { "baro_max_altitutde", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_lr_tm_t, baro_max_altitutde) }, \
         { "gps_max_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_lr_tm_t, gps_max_altitude) }, \
         { "drogue_dpl_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 32, offsetof(mavlink_lr_tm_t, drogue_dpl_ts) }, \
         { "drogue_dpl_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_lr_tm_t, drogue_dpl_max_acc) }, \
         { "dpl_vane_max_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_lr_tm_t, dpl_vane_max_pressure) }, \
         { "main_dpl_altitude_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 40, offsetof(mavlink_lr_tm_t, main_dpl_altitude_ts) }, \
         { "main_dpl_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_lr_tm_t, main_dpl_altitude) }, \
         { "main_dpl_zspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_lr_tm_t, main_dpl_zspeed) }, \
         { "main_dpl_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_lr_tm_t, main_dpl_acc) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_lr_tm_t, cpu_load) }, \
         { "free_heap", NULL, MAVLINK_TYPE_UINT32_T, 0, 116, offsetof(mavlink_lr_tm_t, free_heap) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LR_TM { \
    "LR_TM", \
    24, \
    {  { "liftoff_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lr_tm_t, liftoff_ts) }, \
         { "liftoff_max_acc_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_lr_tm_t, liftoff_max_acc_ts) }, \
         { "liftoff_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_lr_tm_t, liftoff_max_acc) }, \
         { "max_z_speed_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_lr_tm_t, max_z_speed_ts) }, \
         { "max_z_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_lr_tm_t, max_z_speed) }, \
         { "max_airspeed_pitot", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_lr_tm_t, max_airspeed_pitot) }, \
         { "max_speed_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_lr_tm_t, max_speed_altitude) }, \
         { "apogee_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 24, offsetof(mavlink_lr_tm_t, apogee_ts) }, \
         { "apogee_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_lr_tm_t, apogee_lat) }, \
         { "apogee_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_lr_tm_t, apogee_lon) }, \
         { "static_min_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_lr_tm_t, static_min_pressure) }, \
         { "digital_min_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_lr_tm_t, digital_min_pressure) }, \
         { "ada_min_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_lr_tm_t, ada_min_pressure) }, \
         { "baro_max_altitutde", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_lr_tm_t, baro_max_altitutde) }, \
         { "gps_max_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_lr_tm_t, gps_max_altitude) }, \
         { "drogue_dpl_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 32, offsetof(mavlink_lr_tm_t, drogue_dpl_ts) }, \
         { "drogue_dpl_max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_lr_tm_t, drogue_dpl_max_acc) }, \
         { "dpl_vane_max_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_lr_tm_t, dpl_vane_max_pressure) }, \
         { "main_dpl_altitude_ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 40, offsetof(mavlink_lr_tm_t, main_dpl_altitude_ts) }, \
         { "main_dpl_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_lr_tm_t, main_dpl_altitude) }, \
         { "main_dpl_zspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_lr_tm_t, main_dpl_zspeed) }, \
         { "main_dpl_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_lr_tm_t, main_dpl_acc) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_lr_tm_t, cpu_load) }, \
         { "free_heap", NULL, MAVLINK_TYPE_UINT32_T, 0, 116, offsetof(mavlink_lr_tm_t, free_heap) }, \
         } \
}
#endif

/**
 * @brief Pack a lr_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param liftoff_ts [ms] System clock at liftoff
 * @param liftoff_max_acc_ts [ms] System clock at the maximum liftoff acceleration
 * @param liftoff_max_acc [m/s2] Maximum liftoff acceleration
 * @param max_z_speed_ts [ms] System clock at ADA max vertical speed
 * @param max_z_speed [m/s] Max measured vertical speed - ADA
 * @param max_airspeed_pitot [m/s] Max speed read by the pitot tube
 * @param max_speed_altitude [m] Altitude at max speed
 * @param apogee_ts [ms] System clock at apogee
 * @param apogee_lat [deg] Apogee latitude
 * @param apogee_lon [deg] Apogee longitude
 * @param static_min_pressure [Pa] Apogee pressure - Static ports
 * @param digital_min_pressure [Pa] Apogee pressure - Digital barometer
 * @param ada_min_pressure [Pa] Apogee pressure - ADA filtered
 * @param baro_max_altitutde [m] Apogee altitude - Digital barometer
 * @param gps_max_altitude [m] Apogee altitude - GPS
 * @param drogue_dpl_ts [ms] System clock at drouge deployment
 * @param drogue_dpl_max_acc [m/s2] Max acceleration during drouge deployment
 * @param dpl_vane_max_pressure [Pa] Max pressure in the deployment bay during drogue deployment
 * @param main_dpl_altitude_ts [ms] System clock at main chute deployment
 * @param main_dpl_altitude [m] Altittude at main deployment (m.s.l)
 * @param main_dpl_zspeed [m/s] Vertical speed at main deployment (sensor z axis)
 * @param main_dpl_acc [m/s2] Max measured vertical acceleration (sensor z axis) after main parachute deployment
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lr_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t liftoff_ts, uint64_t liftoff_max_acc_ts, float liftoff_max_acc, uint64_t max_z_speed_ts, float max_z_speed, float max_airspeed_pitot, float max_speed_altitude, uint64_t apogee_ts, float apogee_lat, float apogee_lon, float static_min_pressure, float digital_min_pressure, float ada_min_pressure, float baro_max_altitutde, float gps_max_altitude, uint64_t drogue_dpl_ts, float drogue_dpl_max_acc, float dpl_vane_max_pressure, uint64_t main_dpl_altitude_ts, float main_dpl_altitude, float main_dpl_zspeed, float main_dpl_acc, float cpu_load, uint32_t free_heap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LR_TM_LEN];
    _mav_put_uint64_t(buf, 0, liftoff_ts);
    _mav_put_uint64_t(buf, 8, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 16, max_z_speed_ts);
    _mav_put_uint64_t(buf, 24, apogee_ts);
    _mav_put_uint64_t(buf, 32, drogue_dpl_ts);
    _mav_put_uint64_t(buf, 40, main_dpl_altitude_ts);
    _mav_put_float(buf, 48, liftoff_max_acc);
    _mav_put_float(buf, 52, max_z_speed);
    _mav_put_float(buf, 56, max_airspeed_pitot);
    _mav_put_float(buf, 60, max_speed_altitude);
    _mav_put_float(buf, 64, apogee_lat);
    _mav_put_float(buf, 68, apogee_lon);
    _mav_put_float(buf, 72, static_min_pressure);
    _mav_put_float(buf, 76, digital_min_pressure);
    _mav_put_float(buf, 80, ada_min_pressure);
    _mav_put_float(buf, 84, baro_max_altitutde);
    _mav_put_float(buf, 88, gps_max_altitude);
    _mav_put_float(buf, 92, drogue_dpl_max_acc);
    _mav_put_float(buf, 96, dpl_vane_max_pressure);
    _mav_put_float(buf, 100, main_dpl_altitude);
    _mav_put_float(buf, 104, main_dpl_zspeed);
    _mav_put_float(buf, 108, main_dpl_acc);
    _mav_put_float(buf, 112, cpu_load);
    _mav_put_uint32_t(buf, 116, free_heap);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LR_TM_LEN);
#else
    mavlink_lr_tm_t packet;
    packet.liftoff_ts = liftoff_ts;
    packet.liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet.max_z_speed_ts = max_z_speed_ts;
    packet.apogee_ts = apogee_ts;
    packet.drogue_dpl_ts = drogue_dpl_ts;
    packet.main_dpl_altitude_ts = main_dpl_altitude_ts;
    packet.liftoff_max_acc = liftoff_max_acc;
    packet.max_z_speed = max_z_speed;
    packet.max_airspeed_pitot = max_airspeed_pitot;
    packet.max_speed_altitude = max_speed_altitude;
    packet.apogee_lat = apogee_lat;
    packet.apogee_lon = apogee_lon;
    packet.static_min_pressure = static_min_pressure;
    packet.digital_min_pressure = digital_min_pressure;
    packet.ada_min_pressure = ada_min_pressure;
    packet.baro_max_altitutde = baro_max_altitutde;
    packet.gps_max_altitude = gps_max_altitude;
    packet.drogue_dpl_max_acc = drogue_dpl_max_acc;
    packet.dpl_vane_max_pressure = dpl_vane_max_pressure;
    packet.main_dpl_altitude = main_dpl_altitude;
    packet.main_dpl_zspeed = main_dpl_zspeed;
    packet.main_dpl_acc = main_dpl_acc;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LR_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LR_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
}

/**
 * @brief Pack a lr_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param liftoff_ts [ms] System clock at liftoff
 * @param liftoff_max_acc_ts [ms] System clock at the maximum liftoff acceleration
 * @param liftoff_max_acc [m/s2] Maximum liftoff acceleration
 * @param max_z_speed_ts [ms] System clock at ADA max vertical speed
 * @param max_z_speed [m/s] Max measured vertical speed - ADA
 * @param max_airspeed_pitot [m/s] Max speed read by the pitot tube
 * @param max_speed_altitude [m] Altitude at max speed
 * @param apogee_ts [ms] System clock at apogee
 * @param apogee_lat [deg] Apogee latitude
 * @param apogee_lon [deg] Apogee longitude
 * @param static_min_pressure [Pa] Apogee pressure - Static ports
 * @param digital_min_pressure [Pa] Apogee pressure - Digital barometer
 * @param ada_min_pressure [Pa] Apogee pressure - ADA filtered
 * @param baro_max_altitutde [m] Apogee altitude - Digital barometer
 * @param gps_max_altitude [m] Apogee altitude - GPS
 * @param drogue_dpl_ts [ms] System clock at drouge deployment
 * @param drogue_dpl_max_acc [m/s2] Max acceleration during drouge deployment
 * @param dpl_vane_max_pressure [Pa] Max pressure in the deployment bay during drogue deployment
 * @param main_dpl_altitude_ts [ms] System clock at main chute deployment
 * @param main_dpl_altitude [m] Altittude at main deployment (m.s.l)
 * @param main_dpl_zspeed [m/s] Vertical speed at main deployment (sensor z axis)
 * @param main_dpl_acc [m/s2] Max measured vertical acceleration (sensor z axis) after main parachute deployment
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lr_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t liftoff_ts,uint64_t liftoff_max_acc_ts,float liftoff_max_acc,uint64_t max_z_speed_ts,float max_z_speed,float max_airspeed_pitot,float max_speed_altitude,uint64_t apogee_ts,float apogee_lat,float apogee_lon,float static_min_pressure,float digital_min_pressure,float ada_min_pressure,float baro_max_altitutde,float gps_max_altitude,uint64_t drogue_dpl_ts,float drogue_dpl_max_acc,float dpl_vane_max_pressure,uint64_t main_dpl_altitude_ts,float main_dpl_altitude,float main_dpl_zspeed,float main_dpl_acc,float cpu_load,uint32_t free_heap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LR_TM_LEN];
    _mav_put_uint64_t(buf, 0, liftoff_ts);
    _mav_put_uint64_t(buf, 8, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 16, max_z_speed_ts);
    _mav_put_uint64_t(buf, 24, apogee_ts);
    _mav_put_uint64_t(buf, 32, drogue_dpl_ts);
    _mav_put_uint64_t(buf, 40, main_dpl_altitude_ts);
    _mav_put_float(buf, 48, liftoff_max_acc);
    _mav_put_float(buf, 52, max_z_speed);
    _mav_put_float(buf, 56, max_airspeed_pitot);
    _mav_put_float(buf, 60, max_speed_altitude);
    _mav_put_float(buf, 64, apogee_lat);
    _mav_put_float(buf, 68, apogee_lon);
    _mav_put_float(buf, 72, static_min_pressure);
    _mav_put_float(buf, 76, digital_min_pressure);
    _mav_put_float(buf, 80, ada_min_pressure);
    _mav_put_float(buf, 84, baro_max_altitutde);
    _mav_put_float(buf, 88, gps_max_altitude);
    _mav_put_float(buf, 92, drogue_dpl_max_acc);
    _mav_put_float(buf, 96, dpl_vane_max_pressure);
    _mav_put_float(buf, 100, main_dpl_altitude);
    _mav_put_float(buf, 104, main_dpl_zspeed);
    _mav_put_float(buf, 108, main_dpl_acc);
    _mav_put_float(buf, 112, cpu_load);
    _mav_put_uint32_t(buf, 116, free_heap);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LR_TM_LEN);
#else
    mavlink_lr_tm_t packet;
    packet.liftoff_ts = liftoff_ts;
    packet.liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet.max_z_speed_ts = max_z_speed_ts;
    packet.apogee_ts = apogee_ts;
    packet.drogue_dpl_ts = drogue_dpl_ts;
    packet.main_dpl_altitude_ts = main_dpl_altitude_ts;
    packet.liftoff_max_acc = liftoff_max_acc;
    packet.max_z_speed = max_z_speed;
    packet.max_airspeed_pitot = max_airspeed_pitot;
    packet.max_speed_altitude = max_speed_altitude;
    packet.apogee_lat = apogee_lat;
    packet.apogee_lon = apogee_lon;
    packet.static_min_pressure = static_min_pressure;
    packet.digital_min_pressure = digital_min_pressure;
    packet.ada_min_pressure = ada_min_pressure;
    packet.baro_max_altitutde = baro_max_altitutde;
    packet.gps_max_altitude = gps_max_altitude;
    packet.drogue_dpl_max_acc = drogue_dpl_max_acc;
    packet.dpl_vane_max_pressure = dpl_vane_max_pressure;
    packet.main_dpl_altitude = main_dpl_altitude;
    packet.main_dpl_zspeed = main_dpl_zspeed;
    packet.main_dpl_acc = main_dpl_acc;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LR_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LR_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
}

/**
 * @brief Encode a lr_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lr_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lr_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lr_tm_t* lr_tm)
{
    return mavlink_msg_lr_tm_pack(system_id, component_id, msg, lr_tm->liftoff_ts, lr_tm->liftoff_max_acc_ts, lr_tm->liftoff_max_acc, lr_tm->max_z_speed_ts, lr_tm->max_z_speed, lr_tm->max_airspeed_pitot, lr_tm->max_speed_altitude, lr_tm->apogee_ts, lr_tm->apogee_lat, lr_tm->apogee_lon, lr_tm->static_min_pressure, lr_tm->digital_min_pressure, lr_tm->ada_min_pressure, lr_tm->baro_max_altitutde, lr_tm->gps_max_altitude, lr_tm->drogue_dpl_ts, lr_tm->drogue_dpl_max_acc, lr_tm->dpl_vane_max_pressure, lr_tm->main_dpl_altitude_ts, lr_tm->main_dpl_altitude, lr_tm->main_dpl_zspeed, lr_tm->main_dpl_acc, lr_tm->cpu_load, lr_tm->free_heap);
}

/**
 * @brief Encode a lr_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lr_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lr_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lr_tm_t* lr_tm)
{
    return mavlink_msg_lr_tm_pack_chan(system_id, component_id, chan, msg, lr_tm->liftoff_ts, lr_tm->liftoff_max_acc_ts, lr_tm->liftoff_max_acc, lr_tm->max_z_speed_ts, lr_tm->max_z_speed, lr_tm->max_airspeed_pitot, lr_tm->max_speed_altitude, lr_tm->apogee_ts, lr_tm->apogee_lat, lr_tm->apogee_lon, lr_tm->static_min_pressure, lr_tm->digital_min_pressure, lr_tm->ada_min_pressure, lr_tm->baro_max_altitutde, lr_tm->gps_max_altitude, lr_tm->drogue_dpl_ts, lr_tm->drogue_dpl_max_acc, lr_tm->dpl_vane_max_pressure, lr_tm->main_dpl_altitude_ts, lr_tm->main_dpl_altitude, lr_tm->main_dpl_zspeed, lr_tm->main_dpl_acc, lr_tm->cpu_load, lr_tm->free_heap);
}

/**
 * @brief Send a lr_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param liftoff_ts [ms] System clock at liftoff
 * @param liftoff_max_acc_ts [ms] System clock at the maximum liftoff acceleration
 * @param liftoff_max_acc [m/s2] Maximum liftoff acceleration
 * @param max_z_speed_ts [ms] System clock at ADA max vertical speed
 * @param max_z_speed [m/s] Max measured vertical speed - ADA
 * @param max_airspeed_pitot [m/s] Max speed read by the pitot tube
 * @param max_speed_altitude [m] Altitude at max speed
 * @param apogee_ts [ms] System clock at apogee
 * @param apogee_lat [deg] Apogee latitude
 * @param apogee_lon [deg] Apogee longitude
 * @param static_min_pressure [Pa] Apogee pressure - Static ports
 * @param digital_min_pressure [Pa] Apogee pressure - Digital barometer
 * @param ada_min_pressure [Pa] Apogee pressure - ADA filtered
 * @param baro_max_altitutde [m] Apogee altitude - Digital barometer
 * @param gps_max_altitude [m] Apogee altitude - GPS
 * @param drogue_dpl_ts [ms] System clock at drouge deployment
 * @param drogue_dpl_max_acc [m/s2] Max acceleration during drouge deployment
 * @param dpl_vane_max_pressure [Pa] Max pressure in the deployment bay during drogue deployment
 * @param main_dpl_altitude_ts [ms] System clock at main chute deployment
 * @param main_dpl_altitude [m] Altittude at main deployment (m.s.l)
 * @param main_dpl_zspeed [m/s] Vertical speed at main deployment (sensor z axis)
 * @param main_dpl_acc [m/s2] Max measured vertical acceleration (sensor z axis) after main parachute deployment
 * @param cpu_load  CPU load in percentage
 * @param free_heap  Amount of available heap in memory
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lr_tm_send(mavlink_channel_t chan, uint64_t liftoff_ts, uint64_t liftoff_max_acc_ts, float liftoff_max_acc, uint64_t max_z_speed_ts, float max_z_speed, float max_airspeed_pitot, float max_speed_altitude, uint64_t apogee_ts, float apogee_lat, float apogee_lon, float static_min_pressure, float digital_min_pressure, float ada_min_pressure, float baro_max_altitutde, float gps_max_altitude, uint64_t drogue_dpl_ts, float drogue_dpl_max_acc, float dpl_vane_max_pressure, uint64_t main_dpl_altitude_ts, float main_dpl_altitude, float main_dpl_zspeed, float main_dpl_acc, float cpu_load, uint32_t free_heap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LR_TM_LEN];
    _mav_put_uint64_t(buf, 0, liftoff_ts);
    _mav_put_uint64_t(buf, 8, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 16, max_z_speed_ts);
    _mav_put_uint64_t(buf, 24, apogee_ts);
    _mav_put_uint64_t(buf, 32, drogue_dpl_ts);
    _mav_put_uint64_t(buf, 40, main_dpl_altitude_ts);
    _mav_put_float(buf, 48, liftoff_max_acc);
    _mav_put_float(buf, 52, max_z_speed);
    _mav_put_float(buf, 56, max_airspeed_pitot);
    _mav_put_float(buf, 60, max_speed_altitude);
    _mav_put_float(buf, 64, apogee_lat);
    _mav_put_float(buf, 68, apogee_lon);
    _mav_put_float(buf, 72, static_min_pressure);
    _mav_put_float(buf, 76, digital_min_pressure);
    _mav_put_float(buf, 80, ada_min_pressure);
    _mav_put_float(buf, 84, baro_max_altitutde);
    _mav_put_float(buf, 88, gps_max_altitude);
    _mav_put_float(buf, 92, drogue_dpl_max_acc);
    _mav_put_float(buf, 96, dpl_vane_max_pressure);
    _mav_put_float(buf, 100, main_dpl_altitude);
    _mav_put_float(buf, 104, main_dpl_zspeed);
    _mav_put_float(buf, 108, main_dpl_acc);
    _mav_put_float(buf, 112, cpu_load);
    _mav_put_uint32_t(buf, 116, free_heap);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_TM, buf, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
#else
    mavlink_lr_tm_t packet;
    packet.liftoff_ts = liftoff_ts;
    packet.liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet.max_z_speed_ts = max_z_speed_ts;
    packet.apogee_ts = apogee_ts;
    packet.drogue_dpl_ts = drogue_dpl_ts;
    packet.main_dpl_altitude_ts = main_dpl_altitude_ts;
    packet.liftoff_max_acc = liftoff_max_acc;
    packet.max_z_speed = max_z_speed;
    packet.max_airspeed_pitot = max_airspeed_pitot;
    packet.max_speed_altitude = max_speed_altitude;
    packet.apogee_lat = apogee_lat;
    packet.apogee_lon = apogee_lon;
    packet.static_min_pressure = static_min_pressure;
    packet.digital_min_pressure = digital_min_pressure;
    packet.ada_min_pressure = ada_min_pressure;
    packet.baro_max_altitutde = baro_max_altitutde;
    packet.gps_max_altitude = gps_max_altitude;
    packet.drogue_dpl_max_acc = drogue_dpl_max_acc;
    packet.dpl_vane_max_pressure = dpl_vane_max_pressure;
    packet.main_dpl_altitude = main_dpl_altitude;
    packet.main_dpl_zspeed = main_dpl_zspeed;
    packet.main_dpl_acc = main_dpl_acc;
    packet.cpu_load = cpu_load;
    packet.free_heap = free_heap;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_TM, (const char *)&packet, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
#endif
}

/**
 * @brief Send a lr_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lr_tm_send_struct(mavlink_channel_t chan, const mavlink_lr_tm_t* lr_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lr_tm_send(chan, lr_tm->liftoff_ts, lr_tm->liftoff_max_acc_ts, lr_tm->liftoff_max_acc, lr_tm->max_z_speed_ts, lr_tm->max_z_speed, lr_tm->max_airspeed_pitot, lr_tm->max_speed_altitude, lr_tm->apogee_ts, lr_tm->apogee_lat, lr_tm->apogee_lon, lr_tm->static_min_pressure, lr_tm->digital_min_pressure, lr_tm->ada_min_pressure, lr_tm->baro_max_altitutde, lr_tm->gps_max_altitude, lr_tm->drogue_dpl_ts, lr_tm->drogue_dpl_max_acc, lr_tm->dpl_vane_max_pressure, lr_tm->main_dpl_altitude_ts, lr_tm->main_dpl_altitude, lr_tm->main_dpl_zspeed, lr_tm->main_dpl_acc, lr_tm->cpu_load, lr_tm->free_heap);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_TM, (const char *)lr_tm, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_LR_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lr_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t liftoff_ts, uint64_t liftoff_max_acc_ts, float liftoff_max_acc, uint64_t max_z_speed_ts, float max_z_speed, float max_airspeed_pitot, float max_speed_altitude, uint64_t apogee_ts, float apogee_lat, float apogee_lon, float static_min_pressure, float digital_min_pressure, float ada_min_pressure, float baro_max_altitutde, float gps_max_altitude, uint64_t drogue_dpl_ts, float drogue_dpl_max_acc, float dpl_vane_max_pressure, uint64_t main_dpl_altitude_ts, float main_dpl_altitude, float main_dpl_zspeed, float main_dpl_acc, float cpu_load, uint32_t free_heap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, liftoff_ts);
    _mav_put_uint64_t(buf, 8, liftoff_max_acc_ts);
    _mav_put_uint64_t(buf, 16, max_z_speed_ts);
    _mav_put_uint64_t(buf, 24, apogee_ts);
    _mav_put_uint64_t(buf, 32, drogue_dpl_ts);
    _mav_put_uint64_t(buf, 40, main_dpl_altitude_ts);
    _mav_put_float(buf, 48, liftoff_max_acc);
    _mav_put_float(buf, 52, max_z_speed);
    _mav_put_float(buf, 56, max_airspeed_pitot);
    _mav_put_float(buf, 60, max_speed_altitude);
    _mav_put_float(buf, 64, apogee_lat);
    _mav_put_float(buf, 68, apogee_lon);
    _mav_put_float(buf, 72, static_min_pressure);
    _mav_put_float(buf, 76, digital_min_pressure);
    _mav_put_float(buf, 80, ada_min_pressure);
    _mav_put_float(buf, 84, baro_max_altitutde);
    _mav_put_float(buf, 88, gps_max_altitude);
    _mav_put_float(buf, 92, drogue_dpl_max_acc);
    _mav_put_float(buf, 96, dpl_vane_max_pressure);
    _mav_put_float(buf, 100, main_dpl_altitude);
    _mav_put_float(buf, 104, main_dpl_zspeed);
    _mav_put_float(buf, 108, main_dpl_acc);
    _mav_put_float(buf, 112, cpu_load);
    _mav_put_uint32_t(buf, 116, free_heap);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_TM, buf, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
#else
    mavlink_lr_tm_t *packet = (mavlink_lr_tm_t *)msgbuf;
    packet->liftoff_ts = liftoff_ts;
    packet->liftoff_max_acc_ts = liftoff_max_acc_ts;
    packet->max_z_speed_ts = max_z_speed_ts;
    packet->apogee_ts = apogee_ts;
    packet->drogue_dpl_ts = drogue_dpl_ts;
    packet->main_dpl_altitude_ts = main_dpl_altitude_ts;
    packet->liftoff_max_acc = liftoff_max_acc;
    packet->max_z_speed = max_z_speed;
    packet->max_airspeed_pitot = max_airspeed_pitot;
    packet->max_speed_altitude = max_speed_altitude;
    packet->apogee_lat = apogee_lat;
    packet->apogee_lon = apogee_lon;
    packet->static_min_pressure = static_min_pressure;
    packet->digital_min_pressure = digital_min_pressure;
    packet->ada_min_pressure = ada_min_pressure;
    packet->baro_max_altitutde = baro_max_altitutde;
    packet->gps_max_altitude = gps_max_altitude;
    packet->drogue_dpl_max_acc = drogue_dpl_max_acc;
    packet->dpl_vane_max_pressure = dpl_vane_max_pressure;
    packet->main_dpl_altitude = main_dpl_altitude;
    packet->main_dpl_zspeed = main_dpl_zspeed;
    packet->main_dpl_acc = main_dpl_acc;
    packet->cpu_load = cpu_load;
    packet->free_heap = free_heap;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_TM, (const char *)packet, MAVLINK_MSG_ID_LR_TM_MIN_LEN, MAVLINK_MSG_ID_LR_TM_LEN, MAVLINK_MSG_ID_LR_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE LR_TM UNPACKING


/**
 * @brief Get field liftoff_ts from lr_tm message
 *
 * @return [ms] System clock at liftoff
 */
static inline uint64_t mavlink_msg_lr_tm_get_liftoff_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field liftoff_max_acc_ts from lr_tm message
 *
 * @return [ms] System clock at the maximum liftoff acceleration
 */
static inline uint64_t mavlink_msg_lr_tm_get_liftoff_max_acc_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field liftoff_max_acc from lr_tm message
 *
 * @return [m/s2] Maximum liftoff acceleration
 */
static inline float mavlink_msg_lr_tm_get_liftoff_max_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field max_z_speed_ts from lr_tm message
 *
 * @return [ms] System clock at ADA max vertical speed
 */
static inline uint64_t mavlink_msg_lr_tm_get_max_z_speed_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Get field max_z_speed from lr_tm message
 *
 * @return [m/s] Max measured vertical speed - ADA
 */
static inline float mavlink_msg_lr_tm_get_max_z_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field max_airspeed_pitot from lr_tm message
 *
 * @return [m/s] Max speed read by the pitot tube
 */
static inline float mavlink_msg_lr_tm_get_max_airspeed_pitot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field max_speed_altitude from lr_tm message
 *
 * @return [m] Altitude at max speed
 */
static inline float mavlink_msg_lr_tm_get_max_speed_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field apogee_ts from lr_tm message
 *
 * @return [ms] System clock at apogee
 */
static inline uint64_t mavlink_msg_lr_tm_get_apogee_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  24);
}

/**
 * @brief Get field apogee_lat from lr_tm message
 *
 * @return [deg] Apogee latitude
 */
static inline float mavlink_msg_lr_tm_get_apogee_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field apogee_lon from lr_tm message
 *
 * @return [deg] Apogee longitude
 */
static inline float mavlink_msg_lr_tm_get_apogee_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field static_min_pressure from lr_tm message
 *
 * @return [Pa] Apogee pressure - Static ports
 */
static inline float mavlink_msg_lr_tm_get_static_min_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field digital_min_pressure from lr_tm message
 *
 * @return [Pa] Apogee pressure - Digital barometer
 */
static inline float mavlink_msg_lr_tm_get_digital_min_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field ada_min_pressure from lr_tm message
 *
 * @return [Pa] Apogee pressure - ADA filtered
 */
static inline float mavlink_msg_lr_tm_get_ada_min_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field baro_max_altitutde from lr_tm message
 *
 * @return [m] Apogee altitude - Digital barometer
 */
static inline float mavlink_msg_lr_tm_get_baro_max_altitutde(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field gps_max_altitude from lr_tm message
 *
 * @return [m] Apogee altitude - GPS
 */
static inline float mavlink_msg_lr_tm_get_gps_max_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field drogue_dpl_ts from lr_tm message
 *
 * @return [ms] System clock at drouge deployment
 */
static inline uint64_t mavlink_msg_lr_tm_get_drogue_dpl_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  32);
}

/**
 * @brief Get field drogue_dpl_max_acc from lr_tm message
 *
 * @return [m/s2] Max acceleration during drouge deployment
 */
static inline float mavlink_msg_lr_tm_get_drogue_dpl_max_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field dpl_vane_max_pressure from lr_tm message
 *
 * @return [Pa] Max pressure in the deployment bay during drogue deployment
 */
static inline float mavlink_msg_lr_tm_get_dpl_vane_max_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field main_dpl_altitude_ts from lr_tm message
 *
 * @return [ms] System clock at main chute deployment
 */
static inline uint64_t mavlink_msg_lr_tm_get_main_dpl_altitude_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  40);
}

/**
 * @brief Get field main_dpl_altitude from lr_tm message
 *
 * @return [m] Altittude at main deployment (m.s.l)
 */
static inline float mavlink_msg_lr_tm_get_main_dpl_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Get field main_dpl_zspeed from lr_tm message
 *
 * @return [m/s] Vertical speed at main deployment (sensor z axis)
 */
static inline float mavlink_msg_lr_tm_get_main_dpl_zspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  104);
}

/**
 * @brief Get field main_dpl_acc from lr_tm message
 *
 * @return [m/s2] Max measured vertical acceleration (sensor z axis) after main parachute deployment
 */
static inline float mavlink_msg_lr_tm_get_main_dpl_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Get field cpu_load from lr_tm message
 *
 * @return  CPU load in percentage
 */
static inline float mavlink_msg_lr_tm_get_cpu_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Get field free_heap from lr_tm message
 *
 * @return  Amount of available heap in memory
 */
static inline uint32_t mavlink_msg_lr_tm_get_free_heap(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  116);
}

/**
 * @brief Decode a lr_tm message into a struct
 *
 * @param msg The message to decode
 * @param lr_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_lr_tm_decode(const mavlink_message_t* msg, mavlink_lr_tm_t* lr_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lr_tm->liftoff_ts = mavlink_msg_lr_tm_get_liftoff_ts(msg);
    lr_tm->liftoff_max_acc_ts = mavlink_msg_lr_tm_get_liftoff_max_acc_ts(msg);
    lr_tm->max_z_speed_ts = mavlink_msg_lr_tm_get_max_z_speed_ts(msg);
    lr_tm->apogee_ts = mavlink_msg_lr_tm_get_apogee_ts(msg);
    lr_tm->drogue_dpl_ts = mavlink_msg_lr_tm_get_drogue_dpl_ts(msg);
    lr_tm->main_dpl_altitude_ts = mavlink_msg_lr_tm_get_main_dpl_altitude_ts(msg);
    lr_tm->liftoff_max_acc = mavlink_msg_lr_tm_get_liftoff_max_acc(msg);
    lr_tm->max_z_speed = mavlink_msg_lr_tm_get_max_z_speed(msg);
    lr_tm->max_airspeed_pitot = mavlink_msg_lr_tm_get_max_airspeed_pitot(msg);
    lr_tm->max_speed_altitude = mavlink_msg_lr_tm_get_max_speed_altitude(msg);
    lr_tm->apogee_lat = mavlink_msg_lr_tm_get_apogee_lat(msg);
    lr_tm->apogee_lon = mavlink_msg_lr_tm_get_apogee_lon(msg);
    lr_tm->static_min_pressure = mavlink_msg_lr_tm_get_static_min_pressure(msg);
    lr_tm->digital_min_pressure = mavlink_msg_lr_tm_get_digital_min_pressure(msg);
    lr_tm->ada_min_pressure = mavlink_msg_lr_tm_get_ada_min_pressure(msg);
    lr_tm->baro_max_altitutde = mavlink_msg_lr_tm_get_baro_max_altitutde(msg);
    lr_tm->gps_max_altitude = mavlink_msg_lr_tm_get_gps_max_altitude(msg);
    lr_tm->drogue_dpl_max_acc = mavlink_msg_lr_tm_get_drogue_dpl_max_acc(msg);
    lr_tm->dpl_vane_max_pressure = mavlink_msg_lr_tm_get_dpl_vane_max_pressure(msg);
    lr_tm->main_dpl_altitude = mavlink_msg_lr_tm_get_main_dpl_altitude(msg);
    lr_tm->main_dpl_zspeed = mavlink_msg_lr_tm_get_main_dpl_zspeed(msg);
    lr_tm->main_dpl_acc = mavlink_msg_lr_tm_get_main_dpl_acc(msg);
    lr_tm->cpu_load = mavlink_msg_lr_tm_get_cpu_load(msg);
    lr_tm->free_heap = mavlink_msg_lr_tm_get_free_heap(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LR_TM_LEN? msg->len : MAVLINK_MSG_ID_LR_TM_LEN;
        memset(lr_tm, 0, MAVLINK_MSG_ID_LR_TM_LEN);
    memcpy(lr_tm, _MAV_PAYLOAD(msg), len);
#endif
}
