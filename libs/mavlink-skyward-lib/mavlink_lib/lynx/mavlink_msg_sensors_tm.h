#pragma once
// MESSAGE SENSORS_TM PACKING

#define MAVLINK_MSG_ID_SENSORS_TM 184

MAVPACKED(
typedef struct __mavlink_sensors_tm_t {
 uint64_t timestamp; /*< [ms] */
 float bmx160_acc_x; /*< [m/s^2] */
 float bmx160_acc_y; /*< [m/s^2] */
 float bmx160_acc_z; /*< [m/s^2] */
 float bmx160_gyro_x; /*< [rad/s] */
 float bmx160_gyro_y; /*< [rad/s] */
 float bmx160_gyro_z; /*< [rad/s] */
 float bmx160_mag_x; /*< [uT] */
 float bmx160_mag_y; /*< [uT] */
 float bmx160_mag_z; /*< [uT] */
 float bmx160_temp; /*< [deg C] */
 float ms5803_press; /*< [Pa] */
 float ms5803_temp; /*< [deg C] */
 float dpl_press; /*< [Pa] */
 float pitot_press; /*< [Pa] */
 float static_press; /*< [Pa] */
 float lis3mdl_mag_x; /*< [uT] */
 float lis3mdl_mag_y; /*< [uT] */
 float lis3mdl_mag_z; /*< [uT] */
 float lis3mdl_temp; /*< [uT] */
 float gps_lat; /*< [deg] */
 float gps_lon; /*< [deg] */
 float gps_alt; /*< [m] */
 float vbat; /*< [V] */
 float vsupply_5v; /*< [V] */
 uint8_t gps_fix; /*<  */
}) mavlink_sensors_tm_t;

#define MAVLINK_MSG_ID_SENSORS_TM_LEN 105
#define MAVLINK_MSG_ID_SENSORS_TM_MIN_LEN 105
#define MAVLINK_MSG_ID_184_LEN 105
#define MAVLINK_MSG_ID_184_MIN_LEN 105

#define MAVLINK_MSG_ID_SENSORS_TM_CRC 230
#define MAVLINK_MSG_ID_184_CRC 230



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENSORS_TM { \
    184, \
    "SENSORS_TM", \
    26, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sensors_tm_t, timestamp) }, \
         { "bmx160_acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensors_tm_t, bmx160_acc_x) }, \
         { "bmx160_acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensors_tm_t, bmx160_acc_y) }, \
         { "bmx160_acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensors_tm_t, bmx160_acc_z) }, \
         { "bmx160_gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensors_tm_t, bmx160_gyro_x) }, \
         { "bmx160_gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sensors_tm_t, bmx160_gyro_y) }, \
         { "bmx160_gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sensors_tm_t, bmx160_gyro_z) }, \
         { "bmx160_mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sensors_tm_t, bmx160_mag_x) }, \
         { "bmx160_mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sensors_tm_t, bmx160_mag_y) }, \
         { "bmx160_mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sensors_tm_t, bmx160_mag_z) }, \
         { "bmx160_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_sensors_tm_t, bmx160_temp) }, \
         { "ms5803_press", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_sensors_tm_t, ms5803_press) }, \
         { "ms5803_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_sensors_tm_t, ms5803_temp) }, \
         { "dpl_press", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_sensors_tm_t, dpl_press) }, \
         { "pitot_press", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_sensors_tm_t, pitot_press) }, \
         { "static_press", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_sensors_tm_t, static_press) }, \
         { "lis3mdl_mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_sensors_tm_t, lis3mdl_mag_x) }, \
         { "lis3mdl_mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_sensors_tm_t, lis3mdl_mag_y) }, \
         { "lis3mdl_mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_sensors_tm_t, lis3mdl_mag_z) }, \
         { "lis3mdl_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_sensors_tm_t, lis3mdl_temp) }, \
         { "gps_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_sensors_tm_t, gps_lat) }, \
         { "gps_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_sensors_tm_t, gps_lon) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_sensors_tm_t, gps_alt) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 104, offsetof(mavlink_sensors_tm_t, gps_fix) }, \
         { "vbat", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_sensors_tm_t, vbat) }, \
         { "vsupply_5v", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_sensors_tm_t, vsupply_5v) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENSORS_TM { \
    "SENSORS_TM", \
    26, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sensors_tm_t, timestamp) }, \
         { "bmx160_acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensors_tm_t, bmx160_acc_x) }, \
         { "bmx160_acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensors_tm_t, bmx160_acc_y) }, \
         { "bmx160_acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensors_tm_t, bmx160_acc_z) }, \
         { "bmx160_gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensors_tm_t, bmx160_gyro_x) }, \
         { "bmx160_gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sensors_tm_t, bmx160_gyro_y) }, \
         { "bmx160_gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sensors_tm_t, bmx160_gyro_z) }, \
         { "bmx160_mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sensors_tm_t, bmx160_mag_x) }, \
         { "bmx160_mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sensors_tm_t, bmx160_mag_y) }, \
         { "bmx160_mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sensors_tm_t, bmx160_mag_z) }, \
         { "bmx160_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_sensors_tm_t, bmx160_temp) }, \
         { "ms5803_press", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_sensors_tm_t, ms5803_press) }, \
         { "ms5803_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_sensors_tm_t, ms5803_temp) }, \
         { "dpl_press", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_sensors_tm_t, dpl_press) }, \
         { "pitot_press", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_sensors_tm_t, pitot_press) }, \
         { "static_press", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_sensors_tm_t, static_press) }, \
         { "lis3mdl_mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_sensors_tm_t, lis3mdl_mag_x) }, \
         { "lis3mdl_mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_sensors_tm_t, lis3mdl_mag_y) }, \
         { "lis3mdl_mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_sensors_tm_t, lis3mdl_mag_z) }, \
         { "lis3mdl_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_sensors_tm_t, lis3mdl_temp) }, \
         { "gps_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_sensors_tm_t, gps_lat) }, \
         { "gps_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_sensors_tm_t, gps_lon) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_sensors_tm_t, gps_alt) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 104, offsetof(mavlink_sensors_tm_t, gps_fix) }, \
         { "vbat", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_sensors_tm_t, vbat) }, \
         { "vsupply_5v", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_sensors_tm_t, vsupply_5v) }, \
         } \
}
#endif

/**
 * @brief Pack a sensors_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] 
 * @param bmx160_acc_x [m/s^2] 
 * @param bmx160_acc_y [m/s^2] 
 * @param bmx160_acc_z [m/s^2] 
 * @param bmx160_gyro_x [rad/s] 
 * @param bmx160_gyro_y [rad/s] 
 * @param bmx160_gyro_z [rad/s] 
 * @param bmx160_mag_x [uT] 
 * @param bmx160_mag_y [uT] 
 * @param bmx160_mag_z [uT] 
 * @param bmx160_temp [deg C] 
 * @param ms5803_press [Pa] 
 * @param ms5803_temp [deg C] 
 * @param dpl_press [Pa] 
 * @param pitot_press [Pa] 
 * @param static_press [Pa] 
 * @param lis3mdl_mag_x [uT] 
 * @param lis3mdl_mag_y [uT] 
 * @param lis3mdl_mag_z [uT] 
 * @param lis3mdl_temp [uT] 
 * @param gps_lat [deg] 
 * @param gps_lon [deg] 
 * @param gps_alt [m] 
 * @param gps_fix  
 * @param vbat [V] 
 * @param vsupply_5v [V] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensors_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float bmx160_acc_x, float bmx160_acc_y, float bmx160_acc_z, float bmx160_gyro_x, float bmx160_gyro_y, float bmx160_gyro_z, float bmx160_mag_x, float bmx160_mag_y, float bmx160_mag_z, float bmx160_temp, float ms5803_press, float ms5803_temp, float dpl_press, float pitot_press, float static_press, float lis3mdl_mag_x, float lis3mdl_mag_y, float lis3mdl_mag_z, float lis3mdl_temp, float gps_lat, float gps_lon, float gps_alt, uint8_t gps_fix, float vbat, float vsupply_5v)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSORS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, bmx160_acc_x);
    _mav_put_float(buf, 12, bmx160_acc_y);
    _mav_put_float(buf, 16, bmx160_acc_z);
    _mav_put_float(buf, 20, bmx160_gyro_x);
    _mav_put_float(buf, 24, bmx160_gyro_y);
    _mav_put_float(buf, 28, bmx160_gyro_z);
    _mav_put_float(buf, 32, bmx160_mag_x);
    _mav_put_float(buf, 36, bmx160_mag_y);
    _mav_put_float(buf, 40, bmx160_mag_z);
    _mav_put_float(buf, 44, bmx160_temp);
    _mav_put_float(buf, 48, ms5803_press);
    _mav_put_float(buf, 52, ms5803_temp);
    _mav_put_float(buf, 56, dpl_press);
    _mav_put_float(buf, 60, pitot_press);
    _mav_put_float(buf, 64, static_press);
    _mav_put_float(buf, 68, lis3mdl_mag_x);
    _mav_put_float(buf, 72, lis3mdl_mag_y);
    _mav_put_float(buf, 76, lis3mdl_mag_z);
    _mav_put_float(buf, 80, lis3mdl_temp);
    _mav_put_float(buf, 84, gps_lat);
    _mav_put_float(buf, 88, gps_lon);
    _mav_put_float(buf, 92, gps_alt);
    _mav_put_float(buf, 96, vbat);
    _mav_put_float(buf, 100, vsupply_5v);
    _mav_put_uint8_t(buf, 104, gps_fix);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSORS_TM_LEN);
#else
    mavlink_sensors_tm_t packet;
    packet.timestamp = timestamp;
    packet.bmx160_acc_x = bmx160_acc_x;
    packet.bmx160_acc_y = bmx160_acc_y;
    packet.bmx160_acc_z = bmx160_acc_z;
    packet.bmx160_gyro_x = bmx160_gyro_x;
    packet.bmx160_gyro_y = bmx160_gyro_y;
    packet.bmx160_gyro_z = bmx160_gyro_z;
    packet.bmx160_mag_x = bmx160_mag_x;
    packet.bmx160_mag_y = bmx160_mag_y;
    packet.bmx160_mag_z = bmx160_mag_z;
    packet.bmx160_temp = bmx160_temp;
    packet.ms5803_press = ms5803_press;
    packet.ms5803_temp = ms5803_temp;
    packet.dpl_press = dpl_press;
    packet.pitot_press = pitot_press;
    packet.static_press = static_press;
    packet.lis3mdl_mag_x = lis3mdl_mag_x;
    packet.lis3mdl_mag_y = lis3mdl_mag_y;
    packet.lis3mdl_mag_z = lis3mdl_mag_z;
    packet.lis3mdl_temp = lis3mdl_temp;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.gps_alt = gps_alt;
    packet.vbat = vbat;
    packet.vsupply_5v = vsupply_5v;
    packet.gps_fix = gps_fix;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSORS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSORS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSORS_TM_MIN_LEN, MAVLINK_MSG_ID_SENSORS_TM_LEN, MAVLINK_MSG_ID_SENSORS_TM_CRC);
}

/**
 * @brief Pack a sensors_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] 
 * @param bmx160_acc_x [m/s^2] 
 * @param bmx160_acc_y [m/s^2] 
 * @param bmx160_acc_z [m/s^2] 
 * @param bmx160_gyro_x [rad/s] 
 * @param bmx160_gyro_y [rad/s] 
 * @param bmx160_gyro_z [rad/s] 
 * @param bmx160_mag_x [uT] 
 * @param bmx160_mag_y [uT] 
 * @param bmx160_mag_z [uT] 
 * @param bmx160_temp [deg C] 
 * @param ms5803_press [Pa] 
 * @param ms5803_temp [deg C] 
 * @param dpl_press [Pa] 
 * @param pitot_press [Pa] 
 * @param static_press [Pa] 
 * @param lis3mdl_mag_x [uT] 
 * @param lis3mdl_mag_y [uT] 
 * @param lis3mdl_mag_z [uT] 
 * @param lis3mdl_temp [uT] 
 * @param gps_lat [deg] 
 * @param gps_lon [deg] 
 * @param gps_alt [m] 
 * @param gps_fix  
 * @param vbat [V] 
 * @param vsupply_5v [V] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensors_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float bmx160_acc_x,float bmx160_acc_y,float bmx160_acc_z,float bmx160_gyro_x,float bmx160_gyro_y,float bmx160_gyro_z,float bmx160_mag_x,float bmx160_mag_y,float bmx160_mag_z,float bmx160_temp,float ms5803_press,float ms5803_temp,float dpl_press,float pitot_press,float static_press,float lis3mdl_mag_x,float lis3mdl_mag_y,float lis3mdl_mag_z,float lis3mdl_temp,float gps_lat,float gps_lon,float gps_alt,uint8_t gps_fix,float vbat,float vsupply_5v)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSORS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, bmx160_acc_x);
    _mav_put_float(buf, 12, bmx160_acc_y);
    _mav_put_float(buf, 16, bmx160_acc_z);
    _mav_put_float(buf, 20, bmx160_gyro_x);
    _mav_put_float(buf, 24, bmx160_gyro_y);
    _mav_put_float(buf, 28, bmx160_gyro_z);
    _mav_put_float(buf, 32, bmx160_mag_x);
    _mav_put_float(buf, 36, bmx160_mag_y);
    _mav_put_float(buf, 40, bmx160_mag_z);
    _mav_put_float(buf, 44, bmx160_temp);
    _mav_put_float(buf, 48, ms5803_press);
    _mav_put_float(buf, 52, ms5803_temp);
    _mav_put_float(buf, 56, dpl_press);
    _mav_put_float(buf, 60, pitot_press);
    _mav_put_float(buf, 64, static_press);
    _mav_put_float(buf, 68, lis3mdl_mag_x);
    _mav_put_float(buf, 72, lis3mdl_mag_y);
    _mav_put_float(buf, 76, lis3mdl_mag_z);
    _mav_put_float(buf, 80, lis3mdl_temp);
    _mav_put_float(buf, 84, gps_lat);
    _mav_put_float(buf, 88, gps_lon);
    _mav_put_float(buf, 92, gps_alt);
    _mav_put_float(buf, 96, vbat);
    _mav_put_float(buf, 100, vsupply_5v);
    _mav_put_uint8_t(buf, 104, gps_fix);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSORS_TM_LEN);
#else
    mavlink_sensors_tm_t packet;
    packet.timestamp = timestamp;
    packet.bmx160_acc_x = bmx160_acc_x;
    packet.bmx160_acc_y = bmx160_acc_y;
    packet.bmx160_acc_z = bmx160_acc_z;
    packet.bmx160_gyro_x = bmx160_gyro_x;
    packet.bmx160_gyro_y = bmx160_gyro_y;
    packet.bmx160_gyro_z = bmx160_gyro_z;
    packet.bmx160_mag_x = bmx160_mag_x;
    packet.bmx160_mag_y = bmx160_mag_y;
    packet.bmx160_mag_z = bmx160_mag_z;
    packet.bmx160_temp = bmx160_temp;
    packet.ms5803_press = ms5803_press;
    packet.ms5803_temp = ms5803_temp;
    packet.dpl_press = dpl_press;
    packet.pitot_press = pitot_press;
    packet.static_press = static_press;
    packet.lis3mdl_mag_x = lis3mdl_mag_x;
    packet.lis3mdl_mag_y = lis3mdl_mag_y;
    packet.lis3mdl_mag_z = lis3mdl_mag_z;
    packet.lis3mdl_temp = lis3mdl_temp;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.gps_alt = gps_alt;
    packet.vbat = vbat;
    packet.vsupply_5v = vsupply_5v;
    packet.gps_fix = gps_fix;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSORS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSORS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSORS_TM_MIN_LEN, MAVLINK_MSG_ID_SENSORS_TM_LEN, MAVLINK_MSG_ID_SENSORS_TM_CRC);
}

/**
 * @brief Encode a sensors_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensors_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensors_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensors_tm_t* sensors_tm)
{
    return mavlink_msg_sensors_tm_pack(system_id, component_id, msg, sensors_tm->timestamp, sensors_tm->bmx160_acc_x, sensors_tm->bmx160_acc_y, sensors_tm->bmx160_acc_z, sensors_tm->bmx160_gyro_x, sensors_tm->bmx160_gyro_y, sensors_tm->bmx160_gyro_z, sensors_tm->bmx160_mag_x, sensors_tm->bmx160_mag_y, sensors_tm->bmx160_mag_z, sensors_tm->bmx160_temp, sensors_tm->ms5803_press, sensors_tm->ms5803_temp, sensors_tm->dpl_press, sensors_tm->pitot_press, sensors_tm->static_press, sensors_tm->lis3mdl_mag_x, sensors_tm->lis3mdl_mag_y, sensors_tm->lis3mdl_mag_z, sensors_tm->lis3mdl_temp, sensors_tm->gps_lat, sensors_tm->gps_lon, sensors_tm->gps_alt, sensors_tm->gps_fix, sensors_tm->vbat, sensors_tm->vsupply_5v);
}

/**
 * @brief Encode a sensors_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensors_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensors_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensors_tm_t* sensors_tm)
{
    return mavlink_msg_sensors_tm_pack_chan(system_id, component_id, chan, msg, sensors_tm->timestamp, sensors_tm->bmx160_acc_x, sensors_tm->bmx160_acc_y, sensors_tm->bmx160_acc_z, sensors_tm->bmx160_gyro_x, sensors_tm->bmx160_gyro_y, sensors_tm->bmx160_gyro_z, sensors_tm->bmx160_mag_x, sensors_tm->bmx160_mag_y, sensors_tm->bmx160_mag_z, sensors_tm->bmx160_temp, sensors_tm->ms5803_press, sensors_tm->ms5803_temp, sensors_tm->dpl_press, sensors_tm->pitot_press, sensors_tm->static_press, sensors_tm->lis3mdl_mag_x, sensors_tm->lis3mdl_mag_y, sensors_tm->lis3mdl_mag_z, sensors_tm->lis3mdl_temp, sensors_tm->gps_lat, sensors_tm->gps_lon, sensors_tm->gps_alt, sensors_tm->gps_fix, sensors_tm->vbat, sensors_tm->vsupply_5v);
}

/**
 * @brief Send a sensors_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] 
 * @param bmx160_acc_x [m/s^2] 
 * @param bmx160_acc_y [m/s^2] 
 * @param bmx160_acc_z [m/s^2] 
 * @param bmx160_gyro_x [rad/s] 
 * @param bmx160_gyro_y [rad/s] 
 * @param bmx160_gyro_z [rad/s] 
 * @param bmx160_mag_x [uT] 
 * @param bmx160_mag_y [uT] 
 * @param bmx160_mag_z [uT] 
 * @param bmx160_temp [deg C] 
 * @param ms5803_press [Pa] 
 * @param ms5803_temp [deg C] 
 * @param dpl_press [Pa] 
 * @param pitot_press [Pa] 
 * @param static_press [Pa] 
 * @param lis3mdl_mag_x [uT] 
 * @param lis3mdl_mag_y [uT] 
 * @param lis3mdl_mag_z [uT] 
 * @param lis3mdl_temp [uT] 
 * @param gps_lat [deg] 
 * @param gps_lon [deg] 
 * @param gps_alt [m] 
 * @param gps_fix  
 * @param vbat [V] 
 * @param vsupply_5v [V] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensors_tm_send(mavlink_channel_t chan, uint64_t timestamp, float bmx160_acc_x, float bmx160_acc_y, float bmx160_acc_z, float bmx160_gyro_x, float bmx160_gyro_y, float bmx160_gyro_z, float bmx160_mag_x, float bmx160_mag_y, float bmx160_mag_z, float bmx160_temp, float ms5803_press, float ms5803_temp, float dpl_press, float pitot_press, float static_press, float lis3mdl_mag_x, float lis3mdl_mag_y, float lis3mdl_mag_z, float lis3mdl_temp, float gps_lat, float gps_lon, float gps_alt, uint8_t gps_fix, float vbat, float vsupply_5v)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSORS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, bmx160_acc_x);
    _mav_put_float(buf, 12, bmx160_acc_y);
    _mav_put_float(buf, 16, bmx160_acc_z);
    _mav_put_float(buf, 20, bmx160_gyro_x);
    _mav_put_float(buf, 24, bmx160_gyro_y);
    _mav_put_float(buf, 28, bmx160_gyro_z);
    _mav_put_float(buf, 32, bmx160_mag_x);
    _mav_put_float(buf, 36, bmx160_mag_y);
    _mav_put_float(buf, 40, bmx160_mag_z);
    _mav_put_float(buf, 44, bmx160_temp);
    _mav_put_float(buf, 48, ms5803_press);
    _mav_put_float(buf, 52, ms5803_temp);
    _mav_put_float(buf, 56, dpl_press);
    _mav_put_float(buf, 60, pitot_press);
    _mav_put_float(buf, 64, static_press);
    _mav_put_float(buf, 68, lis3mdl_mag_x);
    _mav_put_float(buf, 72, lis3mdl_mag_y);
    _mav_put_float(buf, 76, lis3mdl_mag_z);
    _mav_put_float(buf, 80, lis3mdl_temp);
    _mav_put_float(buf, 84, gps_lat);
    _mav_put_float(buf, 88, gps_lon);
    _mav_put_float(buf, 92, gps_alt);
    _mav_put_float(buf, 96, vbat);
    _mav_put_float(buf, 100, vsupply_5v);
    _mav_put_uint8_t(buf, 104, gps_fix);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSORS_TM, buf, MAVLINK_MSG_ID_SENSORS_TM_MIN_LEN, MAVLINK_MSG_ID_SENSORS_TM_LEN, MAVLINK_MSG_ID_SENSORS_TM_CRC);
#else
    mavlink_sensors_tm_t packet;
    packet.timestamp = timestamp;
    packet.bmx160_acc_x = bmx160_acc_x;
    packet.bmx160_acc_y = bmx160_acc_y;
    packet.bmx160_acc_z = bmx160_acc_z;
    packet.bmx160_gyro_x = bmx160_gyro_x;
    packet.bmx160_gyro_y = bmx160_gyro_y;
    packet.bmx160_gyro_z = bmx160_gyro_z;
    packet.bmx160_mag_x = bmx160_mag_x;
    packet.bmx160_mag_y = bmx160_mag_y;
    packet.bmx160_mag_z = bmx160_mag_z;
    packet.bmx160_temp = bmx160_temp;
    packet.ms5803_press = ms5803_press;
    packet.ms5803_temp = ms5803_temp;
    packet.dpl_press = dpl_press;
    packet.pitot_press = pitot_press;
    packet.static_press = static_press;
    packet.lis3mdl_mag_x = lis3mdl_mag_x;
    packet.lis3mdl_mag_y = lis3mdl_mag_y;
    packet.lis3mdl_mag_z = lis3mdl_mag_z;
    packet.lis3mdl_temp = lis3mdl_temp;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.gps_alt = gps_alt;
    packet.vbat = vbat;
    packet.vsupply_5v = vsupply_5v;
    packet.gps_fix = gps_fix;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSORS_TM, (const char *)&packet, MAVLINK_MSG_ID_SENSORS_TM_MIN_LEN, MAVLINK_MSG_ID_SENSORS_TM_LEN, MAVLINK_MSG_ID_SENSORS_TM_CRC);
#endif
}

/**
 * @brief Send a sensors_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sensors_tm_send_struct(mavlink_channel_t chan, const mavlink_sensors_tm_t* sensors_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sensors_tm_send(chan, sensors_tm->timestamp, sensors_tm->bmx160_acc_x, sensors_tm->bmx160_acc_y, sensors_tm->bmx160_acc_z, sensors_tm->bmx160_gyro_x, sensors_tm->bmx160_gyro_y, sensors_tm->bmx160_gyro_z, sensors_tm->bmx160_mag_x, sensors_tm->bmx160_mag_y, sensors_tm->bmx160_mag_z, sensors_tm->bmx160_temp, sensors_tm->ms5803_press, sensors_tm->ms5803_temp, sensors_tm->dpl_press, sensors_tm->pitot_press, sensors_tm->static_press, sensors_tm->lis3mdl_mag_x, sensors_tm->lis3mdl_mag_y, sensors_tm->lis3mdl_mag_z, sensors_tm->lis3mdl_temp, sensors_tm->gps_lat, sensors_tm->gps_lon, sensors_tm->gps_alt, sensors_tm->gps_fix, sensors_tm->vbat, sensors_tm->vsupply_5v);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSORS_TM, (const char *)sensors_tm, MAVLINK_MSG_ID_SENSORS_TM_MIN_LEN, MAVLINK_MSG_ID_SENSORS_TM_LEN, MAVLINK_MSG_ID_SENSORS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENSORS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sensors_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float bmx160_acc_x, float bmx160_acc_y, float bmx160_acc_z, float bmx160_gyro_x, float bmx160_gyro_y, float bmx160_gyro_z, float bmx160_mag_x, float bmx160_mag_y, float bmx160_mag_z, float bmx160_temp, float ms5803_press, float ms5803_temp, float dpl_press, float pitot_press, float static_press, float lis3mdl_mag_x, float lis3mdl_mag_y, float lis3mdl_mag_z, float lis3mdl_temp, float gps_lat, float gps_lon, float gps_alt, uint8_t gps_fix, float vbat, float vsupply_5v)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, bmx160_acc_x);
    _mav_put_float(buf, 12, bmx160_acc_y);
    _mav_put_float(buf, 16, bmx160_acc_z);
    _mav_put_float(buf, 20, bmx160_gyro_x);
    _mav_put_float(buf, 24, bmx160_gyro_y);
    _mav_put_float(buf, 28, bmx160_gyro_z);
    _mav_put_float(buf, 32, bmx160_mag_x);
    _mav_put_float(buf, 36, bmx160_mag_y);
    _mav_put_float(buf, 40, bmx160_mag_z);
    _mav_put_float(buf, 44, bmx160_temp);
    _mav_put_float(buf, 48, ms5803_press);
    _mav_put_float(buf, 52, ms5803_temp);
    _mav_put_float(buf, 56, dpl_press);
    _mav_put_float(buf, 60, pitot_press);
    _mav_put_float(buf, 64, static_press);
    _mav_put_float(buf, 68, lis3mdl_mag_x);
    _mav_put_float(buf, 72, lis3mdl_mag_y);
    _mav_put_float(buf, 76, lis3mdl_mag_z);
    _mav_put_float(buf, 80, lis3mdl_temp);
    _mav_put_float(buf, 84, gps_lat);
    _mav_put_float(buf, 88, gps_lon);
    _mav_put_float(buf, 92, gps_alt);
    _mav_put_float(buf, 96, vbat);
    _mav_put_float(buf, 100, vsupply_5v);
    _mav_put_uint8_t(buf, 104, gps_fix);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSORS_TM, buf, MAVLINK_MSG_ID_SENSORS_TM_MIN_LEN, MAVLINK_MSG_ID_SENSORS_TM_LEN, MAVLINK_MSG_ID_SENSORS_TM_CRC);
#else
    mavlink_sensors_tm_t *packet = (mavlink_sensors_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->bmx160_acc_x = bmx160_acc_x;
    packet->bmx160_acc_y = bmx160_acc_y;
    packet->bmx160_acc_z = bmx160_acc_z;
    packet->bmx160_gyro_x = bmx160_gyro_x;
    packet->bmx160_gyro_y = bmx160_gyro_y;
    packet->bmx160_gyro_z = bmx160_gyro_z;
    packet->bmx160_mag_x = bmx160_mag_x;
    packet->bmx160_mag_y = bmx160_mag_y;
    packet->bmx160_mag_z = bmx160_mag_z;
    packet->bmx160_temp = bmx160_temp;
    packet->ms5803_press = ms5803_press;
    packet->ms5803_temp = ms5803_temp;
    packet->dpl_press = dpl_press;
    packet->pitot_press = pitot_press;
    packet->static_press = static_press;
    packet->lis3mdl_mag_x = lis3mdl_mag_x;
    packet->lis3mdl_mag_y = lis3mdl_mag_y;
    packet->lis3mdl_mag_z = lis3mdl_mag_z;
    packet->lis3mdl_temp = lis3mdl_temp;
    packet->gps_lat = gps_lat;
    packet->gps_lon = gps_lon;
    packet->gps_alt = gps_alt;
    packet->vbat = vbat;
    packet->vsupply_5v = vsupply_5v;
    packet->gps_fix = gps_fix;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSORS_TM, (const char *)packet, MAVLINK_MSG_ID_SENSORS_TM_MIN_LEN, MAVLINK_MSG_ID_SENSORS_TM_LEN, MAVLINK_MSG_ID_SENSORS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE SENSORS_TM UNPACKING


/**
 * @brief Get field timestamp from sensors_tm message
 *
 * @return [ms] 
 */
static inline uint64_t mavlink_msg_sensors_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field bmx160_acc_x from sensors_tm message
 *
 * @return [m/s^2] 
 */
static inline float mavlink_msg_sensors_tm_get_bmx160_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field bmx160_acc_y from sensors_tm message
 *
 * @return [m/s^2] 
 */
static inline float mavlink_msg_sensors_tm_get_bmx160_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field bmx160_acc_z from sensors_tm message
 *
 * @return [m/s^2] 
 */
static inline float mavlink_msg_sensors_tm_get_bmx160_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field bmx160_gyro_x from sensors_tm message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_sensors_tm_get_bmx160_gyro_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field bmx160_gyro_y from sensors_tm message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_sensors_tm_get_bmx160_gyro_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field bmx160_gyro_z from sensors_tm message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_sensors_tm_get_bmx160_gyro_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field bmx160_mag_x from sensors_tm message
 *
 * @return [uT] 
 */
static inline float mavlink_msg_sensors_tm_get_bmx160_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field bmx160_mag_y from sensors_tm message
 *
 * @return [uT] 
 */
static inline float mavlink_msg_sensors_tm_get_bmx160_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field bmx160_mag_z from sensors_tm message
 *
 * @return [uT] 
 */
static inline float mavlink_msg_sensors_tm_get_bmx160_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field bmx160_temp from sensors_tm message
 *
 * @return [deg C] 
 */
static inline float mavlink_msg_sensors_tm_get_bmx160_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field ms5803_press from sensors_tm message
 *
 * @return [Pa] 
 */
static inline float mavlink_msg_sensors_tm_get_ms5803_press(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field ms5803_temp from sensors_tm message
 *
 * @return [deg C] 
 */
static inline float mavlink_msg_sensors_tm_get_ms5803_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field dpl_press from sensors_tm message
 *
 * @return [Pa] 
 */
static inline float mavlink_msg_sensors_tm_get_dpl_press(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field pitot_press from sensors_tm message
 *
 * @return [Pa] 
 */
static inline float mavlink_msg_sensors_tm_get_pitot_press(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field static_press from sensors_tm message
 *
 * @return [Pa] 
 */
static inline float mavlink_msg_sensors_tm_get_static_press(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field lis3mdl_mag_x from sensors_tm message
 *
 * @return [uT] 
 */
static inline float mavlink_msg_sensors_tm_get_lis3mdl_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field lis3mdl_mag_y from sensors_tm message
 *
 * @return [uT] 
 */
static inline float mavlink_msg_sensors_tm_get_lis3mdl_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field lis3mdl_mag_z from sensors_tm message
 *
 * @return [uT] 
 */
static inline float mavlink_msg_sensors_tm_get_lis3mdl_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field lis3mdl_temp from sensors_tm message
 *
 * @return [uT] 
 */
static inline float mavlink_msg_sensors_tm_get_lis3mdl_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field gps_lat from sensors_tm message
 *
 * @return [deg] 
 */
static inline float mavlink_msg_sensors_tm_get_gps_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field gps_lon from sensors_tm message
 *
 * @return [deg] 
 */
static inline float mavlink_msg_sensors_tm_get_gps_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field gps_alt from sensors_tm message
 *
 * @return [m] 
 */
static inline float mavlink_msg_sensors_tm_get_gps_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field gps_fix from sensors_tm message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_sensors_tm_get_gps_fix(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  104);
}

/**
 * @brief Get field vbat from sensors_tm message
 *
 * @return [V] 
 */
static inline float mavlink_msg_sensors_tm_get_vbat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field vsupply_5v from sensors_tm message
 *
 * @return [V] 
 */
static inline float mavlink_msg_sensors_tm_get_vsupply_5v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Decode a sensors_tm message into a struct
 *
 * @param msg The message to decode
 * @param sensors_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensors_tm_decode(const mavlink_message_t* msg, mavlink_sensors_tm_t* sensors_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sensors_tm->timestamp = mavlink_msg_sensors_tm_get_timestamp(msg);
    sensors_tm->bmx160_acc_x = mavlink_msg_sensors_tm_get_bmx160_acc_x(msg);
    sensors_tm->bmx160_acc_y = mavlink_msg_sensors_tm_get_bmx160_acc_y(msg);
    sensors_tm->bmx160_acc_z = mavlink_msg_sensors_tm_get_bmx160_acc_z(msg);
    sensors_tm->bmx160_gyro_x = mavlink_msg_sensors_tm_get_bmx160_gyro_x(msg);
    sensors_tm->bmx160_gyro_y = mavlink_msg_sensors_tm_get_bmx160_gyro_y(msg);
    sensors_tm->bmx160_gyro_z = mavlink_msg_sensors_tm_get_bmx160_gyro_z(msg);
    sensors_tm->bmx160_mag_x = mavlink_msg_sensors_tm_get_bmx160_mag_x(msg);
    sensors_tm->bmx160_mag_y = mavlink_msg_sensors_tm_get_bmx160_mag_y(msg);
    sensors_tm->bmx160_mag_z = mavlink_msg_sensors_tm_get_bmx160_mag_z(msg);
    sensors_tm->bmx160_temp = mavlink_msg_sensors_tm_get_bmx160_temp(msg);
    sensors_tm->ms5803_press = mavlink_msg_sensors_tm_get_ms5803_press(msg);
    sensors_tm->ms5803_temp = mavlink_msg_sensors_tm_get_ms5803_temp(msg);
    sensors_tm->dpl_press = mavlink_msg_sensors_tm_get_dpl_press(msg);
    sensors_tm->pitot_press = mavlink_msg_sensors_tm_get_pitot_press(msg);
    sensors_tm->static_press = mavlink_msg_sensors_tm_get_static_press(msg);
    sensors_tm->lis3mdl_mag_x = mavlink_msg_sensors_tm_get_lis3mdl_mag_x(msg);
    sensors_tm->lis3mdl_mag_y = mavlink_msg_sensors_tm_get_lis3mdl_mag_y(msg);
    sensors_tm->lis3mdl_mag_z = mavlink_msg_sensors_tm_get_lis3mdl_mag_z(msg);
    sensors_tm->lis3mdl_temp = mavlink_msg_sensors_tm_get_lis3mdl_temp(msg);
    sensors_tm->gps_lat = mavlink_msg_sensors_tm_get_gps_lat(msg);
    sensors_tm->gps_lon = mavlink_msg_sensors_tm_get_gps_lon(msg);
    sensors_tm->gps_alt = mavlink_msg_sensors_tm_get_gps_alt(msg);
    sensors_tm->vbat = mavlink_msg_sensors_tm_get_vbat(msg);
    sensors_tm->vsupply_5v = mavlink_msg_sensors_tm_get_vsupply_5v(msg);
    sensors_tm->gps_fix = mavlink_msg_sensors_tm_get_gps_fix(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENSORS_TM_LEN? msg->len : MAVLINK_MSG_ID_SENSORS_TM_LEN;
        memset(sensors_tm, 0, MAVLINK_MSG_ID_SENSORS_TM_LEN);
    memcpy(sensors_tm, _MAV_PAYLOAD(msg), len);
#endif
}
