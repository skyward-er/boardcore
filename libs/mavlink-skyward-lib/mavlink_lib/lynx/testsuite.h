/** @file
 *    @brief MAVLink comm protocol testsuite generated from lynx.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef LYNX_TESTSUITE_H
#define LYNX_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_lynx(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_lynx(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_ping_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PING_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ping_tc_t packet_in = {
        93372036854775807ULL
    };
    mavlink_ping_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PING_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PING_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ping_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ping_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ping_tc_pack(system_id, component_id, &msg , packet1.timestamp );
    mavlink_msg_ping_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ping_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp );
    mavlink_msg_ping_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ping_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ping_tc_send(MAVLINK_COMM_1 , packet1.timestamp );
    mavlink_msg_ping_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_noarg_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_NOARG_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_noarg_tc_t packet_in = {
        5
    };
    mavlink_noarg_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.command_id = packet_in.command_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_NOARG_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_NOARG_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_noarg_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_noarg_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_noarg_tc_pack(system_id, component_id, &msg , packet1.command_id );
    mavlink_msg_noarg_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_noarg_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command_id );
    mavlink_msg_noarg_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_noarg_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_noarg_tc_send(MAVLINK_COMM_1 , packet1.command_id );
    mavlink_msg_noarg_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_start_launch_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_START_LAUNCH_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_start_launch_tc_t packet_in = {
        93372036854775807ULL
    };
    mavlink_start_launch_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.launch_code = packet_in.launch_code;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_START_LAUNCH_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_START_LAUNCH_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_start_launch_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_start_launch_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_start_launch_tc_pack(system_id, component_id, &msg , packet1.launch_code );
    mavlink_msg_start_launch_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_start_launch_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.launch_code );
    mavlink_msg_start_launch_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_start_launch_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_start_launch_tc_send(MAVLINK_COMM_1 , packet1.launch_code );
    mavlink_msg_start_launch_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_upload_setting_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UPLOAD_SETTING_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_upload_setting_tc_t packet_in = {
        17.0,17
    };
    mavlink_upload_setting_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.setting = packet_in.setting;
        packet1.setting_id = packet_in.setting_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UPLOAD_SETTING_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UPLOAD_SETTING_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_upload_setting_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_upload_setting_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_upload_setting_tc_pack(system_id, component_id, &msg , packet1.setting_id , packet1.setting );
    mavlink_msg_upload_setting_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_upload_setting_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.setting_id , packet1.setting );
    mavlink_msg_upload_setting_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_upload_setting_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_upload_setting_tc_send(MAVLINK_COMM_1 , packet1.setting_id , packet1.setting );
    mavlink_msg_upload_setting_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_aerobrake_angle_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_aerobrake_angle_tc_t packet_in = {
        17.0
    };
    mavlink_set_aerobrake_angle_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.angle = packet_in.angle;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_aerobrake_angle_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_aerobrake_angle_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_aerobrake_angle_tc_pack(system_id, component_id, &msg , packet1.angle );
    mavlink_msg_set_aerobrake_angle_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_aerobrake_angle_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.angle );
    mavlink_msg_set_aerobrake_angle_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_aerobrake_angle_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_aerobrake_angle_tc_send(MAVLINK_COMM_1 , packet1.angle );
    mavlink_msg_set_aerobrake_angle_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_reference_altitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_reference_altitude_t packet_in = {
        17.0
    };
    mavlink_set_reference_altitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ref_altitude = packet_in.ref_altitude;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_altitude_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_reference_altitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_altitude_pack(system_id, component_id, &msg , packet1.ref_altitude );
    mavlink_msg_set_reference_altitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_altitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ref_altitude );
    mavlink_msg_set_reference_altitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_reference_altitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_altitude_send(MAVLINK_COMM_1 , packet1.ref_altitude );
    mavlink_msg_set_reference_altitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_reference_temperature_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_reference_temperature_tc_t packet_in = {
        17.0
    };
    mavlink_set_reference_temperature_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ref_temp = packet_in.ref_temp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_temperature_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_reference_temperature_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_temperature_tc_pack(system_id, component_id, &msg , packet1.ref_temp );
    mavlink_msg_set_reference_temperature_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_temperature_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ref_temp );
    mavlink_msg_set_reference_temperature_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_reference_temperature_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_temperature_tc_send(MAVLINK_COMM_1 , packet1.ref_temp );
    mavlink_msg_set_reference_temperature_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_deployment_altitude_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_deployment_altitude_tc_t packet_in = {
        17.0
    };
    mavlink_set_deployment_altitude_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.dpl_altitude = packet_in.dpl_altitude;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_deployment_altitude_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_deployment_altitude_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_deployment_altitude_tc_pack(system_id, component_id, &msg , packet1.dpl_altitude );
    mavlink_msg_set_deployment_altitude_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_deployment_altitude_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.dpl_altitude );
    mavlink_msg_set_deployment_altitude_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_deployment_altitude_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_deployment_altitude_tc_send(MAVLINK_COMM_1 , packet1.dpl_altitude );
    mavlink_msg_set_deployment_altitude_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_initial_orientation_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_INITIAL_ORIENTATION_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_initial_orientation_tc_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_set_initial_orientation_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.yaw = packet_in.yaw;
        packet1.pitch = packet_in.pitch;
        packet1.roll = packet_in.roll;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_INITIAL_ORIENTATION_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_INITIAL_ORIENTATION_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_initial_orientation_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_initial_orientation_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_initial_orientation_tc_pack(system_id, component_id, &msg , packet1.yaw , packet1.pitch , packet1.roll );
    mavlink_msg_set_initial_orientation_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_initial_orientation_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.yaw , packet1.pitch , packet1.roll );
    mavlink_msg_set_initial_orientation_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_initial_orientation_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_initial_orientation_tc_send(MAVLINK_COMM_1 , packet1.yaw , packet1.pitch , packet1.roll );
    mavlink_msg_set_initial_orientation_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_initial_coordinates_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_INITIAL_COORDINATES_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_initial_coordinates_tc_t packet_in = {
        17.0,45.0
    };
    mavlink_set_initial_coordinates_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_INITIAL_COORDINATES_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_INITIAL_COORDINATES_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_initial_coordinates_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_initial_coordinates_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_initial_coordinates_tc_pack(system_id, component_id, &msg , packet1.latitude , packet1.longitude );
    mavlink_msg_set_initial_coordinates_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_initial_coordinates_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.latitude , packet1.longitude );
    mavlink_msg_set_initial_coordinates_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_initial_coordinates_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_initial_coordinates_tc_send(MAVLINK_COMM_1 , packet1.latitude , packet1.longitude );
    mavlink_msg_set_initial_coordinates_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_telemetry_request_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TELEMETRY_REQUEST_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_telemetry_request_tc_t packet_in = {
        5
    };
    mavlink_telemetry_request_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.board_id = packet_in.board_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TELEMETRY_REQUEST_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TELEMETRY_REQUEST_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_telemetry_request_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_telemetry_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_telemetry_request_tc_pack(system_id, component_id, &msg , packet1.board_id );
    mavlink_msg_telemetry_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_telemetry_request_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.board_id );
    mavlink_msg_telemetry_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_telemetry_request_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_telemetry_request_tc_send(MAVLINK_COMM_1 , packet1.board_id );
    mavlink_msg_telemetry_request_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_raw_event_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RAW_EVENT_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_raw_event_tc_t packet_in = {
        5,72
    };
    mavlink_raw_event_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.Event_id = packet_in.Event_id;
        packet1.Topic_id = packet_in.Topic_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RAW_EVENT_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RAW_EVENT_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_event_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_raw_event_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_event_tc_pack(system_id, component_id, &msg , packet1.Event_id , packet1.Topic_id );
    mavlink_msg_raw_event_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_event_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Event_id , packet1.Topic_id );
    mavlink_msg_raw_event_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_raw_event_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_event_tc_send(MAVLINK_COMM_1 , packet1.Event_id , packet1.Topic_id );
    mavlink_msg_raw_event_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ack_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACK_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ack_tm_t packet_in = {
        5,72
    };
    mavlink_ack_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.recv_msgid = packet_in.recv_msgid;
        packet1.seq_ack = packet_in.seq_ack;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACK_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACK_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ack_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ack_tm_pack(system_id, component_id, &msg , packet1.recv_msgid , packet1.seq_ack );
    mavlink_msg_ack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ack_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.recv_msgid , packet1.seq_ack );
    mavlink_msg_ack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ack_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ack_tm_send(MAVLINK_COMM_1 , packet1.recv_msgid , packet1.seq_ack );
    mavlink_msg_ack_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_nack_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_NACK_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_nack_tm_t packet_in = {
        5,72
    };
    mavlink_nack_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.recv_msgid = packet_in.recv_msgid;
        packet1.seq_ack = packet_in.seq_ack;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_NACK_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_NACK_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nack_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_nack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nack_tm_pack(system_id, component_id, &msg , packet1.recv_msgid , packet1.seq_ack );
    mavlink_msg_nack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nack_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.recv_msgid , packet1.seq_ack );
    mavlink_msg_nack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_nack_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nack_tm_send(MAVLINK_COMM_1 , packet1.recv_msgid , packet1.seq_ack );
    mavlink_msg_nack_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sys_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SYS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sys_tm_t packet_in = {
        93372036854775807ULL,29,96,163,230,41,108,175,242,53,120,187,254,65
    };
    mavlink_sys_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.death_stack = packet_in.death_stack;
        packet1.logger = packet_in.logger;
        packet1.ev_broker = packet_in.ev_broker;
        packet1.pin_obs = packet_in.pin_obs;
        packet1.radio = packet_in.radio;
        packet1.state_machines = packet_in.state_machines;
        packet1.sensors = packet_in.sensors;
        packet1.bmx160_status = packet_in.bmx160_status;
        packet1.ms5803_status = packet_in.ms5803_status;
        packet1.lis3mdl_status = packet_in.lis3mdl_status;
        packet1.gps_status = packet_in.gps_status;
        packet1.internal_adc_status = packet_in.internal_adc_status;
        packet1.ads1118_status = packet_in.ads1118_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SYS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SYS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sys_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.death_stack , packet1.logger , packet1.ev_broker , packet1.pin_obs , packet1.radio , packet1.state_machines , packet1.sensors , packet1.bmx160_status , packet1.ms5803_status , packet1.lis3mdl_status , packet1.gps_status , packet1.internal_adc_status , packet1.ads1118_status );
    mavlink_msg_sys_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.death_stack , packet1.logger , packet1.ev_broker , packet1.pin_obs , packet1.radio , packet1.state_machines , packet1.sensors , packet1.bmx160_status , packet1.ms5803_status , packet1.lis3mdl_status , packet1.gps_status , packet1.internal_adc_status , packet1.ads1118_status );
    mavlink_msg_sys_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sys_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.death_stack , packet1.logger , packet1.ev_broker , packet1.pin_obs , packet1.radio , packet1.state_machines , packet1.sensors , packet1.bmx160_status , packet1.ms5803_status , packet1.lis3mdl_status , packet1.gps_status , packet1.internal_adc_status , packet1.ads1118_status );
    mavlink_msg_sys_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_fmm_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FMM_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_fmm_tm_t packet_in = {
        93372036854775807ULL,29
    };
    mavlink_fmm_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.state = packet_in.state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FMM_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FMM_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fmm_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_fmm_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fmm_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.state );
    mavlink_msg_fmm_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fmm_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.state );
    mavlink_msg_fmm_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_fmm_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fmm_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.state );
    mavlink_msg_fmm_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pin_obs_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PIN_OBS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_pin_obs_tm_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,93372036854777319ULL,101,168,235,46,113,180
    };
    mavlink_pin_obs_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pin_launch_last_change = packet_in.pin_launch_last_change;
        packet1.pin_nosecone_last_change = packet_in.pin_nosecone_last_change;
        packet1.pin_dpl_servo_last_change = packet_in.pin_dpl_servo_last_change;
        packet1.pin_launch_num_changes = packet_in.pin_launch_num_changes;
        packet1.pin_launch_state = packet_in.pin_launch_state;
        packet1.pin_nosecone_num_changes = packet_in.pin_nosecone_num_changes;
        packet1.pin_nosecone_state = packet_in.pin_nosecone_state;
        packet1.pin_dpl_servo_num_changes = packet_in.pin_dpl_servo_num_changes;
        packet1.pin_dpl_servo_state = packet_in.pin_dpl_servo_state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PIN_OBS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PIN_OBS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pin_obs_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_pin_obs_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pin_obs_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.pin_launch_last_change , packet1.pin_launch_num_changes , packet1.pin_launch_state , packet1.pin_nosecone_last_change , packet1.pin_nosecone_num_changes , packet1.pin_nosecone_state , packet1.pin_dpl_servo_last_change , packet1.pin_dpl_servo_num_changes , packet1.pin_dpl_servo_state );
    mavlink_msg_pin_obs_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pin_obs_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.pin_launch_last_change , packet1.pin_launch_num_changes , packet1.pin_launch_state , packet1.pin_nosecone_last_change , packet1.pin_nosecone_num_changes , packet1.pin_nosecone_state , packet1.pin_dpl_servo_last_change , packet1.pin_dpl_servo_num_changes , packet1.pin_dpl_servo_state );
    mavlink_msg_pin_obs_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_pin_obs_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pin_obs_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.pin_launch_last_change , packet1.pin_launch_num_changes , packet1.pin_launch_state , packet1.pin_nosecone_last_change , packet1.pin_nosecone_num_changes , packet1.pin_nosecone_state , packet1.pin_dpl_servo_last_change , packet1.pin_dpl_servo_num_changes , packet1.pin_dpl_servo_state );
    mavlink_msg_pin_obs_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_logger_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOGGER_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_logger_tm_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,963498504,963498712,963498920,963499128,963499336,963499544,19523
    };
    mavlink_logger_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.statTooLargeSamples = packet_in.statTooLargeSamples;
        packet1.statDroppedSamples = packet_in.statDroppedSamples;
        packet1.statQueuedSamples = packet_in.statQueuedSamples;
        packet1.statBufferFilled = packet_in.statBufferFilled;
        packet1.statBufferWritten = packet_in.statBufferWritten;
        packet1.statWriteFailed = packet_in.statWriteFailed;
        packet1.statWriteError = packet_in.statWriteError;
        packet1.statWriteTime = packet_in.statWriteTime;
        packet1.statMaxWriteTime = packet_in.statMaxWriteTime;
        packet1.statLogNumber = packet_in.statLogNumber;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logger_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_logger_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logger_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.statLogNumber , packet1.statTooLargeSamples , packet1.statDroppedSamples , packet1.statQueuedSamples , packet1.statBufferFilled , packet1.statBufferWritten , packet1.statWriteFailed , packet1.statWriteError , packet1.statWriteTime , packet1.statMaxWriteTime );
    mavlink_msg_logger_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logger_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.statLogNumber , packet1.statTooLargeSamples , packet1.statDroppedSamples , packet1.statQueuedSamples , packet1.statBufferFilled , packet1.statBufferWritten , packet1.statWriteFailed , packet1.statWriteError , packet1.statWriteTime , packet1.statMaxWriteTime );
    mavlink_msg_logger_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_logger_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logger_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.statLogNumber , packet1.statTooLargeSamples , packet1.statDroppedSamples , packet1.statQueuedSamples , packet1.statBufferFilled , packet1.statBufferWritten , packet1.statWriteFailed , packet1.statWriteError , packet1.statWriteTime , packet1.statMaxWriteTime );
    mavlink_msg_logger_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_tmtc_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TMTC_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tmtc_tm_t packet_in = {
        93372036854775807ULL,963497880,17859,17963,18067,18171,18275,199,10,77,144,211,22
    };
    mavlink_tmtc_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.parse_state = packet_in.parse_state;
        packet1.n_send_queue = packet_in.n_send_queue;
        packet1.max_send_queue = packet_in.max_send_queue;
        packet1.n_send_errors = packet_in.n_send_errors;
        packet1.packet_rx_success_count = packet_in.packet_rx_success_count;
        packet1.packet_rx_drop_count = packet_in.packet_rx_drop_count;
        packet1.msg_received = packet_in.msg_received;
        packet1.buffer_overrun = packet_in.buffer_overrun;
        packet1.parse_error = packet_in.parse_error;
        packet1.packet_idx = packet_in.packet_idx;
        packet1.current_rx_seq = packet_in.current_rx_seq;
        packet1.current_tx_seq = packet_in.current_tx_seq;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TMTC_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TMTC_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tmtc_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tmtc_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tmtc_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.n_send_queue , packet1.max_send_queue , packet1.n_send_errors , packet1.msg_received , packet1.buffer_overrun , packet1.parse_error , packet1.parse_state , packet1.packet_idx , packet1.current_rx_seq , packet1.current_tx_seq , packet1.packet_rx_success_count , packet1.packet_rx_drop_count );
    mavlink_msg_tmtc_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tmtc_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.n_send_queue , packet1.max_send_queue , packet1.n_send_errors , packet1.msg_received , packet1.buffer_overrun , packet1.parse_error , packet1.parse_state , packet1.packet_idx , packet1.current_rx_seq , packet1.current_tx_seq , packet1.packet_rx_success_count , packet1.packet_rx_drop_count );
    mavlink_msg_tmtc_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tmtc_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tmtc_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.n_send_queue , packet1.max_send_queue , packet1.n_send_errors , packet1.msg_received , packet1.buffer_overrun , packet1.parse_error , packet1.parse_state , packet1.packet_idx , packet1.current_rx_seq , packet1.current_tx_seq , packet1.packet_rx_success_count , packet1.packet_rx_drop_count );
    mavlink_msg_tmtc_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_task_stats_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TASK_STATS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_task_stats_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,689.0,717.0,745.0,773.0,801.0,829.0,857.0,885.0,913.0,941.0,969.0,997.0,1025.0,1053.0
    };
    mavlink_task_stats_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.task_sensors_6ms_min = packet_in.task_sensors_6ms_min;
        packet1.task_sensors_6ms_max = packet_in.task_sensors_6ms_max;
        packet1.task_sensors_6ms_mean = packet_in.task_sensors_6ms_mean;
        packet1.task_sensors_6ms_stddev = packet_in.task_sensors_6ms_stddev;
        packet1.task_sensors_15ms_min = packet_in.task_sensors_15ms_min;
        packet1.task_sensors_15ms_max = packet_in.task_sensors_15ms_max;
        packet1.task_sensors_15ms_mean = packet_in.task_sensors_15ms_mean;
        packet1.task_sensors_15ms_stddev = packet_in.task_sensors_15ms_stddev;
        packet1.task_sensors_20ms_min = packet_in.task_sensors_20ms_min;
        packet1.task_sensors_20ms_max = packet_in.task_sensors_20ms_max;
        packet1.task_sensors_20ms_mean = packet_in.task_sensors_20ms_mean;
        packet1.task_sensors_20ms_stddev = packet_in.task_sensors_20ms_stddev;
        packet1.task_sensors_24ms_min = packet_in.task_sensors_24ms_min;
        packet1.task_sensors_24ms_max = packet_in.task_sensors_24ms_max;
        packet1.task_sensors_24ms_mean = packet_in.task_sensors_24ms_mean;
        packet1.task_sensors_24ms_stddev = packet_in.task_sensors_24ms_stddev;
        packet1.task_sensors_40ms_min = packet_in.task_sensors_40ms_min;
        packet1.task_sensors_40ms_max = packet_in.task_sensors_40ms_max;
        packet1.task_sensors_40ms_mean = packet_in.task_sensors_40ms_mean;
        packet1.task_sensors_40ms_stddev = packet_in.task_sensors_40ms_stddev;
        packet1.task_sensors_1000ms_min = packet_in.task_sensors_1000ms_min;
        packet1.task_sensors_1000ms_max = packet_in.task_sensors_1000ms_max;
        packet1.task_sensors_1000ms_mean = packet_in.task_sensors_1000ms_mean;
        packet1.task_sensors_1000ms_stddev = packet_in.task_sensors_1000ms_stddev;
        packet1.task_ada_min = packet_in.task_ada_min;
        packet1.task_ada_max = packet_in.task_ada_max;
        packet1.task_ada_mean = packet_in.task_ada_mean;
        packet1.task_ada_stddev = packet_in.task_ada_stddev;
        packet1.task_nas_min = packet_in.task_nas_min;
        packet1.task_nas_max = packet_in.task_nas_max;
        packet1.task_nas_mean = packet_in.task_nas_mean;
        packet1.task_nas_stddev = packet_in.task_nas_stddev;
        packet1.task_abk_min = packet_in.task_abk_min;
        packet1.task_abk_max = packet_in.task_abk_max;
        packet1.task_abk_mean = packet_in.task_abk_mean;
        packet1.task_abk_stddev = packet_in.task_abk_stddev;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_task_stats_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_task_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_task_stats_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.task_sensors_6ms_min , packet1.task_sensors_6ms_max , packet1.task_sensors_6ms_mean , packet1.task_sensors_6ms_stddev , packet1.task_sensors_15ms_min , packet1.task_sensors_15ms_max , packet1.task_sensors_15ms_mean , packet1.task_sensors_15ms_stddev , packet1.task_sensors_20ms_min , packet1.task_sensors_20ms_max , packet1.task_sensors_20ms_mean , packet1.task_sensors_20ms_stddev , packet1.task_sensors_24ms_min , packet1.task_sensors_24ms_max , packet1.task_sensors_24ms_mean , packet1.task_sensors_24ms_stddev , packet1.task_sensors_40ms_min , packet1.task_sensors_40ms_max , packet1.task_sensors_40ms_mean , packet1.task_sensors_40ms_stddev , packet1.task_sensors_1000ms_min , packet1.task_sensors_1000ms_max , packet1.task_sensors_1000ms_mean , packet1.task_sensors_1000ms_stddev , packet1.task_ada_min , packet1.task_ada_max , packet1.task_ada_mean , packet1.task_ada_stddev , packet1.task_nas_min , packet1.task_nas_max , packet1.task_nas_mean , packet1.task_nas_stddev , packet1.task_abk_min , packet1.task_abk_max , packet1.task_abk_mean , packet1.task_abk_stddev );
    mavlink_msg_task_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_task_stats_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.task_sensors_6ms_min , packet1.task_sensors_6ms_max , packet1.task_sensors_6ms_mean , packet1.task_sensors_6ms_stddev , packet1.task_sensors_15ms_min , packet1.task_sensors_15ms_max , packet1.task_sensors_15ms_mean , packet1.task_sensors_15ms_stddev , packet1.task_sensors_20ms_min , packet1.task_sensors_20ms_max , packet1.task_sensors_20ms_mean , packet1.task_sensors_20ms_stddev , packet1.task_sensors_24ms_min , packet1.task_sensors_24ms_max , packet1.task_sensors_24ms_mean , packet1.task_sensors_24ms_stddev , packet1.task_sensors_40ms_min , packet1.task_sensors_40ms_max , packet1.task_sensors_40ms_mean , packet1.task_sensors_40ms_stddev , packet1.task_sensors_1000ms_min , packet1.task_sensors_1000ms_max , packet1.task_sensors_1000ms_mean , packet1.task_sensors_1000ms_stddev , packet1.task_ada_min , packet1.task_ada_max , packet1.task_ada_mean , packet1.task_ada_stddev , packet1.task_nas_min , packet1.task_nas_max , packet1.task_nas_mean , packet1.task_nas_stddev , packet1.task_abk_min , packet1.task_abk_max , packet1.task_abk_mean , packet1.task_abk_stddev );
    mavlink_msg_task_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_task_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_task_stats_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.task_sensors_6ms_min , packet1.task_sensors_6ms_max , packet1.task_sensors_6ms_mean , packet1.task_sensors_6ms_stddev , packet1.task_sensors_15ms_min , packet1.task_sensors_15ms_max , packet1.task_sensors_15ms_mean , packet1.task_sensors_15ms_stddev , packet1.task_sensors_20ms_min , packet1.task_sensors_20ms_max , packet1.task_sensors_20ms_mean , packet1.task_sensors_20ms_stddev , packet1.task_sensors_24ms_min , packet1.task_sensors_24ms_max , packet1.task_sensors_24ms_mean , packet1.task_sensors_24ms_stddev , packet1.task_sensors_40ms_min , packet1.task_sensors_40ms_max , packet1.task_sensors_40ms_mean , packet1.task_sensors_40ms_stddev , packet1.task_sensors_1000ms_min , packet1.task_sensors_1000ms_max , packet1.task_sensors_1000ms_mean , packet1.task_sensors_1000ms_stddev , packet1.task_ada_min , packet1.task_ada_max , packet1.task_ada_mean , packet1.task_ada_stddev , packet1.task_nas_min , packet1.task_nas_max , packet1.task_nas_mean , packet1.task_nas_stddev , packet1.task_abk_min , packet1.task_abk_max , packet1.task_abk_mean , packet1.task_abk_stddev );
    mavlink_msg_task_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_dpl_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DPL_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_dpl_tm_t packet_in = {
        93372036854775807ULL,73.0,41,108
    };
    mavlink_dpl_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.servo_position = packet_in.servo_position;
        packet1.fsm_state = packet_in.fsm_state;
        packet1.cutters_enabled = packet_in.cutters_enabled;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DPL_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DPL_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dpl_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_dpl_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dpl_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.fsm_state , packet1.cutters_enabled , packet1.servo_position );
    mavlink_msg_dpl_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dpl_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.fsm_state , packet1.cutters_enabled , packet1.servo_position );
    mavlink_msg_dpl_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_dpl_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dpl_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.fsm_state , packet1.cutters_enabled , packet1.servo_position );
    mavlink_msg_dpl_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ada_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADA_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ada_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,161,228,39,106
    };
    mavlink_ada_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.target_dpl_altitude = packet_in.target_dpl_altitude;
        packet1.kalman_x0 = packet_in.kalman_x0;
        packet1.kalman_x1 = packet_in.kalman_x1;
        packet1.kalman_x2 = packet_in.kalman_x2;
        packet1.vert_speed = packet_in.vert_speed;
        packet1.msl_altitude = packet_in.msl_altitude;
        packet1.ref_pressure = packet_in.ref_pressure;
        packet1.ref_altitude = packet_in.ref_altitude;
        packet1.ref_temperature = packet_in.ref_temperature;
        packet1.msl_pressure = packet_in.msl_pressure;
        packet1.msl_temperature = packet_in.msl_temperature;
        packet1.state = packet_in.state;
        packet1.apogee_reached = packet_in.apogee_reached;
        packet1.aerobrakes_disabled = packet_in.aerobrakes_disabled;
        packet1.dpl_altitude_reached = packet_in.dpl_altitude_reached;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADA_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADA_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ada_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ada_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ada_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.state , packet1.apogee_reached , packet1.aerobrakes_disabled , packet1.dpl_altitude_reached , packet1.target_dpl_altitude , packet1.kalman_x0 , packet1.kalman_x1 , packet1.kalman_x2 , packet1.vert_speed , packet1.msl_altitude , packet1.ref_pressure , packet1.ref_altitude , packet1.ref_temperature , packet1.msl_pressure , packet1.msl_temperature );
    mavlink_msg_ada_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ada_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.state , packet1.apogee_reached , packet1.aerobrakes_disabled , packet1.dpl_altitude_reached , packet1.target_dpl_altitude , packet1.kalman_x0 , packet1.kalman_x1 , packet1.kalman_x2 , packet1.vert_speed , packet1.msl_altitude , packet1.ref_pressure , packet1.ref_altitude , packet1.ref_temperature , packet1.msl_pressure , packet1.msl_temperature );
    mavlink_msg_ada_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ada_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ada_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.state , packet1.apogee_reached , packet1.aerobrakes_disabled , packet1.dpl_altitude_reached , packet1.target_dpl_altitude , packet1.kalman_x0 , packet1.kalman_x1 , packet1.kalman_x2 , packet1.vert_speed , packet1.msl_altitude , packet1.ref_pressure , packet1.ref_altitude , packet1.ref_temperature , packet1.msl_pressure , packet1.msl_temperature );
    mavlink_msg_ada_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_abk_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ABK_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_abk_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,101,168
    };
    mavlink_abk_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.servo_position = packet_in.servo_position;
        packet1.estimated_cd = packet_in.estimated_cd;
        packet1.pid_error = packet_in.pid_error;
        packet1.z = packet_in.z;
        packet1.vz = packet_in.vz;
        packet1.v_mod = packet_in.v_mod;
        packet1.state = packet_in.state;
        packet1.trajectory = packet_in.trajectory;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ABK_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ABK_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_abk_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_abk_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_abk_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.state , packet1.servo_position , packet1.estimated_cd , packet1.pid_error , packet1.z , packet1.vz , packet1.v_mod , packet1.trajectory );
    mavlink_msg_abk_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_abk_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.state , packet1.servo_position , packet1.estimated_cd , packet1.pid_error , packet1.z , packet1.vz , packet1.v_mod , packet1.trajectory );
    mavlink_msg_abk_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_abk_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_abk_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.state , packet1.servo_position , packet1.estimated_cd , packet1.pid_error , packet1.z , packet1.vz , packet1.v_mod , packet1.trajectory );
    mavlink_msg_abk_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_nas_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_NAS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_nas_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,689.0,717.0,745.0,773.0,801.0,829.0,857.0,121
    };
    mavlink_nas_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.x0 = packet_in.x0;
        packet1.x1 = packet_in.x1;
        packet1.x2 = packet_in.x2;
        packet1.x3 = packet_in.x3;
        packet1.x4 = packet_in.x4;
        packet1.x5 = packet_in.x5;
        packet1.x6 = packet_in.x6;
        packet1.x7 = packet_in.x7;
        packet1.x8 = packet_in.x8;
        packet1.x9 = packet_in.x9;
        packet1.x10 = packet_in.x10;
        packet1.x11 = packet_in.x11;
        packet1.x12 = packet_in.x12;
        packet1.ref_pressure = packet_in.ref_pressure;
        packet1.ref_temperature = packet_in.ref_temperature;
        packet1.ref_latitude = packet_in.ref_latitude;
        packet1.ref_longitude = packet_in.ref_longitude;
        packet1.ref_accel_x = packet_in.ref_accel_x;
        packet1.ref_accel_y = packet_in.ref_accel_y;
        packet1.ref_accel_z = packet_in.ref_accel_z;
        packet1.ref_mag_x = packet_in.ref_mag_x;
        packet1.ref_mag_y = packet_in.ref_mag_y;
        packet1.ref_mag_z = packet_in.ref_mag_z;
        packet1.triad_roll = packet_in.triad_roll;
        packet1.triad_pitch = packet_in.triad_pitch;
        packet1.triad_yaw = packet_in.triad_yaw;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.state = packet_in.state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_NAS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_NAS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nas_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_nas_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nas_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.state , packet1.x0 , packet1.x1 , packet1.x2 , packet1.x3 , packet1.x4 , packet1.x5 , packet1.x6 , packet1.x7 , packet1.x8 , packet1.x9 , packet1.x10 , packet1.x11 , packet1.x12 , packet1.ref_pressure , packet1.ref_temperature , packet1.ref_latitude , packet1.ref_longitude , packet1.ref_accel_x , packet1.ref_accel_y , packet1.ref_accel_z , packet1.ref_mag_x , packet1.ref_mag_y , packet1.ref_mag_z , packet1.triad_roll , packet1.triad_pitch , packet1.triad_yaw , packet1.roll , packet1.pitch , packet1.yaw );
    mavlink_msg_nas_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nas_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.state , packet1.x0 , packet1.x1 , packet1.x2 , packet1.x3 , packet1.x4 , packet1.x5 , packet1.x6 , packet1.x7 , packet1.x8 , packet1.x9 , packet1.x10 , packet1.x11 , packet1.x12 , packet1.ref_pressure , packet1.ref_temperature , packet1.ref_latitude , packet1.ref_longitude , packet1.ref_accel_x , packet1.ref_accel_y , packet1.ref_accel_z , packet1.ref_mag_x , packet1.ref_mag_y , packet1.ref_mag_z , packet1.triad_roll , packet1.triad_pitch , packet1.triad_yaw , packet1.roll , packet1.pitch , packet1.yaw );
    mavlink_msg_nas_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_nas_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nas_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.state , packet1.x0 , packet1.x1 , packet1.x2 , packet1.x3 , packet1.x4 , packet1.x5 , packet1.x6 , packet1.x7 , packet1.x8 , packet1.x9 , packet1.x10 , packet1.x11 , packet1.x12 , packet1.ref_pressure , packet1.ref_temperature , packet1.ref_latitude , packet1.ref_longitude , packet1.ref_accel_x , packet1.ref_accel_y , packet1.ref_accel_z , packet1.ref_mag_x , packet1.ref_mag_y , packet1.ref_mag_z , packet1.triad_roll , packet1.triad_pitch , packet1.triad_yaw , packet1.roll , packet1.pitch , packet1.yaw );
    mavlink_msg_nas_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_adc_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADC_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_adc_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0
    };
    mavlink_adc_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pitot_pressure = packet_in.pitot_pressure;
        packet1.dpl_pressure = packet_in.dpl_pressure;
        packet1.static_pressure = packet_in.static_pressure;
        packet1.bat_voltage = packet_in.bat_voltage;
        packet1.bat_voltage_5v = packet_in.bat_voltage_5v;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADC_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADC_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_adc_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.pitot_pressure , packet1.dpl_pressure , packet1.static_pressure , packet1.bat_voltage , packet1.bat_voltage_5v );
    mavlink_msg_adc_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.pitot_pressure , packet1.dpl_pressure , packet1.static_pressure , packet1.bat_voltage , packet1.bat_voltage_5v );
    mavlink_msg_adc_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_adc_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.pitot_pressure , packet1.dpl_pressure , packet1.static_pressure , packet1.bat_voltage , packet1.bat_voltage_5v );
    mavlink_msg_adc_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ms5803_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MS5803_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ms5803_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0
    };
    mavlink_ms5803_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pressure = packet_in.pressure;
        packet1.temperature = packet_in.temperature;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MS5803_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MS5803_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ms5803_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ms5803_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ms5803_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.pressure , packet1.temperature );
    mavlink_msg_ms5803_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ms5803_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.pressure , packet1.temperature );
    mavlink_msg_ms5803_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ms5803_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ms5803_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.pressure , packet1.temperature );
    mavlink_msg_ms5803_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_bmx160_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BMX160_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_bmx160_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0
    };
    mavlink_bmx160_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.acc_x = packet_in.acc_x;
        packet1.acc_y = packet_in.acc_y;
        packet1.acc_z = packet_in.acc_z;
        packet1.gyro_x = packet_in.gyro_x;
        packet1.gyro_y = packet_in.gyro_y;
        packet1.gyro_z = packet_in.gyro_z;
        packet1.mag_x = packet_in.mag_x;
        packet1.mag_y = packet_in.mag_y;
        packet1.mag_z = packet_in.mag_z;
        packet1.temp = packet_in.temp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BMX160_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BMX160_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_bmx160_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_bmx160_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_bmx160_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.temp );
    mavlink_msg_bmx160_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_bmx160_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.temp );
    mavlink_msg_bmx160_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_bmx160_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_bmx160_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.temp );
    mavlink_msg_bmx160_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_lis3mdl_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LIS3MDL_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lis3mdl_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0
    };
    mavlink_lis3mdl_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.mag_x = packet_in.mag_x;
        packet1.mag_y = packet_in.mag_y;
        packet1.mag_z = packet_in.mag_z;
        packet1.temp = packet_in.temp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LIS3MDL_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LIS3MDL_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lis3mdl_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lis3mdl_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lis3mdl_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.temp );
    mavlink_msg_lis3mdl_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lis3mdl_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.temp );
    mavlink_msg_lis3mdl_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lis3mdl_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lis3mdl_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.temp );
    mavlink_msg_lis3mdl_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gps_tm_t packet_in = {
        93372036854775807ULL,179.0,235.0,291.0,241.0,269.0,297.0,325.0,353.0,161,228
    };
    mavlink_gps_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.height = packet_in.height;
        packet1.vel_north = packet_in.vel_north;
        packet1.vel_east = packet_in.vel_east;
        packet1.vel_down = packet_in.vel_down;
        packet1.speed = packet_in.speed;
        packet1.track = packet_in.track;
        packet1.fix = packet_in.fix;
        packet1.n_satellites = packet_in.n_satellites;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gps_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.fix , packet1.latitude , packet1.longitude , packet1.height , packet1.vel_north , packet1.vel_east , packet1.vel_down , packet1.speed , packet1.track , packet1.n_satellites );
    mavlink_msg_gps_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.fix , packet1.latitude , packet1.longitude , packet1.height , packet1.vel_north , packet1.vel_east , packet1.vel_down , packet1.speed , packet1.track , packet1.n_satellites );
    mavlink_msg_gps_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gps_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.fix , packet1.latitude , packet1.longitude , packet1.height , packet1.vel_north , packet1.vel_east , packet1.vel_down , packet1.speed , packet1.track , packet1.n_satellites );
    mavlink_msg_gps_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_hr_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HR_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hr_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,689.0,717.0,745.0,773.0,801.0,829.0,857.0,885.0,913.0,941.0,969.0,997.0,1025.0,1053.0,1081.0,217,28,95,162,229,40,107,174,241,52
    };
    mavlink_hr_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pressure_ada = packet_in.pressure_ada;
        packet1.pressure_digi = packet_in.pressure_digi;
        packet1.pressure_static = packet_in.pressure_static;
        packet1.pressure_dpl = packet_in.pressure_dpl;
        packet1.airspeed_pitot = packet_in.airspeed_pitot;
        packet1.msl_altitude = packet_in.msl_altitude;
        packet1.ada_vert_speed = packet_in.ada_vert_speed;
        packet1.ada_vert_accel = packet_in.ada_vert_accel;
        packet1.acc_x = packet_in.acc_x;
        packet1.acc_y = packet_in.acc_y;
        packet1.acc_z = packet_in.acc_z;
        packet1.gyro_x = packet_in.gyro_x;
        packet1.gyro_y = packet_in.gyro_y;
        packet1.gyro_z = packet_in.gyro_z;
        packet1.mag_x = packet_in.mag_x;
        packet1.mag_y = packet_in.mag_y;
        packet1.mag_z = packet_in.mag_z;
        packet1.gps_lat = packet_in.gps_lat;
        packet1.gps_lon = packet_in.gps_lon;
        packet1.gps_alt = packet_in.gps_alt;
        packet1.vbat = packet_in.vbat;
        packet1.vsupply_5v = packet_in.vsupply_5v;
        packet1.temperature = packet_in.temperature;
        packet1.ab_angle = packet_in.ab_angle;
        packet1.ab_estimated_cd = packet_in.ab_estimated_cd;
        packet1.nas_x = packet_in.nas_x;
        packet1.nas_y = packet_in.nas_y;
        packet1.nas_z = packet_in.nas_z;
        packet1.nas_vx = packet_in.nas_vx;
        packet1.nas_vy = packet_in.nas_vy;
        packet1.nas_vz = packet_in.nas_vz;
        packet1.nas_roll = packet_in.nas_roll;
        packet1.nas_pitch = packet_in.nas_pitch;
        packet1.nas_yaw = packet_in.nas_yaw;
        packet1.nas_bias0 = packet_in.nas_bias0;
        packet1.nas_bias1 = packet_in.nas_bias1;
        packet1.nas_bias2 = packet_in.nas_bias2;
        packet1.ada_state = packet_in.ada_state;
        packet1.fmm_state = packet_in.fmm_state;
        packet1.dpl_state = packet_in.dpl_state;
        packet1.ab_state = packet_in.ab_state;
        packet1.nas_state = packet_in.nas_state;
        packet1.gps_fix = packet_in.gps_fix;
        packet1.pin_launch = packet_in.pin_launch;
        packet1.pin_nosecone = packet_in.pin_nosecone;
        packet1.servo_sensor = packet_in.servo_sensor;
        packet1.logger_error = packet_in.logger_error;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HR_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HR_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hr_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hr_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hr_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.ada_state , packet1.fmm_state , packet1.dpl_state , packet1.ab_state , packet1.nas_state , packet1.pressure_ada , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dpl , packet1.airspeed_pitot , packet1.msl_altitude , packet1.ada_vert_speed , packet1.ada_vert_accel , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.vbat , packet1.vsupply_5v , packet1.temperature , packet1.pin_launch , packet1.pin_nosecone , packet1.servo_sensor , packet1.ab_angle , packet1.ab_estimated_cd , packet1.nas_x , packet1.nas_y , packet1.nas_z , packet1.nas_vx , packet1.nas_vy , packet1.nas_vz , packet1.nas_roll , packet1.nas_pitch , packet1.nas_yaw , packet1.nas_bias0 , packet1.nas_bias1 , packet1.nas_bias2 , packet1.logger_error );
    mavlink_msg_hr_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hr_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.ada_state , packet1.fmm_state , packet1.dpl_state , packet1.ab_state , packet1.nas_state , packet1.pressure_ada , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dpl , packet1.airspeed_pitot , packet1.msl_altitude , packet1.ada_vert_speed , packet1.ada_vert_accel , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.vbat , packet1.vsupply_5v , packet1.temperature , packet1.pin_launch , packet1.pin_nosecone , packet1.servo_sensor , packet1.ab_angle , packet1.ab_estimated_cd , packet1.nas_x , packet1.nas_y , packet1.nas_z , packet1.nas_vx , packet1.nas_vy , packet1.nas_vz , packet1.nas_roll , packet1.nas_pitch , packet1.nas_yaw , packet1.nas_bias0 , packet1.nas_bias1 , packet1.nas_bias2 , packet1.logger_error );
    mavlink_msg_hr_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hr_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hr_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.ada_state , packet1.fmm_state , packet1.dpl_state , packet1.ab_state , packet1.nas_state , packet1.pressure_ada , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dpl , packet1.airspeed_pitot , packet1.msl_altitude , packet1.ada_vert_speed , packet1.ada_vert_accel , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.vbat , packet1.vsupply_5v , packet1.temperature , packet1.pin_launch , packet1.pin_nosecone , packet1.servo_sensor , packet1.ab_angle , packet1.ab_estimated_cd , packet1.nas_x , packet1.nas_y , packet1.nas_z , packet1.nas_vx , packet1.nas_vy , packet1.nas_vz , packet1.nas_roll , packet1.nas_pitch , packet1.nas_yaw , packet1.nas_bias0 , packet1.nas_bias1 , packet1.nas_bias2 , packet1.logger_error );
    mavlink_msg_hr_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_lr_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LR_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lr_tm_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,93372036854777319ULL,93372036854777823ULL,93372036854778327ULL,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,689.0,717.0,745.0,773.0,801.0,963503496
    };
    mavlink_lr_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.liftoff_ts = packet_in.liftoff_ts;
        packet1.liftoff_max_acc_ts = packet_in.liftoff_max_acc_ts;
        packet1.max_z_speed_ts = packet_in.max_z_speed_ts;
        packet1.apogee_ts = packet_in.apogee_ts;
        packet1.drogue_dpl_ts = packet_in.drogue_dpl_ts;
        packet1.main_dpl_altitude_ts = packet_in.main_dpl_altitude_ts;
        packet1.liftoff_max_acc = packet_in.liftoff_max_acc;
        packet1.max_z_speed = packet_in.max_z_speed;
        packet1.max_airspeed_pitot = packet_in.max_airspeed_pitot;
        packet1.max_speed_altitude = packet_in.max_speed_altitude;
        packet1.apogee_lat = packet_in.apogee_lat;
        packet1.apogee_lon = packet_in.apogee_lon;
        packet1.static_min_pressure = packet_in.static_min_pressure;
        packet1.digital_min_pressure = packet_in.digital_min_pressure;
        packet1.ada_min_pressure = packet_in.ada_min_pressure;
        packet1.baro_max_altitutde = packet_in.baro_max_altitutde;
        packet1.gps_max_altitude = packet_in.gps_max_altitude;
        packet1.drogue_dpl_max_acc = packet_in.drogue_dpl_max_acc;
        packet1.dpl_vane_max_pressure = packet_in.dpl_vane_max_pressure;
        packet1.main_dpl_altitude = packet_in.main_dpl_altitude;
        packet1.main_dpl_zspeed = packet_in.main_dpl_zspeed;
        packet1.main_dpl_acc = packet_in.main_dpl_acc;
        packet1.cpu_load = packet_in.cpu_load;
        packet1.free_heap = packet_in.free_heap;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LR_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LR_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lr_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lr_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lr_tm_pack(system_id, component_id, &msg , packet1.liftoff_ts , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.max_z_speed_ts , packet1.max_z_speed , packet1.max_airspeed_pitot , packet1.max_speed_altitude , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.static_min_pressure , packet1.digital_min_pressure , packet1.ada_min_pressure , packet1.baro_max_altitutde , packet1.gps_max_altitude , packet1.drogue_dpl_ts , packet1.drogue_dpl_max_acc , packet1.dpl_vane_max_pressure , packet1.main_dpl_altitude_ts , packet1.main_dpl_altitude , packet1.main_dpl_zspeed , packet1.main_dpl_acc , packet1.cpu_load , packet1.free_heap );
    mavlink_msg_lr_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lr_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.liftoff_ts , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.max_z_speed_ts , packet1.max_z_speed , packet1.max_airspeed_pitot , packet1.max_speed_altitude , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.static_min_pressure , packet1.digital_min_pressure , packet1.ada_min_pressure , packet1.baro_max_altitutde , packet1.gps_max_altitude , packet1.drogue_dpl_ts , packet1.drogue_dpl_max_acc , packet1.dpl_vane_max_pressure , packet1.main_dpl_altitude_ts , packet1.main_dpl_altitude , packet1.main_dpl_zspeed , packet1.main_dpl_acc , packet1.cpu_load , packet1.free_heap );
    mavlink_msg_lr_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lr_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lr_tm_send(MAVLINK_COMM_1 , packet1.liftoff_ts , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.max_z_speed_ts , packet1.max_z_speed , packet1.max_airspeed_pitot , packet1.max_speed_altitude , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.static_min_pressure , packet1.digital_min_pressure , packet1.ada_min_pressure , packet1.baro_max_altitutde , packet1.gps_max_altitude , packet1.drogue_dpl_ts , packet1.drogue_dpl_max_acc , packet1.dpl_vane_max_pressure , packet1.main_dpl_altitude_ts , packet1.main_dpl_altitude , packet1.main_dpl_zspeed , packet1.main_dpl_acc , packet1.cpu_load , packet1.free_heap );
    mavlink_msg_lr_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_test_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TEST_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_test_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,77
    };
    mavlink_test_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pressure_hw = packet_in.pressure_hw;
        packet1.temp_analog = packet_in.temp_analog;
        packet1.temp_imu = packet_in.temp_imu;
        packet1.battery_volt = packet_in.battery_volt;
        packet1.gps_nsats = packet_in.gps_nsats;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TEST_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TEST_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_test_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_test_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_test_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.pressure_hw , packet1.gps_nsats , packet1.temp_analog , packet1.temp_imu , packet1.battery_volt );
    mavlink_msg_test_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_test_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.pressure_hw , packet1.gps_nsats , packet1.temp_analog , packet1.temp_imu , packet1.battery_volt );
    mavlink_msg_test_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_test_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_test_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.pressure_hw , packet1.gps_nsats , packet1.temp_analog , packet1.temp_imu , packet1.battery_volt );
    mavlink_msg_test_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_windtunnel_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WINDTUNNEL_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_windtunnel_tm_t packet_in = {
        93372036854775807ULL,179.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0
    };
    mavlink_windtunnel_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pressure_dpl = packet_in.pressure_dpl;
        packet1.pressure_digital = packet_in.pressure_digital;
        packet1.pressure_differential = packet_in.pressure_differential;
        packet1.pressure_static = packet_in.pressure_static;
        packet1.ab_angle = packet_in.ab_angle;
        packet1.wind_speed = packet_in.wind_speed;
        packet1.log_status = packet_in.log_status;
        packet1.log_num = packet_in.log_num;
        packet1.last_RSSI = packet_in.last_RSSI;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WINDTUNNEL_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WINDTUNNEL_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_windtunnel_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_windtunnel_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_windtunnel_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.pressure_digital , packet1.pressure_differential , packet1.pressure_static , packet1.pressure_dpl , packet1.ab_angle , packet1.wind_speed , packet1.log_status , packet1.log_num , packet1.last_RSSI );
    mavlink_msg_windtunnel_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_windtunnel_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.pressure_digital , packet1.pressure_differential , packet1.pressure_static , packet1.pressure_dpl , packet1.ab_angle , packet1.wind_speed , packet1.log_status , packet1.log_num , packet1.last_RSSI );
    mavlink_msg_windtunnel_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_windtunnel_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_windtunnel_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.pressure_digital , packet1.pressure_differential , packet1.pressure_static , packet1.pressure_dpl , packet1.ab_angle , packet1.wind_speed , packet1.log_status , packet1.log_num , packet1.last_RSSI );
    mavlink_msg_windtunnel_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sensors_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENSORS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sensors_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,689.0,717.0,61
    };
    mavlink_sensors_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.bmx160_acc_x = packet_in.bmx160_acc_x;
        packet1.bmx160_acc_y = packet_in.bmx160_acc_y;
        packet1.bmx160_acc_z = packet_in.bmx160_acc_z;
        packet1.bmx160_gyro_x = packet_in.bmx160_gyro_x;
        packet1.bmx160_gyro_y = packet_in.bmx160_gyro_y;
        packet1.bmx160_gyro_z = packet_in.bmx160_gyro_z;
        packet1.bmx160_mag_x = packet_in.bmx160_mag_x;
        packet1.bmx160_mag_y = packet_in.bmx160_mag_y;
        packet1.bmx160_mag_z = packet_in.bmx160_mag_z;
        packet1.bmx160_temp = packet_in.bmx160_temp;
        packet1.ms5803_press = packet_in.ms5803_press;
        packet1.ms5803_temp = packet_in.ms5803_temp;
        packet1.dpl_press = packet_in.dpl_press;
        packet1.pitot_press = packet_in.pitot_press;
        packet1.static_press = packet_in.static_press;
        packet1.lis3mdl_mag_x = packet_in.lis3mdl_mag_x;
        packet1.lis3mdl_mag_y = packet_in.lis3mdl_mag_y;
        packet1.lis3mdl_mag_z = packet_in.lis3mdl_mag_z;
        packet1.lis3mdl_temp = packet_in.lis3mdl_temp;
        packet1.gps_lat = packet_in.gps_lat;
        packet1.gps_lon = packet_in.gps_lon;
        packet1.gps_alt = packet_in.gps_alt;
        packet1.vbat = packet_in.vbat;
        packet1.vsupply_5v = packet_in.vsupply_5v;
        packet1.gps_fix = packet_in.gps_fix;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENSORS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENSORS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensors_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sensors_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensors_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.bmx160_acc_x , packet1.bmx160_acc_y , packet1.bmx160_acc_z , packet1.bmx160_gyro_x , packet1.bmx160_gyro_y , packet1.bmx160_gyro_z , packet1.bmx160_mag_x , packet1.bmx160_mag_y , packet1.bmx160_mag_z , packet1.bmx160_temp , packet1.ms5803_press , packet1.ms5803_temp , packet1.dpl_press , packet1.pitot_press , packet1.static_press , packet1.lis3mdl_mag_x , packet1.lis3mdl_mag_y , packet1.lis3mdl_mag_z , packet1.lis3mdl_temp , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.gps_fix , packet1.vbat , packet1.vsupply_5v );
    mavlink_msg_sensors_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensors_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.bmx160_acc_x , packet1.bmx160_acc_y , packet1.bmx160_acc_z , packet1.bmx160_gyro_x , packet1.bmx160_gyro_y , packet1.bmx160_gyro_z , packet1.bmx160_mag_x , packet1.bmx160_mag_y , packet1.bmx160_mag_z , packet1.bmx160_temp , packet1.ms5803_press , packet1.ms5803_temp , packet1.dpl_press , packet1.pitot_press , packet1.static_press , packet1.lis3mdl_mag_x , packet1.lis3mdl_mag_y , packet1.lis3mdl_mag_z , packet1.lis3mdl_temp , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.gps_fix , packet1.vbat , packet1.vsupply_5v );
    mavlink_msg_sensors_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sensors_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensors_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.bmx160_acc_x , packet1.bmx160_acc_y , packet1.bmx160_acc_z , packet1.bmx160_gyro_x , packet1.bmx160_gyro_y , packet1.bmx160_gyro_z , packet1.bmx160_mag_x , packet1.bmx160_mag_y , packet1.bmx160_mag_z , packet1.bmx160_temp , packet1.ms5803_press , packet1.ms5803_temp , packet1.dpl_press , packet1.pitot_press , packet1.static_press , packet1.lis3mdl_mag_x , packet1.lis3mdl_mag_y , packet1.lis3mdl_mag_z , packet1.lis3mdl_temp , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.gps_fix , packet1.vbat , packet1.vsupply_5v );
    mavlink_msg_sensors_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_lynx(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ping_tc(system_id, component_id, last_msg);
    mavlink_test_noarg_tc(system_id, component_id, last_msg);
    mavlink_test_start_launch_tc(system_id, component_id, last_msg);
    mavlink_test_upload_setting_tc(system_id, component_id, last_msg);
    mavlink_test_set_aerobrake_angle_tc(system_id, component_id, last_msg);
    mavlink_test_set_reference_altitude(system_id, component_id, last_msg);
    mavlink_test_set_reference_temperature_tc(system_id, component_id, last_msg);
    mavlink_test_set_deployment_altitude_tc(system_id, component_id, last_msg);
    mavlink_test_set_initial_orientation_tc(system_id, component_id, last_msg);
    mavlink_test_set_initial_coordinates_tc(system_id, component_id, last_msg);
    mavlink_test_telemetry_request_tc(system_id, component_id, last_msg);
    mavlink_test_raw_event_tc(system_id, component_id, last_msg);
    mavlink_test_ack_tm(system_id, component_id, last_msg);
    mavlink_test_nack_tm(system_id, component_id, last_msg);
    mavlink_test_sys_tm(system_id, component_id, last_msg);
    mavlink_test_fmm_tm(system_id, component_id, last_msg);
    mavlink_test_pin_obs_tm(system_id, component_id, last_msg);
    mavlink_test_logger_tm(system_id, component_id, last_msg);
    mavlink_test_tmtc_tm(system_id, component_id, last_msg);
    mavlink_test_task_stats_tm(system_id, component_id, last_msg);
    mavlink_test_dpl_tm(system_id, component_id, last_msg);
    mavlink_test_ada_tm(system_id, component_id, last_msg);
    mavlink_test_abk_tm(system_id, component_id, last_msg);
    mavlink_test_nas_tm(system_id, component_id, last_msg);
    mavlink_test_adc_tm(system_id, component_id, last_msg);
    mavlink_test_ms5803_tm(system_id, component_id, last_msg);
    mavlink_test_bmx160_tm(system_id, component_id, last_msg);
    mavlink_test_lis3mdl_tm(system_id, component_id, last_msg);
    mavlink_test_gps_tm(system_id, component_id, last_msg);
    mavlink_test_hr_tm(system_id, component_id, last_msg);
    mavlink_test_lr_tm(system_id, component_id, last_msg);
    mavlink_test_test_tm(system_id, component_id, last_msg);
    mavlink_test_windtunnel_tm(system_id, component_id, last_msg);
    mavlink_test_sensors_tm(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // LYNX_TESTSUITE_H
