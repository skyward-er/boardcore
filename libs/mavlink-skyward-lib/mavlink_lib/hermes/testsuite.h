/** @file
 *    @brief MAVLink comm protocol testsuite generated from hermes.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef HERMES_TESTSUITE_H
#define HERMES_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_hermes(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_hermes(system_id, component_id, last_msg);
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
        93372036854775807ULL,29,96,163,230,41,108,175,242,53,120
    };
    mavlink_sys_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.death_stack = packet_in.death_stack;
        packet1.logger = packet_in.logger;
        packet1.ev_broker = packet_in.ev_broker;
        packet1.pin_obs = packet_in.pin_obs;
        packet1.fmm = packet_in.fmm;
        packet1.sensor_manager = packet_in.sensor_manager;
        packet1.ada = packet_in.ada;
        packet1.tmtc = packet_in.tmtc;
        packet1.ign = packet_in.ign;
        packet1.dpl = packet_in.dpl;
        
        
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
    mavlink_msg_sys_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.death_stack , packet1.logger , packet1.ev_broker , packet1.pin_obs , packet1.fmm , packet1.sensor_manager , packet1.ada , packet1.tmtc , packet1.ign , packet1.dpl );
    mavlink_msg_sys_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.death_stack , packet1.logger , packet1.ev_broker , packet1.pin_obs , packet1.fmm , packet1.sensor_manager , packet1.ada , packet1.tmtc , packet1.ign , packet1.dpl );
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
    mavlink_msg_sys_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.death_stack , packet1.logger , packet1.ev_broker , packet1.pin_obs , packet1.fmm , packet1.sensor_manager , packet1.ada , packet1.tmtc , packet1.ign , packet1.dpl );
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
        93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,77,144,211,22,89
    };
    mavlink_fmm_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pin_launch_last_change = packet_in.pin_launch_last_change;
        packet1.pin_nosecone_last_change = packet_in.pin_nosecone_last_change;
        packet1.state = packet_in.state;
        packet1.pin_launch_num_changes = packet_in.pin_launch_num_changes;
        packet1.pin_launch_state = packet_in.pin_launch_state;
        packet1.pin_nosecone_num_changes = packet_in.pin_nosecone_num_changes;
        packet1.pin_nosecone_state = packet_in.pin_nosecone_state;
        
        
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
    mavlink_msg_fmm_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.state , packet1.pin_launch_last_change , packet1.pin_launch_num_changes , packet1.pin_launch_state , packet1.pin_nosecone_last_change , packet1.pin_nosecone_num_changes , packet1.pin_nosecone_state );
    mavlink_msg_fmm_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fmm_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.state , packet1.pin_launch_last_change , packet1.pin_launch_num_changes , packet1.pin_launch_state , packet1.pin_nosecone_last_change , packet1.pin_nosecone_num_changes , packet1.pin_nosecone_state );
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
    mavlink_msg_fmm_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.state , packet1.pin_launch_last_change , packet1.pin_launch_num_changes , packet1.pin_launch_state , packet1.pin_nosecone_last_change , packet1.pin_nosecone_num_changes , packet1.pin_nosecone_state );
    mavlink_msg_fmm_tm_decode(last_msg, &packet2);
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

static void mavlink_test_sm_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SM_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sm_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,20147,51
    };
    mavlink_sm_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.task_10hz_min = packet_in.task_10hz_min;
        packet1.task_10hz_max = packet_in.task_10hz_max;
        packet1.task_10hz_mean = packet_in.task_10hz_mean;
        packet1.task_10hz_stddev = packet_in.task_10hz_stddev;
        packet1.task_20hz_min = packet_in.task_20hz_min;
        packet1.task_20hz_max = packet_in.task_20hz_max;
        packet1.task_20hz_mean = packet_in.task_20hz_mean;
        packet1.task_20hz_stddev = packet_in.task_20hz_stddev;
        packet1.task_250hz_min = packet_in.task_250hz_min;
        packet1.task_250hz_max = packet_in.task_250hz_max;
        packet1.task_250hz_mean = packet_in.task_250hz_mean;
        packet1.task_250hz_stddev = packet_in.task_250hz_stddev;
        packet1.sensor_state = packet_in.sensor_state;
        packet1.state = packet_in.state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SM_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SM_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sm_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sm_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sm_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.state , packet1.sensor_state , packet1.task_10hz_min , packet1.task_10hz_max , packet1.task_10hz_mean , packet1.task_10hz_stddev , packet1.task_20hz_min , packet1.task_20hz_max , packet1.task_20hz_mean , packet1.task_20hz_stddev , packet1.task_250hz_min , packet1.task_250hz_max , packet1.task_250hz_mean , packet1.task_250hz_stddev );
    mavlink_msg_sm_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sm_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.state , packet1.sensor_state , packet1.task_10hz_min , packet1.task_10hz_max , packet1.task_10hz_mean , packet1.task_10hz_stddev , packet1.task_20hz_min , packet1.task_20hz_max , packet1.task_20hz_mean , packet1.task_20hz_stddev , packet1.task_250hz_min , packet1.task_250hz_max , packet1.task_250hz_mean , packet1.task_250hz_stddev );
    mavlink_msg_sm_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sm_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sm_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.state , packet1.sensor_state , packet1.task_10hz_min , packet1.task_10hz_max , packet1.task_10hz_mean , packet1.task_10hz_stddev , packet1.task_20hz_min , packet1.task_20hz_max , packet1.task_20hz_mean , packet1.task_20hz_stddev , packet1.task_250hz_min , packet1.task_250hz_max , packet1.task_250hz_mean , packet1.task_250hz_stddev );
    mavlink_msg_sm_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ign_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_IGN_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ign_tm_t packet_in = {
        93372036854775807ULL,17651,17755,41,108,175,242,53
    };
    mavlink_ign_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.n_sent_messages = packet_in.n_sent_messages;
        packet1.n_rcv_message = packet_in.n_rcv_message;
        packet1.fsm_state = packet_in.fsm_state;
        packet1.last_event = packet_in.last_event;
        packet1.cmd_bitfield = packet_in.cmd_bitfield;
        packet1.stm32_bitfield = packet_in.stm32_bitfield;
        packet1.avr_bitfield = packet_in.avr_bitfield;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_IGN_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_IGN_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ign_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ign_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ign_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.fsm_state , packet1.last_event , packet1.n_sent_messages , packet1.n_rcv_message , packet1.cmd_bitfield , packet1.stm32_bitfield , packet1.avr_bitfield );
    mavlink_msg_ign_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ign_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.fsm_state , packet1.last_event , packet1.n_sent_messages , packet1.n_rcv_message , packet1.cmd_bitfield , packet1.stm32_bitfield , packet1.avr_bitfield );
    mavlink_msg_ign_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ign_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ign_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.fsm_state , packet1.last_event , packet1.n_sent_messages , packet1.n_rcv_message , packet1.cmd_bitfield , packet1.stm32_bitfield , packet1.avr_bitfield );
    mavlink_msg_ign_tm_decode(last_msg, &packet2);
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
        93372036854775807ULL,73.0,101.0,53,120
    };
    mavlink_dpl_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.cutter_1_test_current = packet_in.cutter_1_test_current;
        packet1.cutter_2_test_current = packet_in.cutter_2_test_current;
        packet1.fsm_state = packet_in.fsm_state;
        packet1.cutter_state = packet_in.cutter_state;
        
        
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
    mavlink_msg_dpl_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.fsm_state , packet1.cutter_1_test_current , packet1.cutter_2_test_current , packet1.cutter_state );
    mavlink_msg_dpl_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dpl_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.fsm_state , packet1.cutter_1_test_current , packet1.cutter_2_test_current , packet1.cutter_state );
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
    mavlink_msg_dpl_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.fsm_state , packet1.cutter_1_test_current , packet1.cutter_2_test_current , packet1.cutter_state );
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
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,963500168,409.0,437.0,465.0,209
    };
    mavlink_ada_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.target_dpl_altitude = packet_in.target_dpl_altitude;
        packet1.kalman_x0 = packet_in.kalman_x0;
        packet1.kalman_x1 = packet_in.kalman_x1;
        packet1.kalman_x2 = packet_in.kalman_x2;
        packet1.kalman_acc_x0 = packet_in.kalman_acc_x0;
        packet1.kalman_acc_x1 = packet_in.kalman_acc_x1;
        packet1.kalman_acc_x2 = packet_in.kalman_acc_x2;
        packet1.ref_pressure = packet_in.ref_pressure;
        packet1.msl_pressure = packet_in.msl_pressure;
        packet1.ref_pressure_mean = packet_in.ref_pressure_mean;
        packet1.ref_pressure_stddev = packet_in.ref_pressure_stddev;
        packet1.ref_pressure_nsamples = packet_in.ref_pressure_nsamples;
        packet1.ref_altitude = packet_in.ref_altitude;
        packet1.ref_temperature = packet_in.ref_temperature;
        packet1.msl_temperature = packet_in.msl_temperature;
        packet1.state = packet_in.state;
        
        
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
    mavlink_msg_ada_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.state , packet1.target_dpl_altitude , packet1.kalman_x0 , packet1.kalman_x1 , packet1.kalman_x2 , packet1.kalman_acc_x0 , packet1.kalman_acc_x1 , packet1.kalman_acc_x2 , packet1.ref_pressure , packet1.msl_pressure , packet1.ref_pressure_mean , packet1.ref_pressure_stddev , packet1.ref_pressure_nsamples , packet1.ref_altitude , packet1.ref_temperature , packet1.msl_temperature );
    mavlink_msg_ada_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ada_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.state , packet1.target_dpl_altitude , packet1.kalman_x0 , packet1.kalman_x1 , packet1.kalman_x2 , packet1.kalman_acc_x0 , packet1.kalman_acc_x1 , packet1.kalman_acc_x2 , packet1.ref_pressure , packet1.msl_pressure , packet1.ref_pressure_mean , packet1.ref_pressure_stddev , packet1.ref_pressure_nsamples , packet1.ref_altitude , packet1.ref_temperature , packet1.msl_temperature );
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
    mavlink_msg_ada_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.state , packet1.target_dpl_altitude , packet1.kalman_x0 , packet1.kalman_x1 , packet1.kalman_x2 , packet1.kalman_acc_x0 , packet1.kalman_acc_x1 , packet1.kalman_acc_x2 , packet1.ref_pressure , packet1.msl_pressure , packet1.ref_pressure_mean , packet1.ref_pressure_stddev , packet1.ref_pressure_nsamples , packet1.ref_altitude , packet1.ref_temperature , packet1.msl_temperature );
    mavlink_msg_ada_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_can_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAN_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_can_tm_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,18067,18171,65,132
    };
    mavlink_can_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.last_sent_ts = packet_in.last_sent_ts;
        packet1.last_rcv_ts = packet_in.last_rcv_ts;
        packet1.n_sent = packet_in.n_sent;
        packet1.n_rcv = packet_in.n_rcv;
        packet1.last_sent = packet_in.last_sent;
        packet1.last_rcv = packet_in.last_rcv;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAN_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAN_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_can_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_tm_pack(system_id, component_id, &msg , packet1.n_sent , packet1.n_rcv , packet1.last_sent , packet1.last_rcv , packet1.last_sent_ts , packet1.last_rcv_ts );
    mavlink_msg_can_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.n_sent , packet1.n_rcv , packet1.last_sent , packet1.last_rcv , packet1.last_sent_ts , packet1.last_rcv_ts );
    mavlink_msg_can_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_can_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_tm_send(MAVLINK_COMM_1 , packet1.n_sent , packet1.n_rcv , packet1.last_sent , packet1.last_rcv , packet1.last_sent_ts , packet1.last_rcv_ts );
    mavlink_msg_can_tm_decode(last_msg, &packet2);
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
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,113,180
    };
    mavlink_adc_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.hw_baro_volt = packet_in.hw_baro_volt;
        packet1.nxp_baro_volt = packet_in.nxp_baro_volt;
        packet1.hw_baro_pressure = packet_in.hw_baro_pressure;
        packet1.nxp_baro_pressure = packet_in.nxp_baro_pressure;
        packet1.battery_voltage = packet_in.battery_voltage;
        packet1.current_sense_1 = packet_in.current_sense_1;
        packet1.current_sense_2 = packet_in.current_sense_2;
        packet1.hw_baro_flag = packet_in.hw_baro_flag;
        packet1.nxp_baro_flag = packet_in.nxp_baro_flag;
        
        
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
    mavlink_msg_adc_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.hw_baro_volt , packet1.nxp_baro_volt , packet1.hw_baro_flag , packet1.nxp_baro_flag , packet1.hw_baro_pressure , packet1.nxp_baro_pressure , packet1.battery_voltage , packet1.current_sense_1 , packet1.current_sense_2 );
    mavlink_msg_adc_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.hw_baro_volt , packet1.nxp_baro_volt , packet1.hw_baro_flag , packet1.nxp_baro_flag , packet1.hw_baro_pressure , packet1.nxp_baro_pressure , packet1.battery_voltage , packet1.current_sense_1 , packet1.current_sense_2 );
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
    mavlink_msg_adc_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.hw_baro_volt , packet1.nxp_baro_volt , packet1.hw_baro_flag , packet1.nxp_baro_flag , packet1.hw_baro_pressure , packet1.nxp_baro_pressure , packet1.battery_voltage , packet1.current_sense_1 , packet1.current_sense_2 );
    mavlink_msg_adc_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_adis_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADIS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_adis_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0
    };
    mavlink_adis_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.acc_x = packet_in.acc_x;
        packet1.acc_y = packet_in.acc_y;
        packet1.acc_z = packet_in.acc_z;
        packet1.gyro_x = packet_in.gyro_x;
        packet1.gyro_y = packet_in.gyro_y;
        packet1.gyro_z = packet_in.gyro_z;
        packet1.compass_x = packet_in.compass_x;
        packet1.compass_y = packet_in.compass_y;
        packet1.compass_z = packet_in.compass_z;
        packet1.temp = packet_in.temp;
        packet1.supply_out = packet_in.supply_out;
        packet1.aux_adc = packet_in.aux_adc;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADIS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADIS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adis_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_adis_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adis_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.compass_x , packet1.compass_y , packet1.compass_z , packet1.temp , packet1.supply_out , packet1.aux_adc );
    mavlink_msg_adis_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adis_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.compass_x , packet1.compass_y , packet1.compass_z , packet1.temp , packet1.supply_out , packet1.aux_adc );
    mavlink_msg_adis_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_adis_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adis_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.compass_x , packet1.compass_y , packet1.compass_z , packet1.temp , packet1.supply_out , packet1.aux_adc );
    mavlink_msg_adis_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mpu_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MPU_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mpu_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0
    };
    mavlink_mpu_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.acc_x = packet_in.acc_x;
        packet1.acc_y = packet_in.acc_y;
        packet1.acc_z = packet_in.acc_z;
        packet1.gyro_x = packet_in.gyro_x;
        packet1.gyro_y = packet_in.gyro_y;
        packet1.gyro_z = packet_in.gyro_z;
        packet1.compass_x = packet_in.compass_x;
        packet1.compass_y = packet_in.compass_y;
        packet1.compass_z = packet_in.compass_z;
        packet1.temp = packet_in.temp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MPU_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MPU_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mpu_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mpu_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mpu_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.compass_x , packet1.compass_y , packet1.compass_z , packet1.temp );
    mavlink_msg_mpu_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mpu_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.compass_x , packet1.compass_y , packet1.compass_z , packet1.temp );
    mavlink_msg_mpu_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mpu_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mpu_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.compass_x , packet1.compass_y , packet1.compass_z , packet1.temp );
    mavlink_msg_mpu_tm_decode(last_msg, &packet2);
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
        93372036854775807ULL,179.0,235.0,291.0,241.0,269.0,297.0,325.0,963499960,161
    };
    mavlink_gps_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.altitude = packet_in.altitude;
        packet1.vel_north = packet_in.vel_north;
        packet1.vel_east = packet_in.vel_east;
        packet1.vel_down = packet_in.vel_down;
        packet1.vel_mag = packet_in.vel_mag;
        packet1.n_satellites = packet_in.n_satellites;
        packet1.fix = packet_in.fix;
        
        
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
    mavlink_msg_gps_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.fix , packet1.lat , packet1.lon , packet1.altitude , packet1.vel_north , packet1.vel_east , packet1.vel_down , packet1.vel_mag , packet1.n_satellites );
    mavlink_msg_gps_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.fix , packet1.lat , packet1.lon , packet1.altitude , packet1.vel_north , packet1.vel_east , packet1.vel_down , packet1.vel_mag , packet1.n_satellites );
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
    mavlink_msg_gps_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.fix , packet1.lat , packet1.lon , packet1.altitude , packet1.vel_north , packet1.vel_east , packet1.vel_down , packet1.vel_mag , packet1.n_satellites );
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
        { 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108 }
    };
    mavlink_hr_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.payload, packet_in.payload, sizeof(uint8_t)*104);
        
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
    mavlink_msg_hr_tm_pack(system_id, component_id, &msg , packet1.payload );
    mavlink_msg_hr_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hr_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.payload );
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
    mavlink_msg_hr_tm_send(MAVLINK_COMM_1 , packet1.payload );
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
        { 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44 }
    };
    mavlink_lr_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.payload, packet_in.payload, sizeof(uint8_t)*40);
        
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
    mavlink_msg_lr_tm_pack(system_id, component_id, &msg , packet1.payload );
    mavlink_msg_lr_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lr_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.payload );
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
    mavlink_msg_lr_tm_send(MAVLINK_COMM_1 , packet1.payload );
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
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,89
    };
    mavlink_test_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pressure_hw = packet_in.pressure_hw;
        packet1.temp_analog = packet_in.temp_analog;
        packet1.temp_imu = packet_in.temp_imu;
        packet1.battery_volt = packet_in.battery_volt;
        packet1.th_cut_1 = packet_in.th_cut_1;
        packet1.th_cut_2 = packet_in.th_cut_2;
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
    mavlink_msg_test_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.pressure_hw , packet1.gps_nsats , packet1.temp_analog , packet1.temp_imu , packet1.battery_volt , packet1.th_cut_1 , packet1.th_cut_2 );
    mavlink_msg_test_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_test_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.pressure_hw , packet1.gps_nsats , packet1.temp_analog , packet1.temp_imu , packet1.battery_volt , packet1.th_cut_1 , packet1.th_cut_2 );
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
    mavlink_msg_test_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.pressure_hw , packet1.gps_nsats , packet1.temp_analog , packet1.temp_imu , packet1.battery_volt , packet1.th_cut_1 , packet1.th_cut_2 );
    mavlink_msg_test_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_hermes(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ping_tc(system_id, component_id, last_msg);
    mavlink_test_noarg_tc(system_id, component_id, last_msg);
    mavlink_test_start_launch_tc(system_id, component_id, last_msg);
    mavlink_test_upload_setting_tc(system_id, component_id, last_msg);
    mavlink_test_telemetry_request_tc(system_id, component_id, last_msg);
    mavlink_test_raw_event_tc(system_id, component_id, last_msg);
    mavlink_test_ack_tm(system_id, component_id, last_msg);
    mavlink_test_nack_tm(system_id, component_id, last_msg);
    mavlink_test_sys_tm(system_id, component_id, last_msg);
    mavlink_test_fmm_tm(system_id, component_id, last_msg);
    mavlink_test_logger_tm(system_id, component_id, last_msg);
    mavlink_test_tmtc_tm(system_id, component_id, last_msg);
    mavlink_test_sm_tm(system_id, component_id, last_msg);
    mavlink_test_ign_tm(system_id, component_id, last_msg);
    mavlink_test_dpl_tm(system_id, component_id, last_msg);
    mavlink_test_ada_tm(system_id, component_id, last_msg);
    mavlink_test_can_tm(system_id, component_id, last_msg);
    mavlink_test_adc_tm(system_id, component_id, last_msg);
    mavlink_test_adis_tm(system_id, component_id, last_msg);
    mavlink_test_mpu_tm(system_id, component_id, last_msg);
    mavlink_test_gps_tm(system_id, component_id, last_msg);
    mavlink_test_hr_tm(system_id, component_id, last_msg);
    mavlink_test_lr_tm(system_id, component_id, last_msg);
    mavlink_test_test_tm(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // HERMES_TESTSUITE_H
