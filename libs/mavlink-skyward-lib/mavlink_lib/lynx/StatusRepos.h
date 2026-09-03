/** @file
 *  @brief Skyward Status Repository
 */
#pragma once

namespace StatusRepo
{
    /* Enumeration of all possible TMs */
    enum MavTMList: uint8_t
    {
         MAV_ACK_TM_ID,
         MAV_NACK_TM_ID,
         MAV_SYS_TM_ID,
         MAV_FMM_TM_ID,
         MAV_PIN_OBS_TM_ID,
         MAV_LOGGER_TM_ID,
         MAV_TMTC_TM_ID,
         MAV_TASK_STATS_TM_ID,
         MAV_DPL_TM_ID,
         MAV_ADA_TM_ID,
         MAV_ABK_TM_ID,
         MAV_NAS_TM_ID,
         MAV_ADC_TM_ID,
         MAV_MS5803_TM_ID,
         MAV_BMX160_TM_ID,
         MAV_LIS3MDL_TM_ID,
         MAV_GPS_TM_ID,
         MAV_HR_TM_ID,
         MAV_LR_TM_ID,
         MAV_TEST_TM_ID,
         MAV_WINDTUNNEL_TM_ID,
         MAV_SENSORS_TM_ID,
        
    };

    /* Struct containing all tms in the form of mavlink structs */
    static struct status_repo_t
    {
         mavlink_ack_tm_t ack_tm;
         mavlink_nack_tm_t nack_tm;
         mavlink_sys_tm_t sys_tm;
         mavlink_fmm_tm_t fmm_tm;
         mavlink_pin_obs_tm_t pin_obs_tm;
         mavlink_logger_tm_t logger_tm;
         mavlink_tmtc_tm_t tmtc_tm;
         mavlink_task_stats_tm_t task_stats_tm;
         mavlink_dpl_tm_t dpl_tm;
         mavlink_ada_tm_t ada_tm;
         mavlink_abk_tm_t abk_tm;
         mavlink_nas_tm_t nas_tm;
         mavlink_adc_tm_t adc_tm;
         mavlink_ms5803_tm_t ms5803_tm;
         mavlink_bmx160_tm_t bmx160_tm;
         mavlink_lis3mdl_tm_t lis3mdl_tm;
         mavlink_gps_tm_t gps_tm;
         mavlink_hr_tm_t hr_tm;
         mavlink_lr_tm_t lr_tm;
         mavlink_test_tm_t test_tm;
         mavlink_windtunnel_tm_t windtunnel_tm;
         mavlink_sensors_tm_t sensors_tm;
        
    } status repo;

    /**
     * Retrieve one of the telemetry structs
     * @req_tm          required telemetry
     * @return          packed mavlink struct of that telemetry
     */
    static mavlink_message_t getTM(uint8_t req_tm, uint8_t sys_id, uint8_t comp_id)
    {
        mavlink_message_t m;

        switch (req_tm) 
        {
             case MavTMList::MAV_ACK_TM_ID:
                mavlink_msg_ack_tm_encode(sys_id, comp_id, &m, &(repo.ack_tm));
                break;
             case MavTMList::MAV_NACK_TM_ID:
                mavlink_msg_nack_tm_encode(sys_id, comp_id, &m, &(repo.nack_tm));
                break;
             case MavTMList::MAV_SYS_TM_ID:
                mavlink_msg_sys_tm_encode(sys_id, comp_id, &m, &(repo.sys_tm));
                break;
             case MavTMList::MAV_FMM_TM_ID:
                mavlink_msg_fmm_tm_encode(sys_id, comp_id, &m, &(repo.fmm_tm));
                break;
             case MavTMList::MAV_PIN_OBS_TM_ID:
                mavlink_msg_pin_obs_tm_encode(sys_id, comp_id, &m, &(repo.pin_obs_tm));
                break;
             case MavTMList::MAV_LOGGER_TM_ID:
                mavlink_msg_logger_tm_encode(sys_id, comp_id, &m, &(repo.logger_tm));
                break;
             case MavTMList::MAV_TMTC_TM_ID:
                mavlink_msg_tmtc_tm_encode(sys_id, comp_id, &m, &(repo.tmtc_tm));
                break;
             case MavTMList::MAV_TASK_STATS_TM_ID:
                mavlink_msg_task_stats_tm_encode(sys_id, comp_id, &m, &(repo.task_stats_tm));
                break;
             case MavTMList::MAV_DPL_TM_ID:
                mavlink_msg_dpl_tm_encode(sys_id, comp_id, &m, &(repo.dpl_tm));
                break;
             case MavTMList::MAV_ADA_TM_ID:
                mavlink_msg_ada_tm_encode(sys_id, comp_id, &m, &(repo.ada_tm));
                break;
             case MavTMList::MAV_ABK_TM_ID:
                mavlink_msg_abk_tm_encode(sys_id, comp_id, &m, &(repo.abk_tm));
                break;
             case MavTMList::MAV_NAS_TM_ID:
                mavlink_msg_nas_tm_encode(sys_id, comp_id, &m, &(repo.nas_tm));
                break;
             case MavTMList::MAV_ADC_TM_ID:
                mavlink_msg_adc_tm_encode(sys_id, comp_id, &m, &(repo.adc_tm));
                break;
             case MavTMList::MAV_MS5803_TM_ID:
                mavlink_msg_ms5803_tm_encode(sys_id, comp_id, &m, &(repo.ms5803_tm));
                break;
             case MavTMList::MAV_BMX160_TM_ID:
                mavlink_msg_bmx160_tm_encode(sys_id, comp_id, &m, &(repo.bmx160_tm));
                break;
             case MavTMList::MAV_LIS3MDL_TM_ID:
                mavlink_msg_lis3mdl_tm_encode(sys_id, comp_id, &m, &(repo.lis3mdl_tm));
                break;
             case MavTMList::MAV_GPS_TM_ID:
                mavlink_msg_gps_tm_encode(sys_id, comp_id, &m, &(repo.gps_tm));
                break;
             case MavTMList::MAV_HR_TM_ID:
                mavlink_msg_hr_tm_encode(sys_id, comp_id, &m, &(repo.hr_tm));
                break;
             case MavTMList::MAV_LR_TM_ID:
                mavlink_msg_lr_tm_encode(sys_id, comp_id, &m, &(repo.lr_tm));
                break;
             case MavTMList::MAV_TEST_TM_ID:
                mavlink_msg_test_tm_encode(sys_id, comp_id, &m, &(repo.test_tm));
                break;
             case MavTMList::MAV_WINDTUNNEL_TM_ID:
                mavlink_msg_windtunnel_tm_encode(sys_id, comp_id, &m, &(repo.windtunnel_tm));
                break;
             case MavTMList::MAV_SENSORS_TM_ID:
                mavlink_msg_sensors_tm_encode(sys_id, comp_id, &m, &(repo.sensors_tm));
                break;
            
            default:
                // TODO: manage error
                break;
        }
       
        return m;
    }

}
