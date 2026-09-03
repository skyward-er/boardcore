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
         MAV_LOGGER_TM_ID,
         MAV_TMTC_TM_ID,
         MAV_SM_TM_ID,
         MAV_IGN_TM_ID,
         MAV_DPL_TM_ID,
         MAV_ADA_TM_ID,
         MAV_CAN_TM_ID,
         MAV_ADC_TM_ID,
         MAV_ADIS_TM_ID,
         MAV_MPU_TM_ID,
         MAV_GPS_TM_ID,
         MAV_HR_TM_ID,
         MAV_LR_TM_ID,
         MAV_TEST_TM_ID,
        
    };

    /* Struct containing all tms in the form of mavlink structs */
    static struct status_repo_t
    {
         mavlink_ack_tm_t ack_tm;
         mavlink_nack_tm_t nack_tm;
         mavlink_sys_tm_t sys_tm;
         mavlink_fmm_tm_t fmm_tm;
         mavlink_logger_tm_t logger_tm;
         mavlink_tmtc_tm_t tmtc_tm;
         mavlink_sm_tm_t sm_tm;
         mavlink_ign_tm_t ign_tm;
         mavlink_dpl_tm_t dpl_tm;
         mavlink_ada_tm_t ada_tm;
         mavlink_can_tm_t can_tm;
         mavlink_adc_tm_t adc_tm;
         mavlink_adis_tm_t adis_tm;
         mavlink_mpu_tm_t mpu_tm;
         mavlink_gps_tm_t gps_tm;
         mavlink_hr_tm_t hr_tm;
         mavlink_lr_tm_t lr_tm;
         mavlink_test_tm_t test_tm;
        
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
             case MavTMList::MAV_LOGGER_TM_ID:
                mavlink_msg_logger_tm_encode(sys_id, comp_id, &m, &(repo.logger_tm));
                break;
             case MavTMList::MAV_TMTC_TM_ID:
                mavlink_msg_tmtc_tm_encode(sys_id, comp_id, &m, &(repo.tmtc_tm));
                break;
             case MavTMList::MAV_SM_TM_ID:
                mavlink_msg_sm_tm_encode(sys_id, comp_id, &m, &(repo.sm_tm));
                break;
             case MavTMList::MAV_IGN_TM_ID:
                mavlink_msg_ign_tm_encode(sys_id, comp_id, &m, &(repo.ign_tm));
                break;
             case MavTMList::MAV_DPL_TM_ID:
                mavlink_msg_dpl_tm_encode(sys_id, comp_id, &m, &(repo.dpl_tm));
                break;
             case MavTMList::MAV_ADA_TM_ID:
                mavlink_msg_ada_tm_encode(sys_id, comp_id, &m, &(repo.ada_tm));
                break;
             case MavTMList::MAV_CAN_TM_ID:
                mavlink_msg_can_tm_encode(sys_id, comp_id, &m, &(repo.can_tm));
                break;
             case MavTMList::MAV_ADC_TM_ID:
                mavlink_msg_adc_tm_encode(sys_id, comp_id, &m, &(repo.adc_tm));
                break;
             case MavTMList::MAV_ADIS_TM_ID:
                mavlink_msg_adis_tm_encode(sys_id, comp_id, &m, &(repo.adis_tm));
                break;
             case MavTMList::MAV_MPU_TM_ID:
                mavlink_msg_mpu_tm_encode(sys_id, comp_id, &m, &(repo.mpu_tm));
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
            
            default:
                // TODO: manage error
                break;
        }
       
        return m;
    }

}
