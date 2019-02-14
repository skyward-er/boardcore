/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <boards/Homeone/ADA/ADA.h>
#include <boards/Homeone/ADA/ADA_config.h>

#include <events/EventBroker.h>

#include <boards/Homeone/Events.h>
#include <boards/Homeone/Topics.h>

#include "Debug.h"

namespace HomeoneBoard
{
namespace FMM
{

/* --- LIFE CYCLE --- */
ADA::ADA()
    : FSM(&ADA::stateCalibrating),
      filter(Matrix{3, 3, P_data}, Matrix{1, 1, R_data}, Matrix{3, 3, Q_data},
             Matrix{1, 3, (float[]){1, 0, 0}})
{
    // Subscribe to topics
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);
    sEventBroker->subscribe(this, TOPIC_ADA);

    // Set state propagation matrix
    // Note that sampling frequency is supposed to be constant and known at
    // compile time. If this is not the case the matrix has to be updated at
    // each iteration

    // clang-format off
    float Phi_data[9] =
                {   1,  samplingPeriod, 0.5 * samplingPeriod * samplingPeriod,
                    0,  1,              samplingPeriod,
                    0,  0,              1
                };
    // clang-format on

    filter.Phi.set(Phi_data);
}

/* --- INSTANCE METHODS --- */

void ADA::update(float pressure)
{
    switch (status.state)
    {
        case ADAState::CALIBRATING:
        {
            // Calibrating state: update calibration data

            // Save old avg to compute var
            float old_avg = calibrationData.avg;

            // Update avg
            calibrationData.avg = (calibrationData.avg * calibrationData.n_samples + pressure) / (calibrationData.n_samples + 1);
            calibrationData.n_samples = calibrationData.n_samples + 1;

            // Update var
            float S_1 = calibrationData.var*(calibrationData.n_samples-1);
            float S   = S_1 + (pressure - old_avg)*(pressure - calibrationData.avg);
            calibrationData.var = S/calibrationData.n_samples;

            if (calibrationData.n_samples >= CALIBRATION_N_SAMPLES) {
                sEventBroker->post({EV_ADA_CALIBRATION_COMPLETE}, TOPIC_ADA);
            }
            break;
        }

        case ADAState::IDLE:
        {
            // Idle state: do nothing
            break;
        }

        case ADAState::SHADOW_MODE:
        {
            // Shadow mode state: update kalman, DO NOT send events
            Matrix y{1, 1, &pressure};
            filter.update(y);
            // Check if the "pressure speed" (hence positive when decending) is
            // positive
            if (filter.X(1) > 0)
            {
                ApogeeDetected apogee_det;
                apogee_det.tick  = miosix::getTick();
                apogee_det.state = status.state;
                logger.log(apogee_det);
            }
            break;
        }

        case ADAState::ACTIVE:
        {
            // Active state send notifications for apogee
            Matrix y{1, 1, &pressure};
            filter.update(y);
            // Check if the "pressure speed" (hence positive when decending) is
            // positive
            if (filter.X(1) > 0)
            {
                sEventBroker->post({EV_ADA_APOGEE_DETECTED}, TOPIC_ADA);
                ApogeeDetected apogee_det;
                apogee_det.tick  = miosix::getTick();
                apogee_det.state = status.state;
                logger.log(apogee_det);
            }
            break;
        }

        case ADAState::FIRST_DESCENT_PHASE:
        {
            // Descent state: send notifications for target altitude reached
            Matrix y{1, 1, &pressure};
            filter.update(y);
            if (filter.X(0) >= dpl_target_pressure_v)
            {
                sEventBroker->post({EV_DPL_ALTITUDE}, TOPIC_ADA);
                DplPressureReached dpl_reached;
                dpl_reached.tick = miosix::getTick();
                logger.log(dpl_reached);
            }
            break;
        }

        case ADAState::END:
        {
            // End state: do nothing
            break;
        }

        case ADAState::UNDEFINED:
        {
            TRACE("ADA Update: Undefined state value \n");
        }

        default:
        {
            TRACE("ADA Update: Unexpected state value \n");
        }
    }
}

/* --- STATES --- */
/**
 * \brief Calibrating state: the ADA calibrates the initial state. This is the
 * initial state.
 *
 * In this state a call to update() will result in a pressure sample being added
 * to the average.
 * The exiting transition to the idle state is triggered by a timeout event.
 */
void ADA::stateCalibrating(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("ADA: Entering stateCalibrating\n");
            status.state = ADAState::CALIBRATING;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("ADA: Exiting stateCalibrating\n");
            break;
        case EV_ADA_CALIBRATION_COMPLETE:
            transition(&ADA::stateIdle);
            break;
        case EV_TC_SET_DPL_PRESSURE:
            const DeploymentPressureEvent& dpl_ev =
                static_cast<const DeploymentPressureEvent&>(ev);
            dpl_target_pressure_v = dpl_ev.dplPressure;
            break;
        default:
            TRACE("ADA stateCalibrating: %d event not handled", ev.sig);
            break;
    }
}

/**
 * \brief Idle state:  ADA is ready and waiting for liftoff
 *
 * In this state a call to update() will have no effect.
 * The exiting transition to the shadow mode state is triggered by the liftoff
 * event.
 */
void ADA::stateIdle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("ADA: Entering stateIdle\n");
            status.state = ADAState::IDLE;
            logger.log(status);
            filter.X(0) = calibrationData.avg;  // Initialize the state with the average
            break;
        case EV_EXIT:
            TRACE("ADA: Exiting stateIdle\n");
            break;
        case EV_LIFTOFF:
            transition(&ADA::stateShadowMode);
            break;
        case EV_TC_SET_DPL_PRESSURE:
            const DeploymentPressureEvent& dpl_ev =
                static_cast<const DeploymentPressureEvent&>(ev);
            dpl_target_pressure_v = dpl_ev.dplPressure;
            break;
        case EV_TC_RESET_CALIBRATION:
            transition(&ADA::stateCalibrating);
            break;
        default:
            TRACE("ADA stateIdle: %d event not handled", ev.sig);
            break;
    }
}

/**
 * \brief Shadow mode state:  ADA is running and logging apogees detected, but
 * is not generating events
 *
 * In this state a call to update() will trigger a one step update of the kalman
 * filter followed by a check of vertical speed sign.
 * The exiting transition to the active state is triggered by a timeout event.
 */
void ADA::stateShadowMode(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("ADA: Entering stateShadowMode\n");
            status.state = ADAState::SHADOW_MODE;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("ADA: Exiting stateShadowMode\n");
            break;
        case EV_TIMEOUT_SHADOW_MODE:
            transition(&ADA::stateActive);
            break;
        default:
            TRACE("ADA stateShadowMode: %d event not handled", ev.sig);
            break;
    }
}

/**
 * \brief Active state:  ADA is running and it generates an event whe apogee is
 * detected
 *
 * In this state a call to update() will trigger a one step update of the kalman
 * filter followed by a check of vertical speed sign.
 * The exiting transition to the descent state is triggered by the apogee
 * reached event (NOT self generated!)
 */
void ADA::stateActive(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("ADA: Entering stateActive\n");
            status.state = ADAState::ACTIVE;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("ADA: Exiting stateActive\n");
            break;
        case EV_APOGEE:
            transition(&ADA::stateFirstDescentPhase);
            break;
        default:
            TRACE("ADA stateActive: %d event not handled", ev.sig);
            break;
    }
}

/**
 * \brief First descent phase state:  ADA is running and it generates an event
 * when a set pressure (pressure of parachute dpl altitude) is reached
 *
 * In this state a call to update() will trigger a one step update of the kalman
 * filter followed by a check of the pressure.
 * The exiting transition to the stop state is triggered by the parachute
 * deployment altitude reached event (NOT self generated!)
 */
void ADA::stateFirstDescentPhase(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("ADA: Entering stateFirstDescentPhase\n");
            status.state = ADAState::FIRST_DESCENT_PHASE;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("ADA: Exiting stateFirstDescentPhase\n");
            break;
        case EV_DPL_ALTITUDE:
            transition(&ADA::stateEnd);
            break;
        default:
            TRACE("ADA stateFirstDescentPhase: %d event not handled", ev.sig);
            break;
    }
}

/**
 * \brief End state:  ADA is stopped
 *
 * In this state a call to update() will have no effect.
 * This is the final state
 */
void ADA::stateEnd(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("ADA: Entering stateEnd\n");
            status.state = ADAState::END;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("ADA: Exiting stateEnd\n");
            break;
        default:
            TRACE("ADA stateEnd: %d event not handled", ev.sig);
            break;
    }
}

}  // namespace FMM
}  // namespace HomeoneBoard
