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
    float Phi_data[9] = {1,
                         samplingPeriod,
                         0.5 * samplingPeriod * samplingPeriod,
                         0,
                         1,
                         samplingPeriod,
                         0,
                         0,
                         1};
    Matrix Phi{3, 3, Phi_data};
    filter.Phi = Phi;
}

/* --- INSTANCE METHODS --- */

void ADA::update(float pressure)
{
    switch (state)
    {
        case ADAState::CALIBRATING:
        {
            // Calibrating state: update the average
            avg = (avg * avg_n_samples + pressure) / (avg_n_samples + 1);
            avg_n_samples = avg_n_samples + 1;
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
            Matrix y{1, 1, &pressure};  // TODO: Check if & is appropriate
            filter.update(y);
            if (filter.X(1) < 0)
            {
                // TODO: Log apogee detected
            }
            break;
        }

        case ADAState::ACTIVE:
        {
            // Active state send notifications for apogee
            Matrix y{1, 1, &pressure};  // TODO: Check if & is appropriate
            filter.update(y);
            if (filter.X(1) < 0)
            {
                sEventBroker->post({EV_ADA_APOGEE_DETECTED}, TOPIC_ADA);
                // TODO: Log apogee detected
            }
            break;
        }

        case ADAState::FIRST_DESCENT_PHASE:
        {
            // Descent state: send notifications for target altitude reached
            Matrix y{1, 1, &pressure};  // TODO: Check if & is appropriate
            filter.update(y);
            if (filter.X(0) <= DESCENT_DPL_ALTITUDE)
            {
                sEventBroker->post({EV_DPL_ALTITUDE}, TOPIC_ADA);
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
            state = ADAState::CALIBRATING;
            cal_delayed_event_id =
                sEventBroker->postDelayed({EV_TIMEOUT_ADA_CALIBRATION},
                                          TOPIC_ADA, TIMEOUT_MS_CALIBRATION);
            break;
        case EV_EXIT:
            TRACE("ADA: Exiting stateCalibrating\n");
            sEventBroker->removeDelayed(cal_delayed_event_id);
            break;
        case EV_TIMEOUT_ADA_CALIBRATION:
            transition(&ADA::stateIdle);
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
            state = ADAState::IDLE;
            break;
        case EV_EXIT:
            TRACE("ADA: Exiting stateIdle\n");
            break;
        case EV_LIFTOFF:
            transition(&ADA::stateShadowMode);
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
            state = ADAState::SHADOW_MODE;
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
            state = ADAState::ACTIVE;
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
 * when a set altitude is reached
 *
 * In this state a call to update() will trigger a one step update of the kalman
 * filter followed by a check of the rocket altitude.
 * The exiting transition to the stop state is triggered by the parachute
 * deployment altitude reached event (NOT self generated!)
 */
void ADA::stateFirstDescentPhase(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("ADA: Entering stateFirstDescentPhase\n");
            state = ADAState::FIRST_DESCENT_PHASE;
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
            state = ADAState::END;
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
