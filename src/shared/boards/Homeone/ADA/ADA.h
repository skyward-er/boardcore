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

#include <boards/Homeone/ADA/ADAStatus.h>
#include <events/FSM.h>
#include <kalman/Kalman.h>
#include "logger/LogProxy.h"

class ADA : public FSM<ADA>
{

public:
    ADA();
    ~ADA() {}
    void update(float pressure);

private:
    void stateCalibrating(const Event& ev);
    void stateIdle(const Event& ev);
    void stateShadowMode(const Event& ev);
    void stateActive(const Event& ev);
    void stateFirstDescentPhase(const Event& ev);
    void stateEnd(const Event& ev);

    uint16_t cal_delayed_event_id = 0;      // Event id for calibration timeout
    ADAStatus status;                       // Variable to store state

    Kalman filter;          // Filter object that perfroms the computations

    // Calibration variables
    ADACalibrationData calibrationData;

    // Sum of all values squared divided by their number, to comupte variance
    float avg_of_squares = 0.0;

    // Parachute deployment pressure volts
    uint16_t   dpl_target_pressure_v = 5000; // Set default value here
    
    // Logger
    LoggerProxy& logger = *(LoggerProxy::getInstance());
};
