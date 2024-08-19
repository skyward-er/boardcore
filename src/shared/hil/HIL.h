/* Copyright (c) 2021-2023 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Emilio Corigliano
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <Singleton.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "HILConfig.h"
#include "HILFlightPhasesManager.h"
#include "HILTransceiver.h"

/**
 * @brief Single interface to the hardware-in-the-loop framework.
 */
class HIL : public Boardcore::Module
{
public:
    HIL(Boardcore::USART &hilSerial,
        HILFlightPhasesManager *flightPhasesManager)
        : flightPhasesManager(flightPhasesManager)
    {
        simulator = new HILTransceiver(hilSerial);
    }

    HILTransceiver *simulator;
    HILFlightPhasesManager *flightPhasesManager;

    /**
     * @brief Start the needed hardware-in-the-loop components.
     */
    [[nodiscard]] bool start()
    {
        return simulator->start() && flightPhasesManager->start();
    }

    void stop() { simulator->stop(); }

    void send(HILConfig::ActuatorData actuatorData)
    {
        simulator->setActuatorData(actuatorData);
    }

    /**
     * @brief Returns if all the schedulers are up and running
     */
    bool isStarted();
};
