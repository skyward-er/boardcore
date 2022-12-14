/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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
#include <Singleton.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/timer/TimestampTimer.h>

#include <iostream>
#include <utils/ModuleManager/ModuleManager.hpp>

#ifndef DEBUG
#define DEBUG
#endif

using namespace Boardcore;
using namespace std;

namespace Boardcore
{
class SensorsModule : public Module
{
protected:
    volatile int dummy = 0;

public:
    virtual int getDummy() { return dummy; };
    virtual void toggleDummy(){};
};

class HILSensors : public SensorsModule
{
public:
    void toggleDummy() override
    {
        dummy = dummy == 0 ? 2000 : 0;
        printf("Test2\n");
    }
};

class Sensors : public SensorsModule
{
public:
    void toggleDummy() override { dummy = dummy == 0 ? 1000 : 0; }
};

class Radio : public Module
{
    volatile int dummy = 0;

public:
    void setDummy(int p) { dummy = p; }
    int getDummy() { return dummy; }
};
}  // namespace Boardcore

int main()
{
    // Create the print logger to print the debug
    PrintLogger logger = Logging::getLogger("TestModuleManager");

    LOG_INFO(logger, "Inserting the HILSensors module");

    // Insert the instance inside the module manager
    if (!ModuleManager::getInstance().insert<SensorsModule>(new HILSensors()))
    {
        LOG_ERR(logger, "Error inserting the HILSensors module");
        return -1;
    }

    LOG_INFO(logger, "Try inserting an already existing module");

    // Useless insertion because of already existing module
    if (ModuleManager::getInstance().insert<SensorsModule>(new Sensors()))
    {
        LOG_ERR(logger, "Error, already inserted module insertion accepted");
        return -1;
    }

    // First correct insertion using the same class
    ModuleManager::getInstance().insert<Radio>(new Radio());

    LOG_INFO(logger, "Checking the default values");

    // Check the output
    if (ModuleManager::getInstance().get<SensorsModule>()->getDummy() != 0 ||
        ModuleManager::getInstance().get<Radio>()->getDummy() != 0)
    {
        LOG_ERR(logger,
                "Error with default values, they should 0 whereas they are: "
                "{:d} {:d}",
                ModuleManager::getInstance().get<SensorsModule>()->getDummy(),
                ModuleManager::getInstance().get<Radio>()->getDummy());
        return -1;
    }

    LOG_INFO(logger, "Toggling the SensorsModule value");
    // Toggle the sensors module and the output should be 2000 because HIL
    // sensors
    ModuleManager::getInstance().get<SensorsModule>()->toggleDummy();

    // Check the Radio didn't change
    if (ModuleManager::getInstance().get<Radio>()->getDummy() != 0)
    {
        LOG_ERR(logger,
                "Error with Radio value. It should be 0 "
                "whereas it is: {:d}",
                ModuleManager::getInstance().get<Radio>()->getDummy());
    }

    // Check the output of the toggle
    if (ModuleManager::getInstance().get<SensorsModule>()->getDummy() != 2000)
    {
        LOG_ERR(logger,
                "Error with SensorsModule value. It should be 2000 "
                "whereas it is: {:d}",
                ModuleManager::getInstance().get<SensorsModule>()->getDummy());
        return -1;
    }

    LOG_INFO(logger, "Toggling the SensorsModule value");
    LOG_INFO(logger, "Set the Radio module value to 1000");
    // Toggle the sensors module again and toggle the radio module
    ModuleManager::getInstance().get<SensorsModule>()->toggleDummy();
    ModuleManager::getInstance().get<Radio>()->setDummy(1000);

    // Check the SensorsModule returned to 0
    if (ModuleManager::getInstance().get<SensorsModule>()->getDummy() != 0)
    {
        LOG_ERR(logger,
                "Error with SensorsModule value. It should be 0 "
                "whereas it is: {:d}",
                ModuleManager::getInstance().get<Radio>()->getDummy());
    }

    // Check that the radio didn't change
    if (ModuleManager::getInstance().get<Radio>()->getDummy() != 1000)
    {
        LOG_ERR(logger,
                "Error with Radio value. It should be 1000 "
                "whereas it is: {:d}",
                ModuleManager::getInstance().get<Radio>()->getDummy());
        return -1;
    }

    LOG_INFO(logger, "Test completed with success!\n");
    LOG_INFO(logger,
             "Now testing the performance compared to a Singleton instance");

    return 0;
}
