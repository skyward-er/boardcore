/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <catch2/catch.hpp>
#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;

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
    void toggleDummy() override { dummy = dummy == 0 ? 2000 : 0; }
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

TEST_CASE("ModuleManager - Insert module")
{
    ModuleManager& modules = ModuleManager::getInstance();
    Sensors* sensors       = new Sensors();

    REQUIRE(modules.insert<SensorsModule>(sensors));
    REQUIRE_FALSE(modules.get<SensorsModule>() == nullptr);

    // Perform a toggle of the module and read according to the expected number
    modules.get<SensorsModule>()->toggleDummy();
    REQUIRE(modules.get<SensorsModule>()->getDummy() == 1000);
}

TEST_CASE("ModuleManager - Insert already existent module")
{
    ModuleManager& modules = ModuleManager::getInstance();
    Sensors* sensors       = new Sensors();
    HILSensors* hil        = new HILSensors();

    REQUIRE(modules.insert<SensorsModule>(sensors));
    REQUIRE_FALSE(modules.insert<SensorsModule>(hil));

    // Verify that the initially inserted module didn't change
    modules.get<SensorsModule>()->toggleDummy();
    REQUIRE(modules.get<SensorsModule>()->getDummy() == 1000);
}

TEST_CASE("ModuleManager - Get without insertion")
{
    ModuleManager& modules = ModuleManager::getInstance();

    REQUIRE(modules.get<SensorsModule>() == nullptr);
}

TEST_CASE("ModuleManager - Insertion after get call")
{
    ModuleManager& modules = ModuleManager::getInstance();
    Sensors* sensors       = new Sensors();
    Radio* radio           = new Radio();

    REQUIRE(modules.insert<Radio>(radio));

    // Trigger the inhibition of further insertions
    REQUIRE_FALSE(modules.get<Radio>() == nullptr);
    REQUIRE_FALSE(modules.insert<Sensors>(sensors));

    REQUIRE_FALSE(modules.get<Radio>() == nullptr);

    // Verify that the initially inserted module didn't change
    modules.get<Radio>()->setDummy(345);
    REQUIRE(modules.get<Radio>()->getDummy() == 345);
}