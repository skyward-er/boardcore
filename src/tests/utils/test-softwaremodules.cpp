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
#include <iostream>
#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;
using namespace std;

namespace Boardcore
{
class SensorsModule : public Module
{
protected:
    int dummy = 0;

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
    int dummy = 0;

public:
    void setDummy(int p) { dummy = p; }
    int getDummy() { return dummy; }
};
}  // namespace Boardcore

int main()
{
    ModuleManager mod{};

    mod.insert<SensorsModule>(new HILSensors());

    // Ignored insertion because a sensorsModule is already present
    mod.insert<SensorsModule>(new Sensors());

    // First correct insertion using the same class
    mod.insert<Radio>(new Radio());

    // Test
    cout << mod.get<SensorsModule>()->getDummy() << ","
         << mod.get<Radio>()->getDummy() << endl;

    mod.get<SensorsModule>()->toggleDummy();

    cout << mod.get<SensorsModule>()->getDummy() << ","
         << mod.get<Radio>()->getDummy() << endl;

    mod.get<Radio>()->setDummy(3000);
    mod.remove<SensorsModule>();

    cout << mod.get<SensorsModule>()->getDummy() << ","
         << mod.get<Radio>()->getDummy() << endl;

    // mod.insert<SensorsModule>(new HILSensors());

    // mod.get<SensorsModule>()->toggleDummy();

    // mod.get<Radio>();

    // cout << mod.get<SensorsModule>()->getDummy() << endl;

    return 0;
}
