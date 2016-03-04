/* Sensors Base Classes
 *
 * Copyright (c) 2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci
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

#ifndef SENSORS_H
#define SENSORS_H
#include <Common.h>
#include <math/Vec3.h>
#include <math/Quaternion.h>

class Sensor {
    public:
        virtual bool init() = 0;
        virtual bool selfTest() = 0;
        uint8_t getLastError() const { return last_error; }
    private:
        uint8_t last_error = 0;
};

class GyroSensor : public virtual Sensor {
    public:
        virtual Quaternion getOrientation() = 0;
};

class AccelSensor : public virtual Sensor {
    public:
        virtual Vec3 getSpeed() = 0;
};

class CompassSensor : public virtual Sensor {
    public:
        virtual Vec3 getCompass() = 0;
};

class TemperatureSensor : public virtual Sensor {
    public:
        virtual float getTemperature() = 0;
};

#endif /* ifndef SENSORS_H */
