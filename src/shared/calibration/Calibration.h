/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Riccardo Musso
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

#pragma once

#include "math/Vec3.h"
#include "math/Matrix.h"

/**
 * This class generalizes all the Calibration classes used to
 * adjust the values read from sensors. It provides a standard interface
 * that tries to integrate well with the new Sensor and SensorData structures.
 *
 * PackedModel is a datatype that could encapsulate all parameters of the model
 * (e.g. Mat4, a std::array of bytes or a custom struct). Choose it carefully
 * because it will be used for the serialization/deserialization of the model.
 *
 * SensorData is the type of the objects the model is applied to (e.g.
 * GyroscopeData, MagnetometerData). This is needed because certain models could
 * work slightly differently depending on it
 */
template <typename PackedModel, typename SensorData, typename... FeedParams>
class AbstractCalibrationModel
{
public:
    virtual void store(PackedModel& output) const = 0;

    virtual void load(const PackedModel& input) = 0;

    virtual void resetToIdentity() = 0;

    virtual void startCalibrationStage() = 0;

    virtual void feed(const FeedParams&... p) = 0;

    virtual void endCalibrationStage() = 0;

    virtual SensorData correct(const SensorData& input) const = 0;
};

/*
 * X, Y and Z always set according to the right hand rule, so that:
 * X is the index
 * Y is the second finger
 * Z is the thumb
 *
 * In the base position, we call "up" the direction of the Z axis, and
 * the four cardinal directions lay on the XY plane (X is "south", Y is "east")
 */
enum class Orientation
{
    UP,
    DOWN,
    NORTH,
    WEST,
    EAST,
    SOUTH
};

Vec3 orientationToVector(Orientation val){
    switch(val){
        case Orientation::UP:
            return Vec3(0.f, 0.f, 1.f);
        case Orientation::DOWN:
            return Vec3(0.f, 0.f, -1.f);
        case Orientation::NORTH:
            return Vec3(-1.f, 0.f, 0.f);
        case Orientation::WEST:
            return Vec3(0.f, -1.f, 0.f);
        case Orientation::EAST:
            return Vec3(0.f, 1.f, 0.f);
        case Orientation::SOUTH:
            return Vec3(1.f, 0.f, 0.f);
    }
}
/*
 * If we know the orientation of the X and Z axis, using the right hand rule
 * we can infer the Y axis
 */
struct AxisOrthoOrientation
{
    Orientation x = Orientation::SOUTH, z = Orientation::UP;

    Mat3 getMatrix() const {
        Vec3 vx, vy, vz;
        
        vx = orientationToVector(x);
        vz = orientationToVector(z);
        vy = vz.cross(vx);
        
        Mat3 mat;
        mat.setVertComponents(vx, vy, vz); 
        return mat;
    }
};

