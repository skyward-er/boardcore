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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "SensorDataExtra.h"

using namespace Eigen;

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
template <typename PackedModel, typename SensorData,
          typename... AdditionalFeedParams>
class AbstractCalibrationModel
{
public:
    virtual void store(PackedModel& output) const = 0;

    virtual void load(const PackedModel& input) = 0;

    virtual void resetToIdentity() = 0;

    virtual void startCalibrationStage() = 0;

    virtual void feed(const SensorData& data,
                      const AdditionalFeedParams&... p) = 0;

    virtual void endCalibrationStage() = 0;

    virtual SensorData correct(const SensorData& input) const = 0;
};

/**
 * X, Y and Z always set according to the right hand rule, so that:
 * X is the index
 * Y is the second finger
 * Z is the thumb
 *
 */
enum class Orientation
{
    POSITIVE_X,
    NEGATIVE_X,
    POSITIVE_Y,
    NEGATIVE_Y,
    POSITIVE_Z,
    NEGATIVE_Z,
};

inline Vector3f orientationToVector(Orientation val)
{
    switch (val)
    {
        case Orientation::POSITIVE_X:
            return {1, 0, 0};
        case Orientation::NEGATIVE_X:
            return {-1, 0, 0};
        case Orientation::POSITIVE_Y:
            return {0, 1, 0};
        case Orientation::NEGATIVE_Y:
            return {0, -1, 0};
        case Orientation::POSITIVE_Z:
            return {0, 0, 1};
        case Orientation::NEGATIVE_Z:
            return {0, 0, -1};
    }
};

/**
 * This struct represents in the most general way any kind of transformation of
 * the reference frame (axis X, Y and Z).
 * This data type is intended to simplify the code, so you shouldn't instantiate
 * this struct directly, but rather use the structures AxisAngleOrientation or
 * AxisOrthoOrientation that will be automatically corrected to this one, thanks
 * to the implicit cast in favor of AxisOrientation.
 *
 * For example:
 *
 * AxisAngleOrientation angles ( PI/2, PI, 0);
 * AxisOrthoOrientation ortho  ( Orientation::NEGATIVE_X,
 * Orientation::POSITIVE_Z );
 *
 * AxisOrientation converted1 = angles; // The implicit cast is supported and
 * recommended AxisOrientation converted2 = ortho;
 *
 * // Now we can use the generated matrix:
 *
 * Vector3f zeta = convertedX.getMatrix() * Vector3f { 0, 0, 1 }
 *
 */
struct AxisOrientation
{
    Matrix3f mat;

    AxisOrientation() : mat(Matrix3f::Identity()) {}

    AxisOrientation(Matrix3f _mat) : mat(_mat) {}

    void setMatrix(Matrix3f _mat) { mat = _mat; }

    Matrix3f getMatrix() const { return mat; }
};

/**
 * This struct uses the three angles yaw, pitch and roll to define the
 * transformation of the reference frame, so according to N.E.D standard we get:
 *
 *         X (north)
 *        /
 *       /
 *      .----> Y (east)
 *      |
 *      |
 *      v
 *     Z (down)
 *
 * Where:
 * Yaw is rotation of Z axis
 * Pitch is rotation of Y axis
 * Roll is rotation of X axis
 */
struct AxisAngleOrientation
{
    float yaw, pitch, roll;

    AxisAngleOrientation() : yaw(0), pitch(0), roll(0) {}

    AxisAngleOrientation(float _yaw, float _pitch, float _roll)
        : yaw(_yaw), pitch(_pitch), roll(_roll)
    {
    }

    operator AxisOrientation() const { return AxisOrientation(getMatrix()); }

    Matrix3f getMatrix() const
    {
        return (AngleAxisf(yaw, Vector3f{0, 0, 1}) *
                AngleAxisf(pitch, Vector3f{0, 1, 0}) *
                AngleAxisf(roll, Vector3f{1, 0, 0}))
            .toRotationMatrix();
    }
};

/**
 * This struct represents the orientation of the reference frame relative
 * to X, Y, Z in the start orientation.
 * If we know the orientation of the X and Y axis, using the right hand rule
 * we can infer the Z axis.
 *
 * For example, if the base reference is:
 *
 *        z
 *        ^
 *        |
 *        |
 *        /----> y
 *       /
 *      x
 *
 * Then if we set x = NEGATIVE_Y, y = POSITIVE_Z, we get:
 *
 *          y   z
 *          ^   ^
 *          |  /
 *          | /
 *   x <----/
 *
 */
struct AxisOrthoOrientation
{
    Orientation xAxis, yAxis;

    AxisOrthoOrientation()
        : xAxis(Orientation::POSITIVE_X), yAxis(Orientation::POSITIVE_Y)
    {
    }

    AxisOrthoOrientation(Orientation _xAxis, Orientation _yAxis)
        : xAxis(_xAxis), yAxis(_yAxis)
    {
    }

    operator AxisOrientation() const { return AxisOrientation(getMatrix()); }

    Matrix3f getMatrix() const
    {
        Vector3f vx, vy, vz;

        vx = orientationToVector(xAxis);
        vy = orientationToVector(yAxis);
        vz = vx.cross(vy);

        Matrix3f mat;
        mat.col(0) << vx;
        mat.col(1) << vy;
        mat.col(2) << vz;
        return mat;
    }
};

