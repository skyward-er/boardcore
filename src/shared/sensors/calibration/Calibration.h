/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "SensorDataExtra.h"
#include "sensors/Sensor.h"

using namespace Eigen;

/**
 * This class is used to adjust the values given by sensors during the flight.
 * An object can be obtained only via deserialization or if produced by an
 * instance of the "CalibrationModel" class.
 *
 * T is the type of sensor data the model is applied to (e.g.
 * GyroscopeData, MagnetometerData). This is needed because certain models could
 * work slightly differently depending on it
 *
 *  Note: derived classes of ValuesCorrector<XX> MUST implement the following
 *  operators (to make possible to store and load the coefficients):
 *
 *  operator << (const XX& t);
 *  operator >> (XX& t);
 *
 *  where XX is a datatype that can fully contain all the coefficients used
 *  by the function correct(input).
 *
 *  Also, an empty constructor must create a neutral instance (identity
 * transformation).
 */
template <typename T>
class ValuesCorrector
{
public:
    /**
     * This method will sets the internal coefficients so that the corrected
     * values are exactly the same of the inputted ones.
     */
    virtual void setIdentity() = 0;

    virtual T correct(const T& input) const = 0;
};

/**
 * AbstractCalibrationModel represents a "factory" of ValuesCorrector instances,
 * and it's necessary to create one. You will always use one of its derived
 * classes, of course.
 *
 * Values given to the feed() function are needed for the training of the model.
 *
 * T is the sensor data type
 * C is a class that extends ValuesCorrector<T>
 */
template <typename T, typename C, typename... AdditionalFeedParams>
class AbstractCalibrationModel
{
public:
    /**
     * Gives to the model a single measurement to store and use to produce the
     * adapted ValueCorrector instance
     *
     * @returns false if the model can't accept the given data (usually because
     * the internal buffers are full)
     */
    virtual bool feed(const T& measurement,
                      const AdditionalFeedParams&... params) = 0;

    /**
     * Creates the best ValuesCorrector instance for the given measurements.
     * Note: you must feed some data to the model before getting the result!
     */
    virtual C computeResult() = 0;

    virtual ~AbstractCalibrationModel(){};
};

/**
 * This class acts like a Sensor driver but incorporates both a Sensor<T>
 * instance and a ValuesCorrector. It can be useful to add a calibration step to
 * alredy existing code that uses the Sensor API.
 */
template <typename SensorData>
class SensorWrapper : public Sensor<SensorData>
{
    using S = Sensor<SensorData>;
    using C = ValuesCorrector<SensorData>;

public:
    SensorWrapper(S* _sensor, C* _calib) : sensor(_sensor), calib(_calib) {}

    S* getSensor() { return sensor; }
    void setSensor(S* s) { sensor = s; }

    C* getValuesCorrector() { return calib; }
    void setValuesCorrector(C* c) { calib = c; }

    void init() override { sensor->init(); }

    bool test() override { return sensor->test(); }

    SensorData sampleImpl() override
    {
        return calib->correct(sensor->sampleImpl());
    }

    SensorErrors getWrappedSensorError() { return sensor->getLastError(); }

private:
    S* sensor;
    C* calib;
};

/**
 * This enum act like versors towards the chosen axis.
 *
 * X, Y and Z always set according to the right hand rule, so that:
 * X is the index
 * Y is the second finger
 * Z is the thumb
 *
 */
enum class Direction : uint8_t
{
    POSITIVE_X = 0,
    NEGATIVE_X,
    POSITIVE_Y,
    NEGATIVE_Y,
    POSITIVE_Z,
    NEGATIVE_Z,
};

constexpr const char* humanFriendlyDirection[]{
    "North", "South", "East", "West", "Down", "Up",
};

inline Vector3f orientationToVector(Direction val)
{
    switch (val)
    {
        case Direction::POSITIVE_X:
            return {1, 0, 0};
        case Direction::NEGATIVE_X:
            return {-1, 0, 0};
        case Direction::POSITIVE_Y:
            return {0, 1, 0};
        case Direction::NEGATIVE_Y:
            return {0, -1, 0};
        case Direction::POSITIVE_Z:
            return {0, 0, 1};
        case Direction::NEGATIVE_Z:
            return {0, 0, -1};
        default:
            /* never happens, added just to shut up the warnings */
            return {0, 0, 0};
    }
}

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
 * AxisOrthoOrientation ortho  ( Direction::NEGATIVE_X,
 * Direction::POSITIVE_Z );
 *
 * // The implicit cast is supported and recommended
 * AxisOrientation converted1 = angles;
 * AxisOrientation converted2 = ortho;
 *
 * // Now we can use the generated matrix:
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
    Direction xAxis, yAxis;

    AxisOrthoOrientation()
        : xAxis(Direction::POSITIVE_X), yAxis(Direction::POSITIVE_Y)
    {
    }

    AxisOrthoOrientation(Direction _xAxis, Direction _yAxis)
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

/**
 * This struct represents axis orientation relative to a reference system.
 * Operatively it simply combines two transformations. It is particularly useful
 * to obtain an AxisOrientation from a reference system generally not N.E.D.
 */
struct AxisRelativeOrientation
{
    AxisOrientation systemOrientation, orientation;

    AxisRelativeOrientation(const AxisOrientation& _systemOrientation,
                            const AxisOrientation& _orientation)
        : systemOrientation(_systemOrientation), orientation(_orientation)
    {
    }

    operator AxisOrientation() const { return {getMatrix()}; }

    Matrix3f getMatrix() const
    {
        return systemOrientation.getMatrix() * orientation.getMatrix();
    }
};
