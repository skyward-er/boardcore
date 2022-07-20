/* Copyright (c) 2021-2022 Skyward Experimental Rocketry
 * Authors: Riccardo Musso, Alberto Nidasio
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

#include <sensors/Sensor.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Boardcore
{

/**
 * This enum is used to represent directions relative to X, Y and X.
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

inline Eigen::Vector3f orientationToVector(Direction direction)
{
    switch (direction)
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
            // never happens, added just to shut up the warnings
            return {0, 0, 0};
    }
}

/**
 * @brief This struct represents in the most general way any kind of
 * transformation of the reference frame (axis X, Y and Z).
 *
 * This class is abstract, use AxisAngleOrientation, AxisOrthoOrientation or
 * AxisRelativeOrientation.
 */
struct AxisOrientation
{
    virtual Eigen::Matrix3f getMatrix() const = 0;
};

/**
 * @brief This struct uses the three angles yaw, pitch and roll to define a
 * transformation.
 *
 * According to N.E.D standard we get:
 *
 *         ^ X (north)
 *        /
 *       /
 *      .----> Y (east)
 *      |
 *      |
 *      v Z (down)
 *
 * Where:
 *-  Yaw is rotation of Z axis
 * - Pitch is rotation of Y axis
 * - Roll is rotation of X axis
 */
struct AxisAngleOrientation : public AxisOrientation
{
    float yaw, pitch, roll;

    AxisAngleOrientation() : yaw(0), pitch(0), roll(0) {}

    AxisAngleOrientation(float _yaw, float _pitch, float _roll)
        : yaw(_yaw), pitch(_pitch), roll(_roll)
    {
    }

    Eigen::Matrix3f getMatrix() const override
    {
        return (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()))
            .toRotationMatrix();
    }
};

/**
 * @brief This struct represents orthogonal rotations.
 *
 * The orientation is specified by describing the X and Y axis position relative
 * to the absolute frame. Then, using the right hand rule, we can infer the Z
 * axis.
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
 *          ^  ^
 *          | /
 *          |/
 *   x <----/
 */
struct AxisOrthoOrientation : public AxisOrientation
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

    Eigen::Matrix3f getMatrix() const override
    {
        Eigen::Vector3f vx, vy, vz;

        vx = orientationToVector(xAxis);
        vy = orientationToVector(yAxis);
        vz = vx.cross(vy);

        Eigen::Matrix3f mat;
        mat.col(0) << vx;
        mat.col(1) << vy;
        mat.col(2) << vz;
        return mat;
    }
};

/**
 * @brief This struct represents axis orientation relative to a reference
 * system.
 *
 * Operatively it simply combines two transformations. It is particularly useful
 * to obtain an AxisOrientation from a reference system generally not N.E.D.
 */
struct AxisRelativeOrientation : public AxisOrientation
{
    const AxisOrientation &orientationA, &orientationB;

    AxisRelativeOrientation(const AxisOrientation& orientationA,
                            const AxisOrientation& orientationB)
        : orientationA(orientationA), orientationB(orientationB)
    {
    }

    Eigen::Matrix3f getMatrix() const override
    {
        return orientationA.getMatrix() * orientationB.getMatrix();
    }
};

}  // namespace Boardcore
