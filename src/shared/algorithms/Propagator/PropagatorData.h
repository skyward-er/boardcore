/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <algorithms/NAS/NASState.h>

#include <Eigen/Core>
#include <ostream>
#include <userde.hpp>

namespace Boardcore
{

/**
 * @brief State of the propagator, taking into account the prediction steps (0
 * if true NAS state) and the propagated NAS
 * @note Can be logged.
 *
 */
struct PropagatorState
{
    uint64_t timestamp;      ///< Prediction timestamp (ARP timestamp) [ms]
    uint32_t nPropagations;  ///< Predictions from last received NAS state
    NASState nas;

    float az                  = 0;
    static constexpr float ax = 0,
                           ay = 0;  ///< only az is used by the propagator
    PropagatorState() : timestamp(0), nPropagations(0), nas() {}

    PropagatorState(uint64_t timestamp, uint32_t nPropagations,
                    NASState nasState)
        : timestamp(timestamp), nPropagations(nPropagations), nas(nasState)
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            PropagatorState,
            FIELD_DEF(timestamp) FIELD_DEF(nPropagations) FIELD_DEF2(nas, n)
                FIELD_DEF2(nas, e) FIELD_DEF2(nas, d) FIELD_DEF2(nas, vn)
                    FIELD_DEF2(nas, ve) FIELD_DEF2(nas, vd) FIELD_DEF2(nas, qx)
                        FIELD_DEF2(nas, qy) FIELD_DEF2(nas, qz) FIELD_DEF2(
                            nas, qw) FIELD_DEF2(nas, bx) FIELD_DEF2(nas, by)
                            FIELD_DEF2(nas, bz) FIELD_DEF(az));
    }

    NASState getNasState() const { return nas; }

    /**
     * @brief Getter for the vector of positions NED
     *
     * @return Eigen::Vector3f the NED position vector
     */
    Eigen::Vector3f getPosition()
    {
        return Eigen::Vector3f(nas.n, nas.e, nas.d);
    }

    /**
     * @brief Setter for the vector of positions NED
     */
    void setPosition(Eigen::Vector3f xProp)
    {
        nas.n = xProp(0);
        nas.e = xProp(1);
        nas.d = xProp(2);
    }

    /**
     * @brief Getter for the vector of velocities NED
     *
     * @return Eigen::Vector3f the NED velocities vector
     */
    Eigen::Vector3f getVelocity()
    {
        return Eigen::Vector3f(nas.vn, nas.ve, nas.vd);
    }

    /**
     * @brief Setter for the vector of velocities NED
     */
    void setVelocity(Eigen::Vector3f vProp)
    {
        nas.vn = vProp(0);
        nas.ve = vProp(1);
        nas.vd = vProp(2);
    }

    /**
     * @brief Setter for the vector acceleration(only z-axis)
     */
    void setZAcceleration(Eigen::Vector3f acc) { az = acc(2); }

    /**
     * @brief Getter for the vector acceleration
     *
     * @return Eigen::Vector3f acceleration
     */
    Eigen::Vector3f getAcceleration() const
    {
        Eigen::Vector3f acc;
        acc(0) = ax;
        acc(1) = ay;
        acc(2) = az;
        return acc;
    }

    /**
     * @brief Getter for the vector of quaternions
     *
     * @return Eigen::Vector3f the quaternions vector
     */
    Eigen::Vector4f getQProp()
    {
        return Eigen::Vector4f(nas.qx, nas.qy, nas.qz, nas.qw);
    }

    /**
     * @brief Setter for the vector of quaternions
     */
    void setQProp(Eigen::Vector4f qProp)
    {
        nas.qx = qProp(0);
        nas.qy = qProp(1);
        nas.qz = qProp(2);
        nas.qw = qProp(3);
    }

    /**
     * @brief Getter for the vector of quaternions' bias
     *
     * @return Eigen::Vector3f the quaternions' bias vector
     */
    Eigen::Vector3f getBProp() { return Eigen::Vector3f(nas.n, nas.e, nas.d); }

    /**
     * @brief Setter for the vector of quaternions' bias
     */
    void setBProp(Eigen::Vector3f bProp)
    {
        nas.bx = bProp(0);
        nas.by = bProp(1);
        nas.bz = bProp(2);
    }
};

}  // namespace Boardcore

