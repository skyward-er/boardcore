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

namespace Boardcore
{

/**
 * @brief Struct containing the state of the propagator. Stores the timestamp of
 * the propagation, the number of propagations since the last NAS rocket packet
 * and the propagated NAS state already divided in significant vectors so that
 * computations with the NAS state are easier.
 * It also exposes a method to retrieve the propagated NAS state as a NASState
 * structure.
 */
struct PropagatorState
{
    uint64_t timestamp;      ///< Prediction timestamp [ms]
    uint32_t nPropagations;  ///< Predictions from last received NAS state
    Eigen::Vector3f x_prop;  ///< Position propagation state NED [m]
    Eigen::Vector3f v_prop;  ///< Speed propagation state NED [m]
    Eigen::Vector4f q_prop;  ///< Quaternion propagation (scalar last)
    Eigen::Vector3f b_prop;  ///< Gyroscope bias propagation

    PropagatorState()
        : timestamp(0), nPropagations(0), x_prop(0, 0, 0), v_prop(0, 0, 0),
          q_prop(0, 0, 0, 1), b_prop(0, 0, 0)
    {
    }

    PropagatorState(const PropagatorState& newState)
        : PropagatorState(newState.timestamp, newState.nPropagations,
                          newState.x_prop, newState.v_prop, newState.q_prop,
                          newState.b_prop)
    {
    }

    PropagatorState(uint64_t timestamp, uint32_t nPropagations,
                    Eigen::Vector3f x_prop, Eigen::Vector3f v_prop,
                    Eigen::Vector4f q_prop, Eigen::Vector3f b_prop)
        : timestamp(timestamp), nPropagations(nPropagations), x_prop(x_prop),
          v_prop(v_prop), q_prop(q_prop), b_prop(b_prop)
    {
    }

    PropagatorState(uint64_t timestamp, uint32_t nPropagations,
                    NASState nasState)
        : timestamp(timestamp), nPropagations(nPropagations),
          x_prop(nasState.n, nasState.e, nasState.d),
          v_prop(nasState.vn, nasState.ve, nasState.vd),
          q_prop(nasState.qx, nasState.qy, nasState.qz, nasState.qw),
          b_prop(nasState.bx, nasState.by, nasState.bz)
    {
    }

    static std::string header()
    {
        return "timestamp,nPropagations,n,e,d,vn,ve,vd,qx,qy,qz,qw,bx,by,bz\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << nPropagations << "," << x_prop(0) << ","
           << x_prop(1) << "," << x_prop(2) << "," << v_prop(0) << ","
           << v_prop(1) << "," << v_prop(2) << "," << q_prop(0) << ","
           << q_prop(1) << "," << q_prop(2) << "," << q_prop(3) << ","
           << b_prop(0) << "," << b_prop(1) << "," << b_prop(2) << "\n";
    }

    NASState getNasState() const
    {
        Eigen::Matrix<float, 13, 1> nasState;
        // cppcheck-suppress constStatement
        nasState << x_prop, v_prop, q_prop, b_prop;
        return NASState(timestamp, nasState);
    }
};

}  // namespace Boardcore
