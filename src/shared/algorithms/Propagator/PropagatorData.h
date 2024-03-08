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

#include <ostream>

namespace Boardcore
{

struct PropagatorState
{
    uint64_t timestamp;
    uint32_t nPropagations;  ///< Predictions from last received NAS state
    Vector3f x_prop;         ///< Position propagation state ENU [m]
    Vector3f v_prop;         ///< Speed propagation state ENU [m]
    Vector4f q_prop;         ///< Quaternion propagation (scalar last)
    Vector3f b_prop;         ///< Gyroscope bias propagation

    PropagatorState()
        : timestamp(0), nPropagations(0), x_prop(0, 0, 0), v_prop(0, 0, 0),
          q_prop(0, 0, 0, 1), b_prop(0, 0, 0)
    {
    }

    PropagatorState(uint64_t timestamp, uint32_t nPropagations, Vector3f x_prop,
                    Vector3f v_prop, Vector4f q_prop, Vector3f b_prop)
        : timestamp(timestamp), nPropagations(nPropagations), x_prop(x_prop),
          v_prop(v_prop), q_prop(q_prop), b_prop(b_prop)
    {
    }

    PropagatorState(uint64_t timestamp, uint32_t nPropagations,
                    NASState nasState)
        : timestamp(timestamp), nPropagations(nPropagations)
    {
        x_prop = Vector3f(nasState.n, nasState.e, nasState.d);
        v_prop = Vector3f(nasState.vn, nasState.ve, nasState.vd);
        q_prop = Vector3f(nasState.qx, nasState.qy, nasState.qz, nasState.qw);
        b_prop = Vector3f(nasState.bx, nasState.by, nasState.bz);
    }

    static std::string header() { return "timestamp,nPropagations\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << nPropagations << "\n";
    }

    NASState getNasState()
    {
        Eigen::Matrix<float, 13, 1> nasState;
        nasState << x_prop, v_prop, q_prop, b_prop;
        return NASState(timestamp, nasState);
    }
};

}  // namespace Boardcore
