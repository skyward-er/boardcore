/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <algorithms/NAS/NASState.h>
#include <mavlink_lib/pyxis/mavlink.h>

namespace Boardcore
{

struct NASStateWrapper
{
    NASState nasState;

    NASStateWrapper() {}
    NASStateWrapper(const NASState& _nasState) : nasState(_nasState) {}
    NASStateWrapper(NASState&& _nasState) : nasState(std::move(_nasState)) {}

    bool unpack(const mavlink_message_t& message)
    {
        if (message.msgid != MAVLINK_MSG_ID_NAS_TM)
            return false;

        // Position [m]
        nasState.n = mavlink_msg_nas_tm_get_nas_n(&message);
        nasState.e = mavlink_msg_nas_tm_get_nas_e(&message);
        nasState.d = mavlink_msg_nas_tm_get_nas_d(&message);

        // Velocity [m/s]
        nasState.vn = mavlink_msg_nas_tm_get_nas_vn(&message);
        nasState.ve = mavlink_msg_nas_tm_get_nas_ve(&message);
        nasState.vd = mavlink_msg_nas_tm_get_nas_vd(&message);

        // Attitude as quaternion
        nasState.qx = mavlink_msg_nas_tm_get_nas_qx(&message);
        nasState.qy = mavlink_msg_nas_tm_get_nas_qy(&message);
        nasState.qz = mavlink_msg_nas_tm_get_nas_qz(&message);
        nasState.qw = mavlink_msg_nas_tm_get_nas_qw(&message);

        // Gyroscope bias
        nasState.bx = mavlink_msg_nas_tm_get_nas_bias_x(&message);
        nasState.by = mavlink_msg_nas_tm_get_nas_bias_y(&message);
        nasState.bz = mavlink_msg_nas_tm_get_nas_bias_z(&message);

        return true;
    }

    mavlink_message_t pack(uint8_t system_id, uint8_t component_id,
                           uint64_t timestamp, uint8_t state,
                           float ref_pressure, float ref_temperature,
                           float ref_latitude, float ref_longitude) const
    {
        mavlink_message_t message;
        mavlink_msg_nas_tm_pack(
            system_id, component_id, &message, timestamp, state, nasState.n,
            nasState.e, nasState.d, nasState.vn, nasState.ve, nasState.vd,
            nasState.qx, nasState.qy, nasState.qz, nasState.qw, nasState.bx,
            nasState.by, nasState.bz, ref_pressure, ref_temperature,
            ref_latitude, ref_longitude);
        return message;
    }

    const NASState& getNASState() const { return nasState; }
    void setNASState(const NASState& _nasState) { nasState = _nasState; }
    void setNASState(NASState&& _nasState) { nasState = std::move(_nasState); }

    operator NASState() { return nasState; }
};

}  // namespace Boardcore