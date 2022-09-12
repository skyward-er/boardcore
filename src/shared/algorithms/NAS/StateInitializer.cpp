/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Marco Cella, Alberto Nidasio
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

#include "StateInitializer.h"

namespace Boardcore
{
StateInitializer::StateInitializer() { x_init << Eigen::MatrixXf::Zero(13, 1); }

void StateInitializer::eCompass(const Eigen::Vector3f acc,
                                const Eigen::Vector3f mag)
{
    // ndr: since this method runs only when the rocket is stationary, there's
    // no need to add the gravity vector because the accelerometers already
    // measure it. This is not true if we consider the flying rocket.

    Eigen::Vector3f am(acc.cross(mag));

    Eigen::Matrix3f R;
    // cppcheck-suppress constStatement
    R << am.cross(acc), am, acc;
    R.col(0).normalize();
    R.col(1).normalize();
    R.col(2).normalize();

    Eigen::Vector4f x_quat = SkyQuaternion::rotationMatrix2quat(R);

    x_init(NAS::IDX_QUAT)     = x_quat(0);
    x_init(NAS::IDX_QUAT + 1) = x_quat(1);
    x_init(NAS::IDX_QUAT + 2) = x_quat(2);
    x_init(NAS::IDX_QUAT + 3) = x_quat(3);
}

void StateInitializer::triad(const Eigen::Vector3f& acc,
                             const Eigen::Vector3f& mag,
                             const Eigen::Vector3f& nedMag)
{
    // The gravity vector is expected to be read inversely because
    // accelerometers read the binding reaction
    Eigen::Vector3f nedAcc(0.0f, 0.0f, -1.0f);

    // Compute the reference triad
    Eigen::Vector3f R1 = nedAcc;
    Eigen::Vector3f R2 = nedAcc.cross(nedMag).normalized();
    Eigen::Vector3f R3 = R1.cross(R2);

    // Compute the measured triad
    Eigen::Vector3f r1 = acc;
    Eigen::Vector3f r2 = acc.cross(mag).normalized();
    Eigen::Vector3f r3 = r1.cross(r2);

    // Compose the reference and measured matrixes
    Eigen::Matrix3f M;
    // cppcheck-suppress constStatement
    M << R1, R2, R3;
    Eigen::Matrix3f m;
    // cppcheck-suppress constStatement
    m << r1, r2, r3;

    // Compute the rotation matrix and the corresponding quaternion
    Eigen::Matrix3f A = M * m.transpose();
    Eigen::Vector4f q = SkyQuaternion::rotationMatrix2quat(A);

    // Save the orientation in the state
    x_init.block<4, 1>(NAS::IDX_QUAT, 0) = q;
}

Eigen::Matrix<float, 13, 1> StateInitializer::getInitX() { return x_init; }

}  // namespace Boardcore
