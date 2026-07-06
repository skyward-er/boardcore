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

using namespace Eigen;

namespace Boardcore
{

StateInitializer::StateInitializer() {}

Vector4f StateInitializer::eCompass(const Vector3f& acc, const Vector3f& mag)
{
    // ndr: since this method runs only when the rocket is stationary, there's
    // no need to add the gravity vector because the accelerometers already
    // measure it. This is not true if we consider the flying rocket.

    Vector3f am(acc.cross(mag));

    Matrix3f R;
    // cppcheck-suppress constStatement
    R << am.cross(acc), am, acc;
    R.col(0).normalize();
    R.col(1).normalize();
    R.col(2).normalize();

    Vector4f x_quat = SkyQuaternion::rotationMatrix2quat(R);

    return x_quat;
}

Vector4f StateInitializer::triad(const Vector3f& acc, const Vector3f& mag,
                                 const Vector3f& nedMag)
{
    Vector3f nedAcc(0.0f, 0.0f, 1.0f);

    // Rotate the NED magnetic vector from right-hand to left-hand
    Matrix3f magSXRot;
    magSXRot << 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f;

    // Apply rotation to the input `nedMag`
    Vector3f nedMagRot = magSXRot * nedMag;

    // Compute the reference triad
    Vector3f R1 = nedAcc;
    Vector3f R2 = nedAcc.cross(nedMagRot).normalized();
    Vector3f R3 = R1.cross(R2);

    // Compute the measured triad
    Vector3f r1 = -acc;
    Vector3f r2 = -acc.cross(mag).normalized();
    Vector3f r3 = r1.cross(r2);

    // Compose the reference and measured matrixes
    Matrix3f M;
    // cppcheck-suppress constStatement
    M << R1, R2, R3;
    Matrix3f m;
    // cppcheck-suppress constStatement
    m << r1, r2, r3;

    // Compute the rotation matrix and the corresponding quaternion
    Matrix3f A = m * M.transpose();
    Vector4f q = SkyQuaternion::rotationMatrix2quat(A);

    // Switch from scalar-last to scalar-first convention
    Vector4f quat;
    quat << q(3), q(0), q(1), q(2);

    return quat;
}

}  // namespace Boardcore
