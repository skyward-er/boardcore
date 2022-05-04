/* Copyright (c) 2020-2022 Skyward Experimental Rocketry
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

#include <sensors/SensorData.h>
#include <sensors/calibration/AxisOrientation.h>
#include <sensors/calibration/Calibration.h>
#include <sensors/calibration/SensorDataExtra.h>

#include <Eigen/Core>

namespace Boardcore
{

/**
 * @brief Applies an offset to the data
 *
 * It stores a vector that will be added to every measurement.
 * Note: The type T must implement >> and << operators with a Vector3f.
 *
 * @tparam T Type used for data items.
 */
template <typename T>
class BiasCorrector : public ValuesCorrector<T>
{
public:
    BiasCorrector();

    BiasCorrector(const Eigen::Vector3f& bias);

    void operator>>(Eigen::Vector3f& rhs);

    void operator<<(const Eigen::Vector3f& rhs);

    void setIdentity() override;

    T correct(const T& data) const override;

private:
    Eigen::Vector3f bias = Eigen::Vector3f::Zero();
};

/**
 * @brief This is the dumbest type of calibration possible: an offset.
 *
 * During the calibration phase it will use a given reference vector (for
 * example the gravitational acceleration), and every time you'll feed the model
 * with a new value, you have to give it the orientation of the sensor, so it
 * can guess the bias.
 *
 * @tparam T
 */
template <typename T>
class BiasCalibration
    : public AbstractCalibration<T, BiasCorrector<T>, AxisOrientation>
{
public:
    BiasCalibration();

    void setReferenceVector(Eigen::Vector3f vec);

    Eigen::Vector3f getReferenceVector();

    bool feed(const T& measured, const AxisOrientation& transform) override;

    bool feed(const T& measured);

    BiasCorrector<T> computeResult();

private:
    Eigen::Vector3f sum, ref;
    unsigned numSamples;
};

template <typename T>
BiasCorrector<T>::BiasCorrector()
{
}

template <typename T>
BiasCorrector<T>::BiasCorrector(const Eigen::Vector3f& bias) : bias(bias)
{
}

template <typename T>
void BiasCorrector<T>::operator>>(Eigen::Vector3f& rhs)
{
    rhs = bias;
}

template <typename T>
void BiasCorrector<T>::operator<<(const Eigen::Vector3f& rhs)
{
    bias = rhs;
}

template <typename T>
void BiasCorrector<T>::setIdentity()
{
    bias = {0, 0, 0};
}

template <typename T>
T BiasCorrector<T>::correct(const T& data) const
{
    T output;
    Eigen::Vector3f tmp;

    data >> tmp;
    tmp += bias;
    output << tmp;

    return output;
}

template <typename T>
BiasCalibration<T>::BiasCalibration()
    : sum(0, 0, 0), ref(0, 0, 0), numSamples(0)
{
}

template <typename T>
void BiasCalibration<T>::setReferenceVector(Eigen::Vector3f vec)
{
    ref = vec;
}

template <typename T>
Eigen::Vector3f BiasCalibration<T>::getReferenceVector()
{
    return ref;
}

/**
 * BiasCalibration accepts an indefinite number of samples,
 * so feed(...) never returns false.
 */
template <typename T>
bool BiasCalibration<T>::feed(const T& measured,
                              const AxisOrientation& transform)
{
    Eigen::Vector3f vec;
    measured >> vec;

    sum += (transform.getMatrix().transpose() * ref) - vec;
    numSamples++;

    return true;
}

template <typename T>
bool BiasCalibration<T>::feed(const T& measured)
{
    return feed(measured, AxisOrthoOrientation());
}

template <typename T>
BiasCorrector<T> BiasCalibration<T>::computeResult()
{
    if (numSamples == 0)
        return {Eigen::Vector3f{0, 0, 0}};
    return {sum / numSamples};
}

}  // namespace Boardcore
