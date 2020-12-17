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

#include "Calibration.h"
#include "math/Vec3.h"
#include "sensors/SensorData.h"

/**
 * This is the dumbest type of calibration possible: it stores a 3d vector 
 * (called "bias") that will be added to every measurement.
 * During the calibration phase it will use a given reference vector (for example the gravitational
 * acceleration for the accelerometer), and every time you'll feed the model with a new
 * value, you have to give it the orientation of the sensor, so it can guess the bias.
*/
template <typename SensorData>
class BiasCalibration : public AbstractCalibrationModel<Vec3, SensorData, Vec3, AxisOrthoOrientation>
{
public:
    BiasCalibration()
       : bias(0.f, 0.f, 0.f), ref(0.f, 0.f, 1.f) {} 
        
    void setReferenceVector(Vec3 vec){
        ref = vec;
    }

    void store(Vec3& out) const override {
        out = bias;
    }

    void load(const Vec3& in) override {
        bias = in;
    }

    void resetToIdentity() override{
        bias.clear();
    }

    void startCalibrationStage() override{
        specificInit();
        sum.clear();
        numSamples = 0;
    }

    void feed(const Vec3& measured, const AxisOrthoOrientation& ortho) override {
        Mat3 mat;
        ortho.getMatrix().setTranspose(mat);
        sum += (mat * ref) - measured;
        numSamples++;
    }

    void endCalibrationStage() override {
        bias = sum / numSamples;
    }

    SensorData correct(const SensorData& data) const override {
        static_assert(sizeof(SensorData) != sizeof(SensorData), "BiasCalibration still doesn't support the given SensorData data type.");
    }

private:
    Vec3 bias, sum, ref;
    unsigned numSamples;

    void specificInit(){}
};

template <>
void BiasCalibration<AccelerometerData>::specificInit(){
    setReferenceVector({ 0.f, 0.f, -1.f });
}

template <>
AccelerometerData BiasCalibration<AccelerometerData>::correct(const AccelerometerData& input) const {
    AccelerometerData output;
    output.accel_x = bias.getX() + input.accel_x;
    output.accel_y = bias.getY() + input.accel_y;
    output.accel_z = bias.getZ() + input.accel_z;
    return output;
}
