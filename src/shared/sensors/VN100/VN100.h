/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <sensors/Sensor.h>
#include "VN100Data.h"
#include "VN100Serial.h"

/**
 * @brief Header class for the VN100 IMU
 */
class VN100 : public Sensor<VN100Data>
{

private:
	
    /**
     * @brief sample action implementation
     */
    VN100Data sampleImpl() override;

public:

    /**
     * @brief Constructor
     */
	VN100();

    /**
     * @brief Init method to initialize the IMU and to set 
     * the user defined working conditions
     * @return Boolean value indicating the operation success state
     */
    bool init() override;

    /**
     * @brief Method to implement the verification process to ensure
     * that the sensor is up and running
     * @return Boolean of the result
     */
    bool selfTest() override;
};