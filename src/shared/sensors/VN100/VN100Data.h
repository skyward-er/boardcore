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


/**
 * @brief data type class
 * @todo use the standard Data types
 */
class VN100Data 
{

private:

    float accel_x;
    float accel_y;
    float accel_z;

public:

    /**
     * @brief Constructor
     */
    VN100Data() : VN100Data(0, 0, 0){}

    /**
     * @brief Constructor with parameters
     */
    VN100Data(float ax, float ay, float az)
    {
        accel_x = ax;
        accel_y = ay;
        accel_z = az;
    }

    /**
     * @brief X acceleration getter
     */
    float getAccelX() { return accel_x; }

    /**
     * @brief Y acceleration getter
     */
    float getAccelY() { return accel_y; }

    /**
     * @brief Z acceleration getter
     */
    float getAccelZ() { return accel_z; }
};