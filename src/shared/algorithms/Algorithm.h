/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

class Algorithm
{
public:
    /**
     * @brief Initializes the Algorithm object, must be called as soon as the
     * object is created.
     * */
    virtual bool init() = 0;

    /**
     * @brief Starts the execution of the algorithm and set the running flag to
     * true.
     * */
    void begin() { running = true; }

    /**
     * @brief Terminates the algorithm's execution and sets the running flag to
     * false.
     * */
    void end() { running = false; }

    /**
     * @brief Checks wether the algorithm is in a running state or not, and
     * eventually calls the @see{step} routine.
     * */
    void update()
    {
        if (running)
        {
            step();
        }
    }

protected:
    /**
     * @brief The actual algorithm step.
     */
    virtual void step() = 0;

    bool running = false;
};
