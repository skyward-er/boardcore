/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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
#include "RegistryMiddleware.h"

#include <stdint.h>
#include <utils/Debug.h>

#include <vector>

namespace Boardcore
{
/**
 * @brief Copies the vector to the buffer for write to the backend
 *
 * @param vector The vector to be written to backend
 * @return true If the copy was successful
 * @return false Otherwise
 */
void RegistryMiddlewareFlash::write(std::vector<uint8_t>& vector)
{
    std::lock_guard<std::mutex> lockBuffers(mutexBuffers);
    if (updateMainBuffer)
    {
        std::unique_lock<std::recursive_mutex> lockMain(mainBuffer.mutex);
        mainBuffer.vector     = vector;
        mainBuffer.needsWrite = true;
    }
    else
    {
        std::unique_lock<std::recursive_mutex> lockSec(secondaryBuffer.mutex);
        secondaryBuffer.vector     = vector;
        secondaryBuffer.needsWrite = true;
    }
    needsToWrite = true;
    writeCondition.notify_all();
};

/**
 * @brief Execute the internal thread for write the buffers to backend
 *
 */
void RegistryMiddlewareFlash::run()
{
    while (true)
    {
        /*! Waits to write section */
        {
            std::unique_lock<std::mutex> lockBuffers(mutexBuffers);
            while (!needsToWrite)
                writeCondition.wait(lockBuffers);
        }

        /*! Writes to the backend */
        if (updateMainBuffer && secondaryBuffer.needsWrite)
        {
            std::unique_lock<std::recursive_mutex> lockSec(
                secondaryBuffer.mutex);
            /*! TODO: write to backend the secondary buffer vector*/
            secondaryBuffer.needsWrite = false;
            secondaryBuffer.vector.clear();
        }
        else if (!updateMainBuffer && mainBuffer.needsWrite)
        {
            std::unique_lock<std::recursive_mutex> lockMain(mainBuffer.mutex);
            /*! TODO: write to backend the main buffer vector*/
            mainBuffer.needsWrite = false;
            mainBuffer.vector.clear();
        }

        /*! Changes the buffer to be updated*/
        {
            std::lock_guard<std::mutex> lockBuffers2(mutexBuffers);
            /*! Inverts the usable and writing buffer */
            updateMainBuffer = !updateMainBuffer;

            /*! In case the buffer needs to write, needsToWrite = true */
            if (updateMainBuffer)
            {
                std::unique_lock<std::recursive_mutex> lockSec2(
                    secondaryBuffer.mutex);
                if (secondaryBuffer.needsWrite)
                    needsToWrite = true;
            }
            else
            {
                std::unique_lock<std::recursive_mutex> lockMain2(
                    mainBuffer.mutex);
                if (mainBuffer.needsWrite)
                    needsToWrite = true;
            }
        }
    }
};

RegistryMiddlewareFlash::RegistryMiddlewareFlash() : RegistryMiddleware()
{
    needsToWrite = false;
};

/**
 * @brief Loads the configuration into the vector, if any configuration exists
 * in the underlying backend.
 *
 * @param vector The vector where the configuration is loaded
 * @return true If the vector has been loaded with a configuration
 * @return false Otherwise
 */
bool RegistryMiddlewareFlash::load(std::vector<uint8_t>& vector)
{
    /*! TODO: will call the backend for load the configuration to the vector */
    return false;
}

/**
 * @brief Clears the buffers and the underlying backend saved configuration
 *
 */
void RegistryMiddlewareFlash::clear()
{
    std::unique_lock<std::mutex> lockBuffers(mutexBuffers);
    std::unique_lock<std::recursive_mutex> lockMain(mainBuffer.mutex);
    std::unique_lock<std::recursive_mutex> lockSec(secondaryBuffer.mutex);
    mainBuffer.vector.clear();
    mainBuffer.needsWrite = false;
    secondaryBuffer.vector.clear();
    secondaryBuffer.needsWrite = false;
    /*! TODO: clear underlying backend*/
}

}  // namespace Boardcore