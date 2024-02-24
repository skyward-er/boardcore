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
#include <ActiveObject.h>
#include <stdint.h>
#include <utils/Debug.h>

#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

constexpr uint32_t entriesReserve = 40;

namespace Boardcore
{
/**
 * @brief Write buffer structs wraps an std::vector<uint8_t> with also its mutex
 * and a flag for specify if it changed from last write.
 */
struct WriteBuffer
{
    std::vector<uint8_t> vector; /*< vector with serialized data*/
    std::recursive_mutex mutex;
    bool needsWrite; /*< True if it is needed a write to the backend */
};

/**
 * @brief Middleware class for saving the buffer to backend. It decouples
 * frontend and backend such that the frontend is blocked just while copying the
 * vector to the middleware while the middleware thread is blocked for the real
 * write to backend.
 *
 */
class RegistryMiddleware : public ActiveObject
{
protected:
    WriteBuffer mainBuffer, secondaryBuffer;
    std::mutex mutexBuffers;
    bool
        updateMainBuffer; /**< True if updates the main buffer while writing the
                           *  secondary and vice-versa, modified by write() only
                           */
    std::thread workerThread;
    bool needsToWrite;
    std::condition_variable writeCondition;

public:
    RegistryMiddleware() : ActiveObject()
    {
        mainBuffer.needsWrite      = false;
        secondaryBuffer.needsWrite = false;
        mainBuffer.vector.reserve(entriesReserve);
        secondaryBuffer.vector.reserve(entriesReserve);
        needsToWrite     = false;
        updateMainBuffer = true;
    };

    /**
     * @brief Copies the vector to the buffer for write to the backend
     *
     * @param vector The vector to be written to backend
     */
    virtual void write(std::vector<uint8_t>& vector) = 0;

    /**
     * @brief Loads the configuration into the vector, if any configuration
     * exists in the underlying backend.
     *
     * @param vector The vector where the configuration is loaded
     * @return true If the vector has been loaded with a configuration
     * @return false Otherwise
     */
    virtual bool load(std::vector<uint8_t>& vector) = 0;

    /**
     * @brief Clears the buffers and the underlying backend saved configuration
     *
     */
    virtual void clear() = 0;
};

class RegistryMiddlewareFlash : public RegistryMiddleware
{
public:
    RegistryMiddlewareFlash();
    /**
     * @brief Copies the vector to the buffer for write to the backend
     *
     * @param vector The vector to be written to backend
     */
    virtual void write(std::vector<uint8_t>& vector) override;

    /**
     * @brief Executed by the internal thread for write the buffers to backend
     */
    virtual void run() override;

    /**
     * @brief Loads the configuration into the vector, if any configuration
     * exists in the underlying backend.
     *
     * @param vector The vector where the configuration is loaded
     * @return true If the vector has been loaded with a configuration
     * @return false Otherwise
     */
    virtual bool load(std::vector<uint8_t>& vector) override;

    /**
     * @brief Clears the buffers and the underlying backend saved configuration
     *
     */
    virtual void clear() override;
};

}  // namespace Boardcore