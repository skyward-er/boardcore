/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Nicolò Caruso, Davide Mor
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

#include <vector>

#include "RegistryTypes.h"

namespace Boardcore
{

/**
 * @brief Registry Backend class used to save and load data to the designated
 * storage/memory.
 */
class RegistryBackend
{
public:
    /**
     * @brief Starts the backend, eventually used in backends that
     * need to start classes and other things e.g. an ActiveObject
     *
     * @return true if successful, false otherwise
     */
    virtual bool start() = 0;

    /**
     * @brief Loads into the buffer the saved configuration from the
     * storage/memory.
     *
     * @param buf The buffer where the data will be loaded from the
     * storage/memory if any is saved.
     *
     * @return true if successful, false otherwise.
     */
    virtual bool load(std::vector<uint8_t>& buf) = 0;

    /**
     * @brief Saves the data in the buf to the storage/memory.
     *
     * @param buf The buf vector with the data to be saved.
     *
     * @return true if successful, false otherwise.
     */
    virtual bool save(std::vector<uint8_t>& buf) = 0;
};

/**
 * @brief Dummy no-op backend
 */
class DummyBackend final : public RegistryBackend
{
public:
    bool start() override { return true; }

    bool load(std::vector<uint8_t>& buf) override { return true; }

    bool save(std::vector<uint8_t>& buf) override { return true; }
};

}  // namespace Boardcore
