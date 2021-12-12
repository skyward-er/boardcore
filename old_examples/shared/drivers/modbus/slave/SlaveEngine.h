/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
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

#include <interfaces/endianness.h>

#include <memory>

#include "../HooksInterface.h"
#include "../PDU.h"
#include "Common.h"

class SlaveEngine
{
public:
    /**
     * Creates an instance of the engine
     * @param hook pointer to an instance of a HooksInterface class, if not
     * provided the SlaveEngine will use a default instance of HooksInterface
     * class.
     * The instance pointed by @param hook will be deleted when the destructor
     * of SlaveEngine is called
     */
    explicit SlaveEngine(HooksInterface* hook = nullptr);

    ~SlaveEngine();

    /**
     * Process a request received from the master.
     * @param request PDU that contains the master's request
     * @return a PDU with the either the response or the exception message
     */
    std::unique_ptr<PDU> ProcessRequest(std::unique_ptr<PDU> request);

private:
    HooksInterface* handlers;  ///< pointer to the handlers' class

    // Helper functions, one for each function code supported
    PDU* DoReadCoils(uint8_t* data);
    PDU* DoWriteCoil(uint8_t* data);
    PDU* DoWriteMultipleCoils(uint8_t* data);
    PDU* DoReadRegisters(uint8_t* data);
    PDU* DoWriteRegister(uint8_t* data);
    PDU* DoWriteMultipleRegisters(uint8_t* data);

    // Unsupported functions
    SlaveEngine(const SlaveEngine& other) = delete;
    SlaveEngine& operator=(const SlaveEngine& other) = delete;
    bool operator==(const SlaveEngine& other)        = delete;
};
