/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <diagnostic/PrintLogger.h>

#include <cstdint>

#include "APIFrames.h"

namespace Boardcore
{

namespace Xbee
{

/**
 * @brief Parses a byte sequence into an Xbee APIFrame
 */
class APIFrameParser
{
public:
    /**
     * @brief Current state of the parser internal state machine
     */
    enum class ParserState
    {
        FIND_START,
        READ_LENGTH_1,
        READ_LENGTH_2,
        READ_FRAME_TYPE,
        READ_FRAME_DATA,
        READ_CHECKSUM
    };

    /**
     * @brief Result of the last parse operation
     *
     */
    enum class ParseResult : uint8_t
    {
        IDLE = 0,  // No frame has been found yet
        PARSING,   // Currently paring a frame
        SUCCESS,   // A frame has been parsed successfully
        FAIL  // The parsed frame was invalid (eg wrong length, wrong checksum)
    };

    APIFrameParser();

    /**
     * @brief Parses a single byte. When this function returns
     * ParseResult:SUCESS, @p frame contains a valid APIFrame
     *
     * @param byte Byte to parse
     * @param frame Frame to be constructed from the parsed data.
     * @return ParseResult If SUCCESS is returned, then the provided frame is
     * valid and can be used
     */
    ParseResult parse(uint8_t byte, APIFrame* frame);

    /**
     * @brief Returns the current state of the parser
     */
    ParserState getParserState() { return parser_state; }

private:
    ParserState parser_state          = ParserState::FIND_START;
    uint16_t current_frame_data_index = 0;

    PrintLogger logger = Logging::getLogger("apiframeparser");
};

}  // namespace Xbee

}  // namespace Boardcore
