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

#include "APIFrameParser.h"

#include "Debug.h"

namespace Xbee
{

APIFrameParser::APIFrameParser() {}

APIFrameParser::ParseResult APIFrameParser::parse(uint8_t byte, APIFrame* frame)
{
    switch (parser_state)
    {
        // Look for the start of frame delimiter
        case ParserState::FIND_START:

            if (byte == START_DELIMITER)
            {
                parser_state = ParserState::READ_LENGTH_1;
            }
            break;
        // Read most significant byte of the length
        case ParserState::READ_LENGTH_1:
            frame->length = byte;
            parser_state  = ParserState::READ_LENGTH_2;

            break;
        // Read least significant byte of the length
        case ParserState::READ_LENGTH_2:
            frame->length |= ((uint16_t)byte << 8) & 0xFF00;
            // At least two frame data bytes (frame_type and a payload)
            if (swapBytes16(frame->length) < 2)
            {
                parser_state = ParserState::FIND_START;
                return ParseResult::FAIL;
            }

            if (frame->getFrameDataLength() > FRAME_DATA_SIZE)
            {
                parser_state = ParserState::FIND_START;
                return ParseResult::FAIL;
            }

            parser_state = ParserState::READ_FRAME_TYPE;
            break;
        // Read frame type
        case ParserState::READ_FRAME_TYPE:
            frame->frame_type = byte;
            parser_state      = ParserState::READ_FRAME_DATA;
            break;
        // Read the data frame
        case ParserState::READ_FRAME_DATA:
            frame->frame_data[current_frame_data_index++] = byte;

            if (current_frame_data_index == frame->getFrameDataLength())
            {
                current_frame_data_index = 0;
                parser_state             = ParserState::READ_CHECKSUM;
            }
            break;
        // Read & verify checksum
        case ParserState::READ_CHECKSUM:
            frame->checksum = byte;
            parser_state    = ParserState::FIND_START;

            if (frame->verifyChecksum())
            {
                return ParseResult::SUCCESS;
            }
            else
            {
                LOG_ERR(logger, "Wrong packet checksum!");
                return ParseResult::FAIL;
            }
            break;
    }

    if (parser_state != ParserState::FIND_START)
    {
        return ParseResult::PARSING;
    }
    else
    {
        return ParseResult::IDLE;
    }
}

}  // namespace Xbee