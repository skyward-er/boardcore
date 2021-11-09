/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

/**
 * @brief enumeration of all the modes supported by the driver
 */
enum LoadCellModes: uint8_t{
    MOD_T,
    MOD_TD
};

/**
 * @brief structure of the output of the load cell in [continuous mode -> ModT]
 */
typedef struct MBLoadCellDataStr
{
    char weight[6];
    char CRLF[2];
} MBLoadCellData;

/**
 * @brief structure of the output of the load cell in [continuous mode -> ModT]
 */
typedef struct DataModTStr
{
    char weight[6];
    char CRLF[2];
} DataModT;

/**
 * @brief structure of the output of the load cell in [continuous mode -> ModTd]
 */
typedef struct DataModTdStr
{
    char beginStr[1];
    char T[1];
    char weightT[6];
    char P[1];
    char weightP[6];
    char endStr[1];
    char ck[2];
    char CR[1];
} DataModTd;