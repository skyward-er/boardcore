/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include "EuRoC-chunk1.h"
#include "EuRoC-chunk2.h"
#include "EuRoC-chunk3.h"
#include "EuRoC-chunk4.h"

constexpr int chunk1Size =
    sizeof(euRoCLogs_chunk1) / sizeof(Boardcore::PressureData);
constexpr int chunk2Size =
    sizeof(euRoCLogs_chunk2) / sizeof(Boardcore::PressureData);
constexpr int chunk3Size =
    sizeof(euRoCLogs_chunk3) / sizeof(Boardcore::PressureData);
constexpr int chunk4Size =
    sizeof(euRoCLogs_chunk4) / sizeof(Boardcore::PressureData);
Boardcore::PressureData
    euRoCLogs[chunk1Size + chunk2Size + chunk3Size + chunk4Size];

void initEuRoCPressureLogs()
{
    std::copy(euRoCLogs_chunk1, euRoCLogs_chunk1 + chunk1Size, euRoCLogs);
    std::copy(euRoCLogs_chunk2, euRoCLogs_chunk2 + chunk2Size,
              euRoCLogs + chunk1Size);
    std::copy(euRoCLogs_chunk3, euRoCLogs_chunk3 + chunk3Size,
              euRoCLogs + chunk1Size + chunk2Size);
    std::copy(euRoCLogs_chunk4, euRoCLogs_chunk4 + chunk4Size,
              euRoCLogs + chunk1Size + chunk2Size + chunk3Size);
}