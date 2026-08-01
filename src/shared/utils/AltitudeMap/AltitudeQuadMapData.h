/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Leonardo Montecchi
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
#include <atomic>

namespace Boardcore
{

#pragma pack(push, 1)  // Disable padding to match binary layout in flash memory

struct MapHeader
{
    float topleftE;
    float topleftN;

    float maxAltitude;
    float minAltitude;

    float stepE;
    float stepN;

    uint16_t numPointsE;
    uint16_t numPointsN;

    uint8_t whoAmI = 0x42;
};

#pragma pack(pop)  // restore default packing

// Coordinates are stored as meters from the target landing point, with the
// target landing point being roughly at (0, 0).

struct MapBoundaries
{
    MapBoundaries()
        : eMax(Units::Length::Meter(NAN)), eMin(Units::Length::Meter(NAN)),
          nMax(Units::Length::Meter(NAN)), nMin(Units::Length::Meter(NAN)) {};

    Units::Length::Meter eMax;
    Units::Length::Meter eMin;
    Units::Length::Meter nMax;
    Units::Length::Meter nMin;
};

template <typename T>
struct __attribute__((packed)) InternalNode
{
    uint8_t marker;
    T size0;
    T size1;
    T size2;
};

namespace Config
{
constexpr std::size_t MAX_QUADTREE_SIZE = 397454;  // Bytes

// The type used to store the offsets of the child nodes in the quadtree. It can
// be either uint16_t or uint32_t, depending on the size of the quadtree.

using QuadOffset = uint32_t;
// using QuadOffset = uint16_t;

}  // namespace Config

}  // namespace Boardcore
