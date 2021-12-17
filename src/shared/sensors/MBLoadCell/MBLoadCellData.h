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

#include "sensors/SensorData.h"

namespace Boardcore
{

/**
 * @brief Enumeration of all the modes supported by the driver.
 */
enum LoadCellModes : uint8_t
{
    ASCII_MOD_TD,
    CONT_MOD_T,
    CONT_MOD_TD
};

/**
 * @brief Enumeration of all the requests in ASCII mode.
 */
enum LoadCellValuesEnum
{
    SET_SETPOINT_1,
    SET_SETPOINT_2,
    SET_SETPOINT_3,
    GET_SETPOINT_1,
    GET_SETPOINT_2,
    GET_SETPOINT_3,
    GROSS_WEIGHT,
    NET_WEIGHT,
    PEAK_WEIGHT,
    RESET_TARE,
    COMMUTE_TO_NET,
    COMMUTE_TO_GROSS
};

/**
 * @brief Type that maps the different requests to their keyword.
 */
typedef std::map<const LoadCellValuesEnum, std::string> LoadCellValues;
static LoadCellValues loadCellValues = {
    {SET_SETPOINT_1, "A"},   {SET_SETPOINT_2, "B"},
    {SET_SETPOINT_3, "C"},   {GET_SETPOINT_1, "a"},
    {GET_SETPOINT_2, "b"},   {GET_SETPOINT_3, "c"},
    {GROSS_WEIGHT, "t"},     {NET_WEIGHT, "n"},
    {PEAK_WEIGHT, "p"},      {RESET_TARE, "z"},
    {COMMUTE_TO_NET, "NET"}, {COMMUTE_TO_GROSS, "GROSS"}};

/**
 * @brief Structure of the errors in the ASCII requests.
 */
enum ReturnsStates
{
    VALID_RETURN,
    RECEPTION_ERROR,  // contains '?'
    EXECUTION_ERROR   // contains '#'
};

/**
 * @brief Structure that stores a data value, with his timestamp and his
 * validity.
 */
struct Data : public LoadCellData
{
    bool valid = false;

    Data() : LoadCellData{0, 0.0}, valid(false) {}

    Data(float data)
        : LoadCellData{TimestampTimer::getTimestamp(), data}, valid(true)
    {
    }

    static std::string header() { return "loadcell_timestamp,weight\n"; }

    void print(std::ostream& os) const
    {
        if (valid)
            os << loadcell_timestamp / 1000000.0 << "," << weight << "\n";
    }
};

/**
 * @brief Structure of the output of the load cell in [continuous mode -> ModT]
 */
struct MBLoadCellSettings
{
    LoadCellModes mode;
    bool gross_mode;
    Data peak_weight;
    Data setpoint1;
    Data setpoint2;
    Data setpoint3;

    /**
     * @brief Updates the correct value with the data passed. Also, memorizes
     * the maximum and minimum value of the gross weight.
     */
    void updateValue(LoadCellValuesEnum val, float data)
    {
        switch (val)
        {
            case PEAK_WEIGHT:
                peak_weight = Data(data);
                break;
            case GET_SETPOINT_1:
                setpoint1 = Data(data);
                break;
            case GET_SETPOINT_2:
                setpoint2 = Data(data);
                break;
            case GET_SETPOINT_3:
                setpoint3 = Data(data);
                break;
            default:
                break;
        }
    }

    /**
     * @brief Prints the structure in a nice way.
     */
    void print() const
    {
        /*if (net_weight.valid)
            TRACE("Net Weight     : %f [Kg]\n", net_weight.data);

        if (gross_weight.valid)
            TRACE("Gross Weight   : %f [Kg]\n", gross_weight.data);
        */
        if (peak_weight.valid)
            TRACE("Peak Weight    : %f [Kg]\n", peak_weight.weight);

        if (setpoint1.valid)
            TRACE("Setpoint 1     : %f [Kg]\n", setpoint1.weight);

        if (setpoint2.valid)
            TRACE("Setpoint 2     : %f [Kg]\n", setpoint2.weight);

        if (setpoint3.valid)
            TRACE("Setpoint 3     : %f [Kg]\n", setpoint3.weight);
    }
};

/**
 * @brief Structure of the output of the load cell in [continuous mode -> ModT]
 */
struct DataModT
{
    char weight[6];
    char CRLF[2];
};

/**
 * @brief Structure of the output of the load cell in [continuous mode -> ModTd]
 */
struct DataModTd
{
    char beginStr[1];
    char T[1];
    char weightT[6];
    char P[1];
    char weightP[6];
    char endStr[1];
    char ck[2];
    char CR[1];
};

/**
 * @brief Structure that contains all the parameters for the request to be sent.
 */
struct DataAsciiRequest
{
    char beginStr[2] = "$";
    char addr[3]     = "01";
    char value[7]    = "";
    char req[6];
    char ck[3];
    char CR[2] = "\r";

    /**
     * @brief In base of the address and the request parameter calculates the
     * checksum.
     */
    void setChecksum()
    {
        uint8_t checksum = 0;
        std::string str;
        str.append(addr);
        str.append(value);
        str.append(req);

        for (unsigned int i = 0; i < str.length(); i++)
        {
            checksum ^= str[i];
        }

        itoa(checksum, ck, 16);
    }

    /**
     * @brief Transforms the request into a string to be sent over serial.
     *
     * @return The string representing the request to be sent.
     */
    std::string to_string()
    {
        std::string str;
        str.append(beginStr);
        str.append(addr);
        str.append(value);
        str.append(req);
        str.append(ck);
        str.append(CR);
        return str;
    }
};

}  // namespace Boardcore