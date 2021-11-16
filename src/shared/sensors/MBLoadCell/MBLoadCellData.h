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
#include <fmt/format.h>

/**
 * @brief enumeration of all the modes supported by the driver
 */
enum LoadCellModes : uint8_t
{
    ASCII_MOD_TD,
    CONT_MOD_T,
    CONT_MOD_TD
};

enum LoadCellValues : char
{
    SET_SETPOINT_1   = 'A',
    SET_SETPOINT_2   = 'B',
    SET_SETPOINT_3   = 'C',
    GET_SETPOINT_1   = 'a',
    GET_SETPOINT_2   = 'b',
    GET_SETPOINT_3   = 'c',
    GROSS_WEIGHT     = 't',
    NET_WEIGHT       = 'n',
    PEAK_WEIGHT      = 'p',
    RESET_TARE       = 'z',
    DECIMALS_READING = 'D'
};

/**
 * @brief structure of the errors in the ASCII requests
 */
enum ReturnsStates
{
    VALID_RETURN,     // min len : 13 (contains '!')
    RECEPTION_ERROR,  // max len : 8 (contains '?', '#')
    EXECUTION_ERROR
};

typedef struct DataStr
{
    float data = 0.0;
    bool valid = false;
} Data;

/**
 * @brief structure of the output of the load cell in [continuous mode -> ModT]
 */
typedef struct MBLoadCellDataStr
{
    Data net_weight;
    Data gross_weight;
    Data peak_weight;
    Data setpoint1;
    Data setpoint2;
    Data setpoint3;
    Data max_weight;
    Data min_weight;

    void updateValue(LoadCellValues val, float data)
    {
        switch (val)
        {
            case GET_SETPOINT_1:
                setpoint1 = {data, true};
                break;
            case GET_SETPOINT_2:
                setpoint2 = {data, true};
                break;
            case GET_SETPOINT_3:
                setpoint3 = {data, true};
                break;
            case NET_WEIGHT:
                net_weight = {data, true};
                break;
            case GROSS_WEIGHT:
                gross_weight = {data, true};

                // memorizing also the maximum gross weight registered
                if (!max_weight.valid || max_weight.data < data)
                    max_weight = {data, true};

                // memorizing also the minimum gross weight registered
                if (!min_weight.valid || min_weight.data > data)
                    min_weight = {data, true};
                break;
            case PEAK_WEIGHT:
                peak_weight = {data, true};
                break;
            default:
                break;
        }
    }

    void print() const
    {
        if (net_weight.valid)
            TRACE("Net Weight     : %f [Kg]\n", net_weight.data);

        if (gross_weight.valid)
            TRACE("Gross Weight   : %f [Kg]\n", gross_weight.data);

        if (peak_weight.valid)
            TRACE("Peak Weight    : %f [Kg]\n", peak_weight.data);

        if (setpoint1.valid)
            TRACE("Setpoint 1     : %f [Kg]\n", setpoint1.data);

        if (setpoint2.valid)
            TRACE("Setpoint 2     : %f [Kg]\n", setpoint2.data);

        if (setpoint3.valid)
            TRACE("Setpoint 3     : %f [Kg]\n", setpoint3.data);

        if (max_weight.valid)
            TRACE("Maximum weight : %f [Kg]\n", max_weight.data);

        if (min_weight.valid)
            TRACE("Minimum weight : %f [Kg]\n", min_weight.data);
    }
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

typedef struct DataAsciiRequestStr
{
    char beginStr[2] = "$";
    char addr[3]     = "01";
    char value[7]    = "";
    char req[2];
    char ck[3];
    char CR[2] = "\r";

    /**
     * @brief in base of the address and the request parameter calculates the
     * checksum
     */
    void setChecksum()
    {
        std::string str = fmt::format("{}{}{}", addr, value, req);
        calculateChecksum(str, ck);
    }

    /**
     * @brief calculates the checksum as in the manual of the Load Cell
     * @param message the string from which will be calculated the checksum
     * @return a pair of chars that represents the checksum in hexadecimal
     */
    void calculateChecksum(std::string message, char *checksum)
    {
        uint8_t ck = 0;
        for (unsigned int i = 0; i < message.length(); i++)
        {
            ck ^= message[i];
        }

        itoa(ck, checksum, 16);
    }

    /**
     * @brief transforms the request into a string to be sent over serial
     * @return the string representing the request to be sent
     */
    std::string to_string()
    {
        std::string str;
        str.append(beginStr);
        str.append(addr);
        if (strlen(value) != 0)
        {
            str.append(value);
        }
        str.append(req);
        str.append(ck);
        str.append(CR);
        return str;
    }
} DataAsciiRequest;