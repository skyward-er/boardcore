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
enum LoadCellModes : uint8_t
{
    ASCII_MOD_TD,
    CONT_MOD_T,
    CONT_MOD_TD
};

enum LoadCellValues : char
{
    SETPOINT_1   = 'a',
    SETPOINT_2   = 'b',
    SETPOINT_3   = 'c',
    GROSS_WEIGHT = 't',
    NET_WEIGHT   = 'n',
    PEAK_WEIGHT  = 'p',
    RESET_TARE   = 'z'
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

    void print() const
    {
        if (net_weight.valid)
            TRACE("Net Weight   : %f [Kg]\n", net_weight.data);

        if (gross_weight.valid)
            TRACE("Gross Weight : %f [Kg]\n", gross_weight.data);

        if (peak_weight.valid)
            TRACE("Peak Weight  : %f [Kg]\n", peak_weight.data);

        if (setpoint1.valid)
            TRACE("Setpoint 1   : %f [Kg]\n", setpoint1.data);

        if (setpoint2.valid)
            TRACE("Setpoint 2   : %f [Kg]\n", setpoint2.data);

        if (setpoint3.valid)
            TRACE("Setpoint 3   : %f [Kg]\n", setpoint3.data);
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
    char req[2];
    char ck[3];
    char CR[2] = "\r";

    void setChecksum()
    {
        char data[4];
        const char *str = this->to_string().c_str();
        for (int i = 0; i < 3; i++)
        {
            data[i] = *(str + 1 + i);
        }
        data[3] = '\0';
        calculateChecksum(data, ck);
    }

    /**
     * @brief calculates the checksum as in the manual of the Load Cell
     * @param message the string from which will be calculated the checksum
     * @return a pair of chars that represents the checksum in hexadecimal
     */
    void calculateChecksum(char *message, char *checksum)
    {
        uint8_t ck = 0;
        for (unsigned int i = 0; i < strlen(message); i++)
        {
            ck ^= message[i];
        }

        char hex[3];
        itoa(ck, hex, 16);

        strcpy(checksum, hex);
    }

    std::string to_string()
    {
        std::string str;
        str.append(beginStr);
        str.append(addr);
        str.append(req);
        str.append(ck);
        str.append(CR);
        return str;
    }
} DataAsciiRequest;

typedef struct DataAsciiCorrectStr
{
    char beginStr[1];
    char addr[2];
    char data[6];
    char req[1];
    char separation[1];
    char ck[2];
    char CR[1];
} DataAsciiCorrect;

typedef struct DataAsciiErrorStr
{
    char beginStr[2];
    char addr[2];
    char errChar[1];
    char separation[1];
    char ck[2];
    char CR[1];
} DataAsciiError;