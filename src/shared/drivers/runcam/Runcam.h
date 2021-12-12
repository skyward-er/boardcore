/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

/*
    *Buttons*
        PowerButton -> near the blue led
        WifiButton -> near the orange led

        Basic info can be found at
https://www.runcam.com/download/split4k/RC_Split_4k_Manual_EN.pdf

    *Leds*
        Blue led slow blink -> Device is recording
        Blue led fast blink -> Error (missing SD card or memory full)
        Blue led still -> Device is in standby and not recording

        Orange led blink -> Firmware is being updated
        Orange led still -> OSD menu is being displayed

    *Changing settings*
        To select a setting press the WifiButton, to move down press the
PowerButton

        1) Stop the recording by pressing the PowerButton, the blue light should
be still 2) Long press the WifiButton the menu will appear on the screen and the
orange light well be still

        The menu shoul be appearing as follow a settings submenu is indicated
with an arrow connecting the setting to the subsetting

        ->Video
            |->Resolution
            |->Loop Recording
            |->Auto Recording
            |->Save and Exit
        ->Image
            |->Saturation
            |->Contrast
            |->Brightness
            |->Sharpness
            |->Exposure
            |->ISO
            |->Image Flip
            |->Metering
            |->Save and Exit
        ->TV-Out
            |->Screen Format
            |->TV Mode
            |->Save and Exit
        ->Micro SD Card
            |->Card Infro
                |->Capacity
                |->Free Space
            |->Format Card
            |->Save and Exit
        ->General
            |->Power Frequency
            |->Volume
            |->Auto Power Up
            |->Auto Shutdown
            |->Reset
            |->Language
            |->Save and Exit
        ->Save and exit


* Class used to control runcam split v4
 * User Manual:
https://www.runcam.com/download/split4k/RC_Split_4k_Manual_EN.pdf
 * Devide Protocol:
https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol
 * Protocl Implementation:
https://github.com/ArduPilot/ardupilot/blob/32482a29db341a1e228d92f682d24a47c1c4c0a4/libraries/AP_Camera/AP_RunCam.h
* crc8 version= Crc8 dvb-s2

* Camera Control 0xCC||0x01||CameraControlAction||crc8
    RCDEVICE_PROTOCOL_SIMULATE_WIFI_BTN = 0xCC010032,	        //Simulate Press
of the Wi-Fi button RCDEVICE_PROTOCOL_SIMULATE_POWER_BTN = 0xCC0101E7,
//Simulate Press of the Power button RCDEVICE_PROTOCOL_CHANGE_MODE =
0xCC01024D,	 //In Standby Mode, long press the Mode Switch button to cycle
through the two modes: Video/OSD settings(Long press wifi button);
*/

#ifndef RUNCAM_H
#define RUNCAM_H

#include <Debug.h>
#include <diagnostic/PrintLogger.h>
#include <fmt/format.h>

#include "RuncamSerial.h"

/**
 * @brief Header class for controlling the Runcam via serial
 */
class Runcam {
 public:
  /**
   * @brief default constructor
   */
  Runcam();

  /**
   * @brief default constructor
   * @param portNumber to use as the serial connection
   */
  Runcam(unsigned int portNumber);

  /**
   * @brief initiate the connection
   */
  bool init();

  /**
   * @brief close the connection
   */
  bool close();

  /**
   * @brief send the message to open the menu
   */
  void openMenu();
  /**
   * @brief send the message to select a settign
   */
  void selectSetting();
  /**
   * @brief send the message to move down
   */
  void moveDown();

 private:
  /**
   * @brief Simulate Click of the Wi-Fi button
   */
  uint32_t SELECT_SETTING = 0xCC010032;

  /**
   * @brief Simulate Click of the Power button
   */
  uint32_t MOVE_DOWN = 0xCC0101E7;

  /**
   * @brief In Standby Mode, long press the Mode Switch button to cycle through
   * the two modes: Video/OSD settings.
   */
  uint32_t OPEN_MENU = 0xCC01024D;

  /**
   * @brief Confiugre Serial Communication
   */
  bool configureSerialCommunication();

  /**
   * @brief default Runcam baud rate
   */
  static const unsigned int defaultBaudRate = 115200;

  /**
   * @brief default USART port number
   */
  static const unsigned int defaultPortNumber = 1;

  /**
   * @brief Check if the connection is already initialized
   */
  bool isInit;

  /**
   * @brief USART port number
   */
  unsigned int portNumber;

  /**
   * @brief serial interface that is needed to communicate
   * with the sensor via ASCII codes
   */
  RuncamSerial *serialInterface;
};
#endif
