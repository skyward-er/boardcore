/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
#include "HomeoneBoard.h"

HomeoneBoard::HomeoneBoard()
{
    setStatus(0x00);
	
	/** ES DA ANAKIN
    mS_MPU9250  = nullptr;
    mS_INEMO    = nullptr;
    mS_FXAS     = nullptr;
    mS_LPS331AP = nullptr;
    mS_MAX21105 = nullptr;
	*/
}

bool AnakinBoard::init()
{
    if (mInited)
        return false;
	
	sLog->logString("Initializing buses\n");
	//TODO initialize buses
	
	/** ES DA ANAKIN
	spiMPU9250::init();
    spiINEMOA::init();
    spiINEMOG::init();
    spiFXAS21002::init();
    spiLPS331AP::init();
	*/
	
	
	sLog->logString("Initializing sensors\n");
	// TODO initialize and check sensors
	
	 
	 sLog->logString("Adding sensors to samplers\n");
	/*
	 * TODO: add SimpleSensors for each sensor
	 * e.g. AddSensor(ACCEL_MPU9250,  DATA_VEC3,  mS_MAX21105->accelDataPtr());
	 *
	 * AddSensor è una funzione di Board
	 */
	 
	/* 
	 * TODO: add SimpleSensors to corresponding samplers
	 * e.g. m100HzDMA.AddSensor(mS_MPU9250);
	 */

    sLog->logString("Adding samplers to scheduler\n");
	/*
	 * TODO: Add samplers to the sEventScheduler with corresponding frequency.
	 *
	 * DAL CODICE DI ANAKIN:
	 *
	 #define ADD_SAMPLER(type, name, rate)                                         \
    sEventScheduler->add(std::bind(&type##SensorSampler::Update, name), rate, \
                         #name "-" #rate "ms", start)
	ADD_SAMPLER(DMA, m100HzDMA, 10);        // 10ms
	 */
	
	setStatus( boardStatusBits.INITIALIZED );
	mInited = true;
	
	return true;
}

//Get e set status sincronizzati

const uint8_t HomeoneBoard::getStatus()
{
	const uint8_t returnedStatus;
	
	Lock<FastMutex> l(statusMutex);
		returnedStatus = boardStatus.status;
	Unlock<FastMutex> u(l);
	
	return returnedStatus;
}

int HomeoneBoard::setStatus(uint8_t bitmask)
{
	Lock<FastMutex> l(statusMutex);
		boardStatus.status |= bitmask;
		boardStatus.last_modified = getTick();
	Unlock<FastMutex> u(l);
	
	return 0;
}