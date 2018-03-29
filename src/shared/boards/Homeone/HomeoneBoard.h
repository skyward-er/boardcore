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

#ifndef BOARDS_HOMEONE_H
#define BOARDS_HOMEONE_H

#include "Board.h"						
#include "Homeone_config.h"				// GPIO and Sensors are here

#include <events/Scheduler.h>			// Defines sEventScheduler
#include <logging/DumbLog.h>		    // Defines sLog

#include <sensors/SensorSampling.h>  


 enum boardStatusBits
 {
	 UNINITIALIZED = 0x00;
	INITIALIZED = 0x01,
	LAUCH_READY = 0x02,
	LAUNCHED = 0x04,
	APOGEE_REACHED = 0x08,
	RECOVERY_MODE = 0x10,
	SD_FULL = 0x20,
	FAULT_DETECTED = 0x40,
	DATA_READY = 0x80
 };

/**
 * Homeone Board
 *
 * L'oggetto board (che è un Singleton e può essere acceduto tramite la
 * macro sBoard definita alla fine del file) per come è pensata è poco più
 * che una classe wrapper per contenere i sensori.
 *
 * Ha il compito di inizializzare i driver dei sensori, definiti come tipo dentro
 * a Homeone_config.h (ognuno col suo bus e con i relativi PIN),
 * e di aggiungerli al sampler della frequenza giusta.
 *
 * Ha inoltre un semplicissimo registro di stato che può essere letto (per
 * esempio dal thread che si occupa di ricevere i comandi da terra) e scritto
 * (per esempio dal thread main).
 **/
class HomeoneBoard : public Singleton<HomeoneBoard>, public Board
{
    friend class Singleton<HomeoneBoard>;

public:

	/**
	 * Initializes the sensors and makes some tests. Called once.
	 * \return init successful
	 **/
    bool init() override;

	/**
	 * Reads the last data of all the sensors.
	 * \return an array of SingleSensors (defined in Board.h).
	 **/
    const std::vector<SingleSensor>& debugGetSensors() const
    {
        return mSensorData;
    }
	
	/**
	 * Synchronized method to access the status of the board.
	 * \return a byte containing the status of the board.
	 **/
	const uint8_t getStatus();
	
	/**
	 * Synchronized method to modify the status of the board.
	 * \arg bitmask 	bitmask for the bits that have to be put to 1
	 * \return 			an error code or 0 if it succeeded
	 **/
	int setStatus(uint8_t bitmask);

private:

	//Private constructor ensures that the class is a Singleton.
    HomeoneBoard();
	
	//Sensors (copy-pasted from Anakin - CHANGE THEM!!)
	fxas_t* mS_FXAS;
    lps331ap_t* mS_LPS331AP;
    max21105_t* mS_MAX21105;
	
	//Samplers 
	SimpleSensorSampler mPressureSampler;			// 2Hz
	SimpleSensorSampler mIMUSampler;					// 250Hz
	SimpleSensorSampler mGPSSampler;					// ??Hz
	
	// Mutex
	miosix::FastMutex statusMutex; 
	
	
	/**
	 * Status of the Board: can be modified with synchronized methods
	 * from the Board class.
	 **/
	struct
	{
		uint8_t status = 0x00;
		long long last_modified;
	} 
	boardStatus;
	
};

#ifdef sBoard
#error YOU ARE TRYING TO USE MULTIPLE BOARDS.
#else
#define sBoard HomeoneBoard::getInstance()
#endif

#endif /* ifndef BOARDS_ANAKINBOARD_H */
