/* Common data structures
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla
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

#pragma once

#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/polymorphic.hpp>

#include <stdint.h>
#include <vector>
#include <memory.h>


enum SampleDataType { 
    DATATYPE_ALTIMETER      = 1, 
    DATATYPE_ACCEL          = 2,
    DATATYPE_GPS            = 3,
    DATATYPE_GYRO           = 4
};

//classe base per i Sample, deve avere anche lei la funzione per la serializzazione
class Sample {
public:
	uint8_t type;
	virtual ~Sample() {};

	template <class Archive>
	void serialize(Archive & ar)
	{
		ar(type);
	}

};

class SampleAltimeter : public Sample {
public:
	float altitude;

	SampleAltimeter() {
		SampleAltimeter(0);
	}

	SampleAltimeter(float altitude) {
		this->type = SampleDataType::DATATYPE_ALTIMETER;
		this->altitude = altitude;
	}

	template <class Archive>
	void serialize(Archive & ar)
	{
		ar(type,altitude);
	}

};

class SampleAccelerometer : public Sample {
public:
	float x;
	float y;
	float z;

	SampleAccelerometer() {
		SampleAccelerometer(0,0,0);
	}

	SampleAccelerometer(float x,float y,float z) {
		this->type = SampleDataType::DATATYPE_ACCEL;
		this->x=x;
		this->y=y;
		this->z=z;
	}

	template <class Archive>
	void serialize(Archive & ar)
	{
		ar(type,x,y,z);
	}


};

class SampleGyro : public Sample {
public:
	float x;
	float y;
	float z;

	SampleGyro() {
		SampleGyro(0,0,0);
	}

	SampleGyro(float x,float y,float z) {
		this->type = SampleDataType::DATATYPE_GYRO;
		this->x=x;
		this->y=y;
		this->z=z;
	}

	template <class Archive>
	void serialize(Archive & ar)
	{
		ar(type,x,y,z);
	}


};



class SampleGPS : public Sample {
public:
	uint8_t gLatitude;
	uint8_t gLongitude;
	double mLatitude;
	double mLongitude;


	SampleGPS() {
		SampleGPS(0,0,0,0);
	}

	SampleGPS(uint8_t gLatitude, uint8_t gLongitude, double mLatitude, double mLongitude) {
		this->type = SampleDataType::DATATYPE_GPS;
		this->gLatitude=gLatitude;
		this->gLongitude=gLongitude;
		this->mLatitude=mLatitude;
		this->mLongitude=mLongitude;
	}

	template <class Archive>
	void serialize(Archive & ar)
	{
		ar(type,gLatitude,gLongitude,mLatitude,mLongitude);
	}


};


class Record {
public:
	uint64_t timestamp;
	std::vector<std::unique_ptr<Sample>> samples;

	template <class Archive>
	void serialize(Archive & ar)
	{
		ar(timestamp,samples);
	}
};

// è necessario registrare le classi che estendono delle classi base per la serializzazione
/*
CEREAL_REGISTER_TYPE(SampleAltimeter)
CEREAL_REGISTER_TYPE(SampleAccelerometer)
CEREAL_REGISTER_TYPE(SampleGyro)
*/

CEREAL_REGISTER_TYPE_WITH_NAME(SampleAltimeter, "alt");
CEREAL_REGISTER_TYPE_WITH_NAME(SampleAccelerometer, "acc");
CEREAL_REGISTER_TYPE_WITH_NAME(SampleGyro, "gyr");
CEREAL_REGISTER_TYPE_WITH_NAME(SampleGPS, "gps");

