/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the LABUST nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <labust/navigation/DVLdataClass.h>
#include <labust/navigation/NavQuestMessages.hpp>
using namespace labust::navigation;


DVLdataClass::DVLdataClass(void){};

DVLdataClass::~DVLdataClass(void){};

void DVLdataClass::parsDVL(const std::string& str)
{
	//LABUST::DVLdataClass DVLoutput;   // class declaration
	std::string sname, sname1;
	int index = 0;

	sname = str;

	if (!(str.compare(0, 8, "$#NQ.RES")))
	{
		//cout << "To je NQ1 poruka" << endl;
		std::stringstream converter;
		index = sname.find(" ");
		Header = sname.substr(0, index); // Header
		//skipped error code
		sname = sname.substr(index + 1);
		index = sname.find(" ");

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Beam1Valid;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Beam2Valid;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Beam3Valid;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Beam4Valid;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Altitude1;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Altitude2;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Altitude3;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Altitude4;
		converter.clear();

		// BottomRadVelocity for each beam, skip
		// WaterRadVelocity for each beam, skip
		// WaterVelocityQuality value: 0-50, 0 - water velocity not valid, 50- perfect, skip
		for (int j = 0; j < 13; j++)
		{ // SKip 12 fields
			sname = sname.substr(index + 1);
			index = sname.find(" ");
		}

		// Instrument DVL.VelocityX,DVL.VelocityY,DVL.VelocityZ and Type (bottom, water, invalid)
		sname1 = sname.substr(0, index);
		converter << sname1; // VelocityX forward speed, forward positive
		converter >> VelocityX;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityY; // VelocityY lateral speed, port positive
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityZ; // VelocityZ forward speed, up positive
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		if (sname1 == "1")
			VelocityType = "Bottom";
		else
		{
			if (sname1 == "2")
				VelocityType = "Water";
			else
				VelocityType = "Invalid";
		}

		// Earth DVL.Velocities and Type (bottom, water, invalid)
		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityNorth;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityEast;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityDown;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		if (sname1 == "1")
			VelocityEarthType = "Bottom";
		else
		{
			if (sname1 == "2")
				VelocityEarthType = "Water";
			else
				VelocityEarthType = "Invalid";
		}

		// Instrument Current.Velocities and Type (bottom, water, invalid)
		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityCurrentX;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityCurrentY;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityCurrentZ;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		if (sname1 == "1")
			VelocityCurrentType = "Botom";
		else
		{
			if (sname1 == "2")
				VelocityCurrentType = "Water";
			else
				VelocityCurrentType = "Invalid";
		}

		// Instrument Current.Velocities and Type (bottom, water, invalid)
		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityCurrentNorth;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityCurrentEast;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityCurrentDown;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		if (sname1 == "1")
			VelocityCurrentEarthType = "Bottom";
		else
		{
			if (sname1 == "2")
				VelocityCurrentEarthType = "Water";
			else
				VelocityCurrentEarthType = "Invalid";
		}

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Roll;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Pitch;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Heading;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Altitude;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Temperature;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Press;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Salinity;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(" ");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> SoundVelocity;
		converter.clear();

	}

	if (!(str.compare(0, 3, ":SA")))
	{
		std::stringstream converter;
		index = sname.find(",");
		sname1 = sname.substr(0, index);
		Header = sname1;

		sname = sname.substr(index + 1);
		index = sname.find(",");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Pitch; // positive for nose up
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(",");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Roll;
		converter.clear();

		sname = sname.substr(index + 1);
		converter << sname;
		converter >> Heading;
		converter.clear();
	}

	if (!(str.compare(0, 3, ":TS")))
	{
		std::stringstream converter;
		index = sname.find(",");
		sname1 = sname.substr(0, index);
		Header = sname1;
		sname = sname.substr(index + 1);

		// Skip first two fields representing date&time, NOT USED
		index = sname.find(",");
		sname = sname.substr(index + 1);
		index = sname.find(",");
		sname = sname.substr(index + 1);

		index = sname.find(",");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Temperature;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(",");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Press;
		converter.clear();

		sname = sname.substr(index + 1);
		converter << sname;
		converter >> SoundVelocity;
		converter.clear();
	}

	if (!(str.compare(0, 3, ":WI")))
	{
		std::stringstream converter;
		index = sname.find(",");
		sname1 = sname.substr(0, index);
		Header = sname1;
		sname = sname.substr(index + 1);

		index = sname.find(",");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityCurrentX;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(",");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityCurrentY;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(",");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityCurrentZ;
		converter.clear();

		// Skip one field, NOT USED
		index = sname.find(",");
		sname = sname.substr(index + 1);

		sname = sname.substr(index + 1,1);
		VelocityCurrentType = sname;
	}

	if (!(str.compare(0, 3, ":BI")))
	{
		std::stringstream converter;
		index = sname.find(",");
		sname1 = sname.substr(0, index);
		Header = sname1;
		sname = sname.substr(index + 1);

		index = sname.find(",");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityX;
		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(",");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityY;

		converter.clear();

		sname = sname.substr(index + 1);
		index = sname.find(",");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> VelocityZ;
		converter.clear();

		// Skip one field, NOT USED
		index = sname.find(",");
		sname = sname.substr(index + 1);

		sname = sname.substr(index + 1,1);
		VelocityType = sname;
	}

	if (!(str.compare(0, 3, ":BD")))
	{
		std::stringstream converter;
		index = sname.find(",");
		sname1 = sname.substr(0, index);
		Header = sname1;
		sname = sname.substr(index + 1);
		// Skip first 3 fields, NOT USED
		index = sname.find(",");
		sname = sname.substr(index + 1);
		index = sname.find(",");
		sname = sname.substr(index + 1);
		index = sname.find(",");
		sname = sname.substr(index + 1);

		index = sname.find(",");
		sname1 = sname.substr(0, index);
		converter << sname1;
		converter >> Altitude;
		converter.clear();
	}

	//std::cout<<str<<" "<<Header<<std::endl;

}
