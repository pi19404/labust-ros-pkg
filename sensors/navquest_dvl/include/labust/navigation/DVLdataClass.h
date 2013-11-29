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
#ifndef DVLDATACLASS_H_
#define DVLDATACLASS_H_

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

namespace labust
{
	namespace navigation
	{
		class DVLdataClass
		{
		public:
			DVLdataClass(void);
			~DVLdataClass(void);
			void parsDVL(const std::string& str);
			std::string Header;
			// std::string	Error;
			bool Beam1Valid,Beam2Valid,Beam3Valid,Beam4Valid;
			float Altitude1, Altitude2, Altitude3, Altitude4;
			//float WaterVelocityX,WaterVelocityY,WaterVelocityZ;
			//int   DVL.WaterVelocityQuality1,DVL.WaterVelocityQuality2,DVL.WaterVelocityQuality3,DVL.WaterVelocityQuality4,...
			float VelocityX, VelocityY, VelocityZ; // VelocityX forward speed, forward positive, VelocityY lateral speed, port positive, VelocityZ, up positive
			float VelocityNorth, VelocityEast, VelocityDown;
			std::string VelocityType, VelocityEarthType, VelocityCurrentType, VelocityCurrentEarthType;
			float VelocityCurrentX, VelocityCurrentY, VelocityCurrentZ;
			float VelocityCurrentNorth, VelocityCurrentEast, VelocityCurrentDown;
			float Roll, Pitch, Heading, Altitude, Temperature;
			float Press, Salinity, SoundVelocity;
			int error_code;
			//int Checksum;
		};
	};
};

#endif
