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
#ifndef SEAMORCOMMS_H_
#define SEAMORCOMMS_H_
#include <labust/vehicles/vehiclesfwd.hpp>
#include <vector>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The function calculates the Seamor checksum.
		 *
		 * \param data The data for which to caluclate the checksum.
		 *
		 * \return Returns the calculated checksum.
		 */
		int getSeamorChecksum(const std::vector<unsigned char>& data);
		/**
		 * The Seamor status message.
		 *
		 * \todo Adjust the types from double to int or float.
		 */
		struct SeamorStatus
		{
			enum {id = 32, length = 18};

			static void decode(const std::vector<unsigned char>& data, SeamorStatus& msg);
			/**
			 * Depth, heading, roll and pitch (in degrees)
			 */
      double depth, heading, roll, pitch;
      /**
       * Thruster currents. Horizontal
       */
      double portVI,stbdVI,portHI,stbdHI;
      /**
        * Ballast and manipulator motor current
        */
      double ballastI, manipI;
      /**
       * Status byte
       */
      int status;
      /*
       * Temperature of electronic can.
       */
      double temperature;
      /**
       * The message timestamp.
       */
      double timeStamp;
		};
		/**
		 * The Seamor control command.
		 */
    struct SeamorCommand
    {
			enum {id=65, length=14};

			/**
			 * Generic constructor.
			 */
			SeamorCommand();

			static void decode(const std::vector<unsigned char>& data, SeamorCommand& msg);
			static void encode(std::vector<unsigned char>& data, const SeamorCommand& msg);

      /**
       * Forces and moments vector
       */
      labust::vehicles::tauMap tau;
      /**
       * Ballast force command
       */
      short ballastF;
      /**
       * Control for autoheading and autodepth
       * 32 = all off, autodepth +16, autoheading +8
       */
      short autopilotCommand;
      /**
       * Control for video selection (0,1,2,3)
       */
      short videoSelector;
      /**
       * Manipulator and light bytes.
       */
      short manipByte, lightByte;
    };
		/**
		 * The Seamor camera status message.
		 */
		struct SeamorCameraStatus
		{
			enum {id=16,length=18};
		};
		/**
		 * The Seamor camera control message.
		 */
    struct SeamorCameraCommand
    {
			enum{id=50,length=20};
    };
	}
}

/* SEAMORCOMMS_H_ */
#endif
