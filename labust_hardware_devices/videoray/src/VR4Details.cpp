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
#include <labust/vehicles/VR4Details.hpp>
#include <labust/tools/StringUtilities.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <cmath>
#include <cstring>

using namespace labust::vehicles;

VRFutaba::VRFutaba()
{
	//Set the buffer sizes.
	outputBuffer.resize(outLen,0);
	inputBuffer.resize(inLen,0);
}

VRFutaba::~VRFutaba(){}

bool VRFutaba::encode(const ThrustVec& thrust)
{
	enum {header1 = 0xFA, header2 = 0xAF, id = 1, flags = 0x03, csr = 0};
	enum {header1_byte = 0, header2_byte,id_byte,flags_byte, csr_byte, size_byte, checksum_byte};
	enum {payload_start = 7, payload_size = 6,
		port_byte = 7, stbd_byte = 9, vert_byte = 11, len = 2};

	outputBuffer[header1_byte] = header1;
	outputBuffer[header2_byte] = header2;
	outputBuffer[id_byte] = id;
	outputBuffer[flags_byte] = flags;
	outputBuffer[csr_byte] = csr;
	outputBuffer[size_byte] = payload_size;
	outputBuffer[checksum_byte] = labust::tools::getChecksum(&outputBuffer[0],payload_start-1);

	//Change to accomodate the existing LabView program
	short int portT = static_cast<short int>(thrust[port]/220.*120.);
	short int stbdT = static_cast<short int>(thrust[stbd]/220.*120.);
	short int vertT = static_cast<short int>(thrust[vert]/220.*120.);

	std::cout<<"Port:"<<portT<<",stbd:"<<stbdT<<std::endl;

	memcpy(&outputBuffer[port_byte], &portT/*&thrust[port]*/, len);
	memcpy(&outputBuffer[stbd_byte], &stbdT/*&thrust[stbd]*/, len);
	memcpy(&outputBuffer[vert_byte], &vertT/*&thrust[vert]*/, len);

	outputBuffer[outLen-1] = labust::tools::getChecksum(&outputBuffer[payload_start],payload_size);

  return true;
}

bool VRFutaba::decode(labust::vehicles::stateMapPtr state)
{
	using namespace labust::vehicles::state;
	//Check if the data header is ok and check checksums.
	enum {header1 = 0xFD, header2 = 0xDF};
	enum {header1_byte = 0, header2_byte};
	enum {payload_start = 7,
	yaw_byte=8, pitch_byte = 10, roll_byte = 12, z_byte = 14};

  //std::cout<<"Data:";
  //for (int i=0; i<inputBuffer.size(); ++i) std::cout<<int(inputBuffer[i])<<",";
  //std::cout<<std::endl;

	//Add some additional checking
	if ((inputBuffer[header1_byte] != header1) || (inputBuffer[header2_byte] != header2)) return false;

	//retVal = retVal && labust::tools::getChecksum(&inputBuffer[0],payload_start-1) == inputBuffer[payload_start-1];
	//retVal = retVal && labust::tools::getChecksum(&inputBuffer[0],inLen-1) == inputBuffer[inLen-1];

	short int temp;
	memcpy(&temp,&inputBuffer[yaw_byte],sizeof(temp));
	(*state)[heading] = temp;
	(*state)[yaw] = labust::math::wrapRad((*state)[heading]*M_PI/180);
	memcpy(&temp,&inputBuffer[pitch_byte],sizeof(temp));
	(*state)[pitch] = labust::math::wrapRad(temp*M_PI/180.);
	memcpy(&temp,&inputBuffer[roll_byte],sizeof(temp));
	(*state)[roll] = labust::math::wrapRad(temp*M_PI/180.);
  memcpy(&temp,&inputBuffer[z_byte],sizeof(temp));
	(*state)[depthPressure] = temp;

  return true;
}

/*
#include "VRFutaba.h"
#include <LABUSTTools.h>
#include <string.h>
#include <iostream>

using LABUST::COMMUNICATION::VRFutaba;

VRFutaba::VRFutaba():
  networkID(vr4),
  flag(full_vr4_info),
  csr(deflt),
  payload(6,0){}

VRFutaba::~VRFutaba(){}

int VRFutaba::populate(uint8* data, int len)
{
  using namespace LABUST::TOOLS;
  int retVal = -1;


  bool flag = (len > 7);
  flag = flag && (calcChks(&data[0],6) == data[6]);
  flag = flag && (calcChks(&data[0],len-1) == data[len-1]);
  flag = flag && ((data[0] == 0xFD) && (data[1] == 0xDF));

  if (flag)
  {
    networkID = data[2];
    flag = data[3];
    csr = data[4];

    if (calcChks(&data[0],6) == data[6])
    {
      retVal = 0;

      uint8 length = data[5];

      payload.resize(length);

      memcpy(&payload[0],&data[7],length);
    }
  }
  else
  {
    std::cout<<"Conditions not satisfied."<<std::endl;
  }


  return retVal;
}

int VRFutaba::fill(uint8* data)
{
  using namespace LABUST::TOOLS;
  data[0] = 0xFA;
  data[1] = 0xAF;
  data[2] = networkID;
  data[3] = flag;
  data[4] = csr;
  data[5] = payload.size();
  data[6] = calcChks(&data[0],6);

  memcpy(&data[7],&payload[0],payload.size());

  int full_len = 8 + payload.size();

  data[full_len-1] = calcChks(&data[0],full_len-1);

  return 0;
}

int VRFutaba::size()
{
 return 8 + payload.size();
}
*/
