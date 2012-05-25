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
#include <labust/vehicles/VR3Details.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <cmath>

using namespace labust::vehicles;

VRSerialComms::VRSerialComms()
{
	//Set the buffer sizes.
	outputBuffer.resize(outLen,0);
	inputBuffer.resize(inLen,0);

	//Set the constant values.
	outputBuffer[0] = header;
	outputBuffer[1] = id;
}
VRSerialComms::~VRSerialComms(){}

bool VRSerialComms::encode(const ThrustVec& thrust)
{
	enum {port_byte = 2, stbd_byte, vert_byte, light_byte, control1, control2};
	enum {manipulator = 0, manipulator_enable, camera_rear = 6};
	enum {tilt = 8, tilt_enable, focus, focus_enable, port_direction, stbd_direction, vert_direction};

  outputBuffer[port_byte] = std::abs(thrust[port]);
  outputBuffer[stbd_byte] = std::abs(thrust[stbd]);
  outputBuffer[vert_byte] = std::abs(thrust[vert]);
  outputBuffer[light_byte] = std::abs(thrust[light]);
  set[port_direction] = !(thrust[port] > 0);
  set[stbd_direction] = thrust[stbd] > 0;
  set[vert_direction] = thrust[vert] > 0;

  //std::cout<<"Thrust:"<<thrust[port]<<","<<thrust[stbd]<<std::endl;

  unsigned short flags = static_cast<unsigned short>(set.to_ulong());
  outputBuffer[control1] = flags%256;
  outputBuffer[control2] = flags/256;

  return true;
}

bool VRSerialComms::decode(labust::vehicles::stateMapPtr state)
{
	using namespace labust::vehicles::state;
	//Check if the data header is ok.
	enum {yaw_byte = 3, depth_byte = 5};

	//Get heading and adjust
  short int value = 360 - (inputBuffer[yaw_byte] + 256*inputBuffer[yaw_byte+1]);
  if (value < 90) (*state)[heading]= value + 270; else (*state)[heading]= value - 90;
  (*state)[yaw] = labust::math::wrapRad((*state)[heading]*M_PI/180);

  //Get pressure and calculate depth.
  (*state)[depthPressure] = inputBuffer[depth_byte] + 256*inputBuffer[depth_byte+1];

  return true;
}
