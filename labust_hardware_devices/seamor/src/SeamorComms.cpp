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
#include <labust/vehicles/SeamorComms.hpp>
#include <labust/tools/TimingTools.hpp>

using namespace labust::vehicles;

int labust::vehicles::getSeamorChecksum(const std::vector<unsigned char>& data)
{
	int sum = 65535;
	for (size_t i = 0; i < data.size(); ++i)
	{
		int temp = data[i];
		for (int j = 0; j < 8; ++j)
		{
			if (temp & 128)
			{
				sum = (sum * 2) | 1;
			}
			else
			{
				sum = sum * 2;
			}
			if (sum & 65536)
			{
				sum = sum ^ 4129;
			}
			sum = sum & 65535;
			temp = (temp * 2) & 255;
		}
	}

	return sum;
}

void SeamorStatus::decode(const std::vector<unsigned char>& data, SeamorStatus& msg)
{
	enum {m_portVI,m_stbdVI,m_portHI,m_stbdHI,
		m_ballastI, m_depthLow, m_depthHigh,
		m_manipI, m_pitch, m_roll, m_status,
		m_headingLow, m_headingHigh, m_temperature};
	const double cc(0.04);
	msg.portVI = data[m_portVI] * cc;
	msg.portHI = data[m_portHI] * cc;
	msg.stbdVI = data[m_stbdVI] * cc;
	msg.stbdHI = data[m_stbdHI] * cc;
	msg.ballastI = data[m_ballastI] * cc;
	msg.manipI = data[m_manipI] * cc;

	msg.depth = (data[m_depthLow]);
	msg.depth = msg.depth + (256. * data[m_depthHigh]);
	msg.depth = msg.depth/ 100.; //to meters
	msg.pitch = (char(data[m_pitch])) / 2.; // degrees
	msg.roll = (char(data[m_roll])) / 2.; // degrees
	msg.heading = (data[m_headingLow] + 256 * data[m_headingHigh]) / 20. + 90; //wrapDeg((data[m_headingLow] + 256 * data[m_headingHigh]) / 20. + 90) * M_PI / 180;
	msg.heading = (msg.heading < 360) ? msg.heading : msg.heading - 360;

	msg.status = data[m_status];
	msg.temperature = data[m_temperature] / 2.;

	msg.timeStamp = labust::tools::unix_time();
}


SeamorCommand::SeamorCommand():
				ballastF(0),
				autopilotCommand(32),
				videoSelector(0),
				manipByte(0),
				lightByte(0){};

void SeamorCommand::decode(const std::vector<unsigned char>& data, SeamorCommand& msg)
{
	enum {m_Z=0,m_Y,m_X,m_N,m_Ballast,
		m_empty, m_ctrlByte1,m_manipByte,
		m_ctrlByte2,m_lightByte, m_crc1,m_crc2};

	msg.tau[tau::X] = char(data[m_X]);
	msg.tau[tau::Y] = char(data[m_Y]);
	msg.tau[tau::Z] = char(data[m_Z]);
	msg.tau[tau::N] = char(data[m_N]);

	msg.ballastF = char(data[m_Ballast]);
	msg.autopilotCommand = char(data[m_ctrlByte1]);
	msg.videoSelector = char(data[m_ctrlByte2]);
	msg.manipByte = char(data[m_manipByte]);
	msg.lightByte = char(data[m_lightByte]);
}

void SeamorCommand::encode(std::vector<unsigned char>& data, const SeamorCommand& msg)
{
	enum {m_Z=2,m_Y,m_X,m_N,m_Ballast,
		m_empty, m_ctrlByte1,m_manipByte,
		m_ctrlByte2,m_lightByte, m_crc1,m_crc2};

	data.resize(SeamorCommand::length);

	data[0] = SeamorCommand::id;
	data[1] = SeamorCommand::length;

	using namespace labust::vehicles;
	data[m_X] = char(msg.tau.at(tau::X));
	data[m_Y] = char(msg.tau.at(tau::Y));
	data[m_Z] = char(msg.tau.at(tau::Z));
	data[m_N] = char(msg.tau.at(tau::N));

	data[m_Ballast] = msg.ballastF;
	data[m_empty] = 255;
	data[m_ctrlByte1] = msg.autopilotCommand;
	data[m_ctrlByte2] = msg.videoSelector;
	data[m_manipByte] = msg.manipByte;
	data[m_lightByte] = msg.lightByte;
	data[m_crc1] = 0;
	data[m_crc2] = 0;

	int sum = getSeamorChecksum(data);

	data[m_crc1] = sum / 256;
	data[m_crc2] = sum % 256;
}
