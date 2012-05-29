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
#include <labust/vehicles/SeamorCommands.h>

#include <boost/algorithm/string.hpp>
/*
#include <LABUSTTools.h>
#include <vector>
#include <iosfwd>
#include "iostream"

 */
int labust::vehicles::getSeamorChecksum(const unsigned char* data, int count)
{
	int sum = 65535;
	for (int i = 0; i < count; ++i)
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

using namespace labust::communication;

SeamorStatus::SeamorStatus():
		depth(0),
    heading(0),
    roll(0),
    pitch(0),
    portVI(0),
    stbdVI(0),
    portHI(0),
    stbdHI(0),
    ballastI(0),
    manipI(0),
    status(0),
    temperature(0){};

SeamorStatus::~SeamorStatus(){};

int SeamorStatus::parseBinary(unsigned char* data, int len)
{
	int retVal = -1;
	if (len == (labust::communication::seamorrawdata::rovStatusLength - 2))
	{
		retVal = 0;
		//Current constant
		double cc = 0.04;
		portVI = data[m_portVI] * cc;
		portHI = data[m_portHI] * cc;
		stbdVI = data[m_stbdVI] * cc;
		stbdHI = data[m_stbdHI] * cc;
		ballastI = data[m_ballastI] * cc;
		manipI = data[m_manipI] * cc;

		depth = (data[m_depthLow]);
		depth = depth + (256. * data[m_depthHigh]);
		depth = depth/ 100.; //to meters
		pitch = ((char) data[m_pitch]) / 2.; // degrees
		roll = ((char) data[m_roll]) / 2.; // degrees
		heading = (data[m_headingLow] + 256 * data[m_headingHigh]) / 20. + 90; //wrapDeg((data[m_headingLow] + 256 * data[m_headingHigh]) / 20. + 90) * M_PI / 180;
		heading = (heading < 360) ? heading : heading - 360;

		status = data[m_status];
		temperature = data[m_temperature] / 2.;
	}

	return retVal;
}

/**
 * Generic constructor
 */
SeamorCommand::SeamorCommand():
		ballastF(0),
    autopilotCommand(32),
    videoSelector(0),
    manipByte(0),
    lightByte(0){};

/**
 * Generic destructor
 */
SeamorCommand::~SeamorCommand(){};

/**
 * Fill data vector from class variables.
 */
int SeamorCommand::parseTau(labust::vehicles::tauMap& newTau)
{
	using namespace labust::vehicles;
	tauC[tau::X] = newTau[tau::X];
	tauC[tau::Y] = newTau[tau::Y];
	tauC[tau::Z] = newTau[tau::Z];
	tauC[tau::N] = newTau[tau::N];
	tauC[tau::K] = 0;
	tauC[tau::M] = 0;
	return 0;
}

int SeamorCommand::parseDataMap(labust::vehicles::strMap& dataMap)
{
	using namespace labust::vehicles;


	std::stringstream buffer;
	if (dataMap.find("AuxMotor") != dataMap.end())
	{
		if (boost::iequals(dataMap["AuxMotor"], "+"))
			ballastF = 64;
		else if (boost::iequals(dataMap["AuxMotor"], "0"))
			ballastF = 0;
		else if (boost::iequals(dataMap["AuxMotor"], "-"))
			ballastF = -64;
	}

	if (dataMap.find("Autopilot") != dataMap.end())
	{
		if (boost::iequals(dataMap["Autopilot"], "Off"))
			autopilotCommand = 32;
		else if (boost::iequals(dataMap["Autopilot"], "AutoHeading"))
			autopilotCommand = 40;
		else if (boost::iequals(dataMap["Autopilot"], "AutoDepth"))
			autopilotCommand = 48;
		else if (boost::iequals(dataMap["Autopilot"], "On"))
			autopilotCommand = 56;
	}

	if (dataMap.find("VideoSelector") != dataMap.end())
	{
		buffer << dataMap["VideoSelector"];
		buffer >> videoSelector;
		buffer.clear();
		buffer.str("");
	}

	if (dataMap.find("Manipulator") != dataMap.end())
	{
		if (boost::iequals(dataMap["Manipulator"], "+"))
			manipByte = 64;
		else if (boost::iequals(dataMap["Manipulator"], "0"))
			manipByte = 0;
		else if (boost::iequals(dataMap["Manipulator"], "-"))
			manipByte = -64;
	}

	if (dataMap.find("Light") != dataMap.end())
	{
		buffer << dataMap["Light"];
		buffer >> lightByte;
		buffer.clear();
		buffer.str("");
	}
	return 0;
}

/**
 * Populate the class from a data vector (received from ROV)
 */
int SeamorCommand::parseBinary(unsigned char* data, int len)
{
	using namespace labust::vehicles;
	int retVal = -1;

	if (data != NULL && len == (seamorrawdata::rovControlLength - 2))
	{
		retVal = 0;
		// deduct 2 because header is not included in passed data
		tauC[tau::X] = (char) data[m_X - 2];
		tauC[tau::Y] = (char) data[m_Y - 2];
		tauC[tau::Z] = (char) data[m_Z - 2];
		tauC[tau::N] = (char) data[m_N - 2];

		ballastF = (char) data[m_Ballast - 2];
		autopilotCommand = (char) data[m_ctrlByte1 - 2];
		videoSelector = (char) data[m_ctrlByte2 - 2];
		manipByte = (char) data[m_manipByte - 2];
		lightByte = (char) data[m_lightByte - 2];
	}
	return retVal;
}

int SeamorCommand::getBinary(unsigned char* o_data)
{
	using namespace labust::vehicles;

	o_data[0] = int(seamorrawdata::rovControl);
	o_data[1] = int(seamorrawdata::rovControlLength);

	/*
	char tauX, tauY, tauZ, tauN;
	COREDATASET::tauMap::const_iterator element;
	element = tau.find(COREDATASET::X);
	tauX = element != tau.end() ? element->second : 0;
	element = tau.find(COREDATASET::Y);
	tauY = element != tau.end() ? element->second : 0;
	element = tau.find(COREDATASET::Z);
	tauZ = element != tau.end() ? element->second : 0;
	element = tau.find(COREDATASET::N);
	tauN = element != tau.end() ? element->second : 0;*/

	o_data[m_X] = char(tauC[tau::X]);
	o_data[m_Y] = char(tauC[tau::Y]);
	o_data[m_Z] = char(tauC[tau::Z]);
	o_data[m_N] = char(tauC[tau::N]);

	o_data[m_Ballast] = ballastF;
	o_data[m_empty] = 255;
	o_data[m_ctrlByte1] = autopilotCommand;
	o_data[m_ctrlByte2] = videoSelector;
	o_data[m_manipByte] = manipByte;
	o_data[m_lightByte] = lightByte;
	o_data[m_crc1] = 0;
	o_data[m_crc2] = 0;

	int sum = labust::vehicles::getSeamorChecksum(o_data, seamorrawdata::rovControlLength);

	o_data[m_crc1] = sum / 256;
	o_data[m_crc2] = sum % 256;
	return 0;
}

SeamorCameraStatus::SeamorCameraStatus() :
        						gain("N/A"),
        						redGain("N/A"),
        						blueGain("N/A"),
        						iris("N/A"),
        						focusDistance("N/A")
{
	float focusInitValues[14] = {0, 2000, 800, 350, 200, 140, 100, 80, 29, 10, 4.7, 2.3, 1, 0};
	std::stringstream buffer;
	float y0, y1, dx, yx;
	for (int i = 16; i <= 192; i++)
	{
		y0 = focusInitValues[(i / 16)];
		y1 = focusInitValues[(i / 16) + 1];
		dx = i % 16;
		yx = y0 * (16 - dx) / 16 + y1 * dx / 16;
		buffer << (yx / 100);
		buffer >> focusDistances[i];
		buffer.clear();
		buffer.str("");
	}
	focusDistances[16] = "Infinity";

	irisSettings[0] = "Closed";
	irisSettings[1] = "F22";
	irisSettings[2] = "F19";
	irisSettings[3] = "F16";
	irisSettings[4] = "F14";
	irisSettings[5] = "F11";
	irisSettings[6] = "F9.6";
	irisSettings[7] = "F8";
	irisSettings[8] = "F6.8";
	irisSettings[9] = "F5.6";
	irisSettings[10] = "F4.8";
	irisSettings[11] = "F4";
	irisSettings[12] = "F3.4";
	irisSettings[13] = "F2.8";
	irisSettings[14] = "F2.4";
	irisSettings[15] = "F2";
	irisSettings[16] = "F1.6";
	irisSettings[17] = "F1.4";

	//'here is the table to convert gain value to value in dB
	gainSettings[0] = "-3";
	gainSettings[1] = "0";
	gainSettings[2] = "2";
	gainSettings[3] = "4";
	gainSettings[4] = "6";
	gainSettings[5] = "8";
	gainSettings[6] = "10";
	gainSettings[7] = "12";
	gainSettings[8] = "14";
	gainSettings[9] = "16";
	gainSettings[10] = "18";
	gainSettings[11] = "20";
	gainSettings[12] = "22";
	gainSettings[13] = "24";
	gainSettings[14] = "26";
	gainSettings[15] = "28";

	//'Note that these values are specific for NTSC
	shutterSettings[0] = "1/4";
	shutterSettings[1] = "1/8";
	shutterSettings[2] = "1/15";
	shutterSettings[3] = "1/30";
	shutterSettings[4] = "1/60";
	shutterSettings[5] = "1/90";
	shutterSettings[6] = "1/100";
	shutterSettings[7] = "1/125";
	shutterSettings[8] = "1/180";
	shutterSettings[9] = "1/250";
	shutterSettings[10] = "1/350";
	shutterSettings[11] = "1/500";
	shutterSettings[12] = "1/725";
	shutterSettings[13] = "1/1000";
	shutterSettings[14] = "1/1500";
	shutterSettings[15] = "1/2000";
	shutterSettings[16] = "1/3000";
	shutterSettings[17] = "1/4000";
	shutterSettings[18] = "1/6000";
	shutterSettings[19] = "1/10000";
};

SeamorCameraStatus::~SeamorCameraStatus()
{
};

int SeamorCameraStatus::parseBinary(unsigned char* data, int len)
{
	if (data != NULL && len == (seamorrawdata::cameraStatusLength - 2))
	{
		tilt = ((signed char) data[0]*256 + (signed char) data[1]) / 65536. * 180; //convert to degrees (approx)
		temperature = (int) data[4] / 2.; //celsius

		zoom = ((int) data[6]*256 + (int) data[5]) / 286.78;
		shutterSpeed = shutterSettings[((int) data[8]) / 4];
		lightSetting = (int) data[9]; /// 2.55; //percentage

		focusDistance = focusDistances[(int) data[7]]; //meters

		std::stringstream blueGainBuffer, redGainBuffer;

		iris = irisSettings[(int) data[10]]; //dB
		/*if ((int) data[11] < 16)
                    redGain = gainSettings[(int) data[11]]; //dB
                //if ((int) data[12] < 16)
                    blueGain = gainSettings[(int) data[12]]; *///dB

		redGainBuffer << (int) data[11]; //dB
		redGain = redGainBuffer.str();

		blueGainBuffer << (int) data[12]; //dB
		blueGain = blueGainBuffer.str();
		if ((int) data[13] < 16)
			gain = gainSettings[(int) data[13]]; //dB
		return 0;
	}
	return -1;
}

SeamorCameraCommand::SeamorCameraCommand() :
        						focus(160),
        						pan(0),
        						tilt(0),
        						light(0),
        						gain(0),
        						zoom(0),
        						redGain(255),
        						blueGain(255),
        						iris("N/A"),
        						shutterSpeed("N/A"),
        						whiteBalance(cameracommands::autoWB),
        						autoExposure(cameracommands::full),
        						laserState(cameracommands::laserOFF),
        						irisInt(0), shutterSpeedInt(0)
{
	irisSettings[0] = "Closed";
	irisSettings[1] = "F22";
	irisSettings[2] = "F19";
	irisSettings[3] = "F16";
	irisSettings[4] = "F14";
	irisSettings[5] = "F11";
	irisSettings[6] = "F9.6";
	irisSettings[7] = "F8";
	irisSettings[8] = "F6.8";
	irisSettings[9] = "F5.6";
	irisSettings[10] = "F4.8";
	irisSettings[11] = "F4";
	irisSettings[12] = "F3.4";
	irisSettings[13] = "F2.8";
	irisSettings[14] = "F2.4";
	irisSettings[15] = "F2";
	irisSettings[16] = "F1.6";
	irisSettings[17] = "F1.4";

	//'Note that these values are specific for NTSC
	shutterSettings[0] = "1/4";
	shutterSettings[1] = "1/8";
	shutterSettings[2] = "1/15";
	shutterSettings[3] = "1/30";
	shutterSettings[4] = "1/60";
	shutterSettings[5] = "1/90";
	shutterSettings[6] = "1/100";
	shutterSettings[7] = "1/125";
	shutterSettings[8] = "1/180";
	shutterSettings[9] = "1/250";
	shutterSettings[10] = "1/350";
	shutterSettings[11] = "1/500";
	shutterSettings[12] = "1/725";
	shutterSettings[13] = "1/1000";
	shutterSettings[14] = "1/1500";
	shutterSettings[15] = "1/2000";
	shutterSettings[16] = "1/3000";
	shutterSettings[17] = "1/4000";
	shutterSettings[18] = "1/6000";
	shutterSettings[19] = "1/10000";

	//'here is the table to convert gain value to value in dB
	gainSettings[0] = "-3";
	gainSettings[1] = "0";
	gainSettings[2] = "2";
	gainSettings[3] = "4";
	gainSettings[4] = "6";
	gainSettings[5] = "8";
	gainSettings[6] = "10";
	gainSettings[7] = "12";
	gainSettings[8] = "14";
	gainSettings[9] = "16";
	gainSettings[10] = "18";
	gainSettings[11] = "20";
	gainSettings[12] = "22";
	gainSettings[13] = "24";
	gainSettings[14] = "26";
	gainSettings[15] = "28";
}

SeamorCameraCommand::~SeamorCameraCommand()
{

}

int SeamorCameraCommand::parseBinary(unsigned char* data, int length)
{
	using namespace labust::vehicles;
	int retVal = -1;

	if (length == (seamorrawdata::cameraControlLength - 2))
	{
		retVal = 0;


		redGain = (int)data[1]; //dB

		blueGain = (int) data[2]; //dB
		if ((int) data[3] < 18)
			iris = irisSettings[(int) data[3]]; //dB
		if ((int) data[4] < 20)
			shutterSpeed = shutterSettings[(int) data[4]];
		gain = data[5];
		focus = data[6];
		zoom = data[7] + 256 * data[8];
		whiteBalance = (cameracommands::wb)data[9];
		autoExposure = (cameracommands::ae)data[10];
		laserState = (cameracommands::laser)data[11];
		pan = data[12];
		tilt = data[13];
		light = data[14];



	}
	return retVal;
}

int SeamorCameraCommand::getBinary(unsigned char* o_data)
{
	using namespace labust::communication;
  o_data[0] = (int) seamorrawdata::cameraControl;
	o_data[1] = (int) seamorrawdata::cameraControlLength;

	o_data[2] = 0;
	o_data[3] = redGain;
	o_data[4] = blueGain;
	o_data[5] = irisInt;
	o_data[6] = shutterSpeedInt;
	o_data[7] = gain;
	o_data[8] = focus;
	o_data[9] = zoom % 256;
	o_data[10] = zoom / 256;
	o_data[11] = (int) whiteBalance;
	o_data[12] = (int) autoExposure;
	o_data[13] = (int) laserState;
	o_data[14] = pan;
	o_data[15] = tilt;
	o_data[16] = light;
	o_data[17] = 16;
	o_data[18] = 0;
	o_data[19] = 0;

	int sum = labust::vehicles::getSeamorChecksum(o_data, seamorrawdata::cameraControlLength);

	o_data[18] = sum / 256;
	o_data[19] = sum % 256;

	return 0;
}

int SeamorCameraCommand::parseDataMap(labust::vehicles::strMap& dataMap)
{
	using namespace labust::vehicles;
	if (dataMap.find("RedGain") != dataMap.end())
	{
		std::stringstream buffer;
		buffer << dataMap["RedGain"];
		buffer >> redGain;

	}

	if (dataMap.find("Light") != dataMap.end())
	{
		std::stringstream buffer;
		buffer << dataMap["Light"];
		buffer >> light;
	}

	if (dataMap.find("BlueGain") != dataMap.end())
	{
		std::stringstream buffer;
		buffer << dataMap["BlueGain"];
		buffer >> blueGain;
	}

	if (dataMap.find("Iris") != dataMap.end())
	{
		std::stringstream buffer;
		char prefix;
		double irisNumber;
		buffer << dataMap["Iris"];
		buffer >> prefix >> irisNumber;
		if (boost::iequals(&prefix, "f"))
		{
			if (irisNumber == 22)
			{
				irisInt = 1;
			}
			else
				if (irisNumber == 19)
				{
					irisInt = 2;
				}
				else
					if (irisNumber == 16)
					{
						irisInt = 3;
					}
					else
						if (irisNumber == 14)
						{
							irisInt = 4;
						}
						else
							if (irisNumber == 11)
							{
								irisInt = 5;
							}
							else
								if (irisNumber == 9.6)
								{
									irisInt = 6;
								}
								else
									if (irisNumber == 8)
									{
										irisInt = 7;
									}
									else
										if (irisNumber == 6.8)
										{
											irisInt = 8;
										}
										else
											if (irisNumber == 5.6)
											{
												irisInt = 9;
											}
											else
												if (irisNumber == 4.8)
												{
													irisInt = 10;
												}
												else
													if (irisNumber == 4)
													{
														irisInt = 11;
													}
													else
														if (irisNumber == 3.4)
														{
															irisInt = 12;
														}
														else
															if (irisNumber == 2.8)
															{
																irisInt = 13;
															}
															else
																if (irisNumber == 2.4)
																{
																	irisInt = 14;
																}
																else
																	if (irisNumber == 2)
																	{
																		irisInt = 15;
																	}
																	else
																		if (irisNumber == 1.6)
																		{
																			irisInt = 16;
																		}
																		else
																			if (irisNumber == 1.4)
																			{
																				irisInt = 17;
																			}

		}
		else
		{
			irisInt = 0;
		}
		iris = irisSettings[irisInt];
	}

	if (dataMap.find("ShutterSpeed") != dataMap.end())
	{
		std::stringstream buffer;
		char a, b;
		int speed;
		buffer << dataMap["ShutterSpeed"];
		buffer >> a >> b >> speed;
		switch (speed)
		{
		case 4:
		{
			shutterSpeedInt = 0;
		}
		break;
		case 8:
		{
			shutterSpeedInt = 1;
		}
		break;
		case 15:
		{
			shutterSpeedInt = 2;
		}
		break;
		case 30:
		{
			shutterSpeedInt = 3;
		}
		break;
		case 60:
		{
			shutterSpeedInt = 4;
		}
		break;
		case 90:
		{
			shutterSpeedInt = 5;
		}
		break;
		case 100:
		{
			shutterSpeedInt = 6;
		}
		break;
		case 125:
		{
			shutterSpeedInt = 7;
		}
		break;
		case 180:
		{
			shutterSpeedInt = 8;
		}
		break;
		case 250:
		{
			shutterSpeedInt = 9;
		}
		break;
		case 350:
		{
			shutterSpeedInt = 10;
		}
		break;
		case 500:
		{
			shutterSpeedInt = 11;
		}
		break;
		case 725:
		{
			shutterSpeedInt = 12;
		}
		break;
		case 1000:
		{
			shutterSpeedInt = 13;
		}
		break;
		case 1500:
		{
			shutterSpeedInt = 14;
		}
		break;
		case 2000:
		{
			shutterSpeedInt = 15;
		}
		break;
		case 3000:
		{
			shutterSpeedInt = 16;
		}
		break;
		case 4000:
		{
			shutterSpeedInt = 17;
		}
		break;
		case 6000:
		{
			shutterSpeedInt = 18;
		}
		break;
		case 10000:
		{
			shutterSpeedInt = 19;
		}
		break;
		default:
			break;
		}
		shutterSpeed = shutterSettings[shutterSpeedInt];
	}

	if (dataMap.find("Gain") != dataMap.end())
	{
		int parsedGain;
		std::stringstream buffer;
		buffer << dataMap["Gain"];
		buffer >> parsedGain;
		if(parsedGain >= 0 && parsedGain % 2 == 0)
		{
			gain = parsedGain / 2 + 1;
		}
		else if(parsedGain == -3)
		{
			gain = 0;
		}
	}

	if (dataMap.find("Focus") != dataMap.end())
	{
		focus = 128;
		if (boost::iequals(dataMap["Focus"], "+"))
		{
			focus+=6;
		}
		else if (boost::iequals(dataMap["Focus"], "-"))
		{
			focus+=5;
		}
		else if (boost::iequals(dataMap["Focus"], "Auto"))
		{
			autoFocus=!autoFocus;
		}
		else if (boost::iequals(dataMap["Focus"], "HighSensitivity"))
		{
			hiSensitivityFocus = !hiSensitivityFocus;
		}
		if(autoFocus)
			focus+=32;
		if(hiSensitivityFocus)
			focus+=64;

	}

	if (dataMap.find("Zoom") != dataMap.end())
	{
		std::stringstream buffer;
		if(boost::iequals(dataMap["Zoom"],"+"))
		{
			zoom = 4;
		}
		else if(boost::iequals(dataMap["Zoom"],"++"))
		{
			zoom = 7;
		}
		else if(boost::iequals(dataMap["Zoom"],"-"))
		{
			zoom = 65529;
		}
		else if(boost::iequals(dataMap["Zoom"],"--"))
		{
			zoom = 65532;
		}
		else
		{
			zoom = 0;
		}
	}

	if (dataMap.find("Tilt") != dataMap.end())
	{
		std::stringstream buffer;
		buffer << dataMap["Tilt"];
		buffer >> tilt;
		if(tilt>127)
			tilt=127;
		if(tilt<-128)
			tilt=-128;
	}

	if (dataMap.find("WB") != dataMap.end())
	{
		if (boost::iequals(dataMap["WB"], "Auto"))
		{
			whiteBalance=cameracommands::autoWB;
		}
		else if (boost::iequals(dataMap["WB"], "Indoor"))
		{
			whiteBalance=cameracommands::indoor;
		}
		else if (boost::iequals(dataMap["WB"], "Outdoor"))
		{
			whiteBalance=cameracommands::outdoor;
		}
		else if (boost::iequals(dataMap["WB"], "ATW"))
		{
			whiteBalance=cameracommands::atw;
		}
		else if (boost::iequals(dataMap["WB"], "Manual"))
		{
			whiteBalance=cameracommands::manualWB;
		}
	}

	if (dataMap.find("AE") != dataMap.end())
	{
		if (boost::iequals(dataMap["AE"], "Full"))
		{
			autoExposure=cameracommands::full;
		}
		else if (boost::iequals(dataMap["AE"], "Manual"))
		{
			autoExposure=cameracommands::manual;
		}
		else if (boost::iequals(dataMap["AE"], "Shutter"))
		{
			autoExposure=cameracommands::shutterPriority;
		}
		else if (boost::iequals(dataMap["AE"], "Iris"))
		{
			autoExposure=cameracommands::irisPriority;
		}
		else if (boost::iequals(dataMap["AE"], "Gain"))
		{
			autoExposure=cameracommands::gainPriority;
		}
		else if (boost::iequals(dataMap["AE"], "Bright"))
		{
			autoExposure=cameracommands::bright;
		}
		else if (boost::iequals(dataMap["AE"], "autoShutter"))
		{
			autoExposure=cameracommands::autoShutter;
		}
		else if (boost::iequals(dataMap["AE"], "autoIris"))
		{
			autoExposure=cameracommands::autoIris;
		}
		else if (boost::iequals(dataMap["AE"], "autoGain"))
		{
			autoExposure=cameracommands::autoGain;
		}
	}

	if (dataMap.find("Laser") != dataMap.end())
	{
		if (boost::iequals(dataMap["Laser"], "On"))
		{
			laserState=cameracommands::laserON;
		}
		else
		{
			laserState=cameracommands::laserOFF;
		}
	}
	return 0;
}




