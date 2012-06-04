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
#include <labust/vehicles/SeamorDriver.h>
#include <labust/tools/TimingTools.hpp>
#include <labust/comms/SerialConfig.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/plugins/PlugableDefs.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>

using namespace labust::vehicles;

SeamorDriver::SeamorDriver(const labust::xml::ReaderPtr reader, std::string configToUse):
		limit(-128, 127),
    seamorResponseWait(100),
    running(false),
    isPassive(true),
    sendRovCommands(false),
    sendCameraCommands(false),
    sendCameraStatus(false),
    serialPort(io),
    lastTauTime(boost::posix_time::microsec_clock::universal_time())
{
	std::string configQuery;
	if (configToUse.empty())
	{
		configQuery = "vehicleDriverConfig[@type='seamorDriver']";
	}
	else
	{
		configQuery = "vehicleDriverConfig[@type='seamorDriver' and @name='" + configToUse + "']";
	}

	std::cout << "Configuring Seamor driver" << std::endl;

	_xmlNode* configNode = NULL;

	if (reader->try_value(configQuery, &configNode))
	{
		std::string tmp = "";
		reader->useNode(configNode);
		if(reader->try_value("param[@name='RovCommandsSending']/@value", &tmp) && boost::iequals(tmp,std::string("ON")))
		{
			sendRovCommands = true;
		}

		if(reader->try_value("param[@name='CameraCommandsSending']/@value", &tmp) && boost::iequals(tmp,std::string("ON")))
		{
			sendCameraCommands = true;
		}

		if(reader->try_value("param[@name='CameraStatusSending']/@value", &tmp) && boost::iequals(tmp,std::string("ON")))
		{
			sendCameraStatus = true;
		}

		if(reader->try_value("param[@name='Joystick']/@value", &tmp) && boost::iequals(tmp,std::string("ON")))
		{
			isPassive = false;
		}

		/*
		 * Added for testing.
		isPassive = false;
		sendRovCommands = true;
		sendCameraCommands = true;
		sendCameraStatus = true;
		*/

		reader->try_value("param[@name='SeamorResponseWait']/@value", &seamorResponseWait);

		labust::comms::serial_configure(*reader,serialPort);
		running = true;
		commsThread = boost::thread(boost::bind(&SeamorDriver::serialFunc,this));
		std::cout << "Seamor driver is running" << std::endl;
	}
	else
	{
		if(configToUse.empty())
		{
			throw std::runtime_error("Unable to start Seamor driver, no configurations of type \"SeamorDriver\" found.");
		}
		else
		{
			throw std::runtime_error("Unable to start Seamor driver, configuration of type \"SeamorDriver\" with name \""+configToUse+"\" found.");
		}
	}
};

SeamorDriver::SeamorDriver(const std::string &configPath, std::string configToUse):
		limit(-128, 127),
		seamorResponseWait(100),
		running(false),
		isPassive(true),
		sendRovCommands(false),
		sendCameraCommands(false),
		sendCameraStatus(false),
		serialPort(io),
		lastTauTime(boost::posix_time::microsec_clock::universal_time())
{
	labust::xml::Reader reader(configPath,true);
	std::string configQuery;
	if (configToUse.empty())
	{
		configQuery = "//vehicleDriverConfig[@type='seamorDriver']";
	}
	else
	{
		configQuery = "//vehicleDriverConfig[@type='seamorDriver' and @name='" + configToUse + "']";
	}


	std::cerr << "Configuring Seamor driver " << std::endl;

	reader.useRootNode();
	_xmlNode* configNode = NULL;

	if (reader.try_value(configQuery, &configNode))
	{
		std::string tmp = "";
		reader.useNode(configNode);
		if(reader.try_value("param[@name='RovCommandsSending']/@value", &tmp) && boost::iequals(tmp,std::string("ON")))
		{
			sendRovCommands = true;
		}

		if(reader.try_value("param[@name='CameraCommandsSending']/@value", &tmp) && boost::iequals(tmp,std::string("ON")))
		{
			sendCameraCommands = true;
		}

		if(reader.try_value("param[@name='CameraStatusSending']/@value", &tmp) && boost::iequals(tmp,std::string("ON")))
		{
			sendCameraStatus = true;
		}
		if(reader.try_value("param[@name='Joystick']/@value", &tmp) && boost::iequals(tmp,std::string("ON")))
		{
			isPassive = false;
		}

		reader.try_value("param[@name='SeamorResponseWait']/@value", &seamorResponseWait);

		labust::comms::serial_configure(reader,serialPort);
		running = true;
		commsThread = boost::thread(boost::bind(&SeamorDriver::serialFunc,this));
		std::cout << "Seamor driver is running" << std::endl;
	}
	else
	{
		if(configToUse.empty())
		{
			throw std::runtime_error("Unable to start Seamor driver, no configurations of type \"SeamorDriver\" found.");
		}
		else
		{
			throw std::runtime_error("Unable to start Seamor driver, no configuration of type \"SeamorDriver\" with name \""+configToUse+"\" found.");
		}
	}
};

SeamorDriver::~SeamorDriver()
{
	running = false;
	commsThread.join();
};

void SeamorDriver::setTAU(const labust::vehicles::tauMapRef tau)
{
	/**
	 * SeamorDriver protocol does the allocation on the vehicle
	 * We set TAU forces directly. Force and torque domain
	 * is [-128,127>.
	 */

	if (!isPassive)
	{
		lastTauTime = boost::posix_time::microsec_clock::universal_time();
		/*
		COREDATASET::tauMap::const_iterator element;
		double tauX, tauY, tauZ, tauN;
		element = tau.find(COREDATASET::X);
		tauX = element != tau.end() ? element->second : 0;
		element = tau.find(COREDATASET::Y);
		tauY = element != tau.end() ? element->second : 0;
		element = tau.find(COREDATASET::Z);
		tauZ = element != tau.end() ? element->second : 0;
		element = tau.find(COREDATASET::N);
		tauN = element != tau.end() ? element->second : 0;
		*/

		using namespace labust::math;
		using namespace labust::vehicles;
		tauMap tauC;

		tauC[tau::X] = coerce(tau[tau::X], limit);
		tauC[tau::Y] = coerce(tau[tau::Y], limit);
		tauC[tau::Z] = coerce(tau[tau::Z], limit);
		tauC[tau::N] = coerce(tau[tau::N], limit);

		boost::mutex::scoped_lock lockRovCommand(rovCommand.mutex);
		if (rovCommand.parseTau(tauC))
		{
			throw std::runtime_error("Seamor::setCommand: rovCommand parsing error");
		}
	}
};

void SeamorDriver::setCommand(const labust::vehicles::strMapRef data)
{
	if(data.find("Active")!=data.end())
	{
		isPassive = false;
	}
	if(data.find("Passive")!=data.end())
	{
		isPassive = true;
	}
	labust::vehicles::strMap::const_iterator element;
	element = data.find("RovCommandsSending");
	if(element!=data.end())
	{
		sendRovCommands = boost::iequals(element->second,std::string("ON"));
	}

	element = data.find("CameraCommandsSending");
	if(element!=data.end())
	{
		sendCameraCommands = boost::iequals(element->second,std::string("ON"));
	}

	element = data.find("CameraStatusSending");
	if(element!=data.end())
	{
		sendCameraStatus = boost::iequals(element->second,std::string("ON"));
	}
	if (!isPassive)
	{
		labust::vehicles::strMap dataMap = data;
		boost::mutex::scoped_lock lockRovCommand(rovCommand.mutex);
		if (rovCommand.parseDataMap(dataMap))
		{
			throw std::runtime_error("Seamor::setCommand: rovCommand parsing error.");
		}

		lockRovCommand.unlock();
		boost::mutex::scoped_lock lockCameraCommand(cameraCommand.mutex);
		if (cameraCommand.parseDataMap(dataMap))
		{
			throw std::runtime_error("Seamor::setCommand: cameraCommand parsing error.");
		}
	}
}

void SeamorDriver::getState(labust::vehicles::stateMapRef states)
{
	boost::mutex::scoped_lock lockRovCommand(rovCommand.mutex);
	using namespace labust::vehicles;
	states[state::X_e] = rovCommand.tauC[tau::X];
	states[state::Y_e] = rovCommand.tauC[tau::Y];
	states[state::Z_e] = rovCommand.tauC[tau::Z];
	states[state::N_e] = rovCommand.tauC[tau::N];

	boost::mutex::scoped_lock lock(rovState.mutex);
	states[state::yaw] = labust::math::wrapDeg(rovState.heading) * M_PI / 180;
	states[state::z] = rovState.depth;
	states[state::roll] = rovState.roll * M_PI / 180;
	states[state::pitch] = rovState.pitch * M_PI / 180;
}

void SeamorDriver::getData(labust::vehicles::strMapPtr data2)
{
	labust::vehicles::strMapRef data = *data2;
	std::stringstream buffer;
	{
		boost::mutex::scoped_lock lock(rovState.mutex);
		buffer << rovState.depth << " "
				<< rovState.heading << " "
				<< rovState.pitch << " "
				<< rovState.roll << " "
				<< rovState.portHI << " "
				<< rovState.stbdHI << " "
				<< rovState.portVI << " "
				<< rovState.stbdVI << " "
				<< rovState.ballastI << " "
				<< rovState.manipI << " "
				<< rovState.temperature << " "
				<< rovState.status << " "
				<< rovState.timeStamp << " ";
	}

	if(sendRovCommands)
	{
		using namespace labust::vehicles;
		boost::mutex::scoped_lock lock(rovCommand.mutex);
		buffer << rovCommand.ballastF << " "
				<< rovCommand.autopilotCommand << " "
				<< rovCommand.videoSelector << " "
				<< rovCommand.lightByte << " "
				<< rovCommand.manipByte << " "
				<< rovCommand.tauC[tau::X] << " "
				<< rovCommand.tauC[tau::Y] << " "
				<< rovCommand.tauC[tau::Z] << " "
				<< rovCommand.tauC[tau::N] << " ";

	}

	if(sendCameraStatus)
	{
		boost::mutex::scoped_lock lock(cameraStatus.mutex);
		buffer << cameraStatus.blueGain << " "
				<< cameraStatus.focusDistance << " "
				<< cameraStatus.iris << " "
				<< cameraStatus.gain << " "
				<< cameraStatus.lightSetting << " "
				<< cameraStatus.redGain << " "
				<< cameraStatus.temperature << " "
				<< cameraStatus.tilt << " "
				<< cameraStatus.zoom << " "
				<< cameraStatus.timeStamp << " "
				<< cameraStatus.shutterSpeed << " ";
	}

	if(sendCameraCommands)
	{
		boost::mutex::scoped_lock lock(cameraCommand.mutex);
		buffer << cameraCommand.autoExposure << " "
				<< cameraCommand.blueGain << " "
				<< cameraCommand.focus << " "
				<< cameraCommand.gain << " "
				<< cameraCommand.iris << " "
				<< cameraCommand.laserState << " "
				<< cameraCommand.light << " "
				<< cameraCommand.pan << " "
				<< cameraCommand.redGain << " "
				<< cameraCommand.shutterSpeed << " "
				<< cameraCommand.tilt << " "
				<< cameraCommand.whiteBalance << " "
				<< cameraCommand.zoom << " ";

	}
	//std::cout<<buffer.str()<<std::endl;
	buffer >> data["Depth"]
	       >> data["Heading"]
	       >> data["Pitch"]
	       >> data["Roll"]
	       >> data["PortHI"]
	       >> data["StbdHI"]
	       >> data["PortVI"]
	       >> data["StbdVI"]
	       >> data["BallastI"]
	       >> data["ManipulatorI"]
	       >> data["RovTemperature"]
	       >> data["RovStatus"]
	       >> data["RovDataTimestamp"];
	if(sendRovCommands)
	{
		buffer >> data["AuxMotorCommand"]
		       >> data["AutopilotCommand"]
		       >> data["VideoSelector"]
		       >> data["LightCommand"]
		       >> data["ManipulatorCommand"]
		       >> data["TauXCommand"]
		       >> data["TauYCommand"]
		       >> data["TauZCommand"]
		       >> data["TauNCommand"];
	}

	if(sendCameraStatus)
	{
		buffer >> data["CameraBlueGain"]
		       >> data["CameraFocusDistance"]
		       >> data["CameraIris"]
		       >> data["CameraGain"]
		       >> data["CameraLightSetting"]
		       >> data["CameraRedGain"]
		       >> data["CameraTemperature"]
		       >> data["CameraTilt"]
		       >> data["CameraZoom"]
		       >> data["CameraDataTimestamp"]
		       >> data["CameraShutter"];
	}
	if(sendCameraCommands)
	{
		buffer >> data["CameraAECommand"]
		       >> data["CameraBlueGainCommand"]
		       >> data["CameraFocusCommand"]
		       >> data["CameraGainCommand"]
		       >> data["CameraIrisCommand"]
		       >> data["CameraLaserCommand"]
		       >> data["CameraLightCommand"]
		       >> data["CameraPanCommand"]
		       >> data["CameraRedGainCommand"]
		       >> data["CameraShutterCommand"]
		       >> data["CameraTiltCommand"]
		       >> data["CameraWBCommand"]
		       >> data["CameraZoomCommand"];
	}
}

void SeamorDriver::serialFunc()
{
	while(running)
	{//std::cout<<"sf"<<std::endl;
		if (isPassive)
		{
			passiveSerial();
		}
		else
		{
			activeSerial();
		}
	}
}

void SeamorDriver::passiveSerial()
{
	typedef unsigned char uint8;
	char firstByte, secondByte;
	//for init, read a byte from input
	boost::asio::read(serialPort,boost::asio::buffer((void*)&firstByte,1));

	while (running)
	{
		if(!isPassive)
			break;
		boost::asio::read(serialPort,boost::asio::buffer((void*)&secondByte,1));

		//when a seamor message is found, read it entirely.
		// reads also read two extra bytes, the two extra bytes at the
		// end are stored as new first and second byte for further parsing
		if (firstByte == labust::communication::seamorrawdata::cameraStatus && secondByte == labust::communication::seamorrawdata::cameraStatusLength)
		{
			int length = labust::communication::seamorrawdata::cameraStatusLength - 1;
			uint8 *data = new uint8[length];
			boost::asio::read(serialPort,boost::asio::buffer(data,length));

			//std::cout << "cs" << std::endl;
			boost::mutex::scoped_lock lockCameraStatus(cameraStatus.mutex);
			cameraStatus.parseBinary(data, (length-1));
			cameraStatus.timeStamp = labust::tools::unix_time_string(true);
			firstByte = (int) (data)[length-1];
			delete(data);
		}
		else if (firstByte == labust::communication::seamorrawdata::cameraControl && secondByte == labust::communication::seamorrawdata::cameraControlLength)
		{
			int length = labust::communication::seamorrawdata::cameraControlLength - 1;
			uint8 *data = new uint8[length];
			boost::asio::read(serialPort,boost::asio::buffer(data,length));
			// std::cout << "cc" << std::endl;
			if (isPassive)
			{
				boost::mutex::scoped_lock lockCameraCommand(cameraCommand.mutex);
				cameraCommand.parseBinary(data, (length - 1));
			}
			firstByte = (int) (data[length - 1]);
			delete(data);
		}
		else if (firstByte == labust::communication::seamorrawdata::rovControl && secondByte == labust::communication::seamorrawdata::rovControlLength)
		{
			int length = labust::communication::seamorrawdata::rovControlLength - 1;
			uint8 *data = new uint8[length];
			boost::asio::read(serialPort,boost::asio::buffer(data,length));
			//  std::cout << "sc" << std::endl;
			if (isPassive)
			{
				boost::mutex::scoped_lock lockRovCommand(rovCommand.mutex);
				rovCommand.parseBinary(data, (length - 1));
			}
			firstByte = (int) (data[length - 1]);
			delete(data);
		}
		else if (firstByte == labust::communication::seamorrawdata::rovStatus && secondByte == labust::communication::seamorrawdata::rovStatusLength)
		{
			int length = labust::communication::seamorrawdata::rovStatusLength - 1;
			uint8 *data = new uint8[length];
			boost::asio::read(serialPort, boost::asio::buffer(data, length));
			//  std::cout << "ss" << std::endl;
			boost::mutex::scoped_lock lockRovState(rovState.mutex);
			rovState.parseBinary(data, (length - 1));
			rovState.timeStamp = labust::tools::unix_time_string(true);
			firstByte = (int) (data[length - 1]);
			delete(data);
		}
		else
		{
			firstByte = secondByte;
		}
	}

}

void SeamorDriver::activeSerial()
{
	typedef unsigned char uint8;
	uint8 rovCommandData[labust::communication::seamorrawdata::rovControlLength];
	uint8 cameraCommandData[labust::communication::seamorrawdata::cameraControlLength];

	char firstByte, secondByte;
	boost::asio::read(serialPort, boost::asio::buffer(&firstByte, 1));
	while (running)
	{
		if(isPassive)
			break;

		boost::posix_time::time_duration tauAge = boost::posix_time::microsec_clock::universal_time() - lastTauTime;

		if (tauAge.total_milliseconds() > maxTauAgeMs)
		{ //if tau is too old, set it to zero
			boost::mutex::scoped_lock lockRovCommand(rovCommand.mutex);
			using namespace labust::vehicles;
			rovCommand.tauC[tau::X] =
			rovCommand.tauC[tau::Y] =
			rovCommand.tauC[tau::Z] =
			rovCommand.tauC[tau::K] =
			rovCommand.tauC[tau::M] =
			rovCommand.tauC[tau::N] = 0;
		}


		boost::asio::read(serialPort, boost::asio::buffer(&secondByte, 1));

		if (firstByte == labust::communication::seamorrawdata::rovStatus && secondByte == labust::communication::seamorrawdata::rovStatusLength)
		{
			int length = labust::communication::seamorrawdata::rovStatusLength - 2;
			uint8 *data = new uint8[length];
			boost::asio::read(serialPort,boost::asio::buffer(data,length));

			boost::mutex::scoped_lock lockRovState(rovState.mutex);
			rovState.parseBinary(data, length);
			rovState.timeStamp = labust::tools::unix_time_string(true);
			lockRovState.unlock();
			delete(data);

			{
				boost::mutex::scoped_lock lockCameraCommand(cameraCommand.mutex);
				cameraCommand.getBinary(cameraCommandData);
				lockCameraCommand.unlock();
				boost::asio::write(serialPort,boost::asio::buffer(cameraCommandData,labust::communication::seamorrawdata::cameraControlLength));

			}
			boost::this_thread::sleep(boost::posix_time::milliseconds(seamorResponseWait));
			boost::asio::read(serialPort, boost::asio::buffer(&firstByte, 1));
		}
		else if (firstByte == labust::communication::seamorrawdata::cameraStatus && secondByte == labust::communication::seamorrawdata::cameraStatusLength)
		{
			int length = labust::communication::seamorrawdata::cameraStatusLength - 2;
			uint8 *data = new uint8[length];
			boost::asio::read(serialPort,boost::asio::buffer(data,length));

			boost::mutex::scoped_lock lockCameraState(cameraStatus.mutex);
			cameraStatus.parseBinary(data, length);
			cameraStatus.timeStamp = labust::tools::unix_time_string(true);
			lockCameraState.unlock();
			delete(data);

			{
				boost::mutex::scoped_lock lockRovCommand(rovCommand.mutex);
				rovCommand.getBinary(rovCommandData);
				boost::asio::write(serialPort, boost::asio::buffer(rovCommandData, labust::communication::seamorrawdata::rovControlLength));

			}
			boost::this_thread::sleep(boost::posix_time::milliseconds(seamorResponseWait));
			boost::asio::read(serialPort, boost::asio::buffer(&firstByte, 1));
		}
		else
		{
			firstByte = secondByte;
		}
	}
}

void SeamorDriver::setGuidance(const labust::vehicles::guidanceMapRef guidance)
{
	throw std::runtime_error("SeamorDriver::setGuidance not implemented.");
}

LABUST_EXTERN
{
	LABUST_EXPORT VehicleFactoryPtr createVehicleFactory()
  {
		return VehicleFactoryPtr(new VehicleFactory::Impl<SeamorDriver>());
  }
};












