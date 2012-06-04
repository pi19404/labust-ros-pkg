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
#ifndef SEAMORDRIVER_H_
#define SEAMORDRIVER_H_

#include <labust/vehicles/SeamorCommands.h>
#include <labust/vehicles/VehicleDriver.hpp>
#include <labust/plugins/PlugableDefs.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/xml/xmlfwd.hpp>

#include <boost/noncopyable.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/asio/serial_port.hpp>

namespace labust
{
	namespace vehicles
	{
		/**
		 * This class implements the Seamor vehicle driver.
		 */
		class SeamorDriver : public Driver, private boost::noncopyable
		{
			enum {maxTauAgeMs = 800};
		public:

			/**
			 * Constructor from a preloaded XML config file
			 * Searches the configuration file for a <vehicleDriverConfig> element of type "SeamorDriver" and optionally with a specified name
			 *
			 * \param reader XMLReader with a preloaded configuration file
			 * \param configToUse name of configuration element to use, if empty, first one will be used
			 * \throws VehicleException if configuration is not found in config file
			 */
			SeamorDriver(const labust::xml::ReaderPtr reader, std::string configToUse = "");

			/**
			 * Constructor from path to config file
			 * Searches the configuration file for a <vehicleDriverConfig> element of type "SeamorDriver" and optionally with a specified name
			 *
			 * \param configPath path to configuration file
			 * \param configToUse name of configuration element to use, if empty, first one will be used
			 * \throws VehicleException if configuration is not found in config file
			 */
			SeamorDriver(const std::string &configPath, std::string configToUse = "");

			virtual ~SeamorDriver();

			/**
			 * \see LABUST::VEHICLES::Driver::setTAU
			 */
			LABUST_EXPORT void setTAU(const labust::vehicles::tauMapRef tau);

			/**
			 * Future guidance algorithms may be governed using this function
			 *
			 * \see LABUST::VEHICLES::Driver::setGuidance
			 */
			LABUST_EXPORT void setGuidance(const labust::vehicles::guidanceMapRef guidance);
			/**
			 * \see LABUST::VEHICLES::Driver::setOptional
			 *
			 * Seamor optional commands:
			 *
			 * *** Driver related: ***
			 *
			 * Active: (no param) makes driver interface active (command from joystick)
			 * Passive: (no param) makes driver interface passive
			 * SendRovCommands: "ON", "OFF" - turns sending of rov commands in the extended dataset
			 * SendCameraCommands: "ON", "OFF" - turns sending of camera commands in the extended dataset
			 * SendCameraStatus: "ON", "OFF" - turns sending of camera status in the extended dataset
			 *
			 * *** Rov related: ***
			 *
			 * AuxMotor: "+","0","-"
			 * Autopilot: "Off", "AutoHeading", "AutoDepth", "On"
			 * VideoSelector: 0,1,2,3
			 * Manupulator: "+","0","-"
			 * Light: 0 - 127
			 *
			 * *** Camera related: ***
			 *
			 * RedGain:     0 - 255
			 * BlueGain:    0 - 255
			 * Iris:        closed, f22, f19, f16, f14, f11, f9.6, f8, f6.8, f5.6, f4.8, f4, f3.4, f2.8, f2.4, f2, f1.6, f1.4
			 * ShutterSpeed:1/4, 1/8, 1/15, 1/30, 1/60, 1/90, 1/100, 1/125, 1/180, 1/250, 1/350, 1/500, 1/725, 1/1000, 1/1500, 1/2000, 1/3000, 1/4000, 1/6000, 1/10000
			 * Gain:        -3, 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28 (dB)
			 * Focus:       Auto (toggle), HiSensitivity  (toggle), +, -
			 * Zoom:        ++ , +, -, -- (double=fast)
			 * Tilt:        -128 to 127
			 * WB:          Auto, indoor, outdoor, atw, manual
			 * AE:          full, manual, shutter, iris, || (others not working properly)gain, brightness, autoShutter, autoIris, autoGain
			 * Laser:       on, off
			 */
			void setCommand(const labust::vehicles::strMapRef data);

			void setCommand(const labust::apps::stringRef data){};

			/**
			 * \see LABUST::VEHICLES::Driver::getState
			 */
			LABUST_EXPORT void getState(labust::vehicles::stateMapRef state);

			/**
			 * \see LABUST::VEHICLES::Driver::getData
			 *
			 * Data provided:
			 *	Depth: depth of the ROV, metres
			 *	Heading: heading of the ROV, degrees
			 *	Pitch: pitch of ROV, degrees
			 *	Roll: roll of ROV, degrees
			 *	PortHI: current, port horizontal thruster, amps
			 *	StbdHI: current, starboard horizontal thruster, amps
			 *	PortVI: current, port vertical thruster, amps
			 *	StbdVI: current, starboard vertical thruster, amps
			 *	BallastI: current, ballast motor, amps
			 *	ManipulatorI: current, manipulator, amps
			 *	RovTemperature: temperature of rov electronic can, celsius
			 *	RovStatus: status of ROV
			 *	RovDataTimestamp: unix timestamp of most recent measurement
			 *
			 *	FOLLOWING DATA PROVIDED ONLY IF ROV COMMAND SENDING IS TURNED ON
			 *	AuxMotorCommand: 64, 0, -64
			 *	AutopilotCommand: 32 = off, 40 = Autoheading, 48 = autodepth, 56 = on
			 *	VideoSelector: 0,1,2,3
			 *	LightCommand: 0 - 127
			 *	ManipulatorCommand: 64, 0, -64
			 *	TauXCommand: -100 to 100
			 *	TauYCommand: -100 to 100
			 *	TauZCommand: -100 to 100
			 *	TauNCommand: -100 to 100
			 *
			 *	FOLLOWING DATA PROVIDED ONLY IF CAMERA STATUS SENDING IS TURNED ON
			 *	CameraBlueGain: 0 - 255
			 *	CameraFocusDistance: metres
			 *	CameraIris: see iris settings in commands
			 *	CameraGain: see gain settings in commands
			 *	CameraLightSetting: see gain settings in commands
			 *	CameraRedGain: 0 - 255
			 *	CameraTemperature: temperature of camera can, celsius
			 *	CameraTilt: degrees, -90 to 90
			 *	CameraZoom: 0 - 255
			 *	CameraDataTimestamp: timestamp of most recent data
			 *	CameraShutter: see shutter settings
			 *
			 *	FOLLOWING DATA PROVIDED ONLY IF CAMERA COMMAND SENDING IS TURNED ON
			 *	CameraAECommand: see commands in setoptional
			 *	CameraBlueGainCommand:
			 *	CameraFocusCommand:
			 *	CameraGainCommand:
			 *	CameraIrisCommand:
			 *	CameraLaserCommand:
			 *	CameraLightCommand:
			 *	CameraPanCommand:
			 *	CameraRedGainCommand:
			 *	CameraShutterCommand:
			 *	CameraTiltCommand:
			 *	CameraWBCommand:
			 *	CameraZoomCommand:
			 */
			void getData(const labust::apps::stringPtr data){};
			void getData(labust::vehicles::strMapPtr data);

		private:

			void serialFunc();

			void passiveSerial();

			void activeSerial();

			/**
			 * Vehicle force and torque limit
			 */
			labust::math::Limit<int> limit;
			/**
			 * Control variable for the comms thread
			 */

			int seamorResponseWait;

			bool running;
			/**
			 * Control variable that determines whether we are only listening
			 */
			bool isPassive;

			bool sendRovCommands, sendCameraCommands, sendCameraStatus;

			/**
			 * Seamor vehicle state
			 */
			labust::communication::SeamorStatus rovState;
			/**
			 * Seamor console command
			 */
			labust::communication::SeamorCommand rovCommand;
			/**
			 * Input thread
			 */
			boost::thread commsThread;

			boost::asio::io_service io;
			boost::asio::serial_port serialPort;
			boost::mutex commsMutex;

			labust::communication::SeamorCameraStatus cameraStatus;

			labust::communication::SeamorCameraCommand cameraCommand;

			boost::posix_time::ptime lastTauTime;
		};
	};
};
/* SEAMORDRIVER_H_ */
#endif
