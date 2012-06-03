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
#ifndef SEAMOR_HPP_
#define SEAMOR_HPP_
#include <labust/vehicles/SeamorComms.hpp>
#include <labust/vehicles/VehicleDriver.hpp>
#include <labust/plugins/PlugableDefs.hpp>
#include <labust/math/Limits.hpp>
#include <labust/xml/xmlfwd.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/system/error_code.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_service.hpp>

#include <vector>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The class is the implementation of the Seamor vehicle control driver.
		 *
		 * \todo Polymorphic messages.
		 */
		class Seamor : public virtual Driver
		{
			typedef boost::shared_ptr<boost::thread> ThreadPtr;

			enum {Sync=0,CameraStatus, CameraCommand, Status, Command};

		public:
			/**
			 * Main constructor that configures the vehicle from XML configuration file.
			 *
			 * \param reader Pointer to the reader with the configuration XML.
			 * \param id Identification name of the configuration element.
			 */
			Seamor(const labust::xml::ReaderPtr reader, const std::string& id = "");
			/**
			 * Generic destructor.
			 */
			~Seamor();

			/**
			 * \overload labust::vehicles::Driver::setTAU
			 */
			LABUST_EXPORT void setTAU(const labust::vehicles::tauMapRef tau);
			/**
			 * \overload labust::vehicles::Driver::getState
			 */
			LABUST_EXPORT void getState(labust::vehicles::stateMapPtr states);
			/**
			 *	\overload labust::vehicles::Driver::setGuidance
			 */
			LABUST_EXPORT void setGuidance(const labust::vehicles::guidanceMapRef guidance);
			/**
			 * \overload labust:::vehicles::Driver::setCommand
			 */
			LABUST_EXPORT void setCommand(const labust::apps::stringRef commands);
			/**
			 * \overload labust::vehicles::Driver::getData
			 */
			LABUST_EXPORT void getData(const labust::apps::stringPtr data);

		private:
			/**
			 * The method configures the class with the XML configuration.
			 *
			 * \param reader Pointer to the reader with the configuration XML.
			 * \param id Identification name of the configuration element.
			 */
			void configure(const labust::xml::ReaderPtr reader, const std::string& id);
			/**
			 * Starts the data receiving process.
			 */
			void start_receive();
			/**
			 * The method handles incoming data.
			 *
			 * \param e The error that occured during transfer.
			 * \param transferred Number of bytes that were transfered.
			 */
			void handleInput(const boost::system::error_code& error, const size_t transferred);
			/**
			 * The method handles incoming data.
			 *
			 * \param e The error that occured during transfer.
			 * \param transferred Number of bytes that were transfered.
			 */
			void handleOutput(const boost::system::error_code& error, const size_t transferred);
			/**
			 * The method checks if the receive is synchronized.
			 */
			void checkSync();

			/**
			 * I/O service for the serial port.
			 */
			boost::asio::io_service io;
			/**
			 * The serial port.
			 */
			boost::asio::serial_port port;
			/**
			 * The incoming thread handler.
			 */
			ThreadPtr comms;
			/**
			  * The next message and desired read size.
				*/
			size_t nextMsg, numToRead;
			/**
			 * The communication input buffer.
			 */
			std::vector<unsigned char> inputBuffer;
			/**
			 * The communication output buffer.
			 */
			std::vector<unsigned char> outputBuffer;
			/**
			 * ROV status.
			 */
			SeamorStatus rovStatus;
			/**
			 * ROV command.
			 */
			SeamorCommand rovCommand;
			/**
			 * Camera status.
			 */
			SeamorCameraStatus cameraStatus;
			/**
			 * Camera command.
			 */
			SeamorCameraCommand cameraCommand;
			/**
			 * I/O thread.
			 */
			boost::thread commsThread;
			/**
			 * Data mutex.
			 */
			boost::mutex dataMux;
			/**
			 * Thruster affine mappings.
			 */
			double Tnn,_Tnn;
			/**
			 * The revolution limit.
			 */
			const labust::math::Limit<int> revLimit;
		};
	}
}


/* SEAMOR_HPP_ */
#endif
