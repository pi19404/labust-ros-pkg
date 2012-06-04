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
#ifndef VRPRO_HPP_
#define VRPRO_HPP_
#include <labust/vehicles/VRComms.hpp>
#include <labust/vehicles/VehicleDriver.hpp>
#include <labust/xml/xmlfwd.hpp>
#include <labust/plugins/PlugableDefs.hpp>
#include <labust/math/Limits.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The class implements the vehicle driver for the VideoRay family.
		 *
		 * \todo Add thread for receive avoids blocking on setTau if vehicle is disconnected.
		 * \todo Synced comms eliminates the threading need and will work fine.
		 * \todo Add automatic protocol identification.
		 * \todo Add automatic serial port detection and generalize.
		 */
		class VRPro : public virtual Driver
		{
		public:
			/**
			 * Main constructor. Take a XML reader pointer and configures the controller.
			 *
			 * \param reader Pointer to the XMLReader object containing the parameters.
			 * \param id Identification class.
			 */
			VRPro(const labust::xml::ReaderPtr reader, const std::string& id = "");
			/**
			 * Generic destructor.
			 */
			~VRPro();

			/**
			 * \overload labust::vehicles::Driver::setTAU
			 */
			LABUST_EXPORT void setTAU(const labust::vehicles::tauMapRef tau);
			/**
			 * \overload labust::vehicles::Driver::getState
			 */
			LABUST_EXPORT void getState(labust::vehicles::stateMapRef state);
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
			 * The method configures the driver through a XML config file.
			 *
			 * \param reader Pointer to the XMLReader object containing the parameters.
			 * \param id Identification class.
			 */
			void configure(const labust::xml::ReaderPtr& reader, const std::string& id);
			/**
			 * The method handles incoming data.
			 *
			 * \param e The error that occured during transfer.
			 * \param transferred Number of bytes that were transfered.
			 */
			void handleInput(const boost::system::error_code& error, const size_t transferred);
			/**
			 * The method starts the receiving loop.
			 */
			void start_receive();
			/**
			 * The method calculates the depth scale parameters.
			 *
			 * \param low The low pressure value.
			 * \param high The high pressure value.
			 * \param psiHigh The high calibration pressure.
			 * \param psiLow The low calibration pressure.
			 */
			void depthScale(int low, int high, double psiHigh, double psiLow = 0);

			/**
			 * Serial port I/O service.
			 */
			boost::asio::io_service io;
			/**
			 * Serial port.
			 */
			boost::asio::serial_port port;
			/**
			 * The communication layer.
			 */
			VRCommsPtr comms;
			/**
			 * The thruster vector.
			 */
			VRComms::ThrustVec thrusters;
			/**
			 * Revolution limiter.
			 */
			labust::math::Limit<int> revLimit;
			/**
			 * The light limit.
			 */
			labust::math::Limit<int> lightLimit;
			/**
			 * Thruster affine mappings.
			 */
			double Tnn,_Tnn;
			/**
			 * The vehicle state
			 */
			labust::vehicles::stateMapPtr states;
			/**
			 * Depth scale parameters.
			 */
			double xDepth, yDepth;
			/**
			 * The heading unwrapper.
			 */
			labust::math::unwrap unwrap;
		};
	}
}
/* VRPRO_HPP_ */
#endif
