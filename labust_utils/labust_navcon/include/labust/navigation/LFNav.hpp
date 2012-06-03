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
#ifndef LFNAV_HPP_
#define LFNAV_HPP_
#include <labust/navigation/navfwd.hpp>
#include <labust/navigation/NavDriver.hpp>
#include <labust/navigation/KFCore.hpp>
#include <labust/navigation/LFModel.hpp>
#include <labust/xml/xmlfwd.hpp>

#include <deque>

namespace labust
{
	namespace navigation
	{
		/**
		 * The class implements the LineFollowing navigation.
		 *
		 * \todo Add configuration for the delay.
		 * \todo Specify commands for line recalculation.
		 * \todo Specify commands for filter model reconfiguration.
		 */
		class LFNav : public virtual Driver
		{
			/**
			 * The EKF with line following model filter class.
			 */
			typedef labust::navigation::KFCore<labust::navigation::LFModel> Nav;
		public:
			/**
			 * Main constructor. Take a XML reader pointer and configures the controller.
			 *
			 * \param reader Pointer to the XMLReader object containing the parameters.
			 * \param id Identification class.
			 */
			LFNav(const labust::xml::ReaderPtr reader, const std::string& id = "");

			/**
			 *	The method performs the prediction step based on the given force input vector.
			 *
			 *	\param tau The input forces and torques vector.
			 */
			void prediction(const labust::vehicles::tauMapRef tau);
			/**
			 * The method performs the correction step of the filter based on supplied state measurement
			 * vector.
			 *
			 * \param measurement The measurement data.
			 * \param stateHat The estimated state values.
			 */
			void correction(const labust::vehicles::stateMapRef measurement, labust::vehicles::stateMapPtr stateHat);
      /**
       * The method can be used to command additional configuration options.
       *
       * \param data User specific map.
       */
      void setCommand(const labust::apps::stringRef cmd);
      /**
       * This method is used to return random data.
       *
       * \param data Peripheral data values will be returned
       */
      void getData(labust::apps::stringPtr data){};

		private:
      /**
       * Configures the driver using an XML file.
       *
       * \param reader Pointer to the XMLReader object containing the parameters.
			 * \param id Identification class.
       */
			void configure(const labust::xml::ReaderPtr& reader, const std::string& id);
			/**
			 * The method recalculates the filter when delayed measurements arrive.
			 */
			void recalculate(const labust::vehicles::stateMapRef measurement);
			/**
			 * Reset surge estimate. Added to avoid estimating the surge speed.
			 */
			void resetSurge(double u);


			/**
			 * The EKF filter.
			 */
			Nav nav;
			/**
			 * The measurement and input buffer.
			 */
			std::deque<Nav::vector > measM, tauM;
			/**
			 * The saved state vector.
			 */
			Nav::output_type stateM;
			/**
			 * The saved covariance matrix.
			 */
			Nav::matrix covM;
			/**
			 * The iteration counter.
			 */
			size_t iteration;
			/**
			 * Number of iterations delayed and iteration update frequency.
			 */
			size_t delayMeas, receiveFreq;
		};
	}
}

 /* LFNAV_HPP_ */
#endif
