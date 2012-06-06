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
#ifndef HDCONTROLLER_HPP_
#define HDCONTROLLER_HPP_
#include <labust/control/ControlDriver.hpp>
#include <labust/xml/xmlfwd.hpp>
#include <labust/plugins/PlugableDefs.hpp>
#include <labust/vehicles/vehiclesfwd.hpp>
#include <labust/control/PIDController.hpp>
#include <labust/control/controltune.hpp>

namespace labust
{
	namespace control
	{
		/**
		 * The generic heading and depth controller for simple ROVs.
		 */
		class HDController : public virtual Driver
		{
		public:
			/**
			 * Main constructor. Take a XML reader pointer and configures the controller.
			 *
			 * \param reader Pointer to the XMLReader object containing the parameters.
			 */
			HDController(const labust::xml::ReaderPtr reader);

			/**
			 * \override labust::apps::App::setCommand.
			 */
			LABUST_EXPORT void setCommand(const labust::apps::stringRef cmd);
			/**
			 * \override labust::apps::App::getDate.
			 */
			LABUST_EXPORT void getData(const labust::apps::stringPtr data);

			/**
			 * \override labust::control::Driver::getTAU
			 */
			LABUST_EXPORT void getTAU(const labust::vehicles::stateMapRef stateRef,
					const labust::vehicles::stateMapRef state, const labust::vehicles::tauMapRef tau);

			PP_LABUST_IN_CLASS_ADAPT_TO_XML(HDController)

		private:
			/**
			 * Configures the controller based on the supplied XML.
			 *
			 * \param reader Pointer to the XMLReader object containing the parameters.
			 */
			void configure(const labust::xml::ReaderPtr reader);

			/**
			 * The heading and depth IPD controller.
			 */
			labust::control::IPD heading, depth;
			/**
			 * The tuning parameters.
			 */
			labust::control::TuningParameters headingParams, depthParams;
		};

		PP_LABUST_MAKE_CLASS_XML_OPERATORS(,HDController,
				(TuningParameters, headingParams)
				(TuningParameters, depthParams))

	  #include<labust/xml/adapt_class_undef.hpp>
	}
}

/* HDCONTROLLER_HPP_ */
#endif
