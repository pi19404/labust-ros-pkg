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
#ifndef LFCONTROLLER_HPP_
#define LFCONTROLLER_HPP_
#include <labust/control/ControlDriver.hpp>
#include <labust/xml/xmlfwd.hpp>
#include <labust/control/PIDController.hpp>
#include <labust/plugins/PlugableDefs.hpp>

#include <boost/array.hpp>

namespace labust
{
	namespace control
	{
		/**
		 * The generic line following controller. Implements 3D line following as specified in
		 * Mišković, Nikola; Nađ, Đula; Vukić, Zoran: "3D Line Following for Unmanned Underwater Vehicles",
		 * Brodogradnja, 61 (2010) , 2; 121-129.
		 *
		 * \todo Consider keeping a Parameter map once loaded from the XML if needed.
		 * \todo Initialization of the PD derivator ? or yaw-rate reference limit.
		 * \todo Run-time parameter retuning.
		 * \todo Surge speed in parameter recalculation.
		 */
		class LFController : public virtual Driver
		{
		public:
			/**
			 * Main constructor. Take a XML reader pointer and configures the controller.
			 *
			 * \param reader Pointer to the XMLReader object containing the parameters.
			 */
			LFController(const labust::xml::ReaderPtr reader);

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
					const labust::vehicles::stateMapRef state, const labust::vehicles::tauMapPtr tau);
			/**
			 * The method tunes the parameters of the controller based on the desired
			 * binomial transfer function and model parameters.
			 * Following parameters are required for the horizontal controller:
			 *  1. w_h - frequency of the horizontal controller binomial,
			 *  2. alpha_r - inertia parameter of the yaw-rate model,
			 *  3. beta_r or beta_rr - linear or quadratic damping parameter of the yaw-rate model,
			 *  4. Max_N - torque limit.
			 * Following parameters are required for the vertical controller:
			 *  1. w_v - frequency of the vertical controller binomial,
			 *  2. alpha_w - inertia parameter of the heave model,
			 *  3. beta_w - linear damping parameter of the heave model,
			 *  4. Xi - the line orientation in the vertical plane,
			 *  5. Max_Z - force limit.
			 * Following parameters are required:
			 *  Ts - sampling time in seconds.
			 * Following parameters are optional:
			 *  attack_angle - the maximum horizontal approach angle.
			 *
			 * \param data Parameter map.
			 *
			 * \return Returns true if the controller tuning was successful.
			 */
			bool tuneController(const labust::vehicles::dataMapRef data);
			/**
			 * The method override allows for vertical controller re-tuning when
			 * the line orientation changes.
			 *
			 * \param Xi The vertical orientation of the line.
			 */
			bool tuneController(double Xi);
			/**
			 * The method returns the parameter map for possible runtime changes.
			 */
			inline labust::vehicles::dataMapRef getParamMap(){return this->paramMap;};
			/**
			 * Method that checks if the controller is tunned.
			 *
			 * \return True if the controller is tunned. False otherwise.
			 */
			inline bool isTunned(){return this->tunned;};

		private:
			/**
			 * Configures the controller based on the XML file.
			 *
			 * \param reader Pointer to the XMLReader object containing the parameters.
			 */
			void configure(const labust::xml::ReaderPtr& reader);
			/**
			 * Performs real-time adjustment of the line-following controller.
			 *
			 * \param surge The surge speed of the vehicle.
			 */
			void adjustDH(double surge);
			/**
			 * The horizontal distance PD controller. It has a
			 * limit on the P output value.
			 */
			lfPD dh_controller;
			/**
			 * The yaw-rate and vertical distance controllers.
			 */
			IPD r_controller,dv_controller;
			/**
			 * Flag to indicate a nonlinear model and required feedback linearization.
			 */
			bool useFL;
			/**
			 * Flag to indicate that the controller is tunned.
			 */
			bool tunned;
			/**
			 * The nonlinear part of the yaw-rate model. Used for feedback linearization.
			 */
			double beta_rr;
			/**
			 * The maximum approach angle.
			 */
			double aAngle;
			/**
			 * The model function parameters for the horizontal and vertical.
			 */
			boost::array<double,4> mfH,mfV;
			/**
			 * Enumerator for model function access.
			 */
			enum {a1h=0,a1v=0, a2h=1,a2v=1, a3h=2,a3v=2, a4h=3};
			/**
			 * Parameter map.
			 */
			labust::vehicles::dataMap paramMap;
		};

		typedef boost::shared_ptr<LFController> LFControllerPtr;
	};
};

/* LFCONTROLLER_HPP_ */
#endif
