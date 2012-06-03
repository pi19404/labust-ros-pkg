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
#ifndef UUVAPP_HPP_
#define UUVAPP_HPP_
#include <labust/apps/AppInterface.hpp>
#include <labust/xml/xmlfwd.hpp>
#include <labust/vehicles/vehiclesfwd.hpp>
#include <labust/control/PIDController.hpp>
#include <labust/control/LFController.hpp>
#include <labust/control/SOIdentification.hpp>
#include <labust/tools/TimingTools.hpp>
#include <labust/navigation/KFCore.hpp>
#include <labust/navigation/LFModel.hpp>
#include <labust/navigation/LFNav.hpp>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The class integrates control, navigation and the UUV plugin into a modular
		 * infrastructure.
		 *
		 * \todo force surge speed in estimation.
		 * \todo Add delay compensation
		 * \todo Add external identification experiment specifications.
		 * \todo Integrate an automaton to handle identification.
		 * \todo Create a higher level mission controller (Petri Net based ?)
		 * \todo Extract for code reuse without the ModemData stuff.
		 * \todo  !!!!!!!! Reconfigure the Kalman filter after parameters identification.!!!!!
		 * \todo Save identification parameters.
		 * \todo Add currents external
		 * \todo Cut apart the UUVApp
		 * \todo Add sonar depth specification
		 */
		class UUVApp : labust::apps::App
		{
			typedef labust::navigation::KFCore<labust::navigation::LFModel> LFNav;
			typedef boost::shared_ptr<LFNav> LFNavPtr;

			enum {identHeading, identDepth, identFinished, identNone};

		public:
			/**
			 * The UV modes.
			 */
			typedef enum {idle, manual, manualControl, lineFollowing, identification} UVMode;

			/**
			 * Main constructor. Configures the system using the XML configuration file.
			 *
			 * \param reader Pointer to the loaded XML configuration file.
			 * \param id Identification class.
			 */
			UUVApp(const labust::xml::ReaderPtr reader, const std::string& id);

      /**
       * \override labust::apps::App::setCommand.
       */
      void setCommand(const labust::apps::stringRef cmd);
      /**
       * \override labust::apps::App::getDate.
       */
      void getData(const labust::apps::stringPtr data);


		protected:











			/**
			 * Tune the depth controller based on model parameters.
			 *
			 * \param param The tuning parameters.
			 */
			inline void tuneDepthController(const TuningParameters& param)
			{
				this->tuneController(param,&depthCon);
			}
			/**
			 * Tune the heading controller based on model parameters.
			 *
			 * \param param The tuning parameters.
			 */
			inline void tuneHeadingController(const TuningParameters& param)
			{
				this->tuneController(param,&yawCon);
			}

			/**
			 * The method processes the arrived modem data.
			 */
			void newModemData(const labust::xml::GyrosReader& reader);
			/**
			 * The method sets the current values commanded by MOOS.
			 *
			 * \param current The current vector.
			 */
			void setCurrents(const boost::array<double,3>& current);

			/**
			 * Method perfoms one execution iteration.
			 *
			 * \return Returns the estimated states.
			 */
			const labust::vehicles::stateMapRef step();
			/**
			 * Stop all action in the UUV.
			 */
			void stop();
			/**
			 * Set open loop surge force during line following.
			 *
			 * \param tauX The desired surgeForce.
			 */
			void setLFTauX(double tauX){this->surgeForce = tauX;};
			/**
			 * The method returns a reference to the real vehicle measurements.
			 */
			const labust::vehicles::stateMapRef getMeasurements(){return *this->measState;};

			/**
			 * The method returns the commanded tau.
			 */
			labust::vehicles::tauMap getTau(){return this->tau;};


		private:
			/**
			 * Calculate the TAU vector based on the current mode.
			 */
			void calculateTau();
			/**
			 * Extract measurements from the line following package.
			 */
			void extractMeasurements(std::map<std::string, double>& values);
			void extractMeasurements2(std::map<std::string, double>& values);
			/**
			 * The UUV identification step.
			 */
			void doIdentification();
			/**
			 * Identification state changer.
			 */
			void nextIdentificationStep();

			/**
			 * Vehicle plugin
			 */
			labust::vehicles::VehiclePluginPtr plugin;
			/**
			 * UUV simulator pointer
			 */
			labust::vehicles::DriverPtr uuv;
			/**
			 * The line following controller.
			 */
			labust::control::LFControllerPtr lfCon;
			/**
			 * The I-SO controller.
			 */
			labust::control::SOIdentification soIdent;
			/**
			 * The identification parameters.
			 */
			std::pair<double, double> headingSOId, depthSOId;
			/**
			 * The Kalman filter state estimator.
			 */
			LFNavPtr nav;

			labust::navigation::LFNav* nav2;

			/**
			 * The heading and depth controllers.
			 */
			labust::control::PID yawCon, depthCon;
			/**
			 * The controller parameters.
			 */
			TuningParameters yawParam, depthParam;
			/**
			 * Operation mode.
			 */
			UVMode mode;
			/**
			 * Last calculated TAU vector.
			 */
			labust::vehicles::tauMap tau,tauk_1;
			/**
			 * Current vehicle state.
			 */
			labust::vehicles::stateMapPtr state, measState;
			/**
			 * Desired states.
			 */
			labust::vehicles::stateMap stateRef;
			/**
			 * The Target information.
			 */
			double targetDepth;
			/**
			 * The calculated UUV position.
			 */
			double xuuv,yuuv;
			/**
			 * New measurement arrived.
			 */
			bool newMeasurement;
			/**
			 * The timeout watchdog.
			 */
			boost::shared_ptr<labust::tools::watchdog> watchdog;
			/**
			 * Forward surge force for the open-loop case.
			 */
			double surgeForce;
			/**
			 * Identification step tracker.
			 */
			int identificationStep;
			/**
			 * Sampling step of the application.
			 */
			double Ts;
			/**
			 * Identification results saving flag.
			 */
			bool saveIdentification;

			/**
			 * Addition for recalculation
			 */
			LFNav::matrix tauM;
			LFNav::matrix measM;
			LFNav::output_type stateM;
			LFNav::matrix convP;
			int iterCnt;

			LFNav::matrix delayMeas;

		public:
			double range,bearing,usvx,usvy;
		};
	}
}

/* UUVAPP_HPP_ */
#endif
