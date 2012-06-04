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
#ifndef UVAPP_HPP_
#define UVAPP_HPP_
#include <labust/apps/AppInterface.hpp>
#include <labust/xml/xmlfwd.hpp>
#include <labust/vehicles/vehiclesfwd.hpp>
#include <labust/navigation/navfwd.hpp>
#include <labust/control/controlfwd.hpp>
#include <labust/plugins/PluginLoader.hpp>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The class integrates control, navigation and the UUV plugin into a modular
		 * infrastructure.
		 *
		 * \todo
		 */
		class UVApp : public virtual labust::apps::App
		{
			typedef labust::plugins::Loader<
					labust::vehicles::VehiclePluginPtr,
					labust::vehicles::DriverPtr > VehicleLoader;
			typedef labust::plugins::Loader<
					labust::navigation::NavigationPluginPtr,
					labust::navigation::DriverPtr > NavigationLoader;
			typedef labust::plugins::Loader<
					labust::control::ControlPluginPtr,
					labust::control::DriverPtr > ControlLoader;
		public:
			/**
			 * The operational modes.
			 */
			typedef enum {idle, manual, headingDepth, lineFollowing, identification} UVMode;
			/**
			 * The UV application pointer.
			 */
			typedef boost::shared_ptr<UVApp> Ptr;
			/**
			 * Main constructor. Configures the system using the XML configuration file.
			 *
			 * \param reader Pointer to the loaded XML configuration file.
			 * \param id Identification class.
			 */
			UVApp(const labust::xml::ReaderPtr reader, const std::string& id);
			/**
			 * Generic virtual destructor.
			 */
			virtual ~UVApp(){};

      /**
       * \override labust::apps::App::setCommand.
       */
      void setCommand(const labust::apps::stringRef cmd){};
      /**
       * \override labust::apps::App::getDate.
       */
      void getData(const labust::apps::stringPtr data){};

      /**
       * Stop all operation and go to idle mode.
       */
      void stop();

      /**
       * Performs one cycle of acquisition, navigation and control.
       *
       * \param measurements The acquired external measurements.
       */
      virtual void step(labust::vehicles::stateMapRef measurements);

      /**
       * Sets the desired mode of operation.
       */
      inline void setUVMode(UVMode mode){this->mode = mode;};

		protected:
			/**
			 * Calculate the next tau vector.
			 */
			void calculateTau();
      /**
       * The three controller options: Heading+Depth, LineFollowing, Manual.
       */
      ControlLoader hdCon, lfCon, manCon, ident;
      /**
       * The navigation.
       */
      NavigationLoader nav;
      /**
       * The vehicle.
       */
      VehicleLoader uuv;
      /**
       * The reference, measured and estimated states.
       */
      labust::vehicles::stateMap stateRef, stateHat, stateMeas;
      /**
       * The current and last tau vector.
       */
      labust::vehicles::tauMap tau, tauk_1;
      /**
       * The UV operational mode.
       */
      UVMode mode;
  	};
	}
}

/* UVAPP_HPP_ */
#endif
