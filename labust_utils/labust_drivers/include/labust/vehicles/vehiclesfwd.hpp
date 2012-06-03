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
#ifndef VEHICLESFWD_HPP_
#define VEHICLESFWD_HPP_
#include <labust/plugins/Factory.hpp>
#include <labust/plugins/DLLoad.hpp>
#include <labust/xml/xmlfwd.hpp>

#include <boost/shared_ptr.hpp>

#include <map>
#include <string>

namespace labust
{
	namespace vehicles
	{
  	/**
     * These are shortcuts to the TAU map parameters.
     * X,Y,Z are forces in body frame x,y,z directions.
     * K,M,N are torques around the body frame x,y,z axes.
     */
		namespace tau
		{
			typedef enum {X=0, Y, Z, K, M, N} tau;
		}
    /**
     * This map represents the force and torque vector used in marine applications.
     * X, Y, Z represent forces in the body x, y and z axes direction respectively.
     * K, M, N represent moments around the body x, y and z axes respectively.
     *
     * For more information consult the "Guidance and Control of Ocean Vehicles"
     * by Thor I. Fossen.
     */
		typedef std::map<int, double> tauMap;
		typedef tauMap& tauMapRef;
		typedef boost::shared_ptr<tauMap> tauMapPtr;

    /**
     * These are shortcuts to the state map parameters. Usually
     * a state map can contain any parameter that we define as
     * a state.
     *
     * \todo Consider adding xhat, yhat ... for estimated variables as separate states.
     */
		namespace state
		{
			typedef enum {
				//Nu states are in m/s and radians/s
				u = 0,v,w,p,q,r,
				//ETA states are in meters and radians
				x, y, z, roll, pitch,yaw,
				//Geostates
				lat,lon,heading,altitude,
				//Line following states
				dH,dV,
				//Sensor states
				depthPressure,

				//External forces
				X_e,Y_e,Z_e,K_e,M_e,N_e
			} states;
		};

    /**
     * This map contains various states that we defined.
     */
    typedef std::map<int, double> stateMap;
    typedef stateMap& stateMapRef;
    typedef boost::shared_ptr<stateMap> stateMapPtr;
    /**
     * This type contains a guidance map that allows control of states.
     */
    typedef std::map<int, double> guidanceMap;
    typedef guidanceMap& guidanceMapRef;
    typedef boost::shared_ptr<stateMap> guidanceMapPtr;
    /**
     * These is a generic map for data-exchange.
     */
    typedef std::map<std::string, double> dataMap;
    typedef dataMap& dataMapRef;
    typedef boost::shared_ptr<dataMap> dataMapPtr;
    typedef std::map<std::string, std::string> strMap;
    typedef strMap& strMapRef;
    typedef boost::shared_ptr<strMap> strMapPtr;
    /**
     * Vehicle forward declaration
     */
    class Driver;
    typedef boost::shared_ptr<Driver> DriverPtr;
    /**
     * Plugin factory declarations
     */
    typedef labust::plugins::TmplPluginFactory<
      Driver,
      const labust::xml::ReaderPtr> VehicleFactory;
    typedef VehicleFactory::AbstractFactory* VehicleFactoryPtr;

    typedef labust::plugins::DLLoad<VehicleFactory> VehiclePlugin;
    typedef boost::shared_ptr<VehiclePlugin> VehiclePluginPtr;
	}
}

/* VEHICLESFWD_HPP_ */
#endif
