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
#ifndef VEHICLEDRIVER_HPP_
#define VEHICLEDRIVER_HPP_
#include <labust/vehicles/vehiclesfwd.hpp>
#include <labust/apps/AppInterface.hpp>

namespace labust
{
  namespace vehicles
  {
    /**
     * This class specifies an configurable vehicle interface. Users should inherit from this class when
     * the driver will use or implement guidance algorithms inside. Also, for advanced usage of vehicles,
     * i.e. full implementation, you should implement this interface.
     */
    class Driver : public virtual labust::apps::App
    {
    public:
      /**
       * Virtual destructor. Remove the debugging message and the iostream include.
       */
      virtual ~Driver(){};
      /**
       * Method to set the force and torque vector. This approach abstracts different
       * vehicles fairly well. We can achieve control of all types of vehicles by simply
       * using specifying the desired forces and torque that should be applied via
       * thrusters and control surfaces.
       *
       * This method should throw a VehicleException.
       *
       * \param tau Force and torque map.
       * \see LABUST::VEHICLES::COREDATASET::tauVector
       */
      virtual void setTAU(const labust::vehicles::tauMapRef tau) = 0;
      /**
       * This method should be used to return the state vector. Number of states are vehicle
       * dependent. For this reason we use a map. Users can search the states they need by key
       * and can easily detect that states are missing.
       *
       * This method should no throw.
       *
       * \param states Different vehicle states.
       */
      virtual void getState(labust::vehicles::stateMapPtr state) = 0;
      /**
       * Some vehicles support guidance set points, ie. desired speed and heading, depth, waypoint, etc.
       * You can transmit these set-points through this method.
       *
       * This method should throw a VehicleDriverException.
       *
       * \param guidance Desired set points and similar guidance values.
       */
      virtual void setGuidance(const labust::vehicles::guidanceMapRef guidance) = 0;
    };
  };
};
/* VEHICLEDRIVER_HPP_ */
#endif
