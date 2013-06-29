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
*
*  Author: Dula Nad
*  Created: 01.02.2013.
*********************************************************************/
#ifndef SIMCORE_HPP_
#define SIMCORE_HPP_
#include <labust/simulation/RBModel.hpp>
#include <labust/ros/SimSensors.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace labust
{
  namespace simulation
  {
  	/**
  	 * The method populates the rigid body model from the
  	 *
  	 * \param nh The node handle to the configuration.
  	 * \param model The rigid body model to configure.
  	 */
  	void configureModel(const ros::NodeHandle& nh, RBModel& model);

    /**
     *  This class implements core functionality of the ROS uvsim node.
     */
    class SimCore
    {
    public:
    	/**
    	 * The generic constructor.
    	 */
    	SimCore();

    	/**
    	 * The method initializes the ROS node and configures the model from the ROS parameters.
    	 */
    	void onInit();

    private:
    	/**
    	 * Output the model parameters info.
    	 */
    	void modelReport();
    	/**
    	 * The rigid body model implementation.
    	 */
    	RBModel model;
    	/**
    	 * The frame transform broadcaster.
    	 */
			tf::TransformBroadcaster broadcast;
    	/**
    	 * The frame transform listener.
    	 */
			tf::TransformListener listener;
    };
  }
}

/* SIMCORE_HPP_ */
#endif
