/*********************************************************************
 * lowLevelConfigure.hpp
 *
 *  Created on: Mar 7, 2014
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, LABUST, UNIZG-FER
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

#ifndef LOWLEVELCONFIGURE_HPP_
#define LOWLEVELCONFIGURE_HPP_

#include <auv_msgs/Bool6Axis.h>
#include <labust_uvapp/ConfigureVelocityController.h>
#include <ros/ros.h>
#include <labust_mission/serviceCall.hpp>

extern ros::NodeHandle *nh_ptr;


namespace utils {

	class LowLevelConfigure {

	public:

		LowLevelConfigure();

		/*************************************************************
		 *** Class functions
		 ************************************************************/
		void start();

		void LL_VELconfigure(bool enable, int x, int y, int z, int pitch, int roll, int yaw);

		void AXESconfigure(auv_msgs::Bool6Axis& nu_bool);

		/*************************************************************
		 *** Class variables
		 ************************************************************/

		auv_msgs::Bool6Axis nuAxis;
		labust_uvapp::ConfigureVelocityController velConConf;

		ros::ServiceClient clientConfigureAxes;
		ros::ServiceClient clientConfigureVelocitiyController;
	};

	LowLevelConfigure::LowLevelConfigure() // nuAxis(false),
											// velConConf(0)
	{
		nuAxis.x = false;
		nuAxis.y = false;
		nuAxis.z = false;
		nuAxis.pitch = false;
		nuAxis.roll = false;
		nuAxis.yaw = false;

		for( int i = 0; i<=5; i++){
			velConConf.request.desired_mode[i] = 0;
		}
	}

	void LowLevelConfigure::start(){

		clientConfigureAxes = nh_ptr->serviceClient<navcon_msgs::ConfigureAxes>("ConfigureAxes");
		clientConfigureVelocitiyController = nh_ptr->serviceClient<labust_uvapp::ConfigureVelocityController>("ConfigureVelocityController");
	}

	/*
	 * Low-level velocity controller configure
	 */
	void LowLevelConfigure::LL_VELconfigure(bool enable, int x, int y, int z, int roll, int pitch, int yaw){

		if(enable){
			// Provjeriti je li potreban reset svih ili u if dodati.
			nuAxis.x = false;
			nuAxis.y = false;
			nuAxis.z = false;
			nuAxis.pitch = false;
			nuAxis.roll = false;
			nuAxis.yaw = false;

			if(velConConf.request.desired_mode[0] = x){ nuAxis.x = true;}
			if(velConConf.request.desired_mode[1] = y){nuAxis.y = true;}
			if(velConConf.request.desired_mode[2] = z){nuAxis.z = true;}
			if(velConConf.request.desired_mode[3] = roll){nuAxis.roll = true;}
			if(velConConf.request.desired_mode[4] = pitch){ nuAxis.pitch = true;}
			if(velConConf.request.desired_mode[5] = yaw){nuAxis.yaw = true;}

			AXESconfigure(nuAxis);
			utils::callService<labust_uvapp::ConfigureVelocityController>(clientConfigureVelocitiyController,velConConf);
		} else {

			/* Disable all degrees of freedom */
			for( int i = 0; i<=5; i++){
				velConConf.request.desired_mode[i] = 0;
			}

			nuAxis.x = false;
			nuAxis.y = false;
			nuAxis.z = false;
			nuAxis.pitch = false;
			nuAxis.roll = false;
			nuAxis.yaw = false;

			AXESconfigure(nuAxis);
			utils::callService<labust_uvapp::ConfigureVelocityController>(clientConfigureVelocitiyController,velConConf);
		}
	}

	/*
	 * Configure axes to control
	 */

	void LowLevelConfigure::AXESconfigure(auv_msgs::Bool6Axis& nu_bool){

		navcon_msgs::ConfigureAxes nu_cfg;
		nu_cfg.request.disable_axis = nu_bool;
		utils::callService<navcon_msgs::ConfigureAxes>(clientConfigureAxes, nu_cfg);
	}

}

#endif /* LOWLEVELCONFIGURE_HPP_ */
