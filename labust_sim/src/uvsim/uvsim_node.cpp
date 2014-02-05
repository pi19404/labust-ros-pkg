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
#include <labust/ros/SimCore.hpp>
#include <labust/ros/SimSensors.hpp>
#include <labust/tools/conversions.hpp>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

///\todo Edit the class loading to be loaded from the rosparam server.
int main(int argc, char* argv[])
{
	ros::init(argc,argv,"uvsim");
	ros::NodeHandle nh;

	//Sensor loaders
//	pluginlib::ClassLoader<labust::simulation::SimSensorInterface>
//		sim_loader("labust_sim", "labust::simulation::SimSensorInterface");

	labust::simulation::SimCore simulator;

//	typedef std::pair<std::string, std::string> NameTopicPair;
//	std::vector< NameTopicPair >
//		list({
//		NameTopicPair("labust::simulation::ImuSensor","imu"),
//		NameTopicPair("labust::simulation::GPSSensor","fix")});
//
//	try
//	{
//		using namespace labust::simulation;
//		for (auto it = list.begin(); it != list.end(); ++it)
//		{
//			SimSensorInterface::Ptr sensor(sim_loader.createInstance(it->first));
//			sensor->configure(nh,it->second);
//			simulator.addSensor(sensor);
//		}
//	}
//	catch(pluginlib::PluginlibException& ex)
//	{
//	  //handle the class failing to load
//	  ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
//	}

	ros::spin();
	return 0;
}
