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
#include <labust/gearth/CaddyKML.hpp>

#include <auv_msgs/NavSts.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

///\todo Edit the class loading to be loaded from the rosparam server.
int main(int argc, char* argv[])
{
	ros::init(argc,argv,"caddy_ge");
	ros::NodeHandle nh,ph("~");

	labust::gearth::CaddyKML kml;
	ros::Subscriber platform = nh.subscribe<auv_msgs::NavSts>("platform",
			1, &labust::gearth::CaddyKML::addShipPosition, &kml);
	ros::Subscriber diver = nh.subscribe<auv_msgs::NavSts>("diver",
			1, &labust::gearth::CaddyKML::addVehiclePosition, &kml);
	ros::Subscriber diver_origin = nh.subscribe<geometry_msgs::Point>("diver_origin",
			1, &labust::gearth::CaddyKML::setDiverOrigin, &kml);

	double freq(2);
	ph.param("rate",freq,freq);
	ros::Rate rate(freq);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	ros::spin();
	return 0;
}




