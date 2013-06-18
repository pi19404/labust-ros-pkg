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
 *  Created: 06.05.2013.
 *********************************************************************/
#include <labust/control/HLManager.hpp>
#include <labust_uvapp/ConfigureVelocityController.h>
#include <labust_uvapp/EnableControl.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <labust/tools/rosutils.hpp>
#include <cart2/HLMessage.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

using labust::control::HLManager;

HLManager::HLManager():
			nh(ros::NodeHandle()),
			ph(ros::NodeHandle("~")),
			lastEst(ros::Time::now()),
			launchTime(ros::Time::now()),
			launchDetected(false),
			timeout(5),
			mode(HLManager::stop),
			type(HLManager::bArt),
			safetyRadius(2),
			safetyDistance(20),
			safetyTime(15),
			safetyTime2(2*safetyTime),
			gyroYaw(0),
			circleRadius(10),
			s(0),
			fixValidated(false)
{this->onInit();}

void HLManager::onInit()
{
	//Fill controller names
	controllers.insert(std::make_pair("LF",false));
	controllers.insert(std::make_pair("DP",false));
	controllers.insert(std::make_pair("VT",false));
	controllers.insert(std::make_pair("HDG",false));

	//Initialize publishers
	refPoint = nh.advertise<geometry_msgs::PointStamped>("ref_point", 1);
	refTrack = nh.advertise<auv_msgs::NavSts>("ref_track", 1);
	openLoopSurge = nh.advertise<std_msgs::Float32>("open_loop_surge", 1);
	curMode = nh.advertise<std_msgs::Int32>("current_mode", 1);
	refHeading = nh.advertise<std_msgs::Float32>("heading_ref", 1);
	hlMessagePub = nh.advertise<cart2::HLMessage>("hl_diagnostics", 1);
	sfPub = nh.advertise<auv_msgs::NavSts>("sf_diagnostics", 1);

	//Initialze subscribers
	state = nh.subscribe<auv_msgs::NavSts>("stateHat", 1,
			&HLManager::onVehicleEstimates,this);
	launch = nh.subscribe<std_msgs::Bool>("launched", 1,
			&HLManager::onLaunch,this);
	gpsData = nh.subscribe<sensor_msgs::NavSatFix>("gps", 1,
			&HLManager::onGPSData,this);
	virtualTargetTwist = nh.subscribe<geometry_msgs::TwistStamped>("virtual_target_twist", 1,
			&HLManager::onVTTwist,this);
	imuMeas = nh.subscribe<sensor_msgs::Imu>("imu", 1,
			&HLManager::onImuMeas,this);

	//Configure service
	modeServer = nh.advertiseService("SetHLMode",
			&HLManager::setHLMode, this);
	bool isBart(false);
	nh.param("hl_manager/timeout",timeout,timeout);
	nh.param("hl_manager/radius",safetyRadius,safetyRadius);
	nh.param("hl_manager/safetyDistance",safetyDistance,safetyDistance);
	nh.param("hl_manager/safetyTime",safetyTime,safetyTime);
	nh.param("hl_manager/safetyTime2",safetyTime2,safetyTime2);
	nh.param("hl_manager/circleRadius", circleRadius, circleRadius);
	nh.param("hl_manager/isBart",isBart,isBart);
	nh.param("LocalOriginLat",originLat,originLat);
	nh.param("LocalOriginLon",originLon,originLon);
	ph.param("LocalFixSim",fixValidated, fixValidated);
}

bool HLManager::setHLMode(cart2::SetHLMode::Request& req,
		cart2::SetHLMode::Response& resp)
{
	if (req.mode >= lastMode)
	{
		ROS_ERROR("Faulty mode reuqest.");
		this->fullStop();
		return false;
	}
	//If in latitude/longitude convert to meters
	if (req.ref_point.header.frame_id == "worldLatLon")
	{
		std::pair<double, double> location = labust::tools::deg2meter(req.ref_point.point.x-originLat,
				req.ref_point.point.y-originLon,
				originLat);
		this->point.point.x = location.first;
		this->point.point.y = location.second;
	}
	else
	{
		this->point = req.ref_point;
	}

	this->point.header.frame_id = "local";
	point.header.stamp = ros::Time::now();
	refPoint.publish(point);
	hlDiagnostics.ref_point = point;

	if (req.radius) circleRadius = req.radius;
	hlDiagnostics.radius = circleRadius;

	std_msgs::Float32 surge;
	surge.data = req.surge;
	hlDiagnostics.surge = req.surge;
	openLoopSurge.publish(surge);

	std_msgs::Int32 cmode;
	cmode.data = req.mode;
	hlDiagnostics.mode = req.mode;
	curMode.publish(cmode);

	std_msgs::Float32 deshdg;
	deshdg.data = req.yaw;
	hlDiagnostics.yaw = req.yaw;
	refHeading.publish(deshdg);

	//Check if the mode is already active
	if (this->mode == req.mode) return true;
	//Else handle the mode change
	mode = req.mode;

	ros::ServiceClient client =
				nh.serviceClient<labust_uvapp::ConfigureVelocityController>("ConfigureVelocityController");
	labust_uvapp::ConfigureVelocityController srv;
	for (int32_t i=srv.request.u; i<= srv.request.r;++i)
	{
		srv.request.desired_mode[i] = srv.request.DisableAxis;
	}

	srv.request.desired_mode[srv.request.u] = srv.request.ControlAxis;
	srv.request.desired_mode[srv.request.r] = srv.request.ControlAxis;

	geometry_msgs::TwistStampedPtr fakeTwist(new geometry_msgs::TwistStamped());
	disableControllerMap();

	switch (mode)
	{
	case manual:
		ROS_INFO("Set to manual mode.");
		srv.request.desired_mode[srv.request.u] = srv.request.ManualAxis;
		srv.request.desired_mode[srv.request.r] = srv.request.ManualAxis;
		break;
	case gotoPoint:
		ROS_INFO("Set to GoTo mode.");
		controllers["LF"] = true;
		break;
	case stationKeeping:
		ROS_INFO("Set to Station keeping mode.");
		controllers["DP"] = true;
		break;
	case circle:
	  ROS_INFO("Set to Circle mode.");
		controllers["VT"] = true;
		s = 0;
		this->onVTTwist(fakeTwist);
		break;
	case heading:
		ROS_INFO("Set to Heading mode.");
		srv.request.desired_mode[srv.request.u] = srv.request.ManualAxis;
		controllers["HDG"] = true;
		break;
	case headingSurge:
		ROS_INFO("Set to Heading + constant surge mode.");
		controllers["HDG"] = true;
		break;
	case vtManual:
		ROS_INFO("Set to vt manual mode.");
		srv.request.desired_mode[srv.request.u] = srv.request.ManualAxis;
		srv.request.desired_mode[srv.request.r] = srv.request.ManualAxis;
		controllers["VT"] = true;
		s = 0;
		this->onVTTwist(fakeTwist);
		break;
	case stop:
		ROS_INFO("Stopping.");
		return this->fullStop();
		break;
	default:
	  ROS_ERROR("Wrong mode selected:%d",mode);
	  break;
	}

	return client.call(srv) && configureControllers();
}

void HLManager::disableControllerMap()
{
	for (ControllerMap::iterator it=controllers.begin();
			it != controllers.end(); ++it)
	{
		it->second = false;
	}
}

bool HLManager::fullStop()
{
	labust_uvapp::EnableControl srv;
	srv.request.enable = false;
	ros::ServiceClient client = nh.serviceClient<labust_uvapp::EnableControl>("VelCon_enable");
	bool serviceFlag;
	if (!((serviceFlag=client.call(srv))))
	{
	  ROS_ERROR("Failed to call velocity control configuration service.");
	  return false;
	}

	disableControllerMap();

	if (!configureControllers()) return false;

	ROS_INFO("Stopping all axes.\n");
	return true;
}

bool HLManager::configureControllers()
{
	bool success = true;
	for (ControllerMap::const_iterator it=controllers.begin();
			it != controllers.end(); ++it)
	{
		ros::ServiceClient client = nh.serviceClient<labust_uvapp::EnableControl>(it->first + "_enable");
		labust_uvapp::EnableControl srv;
		srv.request.enable = it->second;
		if (!client.call(srv))
		{
		  ROS_ERROR("Failed to call the %s configuration service.",it->first.c_str());
		  success = false;
		}
	}

	return success;
}

void HLManager::onVTTwist(const geometry_msgs::TwistStamped::ConstPtr& twist)
{
	if (mode == circle || mode == vtManual)
	{
		//\todo Generalize this 0.1 with Ts.
		s +=twist->twist.linear.x*0.1;

		//Circle
		if (s>=2*circleRadius*M_PI) s=s-2*circleRadius*M_PI;
		else if (s<0) s=2*circleRadius*M_PI-s;

		double xRabbit = point.point.x + circleRadius*cos(s/circleRadius);
		double yRabbit = point.point.y + circleRadius*sin(s/circleRadius);
		double gammaRabbit=labust::math::wrapRad(s/circleRadius)+M_PI/2;

		tf::Transform transform;
		transform.setOrigin(tf::Vector3(xRabbit, yRabbit, 0));
		Eigen::Quaternion<float> q;
		labust::tools::quaternionFromEulerZYX(0,0,gammaRabbit, q);
		transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
		broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local", "serret_frenet_frame"));

		auv_msgs::NavStsPtr msg(new auv_msgs::NavSts());
		msg->position.north = xRabbit;
		msg->position.east = yRabbit;
		msg->body_velocity.x = twist->twist.linear.x;
		msg->orientation.yaw = gammaRabbit;
		sfPub.publish(msg);
	}
}

void HLManager::onImuMeas(const sensor_msgs::Imu::ConstPtr& imu)
{
	if (this->launchDetected)
	{
		double Ts(0.1);
		gyroYaw+=imu->angular_velocity.z*Ts;
		ROS_INFO("Desired gyro yaw: %f",gyroYaw);
	}
	else
	{
		gyroYaw = 0;
	}
}

void HLManager::onVehicleEstimates(const auv_msgs::NavSts::ConstPtr& estimate)
{
	this->stateHat = *estimate;
	lastEst = ros::Time::now();
};

void HLManager::onGPSData(const sensor_msgs::NavSatFix::ConstPtr& fix)
{
	this->fix = *fix;
	this->fix.header.stamp = ros::Time::now();

	//In case we didn't have a fix on launch, but now we get one.
	if (!fixValidated)
	{
		originLat = fix->latitude;
		originLon = fix->longitude;
		fixValidated = true;
	}
};

void HLManager::onLaunch(const std_msgs::Bool::ConstPtr& isLaunched)
{
	if ((type== bArt) && (mode==stop) && (!launchDetected && isLaunched->data))
	{
		ROS_INFO("Launch detected");
		launchDetected = isLaunched->data;
		launchTime = ros::Time::now();

		//Init the gyroYaw
		gyroYaw = 0;

		//Check if fix is valid
		if ((fixValidated = (fix.header.stamp - ros::Time::now()).sec < safetyTime))
		{
			originLat = fix.latitude;
			originLon = fix.longitude;
		}
	}
	else
	{
		ROS_INFO("Ignoring launch while in %d mode.", mode);
	}
}

void HLManager::calculateBArtPoint(double heading)
{
	//Get current heading and calculate the desired point
	point.point.x = stateHat.position.north + safetyDistance*cos(heading);
	point.point.y = stateHat.position.east + safetyDistance*sin(heading);
	point.point.z = 0;
	point.header.frame_id = "local";
}

void HLManager::safetyTest()
{
	bool estTimeout = (ros::Time::now() - lastEst).toSec() > timeout;

	if (estTimeout)
	{
		ROS_WARN("Timeout on the control channel. Controlled axes will be disabled.");

		//Some action
		//Turn off all control, switch to manual ?
		this->fullStop();
	}
}

void HLManager::bArtStep()
{
		//Check that enough time has passed
	  double dT = (ros::Time::now() - launchTime).toSec();
		double sft2 = 5;
		if ((dT > safetyTime) && (dT < (safetyTime + sft2)))
		{
			ROS_INFO("Heading only mode.");
			//Switch to combined heading
			cart2::SetHLMode srv;
			srv.request.mode = srv.request.HeadingSurge;
			srv.request.yaw = stateHat.orientation.yaw-gyroYaw;
			srv.request.surge = 0;
			this->setHLMode(srv.request, srv.response);
		}

		if ((dT > (safetyTime+sft2)) && (dT < safetyTime2))
		{
			ROS_INFO("Heading + surge mode.");
			//Switch to combined heading
			cart2::SetHLMode srv;
			srv.request.mode = srv.request.HeadingSurge;
			srv.request.yaw = stateHat.orientation.yaw-gyroYaw;
			srv.request.surge = 0.5;
			this->setHLMode(srv.request, srv.response);
		}

		if (dT >= safetyTime2)
		{
			launchDetected = false;
			//Switch to station keeping
			cart2::SetHLMode srv;
			srv.request.mode = srv.request.GoToPoint;
			this->calculateBArtPoint(stateHat.orientation.yaw-gyroYaw);
			srv.request.ref_point = point;
			srv.request.surge = 0.5;
			this->setHLMode(srv.request, srv.response);
		}
}

void HLManager::step()
{
	if ((type == bArt) && (launchDetected))
	{
		//Publish the launch information
		this->bArtStep();
	};

	//Check distance to point
	double dx(point.point.x - stateHat.position.north);
	double dy(point.point.y - stateHat.position.east);
	double dist(sqrt(dx*dx+dy*dy));
	double relAngle(atan2(dy,dx));
	double angleDiff(fabs(labust::math::wrapRad(relAngle - stateHat.orientation.yaw)));

	if (mode == gotoPoint)
	{
		//If distance to point is OK or the vehicle missed the point
		if (dist < safetyRadius || ((dist < 2*safetyRadius) && (angleDiff > M_PI/2)))
		{
			//Switch to station keeping
			cart2::SetHLMode srv;
			srv.request.mode = srv.request.StationKeeping;
			srv.request.ref_point = point;
			this->setHLMode(srv.request, srv.response);
		}
	}
}

void HLManager::start()
{
	ros::Rate rate(10);

	while (nh.ok())
	{
		if (fixValidated)
		{
			tf::Transform transform;
			transform.setOrigin(tf::Vector3(originLon, originLat, 0));
			transform.setRotation(tf::createQuaternionFromRPY(0,0,0));
			broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/worldLatLon", "/world"));
			transform.setOrigin(tf::Vector3(0, 0, 0));
			Eigen::Quaternion<float> q;
			labust::tools::quaternionFromEulerZYX(M_PI,0,M_PI/2,q);
			transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
			broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "local"));
		}
		this->safetyTest();
		this->step();
		hlMessagePub.publish(hlDiagnostics);
		rate.sleep();
		ros::spinOnce();
	}
}
