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
 *  Created: 02.04.2013.
 *********************************************************************/
#include <labust/tritech/USBLFilter.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <labust/math/uBlasOperations.hpp>
#include <pluginlib/class_list_macros.h>
#include <labust/math/NumberManipulation.hpp>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <auv_msgs/NavSts.h>

PLUGINLIB_DECLARE_CLASS(usbl,USBLFilter,labust::tritech::USBLFilter, nodelet::Nodelet)

using namespace labust::tritech;

USBLFilter::USBLFilter():
	listener(buffer),
	timeout(150),
	iteration(0),
	maxSpeed(0.5){};

USBLFilter::~USBLFilter(){};

void USBLFilter::onInit()
{
	ros::NodeHandle nh = this->getNodeHandle();
	usblSub = nh.subscribe<geometry_msgs::PointStamped>("usbl_nav", 1, boost::bind(&USBLFilter::onUsbl,this,_1));
	navPub = nh.advertise<auv_msgs::NavSts>("usblFiltered",1);

	filter.initModel();
	configureModel(nh);
	worker = boost::thread(boost::bind(&USBLFilter::run,this));
}

void USBLFilter::configureModel(ros::NodeHandle& nh)
{
	std::string sQ,sW,sV,sR,sP,sx0;
	nh.getParam("usbl_filter/Q", sQ);
	nh.getParam("usbl_filter/W", sW);
	nh.getParam("usbl_filter/V", sV);
	nh.getParam("usbl_filter/R", sR);
	nh.getParam("usbl_filter/P", sP);
	nh.getParam("usbl_filter/x0", sx0);
	try
	{
		KFilter::matrix Q,W,V,R,P;
		KFilter::vector x0;

		boost::numeric::ublas::matrixFromString(sQ,Q);
		boost::numeric::ublas::matrixFromString(sR,R);
		boost::numeric::ublas::matrixFromString(sW,W);
		boost::numeric::ublas::matrixFromString(sV,V);
		boost::numeric::ublas::matrixFromString(sR,R);
		boost::numeric::ublas::matrixFromString(sP,P);
		std::stringstream ss(sx0);
		boost::numeric::ublas::operator >>(ss,x0);

		double dT(0.1);
		nh.param("sampling_time",dT,dT);
		filter.setTs(dT);
		filter.setStateParameters(W,Q);
		filter.setMeasurementParameters(V,R);
		filter.setStateCovariance(P);
		filter.setState(x0);
	}
	catch (std::exception& e)
	{
		NODELET_ERROR("USBLFilter:: Model configuration failed, %s",
				e.what());
	}
}

void USBLFilter::onUsbl(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	NODELET_DEBUG("New update: %f %f %f\n",msg->point.x, msg->point.y, msg->point.z);

	boost::mutex::scoped_lock lock(dataMux);

	geometry_msgs::TransformStamped transform;
	try
	{
		//Take position two seconds from the past
		ros::Time now(ros::Time::now()), desired(now - ros::Duration(0));
		transform = buffer.lookupTransform("local", "base_link", ros::Time(0));
	}
	catch (tf2::TransformException& ex)
	{
		NODELET_ERROR("%s",ex.what());
	}

	KFilter::input_type vec(2);
	vec(0) = transform.transform.translation.x + msg->point.x;
	vec(1) = transform.transform.translation.y + msg->point.y;
	depth = transform.transform.translation.z + (-msg->point.z);

	double inx, iny;
	filter.calculateXYInovationVariance(filter.getStateCovariance(),inx,iny);
	const KFilter::vector& xy(filter.getState());
	double outlierR = 1;
	bool outlier = sqrt(pow(xy(KFilter::xp)-vec(0),2) +
			pow(xy(KFilter::yp)-vec(1),2)) > outlierR*sqrt(inx*inx + iny*iny);

	if (!outlier)
	{
		filter.correct(vec);

		//Limit diver speed
		KFilter::vector newState(filter.getState());
		newState(KFilter::Vv) = labust::math::coerce(
				newState(KFilter::Vv), -maxSpeed, maxSpeed);
		filter.setState(newState);
		iteration = 0;
	}
	else
	{
		NODELET_INFO("Outlier rejected: current: (%f,%f) - measurement: (%f,%f) - inovation: (%f,%f)",
				xy(KFilter::xp), xy(KFilter::yp),
				vec(0), vec(1), inx, iny);
	}
};

void USBLFilter::run()
{
	///\todo Add variable rate of filter estimates.
	ros::Rate rate(10);
	timeout = 100;

	while (ros::ok())
	{
		boost::mutex::scoped_lock lock(dataMux);

		if (++iteration > timeout)
		{
			NODELET_INFO("Timeout on USBL measurements. Setting diver speed to zero. Iteration: %d",
					iteration);
			//Stop the diver speed if lost
			KFilter::vector newState(filter.getState());
			newState(KFilter::Vv) = 0;
			filter.setState(newState);
		}

		filter.predict();
		const KFilter::vector& state = filter.getState();
		auv_msgs::NavSts::Ptr odom(new auv_msgs::NavSts());
		odom->header.stamp = ros::Time::now();
		odom->header.frame_id = "local";
		odom->position.north = state(KFilter::xp);
		odom->position.east = state(KFilter::yp);
		odom->position.depth = depth;
		odom->orientation.yaw = state(KFilter::psi);
		const KFilter::matrix& variance = filter.getStateCovariance();
		odom->orientation_variance.yaw = variance(KFilter::psi, KFilter::psi);
		odom->position_variance.north = variance(KFilter::xp, KFilter::xp);
		odom->position_variance.east = variance(KFilter::yp, KFilter::yp);

		//\todo Add covariance data
		odom->body_velocity.x = state(KFilter::Vv);
		//Free the filter lock
		lock.unlock();

		try
		{
			geometry_msgs::TransformStamped transformDeg;
			transformDeg = buffer.lookupTransform("worldLatLon", "local", ros::Time(0));

			std::pair<double, double> diffAngle = labust::tools::meter2deg(state(KFilter::xp),
					state(KFilter::yp),
					//The latitude angle
					transformDeg.transform.translation.y);
			odom->global_position.latitude = transformDeg.transform.translation.y + diffAngle.first;
			odom->global_position.longitude = transformDeg.transform.translation.x + diffAngle.second;
		}
		catch(tf2::TransformException& ex)
		{
			NODELET_ERROR("%s",ex.what());
		}

		//\todo Add covariance data
		navPub.publish(odom);
		rate.sleep();
	}
}


