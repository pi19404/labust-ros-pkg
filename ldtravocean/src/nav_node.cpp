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
 *  Author : Dula Nad
 *  Created: 26.03.2013.
 *********************************************************************/
#include <labust/navigation/KFCore.hpp>
#include <labust/navigation/LDTravModel.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/tools/DynamicsLoader.hpp>
#include <labust/simulation/DynamicsParams.hpp>
#include <labust/navigation/KFModelLoader.hpp>


#include <kdl/frames.hpp>
#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyForceReq.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

#include <boost/bind.hpp>
#include <sstream>

typedef labust::navigation::KFCore<labust::navigation::LDTravModel> KFNav;
tf::TransformListener* listener;

ros::Time t;
double g_acc(9.81), rho(1025);

void handleTau(KFNav::vector& tauIn, const auv_msgs::BodyForceReq::ConstPtr& tau)
{
	tauIn(KFNav::X) = tau->wrench.force.x;
	tauIn(KFNav::Y) = tau->wrench.force.y;
	tauIn(KFNav::Z) = tau->wrench.force.z;
	tauIn(KFNav::N) = tau->wrench.torque.z;
};

void handleGPS(KFNav::vector& measurement, KFNav::vector& newFlag, const sensor_msgs::NavSatFix::ConstPtr& data)
{
	//Calculate to X-Y tangent plane
	tf::StampedTransform transformDeg, transformLocal;
	try
	{
		listener->lookupTransform("local", "gps_frame", ros::Time(0), transformLocal);
		listener->lookupTransform("/worldLatLon", "local", ros::Time(0), transformDeg);

		std::pair<double,double> posxy =
				labust::tools::deg2meter(data->latitude - transformDeg.getOrigin().y(),
						data->longitude - transformDeg.getOrigin().x(),
						transformDeg.getOrigin().y());

		//correct for lever arm shift during pitch and roll
		tf::Vector3 pos = tf::Vector3(posxy.first, posxy.second,0);

		//xy(x) = pos.x();
		//xy(y) = pos.y();
		measurement(KFNav::xp) = posxy.first;
		newFlag(KFNav::xp) = 1;
		measurement(KFNav::yp) = posxy.second;
		newFlag(KFNav::yp) = 1;
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}
};

void handleImu(KFNav::vector& rpy,  const sensor_msgs::Imu::ConstPtr& data)
{
	enum {r,p,y,newMsg};
	double roll,pitch,yaw;
	static labust::math::unwrap unwrap;

	tf::StampedTransform transform;
	try
	{
		t = data->header.stamp;
		listener->lookupTransform("base_link", "imu_frame", ros::Time(0), transform);
		tf::Quaternion meas(data->orientation.x,data->orientation.y,
				data->orientation.z,data->orientation.w);
		tf::Quaternion result = meas*transform.getRotation();

		KDL::Rotation::Quaternion(result.x(),result.y(),result.z(),result.w()).GetEulerZYX(yaw,pitch,roll);
		/*labust::tools::eulerZYXFromQuaternion(
				Eigen::Quaternion<float>(result.x(),result.y(),
						result.z(),result.w()),
				roll,pitch,yaw);*/
		//ROS_INFO("Received RPY:%f,%f,%f",roll,pitch,yaw);

		rpy(r) = roll;
		rpy(p) = pitch;
		rpy(y) = unwrap(yaw);
		rpy(newMsg) = 1;
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}
};

void handleDvl(KFNav::vector& measurement, KFNav::vector& newFlag,
		const geometry_msgs::TwistStamped::ConstPtr& data)
{
	measurement(KFNav::u) = data->twist.linear.x;
	newFlag(KFNav::u)=1;
	measurement(KFNav::v) = data->twist.linear.y;
	newFlag(KFNav::v)=1;
	//measurement(KFNav::w) = data->twist.linear.z;
	//newFlag(KFNav::w)=1;
};

void handlePressure(KFNav::vector& measurement, KFNav::vector& newFlag,
		const std_msgs::Float32::ConstPtr& data)
{
	measurement(KFNav::zp) = data->data;
	newFlag(KFNav::zp)=1;
};

void configureNav(KFNav& nav, ros::NodeHandle& nh)
{
	ROS_INFO("Configure navigation.");

	labust::simulation::DynamicsParams params;
	labust::tools::loadDynamicsParams(nh, params);

	ROS_INFO("Loaded dynamics params.");

	KFNav::ModelParams surge,sway,heave,yaw;
	surge.alpha = params.m + params.Ma(0,0);
	sway.alpha = params.m + params.Ma(1,1);
	heave.alpha = params.m + params.Ma(2,2);
	yaw.alpha = params.Io(2,2) + params.Ma(5,5);
	surge.beta = params.Dlin(0,0);
	sway.beta = params.Dlin(1,1);
	sway.beta = params.Dlin(2,2);
	yaw.beta = params.Dlin(5,5);
	surge.betaa = params.Dquad(0,0);
	sway.betaa = params.Dquad(1,1);
	sway.betaa = params.Dquad(2,2);
	yaw.betaa = params.Dquad(5,5);
	nav.setParameters(surge,sway,heave,yaw);

	g_acc = params.g_acc;
	rho = params.rho;

	nav.initModel();
	labust::navigation::kfModelLoader(nav, nh, "ekfnav");
}

//consider making 2 corrections based on arrived measurements (async)
int main(int argc, char* argv[])
{
	ros::init(argc,argv,"rov_nav");
	ros::NodeHandle nh;
	KFNav nav;
	//Configure the navigation
	configureNav(nav,nh);

	double outlierR;
	nh.param("outlier_radius",outlierR,1.0);

	//Publishers
	ros::Publisher stateHat = nh.advertise<auv_msgs::NavSts>("stateHat",1);
	ros::Publisher stateMeas = nh.advertise<auv_msgs::NavSts>("meas",1);
	ros::Publisher currentTwist = nh.advertise<geometry_msgs::TwistStamped>("currentsHat",1);
	ros::Publisher bodyFlowFrame = nh.advertise<geometry_msgs::TwistStamped>("body_flow_frame_twist",1);
	//Subscribers
	KFNav::vector tau(KFNav::vector::Zero(KFNav::inputSize)),
			measurements(KFNav::vector::Zero(KFNav::stateNum)),
			newMeasFlag(KFNav::vector::Zero(KFNav::stateNum));
	KFNav::vector rpy(KFNav::vector::Zero(3+1));

	tf::TransformBroadcaster broadcast;
	listener = new tf::TransformListener();

	ros::Subscriber tauAch = nh.subscribe<auv_msgs::BodyForceReq>("tauAch", 1,
			boost::bind(&handleTau,boost::ref(tau),_1));
	ros::Subscriber navFix = nh.subscribe<sensor_msgs::NavSatFix>("gps", 1,
			boost::bind(&handleGPS,boost::ref(measurements),boost::ref(newMeasFlag),_1));
	ros::Subscriber imu = nh.subscribe<sensor_msgs::Imu>("imu", 1,
			boost::bind(&handleImu,boost::ref(rpy),_1));
	ros::Subscriber dvl = nh.subscribe<geometry_msgs::TwistStamped>("dvl", 1,
			boost::bind(&handleDvl,boost::ref(measurements),boost::ref(newMeasFlag),_1));
	ros::Subscriber pressure = nh.subscribe<sensor_msgs::FluidPressure>("pressure", 1,
			boost::bind(&handlePressure,boost::ref(measurements),boost::ref(newMeasFlag),_1));

	ros::Rate rate(10);

	nav_msgs::Odometry odom;
	auv_msgs::NavSts meas,state;
	geometry_msgs::TwistStamped current, flowspeed;
	meas.header.frame_id = "local";
	state.header.frame_id = "local";
	current.header.frame_id = "local";

	labust::math::unwrap unwrap;

	while (ros::ok())
	{
		nav.predict(tau);
		if (rpy(3))
		{
			ROS_INFO("Set the yaw.");
			double yaw = unwrap(rpy(2));
			measurements(KFNav::psi) = yaw;
			newMeasFlag(KFNav::psi) = 1;
		}

		bool anyNew = false;
		for (size_t i=0; i<newMeasFlag.size(); ++i) if ((anyNew = (newMeasFlag(i)))) break;

		if (anyNew)
		{
			ROS_INFO("Do update.");
			nav.correct(nav.update(measurements, newMeasFlag));
			ROS_INFO("Correction step.");
		}

		meas.orientation.roll = rpy(0);
		meas.orientation.pitch = rpy(1);
		meas.orientation.yaw = rpy(2);
		meas.position.north = measurements(KFNav::xp);
		meas.position.east = measurements(KFNav::yp);
		meas.position.depth = measurements(KFNav::zp);
		meas.body_velocity.x = measurements(KFNav::u);
		meas.body_velocity.y = measurements(KFNav::v);
		meas.body_velocity.z = measurements(KFNav::w);
		stateMeas.publish(meas);

		const KFNav::vector& estimate = nav.getState();
		state.body_velocity.x = estimate(KFNav::u);
		state.body_velocity.y = estimate(KFNav::v);
		state.body_velocity.z = estimate(KFNav::w);
		state.orientation_rate.yaw = estimate(KFNav::r);
		state.position.north = estimate(KFNav::xp);
		state.position.east = estimate(KFNav::yp);
		state.position.depth = estimate(KFNav::zp);
		current.twist.linear.x = estimate(KFNav::xc);
		current.twist.linear.y = estimate(KFNav::yc);

		const KFNav::matrix& covariance = nav.getStateCovariance();
		state.position_variance.north = covariance(KFNav::xp, KFNav::xp);
		state.position_variance.east = covariance(KFNav::yp, KFNav::yp);
		state.position_variance.depth = covariance(KFNav::zp,KFNav::zp);
		state.orientation_variance.yaw =  covariance(KFNav::psi, KFNav::psi);

		try
		{
			tf::StampedTransform transformDeg;
			listener->lookupTransform("/worldLatLon", "local", ros::Time(0), transformDeg);

			std::pair<double, double> diffAngle = labust::tools::meter2deg(state.position.north,
					state.position.east,
					//The latitude angle
					transformDeg.getOrigin().y());
			state.global_position.latitude = transformDeg.getOrigin().y() + diffAngle.first;
			state.global_position.longitude = transformDeg.getOrigin().x() + diffAngle.second;
		}
		catch(tf::TransformException& ex)
		{
			ROS_WARN("Unable to set global position. %s",ex.what());
		}

		state.orientation.yaw = labust::math::wrapRad(estimate(KFNav::psi));

		tf::StampedTransform transform;
		transform.setOrigin(tf::Vector3(estimate(KFNav::xp), estimate(KFNav::yp), 0.0));
		Eigen::Quaternion<float> q;
		labust::tools::quaternionFromEulerZYX(rpy(0),rpy(1),estimate(KFNav::psi),q);
		transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
		broadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local", "base_link"));

		//Calculate the flow frame (instead of heading use course)
		double xdot,ydot;
		nav.getNEDSpeed(xdot,ydot);
		labust::tools::quaternionFromEulerZYX(rpy(0),rpy(1),atan2(ydot,xdot),q);
		transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
		broadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local", "base_link_flow"));

		flowspeed.twist.linear.x = xdot;
		flowspeed.twist.linear.y = ydot;
		bodyFlowFrame.publish(flowspeed);
		currentTwist.publish(current);
		state.header.stamp = t;
		stateHat.publish(state);

		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}


