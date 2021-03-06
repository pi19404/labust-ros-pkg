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
#include <labust/navigation/XYModel.hpp>
#include <labust/math/uBlasOperations.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/tools/DynamicsLoader.hpp>
#include <labust/simulation/DynamicsParams.hpp>

#include <kdl/frames.hpp>
#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyForceReq.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

#include <boost/bind.hpp>
#include <sstream>
#include <fstream>

typedef labust::navigation::KFCore<labust::navigation::XYModel> KFNav;
tf::TransformListener* listener;

ros::Time t;
KFNav::vector measurement(KFNav::zeros(KFNav::stateNum)), newMeas(KFNav::zeros(KFNav::stateNum));

void handleTau(KFNav::vector& tauIn, const auv_msgs::BodyForceReq::ConstPtr& tau)
{
	tauIn(KFNav::X) = tau->wrench.force.x;
	tauIn(KFNav::Y) = tau->wrench.force.y;
	tauIn(KFNav::N) = tau->wrench.torque.z;
};

void handleGPS(KFNav::vector& xy, const sensor_msgs::NavSatFix::ConstPtr& data)
{
	enum {x,y,newMsg};
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
		xy(x) = posxy.first;
		xy(y) = posxy.second;
		xy(newMsg) = 1;

		//For generic updates
		newMeas(KFNav::xp) = 1;
		newMeas(KFNav::yp) = 1;
		measurement(KFNav::xp) = posxy.first;
		measurement(KFNav::yp) = posxy.second;
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}
};

void handleImu(KFNav::vector& rpy, const sensor_msgs::Imu::ConstPtr& data)
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
		rpy(y) = yaw;
		rpy[newMsg] = 1;

		//For generic updates
		newMeas(KFNav::psi) = 1;
		newMeas(KFNav::r) = 1;
		measurement(KFNav::psi) = unwrap(yaw);
		measurement(KFNav::r) = data->angular_velocity.z;
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}
};

void configureNav(KFNav& nav, ros::NodeHandle& nh)
{
	ROS_INFO("Configure navigation.");

	labust::simulation::DynamicsParams params;
	labust::tools::loadDynamicsParams(nh, params);

	ROS_INFO("Loaded dynamics params.");

	KFNav::ModelParams surge,sway,yaw;
	surge.alpha = params.m + params.Ma(0,0);
	sway.alpha = params.m + params.Ma(1,1);
	yaw.alpha = params.Io(2,2) + params.Ma(5,5);
	surge.beta = params.Dlin(0,0);
	sway.beta = params.Dlin(1,1);
	yaw.beta = params.Dlin(5,5);
	surge.betaa = params.Dquad(0,0);
	sway.betaa = params.Dquad(1,1);
	yaw.betaa = params.Dquad(5,5);

	std::string sQ,sW,sV,sR,sP,sx0;
	nh.getParam("ekfnav/Q", sQ);
	nh.getParam("ekfnav/W", sW);
	nh.getParam("ekfnav/V", sV);
	nh.getParam("ekfnav/R", sR);
	nh.getParam("ekfnav/P", sP);
	nh.getParam("ekfnav/x0", sx0);
	KFNav::matrix Q,W,V,R,P;
	KFNav::vector x0;
	boost::numeric::ublas::matrixFromString(sQ,Q);
	boost::numeric::ublas::matrixFromString(sW,W);
	boost::numeric::ublas::matrixFromString(sV,V);
	boost::numeric::ublas::matrixFromString(sR,R);
	boost::numeric::ublas::matrixFromString(sP,P);
	std::stringstream ss(sx0);
	boost::numeric::ublas::operator >>(ss,x0);

	double dT(0.1);
	nh.param("sampling_time",dT,dT);
	nav.setTs(dT);
	nav.setParameters(surge,sway,yaw);
	nav.setStateParameters(W,Q);
	nav.setMeasurementParameters(V,R);
	nav.setStateCovariance(P);
	nav.initModel();
	nav.setState(x0);
}

void offline_sim(const std::string& filename, KFNav& ekf)
{
	std::fstream file(filename.c_str());
	std::ofstream log("test.csv");
	std::cout<<"Reading log file:"<<filename<<std::endl;

	//Define a desired structure
	log<<"%element: off_data\n"
	"% body_velocity.x\n"
	"% body_velocity.y\n"
	"% orientation_rate.yaw\n"
	"% position.north\n"
	"% position.east\n"
	"% orientation.yaw\n"
	"% current.xc\n"
	"% current.yc\n"
	"% b1\n"
	"% b2\n"
	"%element: off_cov \n"
  "% u_cov\n"
	"% v_cov\n"
	"% r_cov\n"
	"% x_cov\n"
	"% y_cov\n"
	"% psi_cov\n"
	"% xc_cov\n"
	"% yc_cov\n"
	"% b1_cov\n"
	"% b2_cov\n";

	double tauX,tauY,tauN,x,y,psi,yaw_rate;
	file>>tauX>>tauY>>tauN>>x>>y>>psi>>yaw_rate;

	KFNav::vector state(KFNav::zeros(KFNav::stateNum));
	state(KFNav::xp) = x;
	state(KFNav::yp) = y;
	state(KFNav::psi) = psi;

	ekf.setState(state);

	double lastx,lasty;
	labust::math::unwrap unwrap;

	int i=0;
	while(!file.eof() && ros::ok())
	{
		file>>tauX>>tauY>>tauN>>x>>y>>psi>>yaw_rate;
		//psi -= 2*M_PI*3.0/180;
		KFNav::input_type input(KFNav::inputSize);
		input(KFNav::X) = tauX;
		input(KFNav::Y) = tauY;
		input(KFNav::N) = tauN;
		ekf.predict(input);

		KFNav::vector newMeas(KFNav::zeros(KFNav::stateNum));
		KFNav::vector measurement(KFNav::zeros(KFNav::stateNum));

		double yaw = unwrap(psi);

		newMeas(KFNav::psi) = 1;
		newMeas(KFNav::r) = 1;
		measurement(KFNav::psi) = yaw;
		measurement(KFNav::r) = yaw_rate;

		if (i%10 == 0)
		{
			//if (!((lastx == x) && (lasty == y)))
			newMeas(KFNav::xp) = 1;
			newMeas(KFNav::yp) = 1;
			measurement(KFNav::xp) = x;
			measurement(KFNav::yp) = y;
		}

		ekf.correct(ekf.update(measurement, newMeas));

		lastx=x;
		lasty=y;

		//std::cout<<"Meas:"<<x<<","<<y<<","<<M_PI*psi/180<<std::endl;
		//std::cout<<"Estimate:"<<ekf.getState()(KFNav::xp)<<",";
		//std::cout<<ekf.getState()(KFNav::yp)<<","<<ekf.getState()(KFNav::psi)<<std::endl;
		//std::cout<<ekf.traceP()<<std::endl;

		log<<ekf.getState()(0);
		for (size_t j = 1; j< ekf.getState().size(); ++j)
			log<<","<<((j==KFNav::psi)?labust::math::wrapRad(ekf.getState()(j)):ekf.getState()(j));
		for (size_t j = 0; j< ekf.getState().size(); ++j)	log<<","<<ekf.getStateCovariance()(j,j);
		log<<"\n";

		++i;
	}
}

//consider making 2 corrections based on arrived measurements (async)
int main(int argc, char* argv[])
{
	ros::init(argc,argv,"pladypos_nav");
	ros::NodeHandle nh;
	KFNav nav;
	//Configure the navigation
	configureNav(nav,nh);

	double outlierR;
	nh.param("outlier_radius",outlierR,1.0);

	/////////////////////////////////////////
	//ADDED FOR LOG-BASED NAVIGATION TUNING
	bool offline(false);
	nh.param("offline",offline,false);
	std::string filename("");
	nh.param("offline_file",filename,filename);

	if (offline)
	{
		offline_sim(filename, nav);
		return 0;
	}
	////////////////////////////////////////

	//Publishers
	ros::Publisher stateHat = nh.advertise<auv_msgs::NavSts>("stateHat",1);
	ros::Publisher stateMeas = nh.advertise<auv_msgs::NavSts>("meas",1);
	ros::Publisher currentTwist = nh.advertise<geometry_msgs::TwistStamped>("currentsHat",1);
	ros::Publisher bodyFlowFrame = nh.advertise<geometry_msgs::TwistStamped>("body_flow_frame_twist",1);
	//Subscribers
	KFNav::vector tau(KFNav::zeros(KFNav::inputSize)),xy(KFNav::zeros(2+1)),rpy(KFNav::zeros(3+1));

	tf::TransformBroadcaster broadcast;
	listener = new tf::TransformListener();

	ros::Subscriber tauAch = nh.subscribe<auv_msgs::BodyForceReq>("tauAch", 1, boost::bind(&handleTau,boost::ref(tau),_1));
	ros::Subscriber navFix = nh.subscribe<sensor_msgs::NavSatFix>("gps", 1, boost::bind(&handleGPS,boost::ref(xy),_1));
	ros::Subscriber imu = nh.subscribe<sensor_msgs::Imu>("imu", 1, boost::bind(&handleImu,boost::ref(rpy),_1));

	ros::Rate rate(10);

	auv_msgs::NavSts meas,state;
	geometry_msgs::TwistStamped current, flowspeed;
	meas.header.frame_id = "local";
	state.header.frame_id = "local";
	current.header.frame_id = "local";

	labust::math::unwrap unwrap;

	while (ros::ok())
	{
		nav.predict(tau);

		bool newArrived(false);

		for(size_t i=0; i<newMeas.size(); ++i)
		{
			if ((newArrived = newMeas(i))) break;
		}

		if (newArrived)
		{
			//Generic measurment outlier rejection
			bool outlier = false;
			double x(nav.getState()(KFNav::xp)), y(nav.getState()(KFNav::yp));
			double inx(0),iny(0);
			nav.calculateXYInovationVariance(nav.getStateCovariance(),inx,iny);
			outlier = sqrt(pow(x-xy(0),2) + pow(y-xy(1),2)) > outlierR*sqrt(inx*inx + iny*iny);
			if (outlier)
			{
				ROS_INFO("Outlier rejected: meas(%f, %f), estimate(%f,%f), inovationCov(%f,%f)",xy(0),xy(1),x,y,inx,iny);
				newMeas(KFNav::xp) = 0;
				newMeas(KFNav::yp) = 0;
			}

			nav.correct(nav.update(measurement, newMeas));
			//Clear measurements
			newMeas = KFNav::zeros(KFNav::stateNum);
		}

//		if (rpy(3))
//		{
//			double yaw = unwrap(rpy(2));
//
//			bool outlier = false;
//			double x(nav.getState()(KFNav::xp)), y(nav.getState()(KFNav::yp));
//			double inx(0),iny(0);
//			nav.calculateXYInovationVariance(nav.getStateCovariance(),inx,iny);
//			outlier = sqrt(pow(x-xy(0),2) + pow(y-xy(1),2)) > outlierR*sqrt(inx*inx + iny*iny);
//			if (outlier)
//			{
//				ROS_INFO("Outlier rejected: meas(%f, %f), estimate(%f,%f), inovationCov(%f,%f)",xy(0),xy(1),x,y,inx,iny);
//				xy(2) = 0;
//			}
//
//			if (xy(2) == 1 && !outlier)
//			{
//				ROS_INFO("XY correction: meas(%f, %f), estimate(%f,%f), inovationCov(%f,%f,%d)",xy(0),xy(1),x,y,inx,iny,nav.getInovationCovariance().size1());
//				nav.correct(nav.fullUpdate(xy(0),xy(1),yaw));
//				xy(2) = 0;
//			}
//			else
//			{
//				ROS_INFO("Heading correction:%f",yaw);
//				nav.correct(nav.yawUpdate(yaw));
//			}
//			rpy(3) = 0;
//		}

		meas.orientation.roll = rpy(0);
		meas.orientation.pitch = rpy(1);
		meas.orientation.yaw = rpy(2);
		meas.orientation_rate.yaw = measurement(KFNav::r);
		meas.position.north = xy(0);
		meas.position.east = xy(1);
		meas.header.stamp = ros::Time::now();
		stateMeas.publish(meas);

		const KFNav::vector& estimate = nav.getState();
		state.body_velocity.x = estimate(KFNav::u);
		state.body_velocity.y = estimate(KFNav::v);
		state.orientation_rate.yaw = estimate(KFNav::r);
		state.position.north = estimate(KFNav::xp);
		state.position.east = estimate(KFNav::yp);
		current.twist.linear.x = estimate(KFNav::xc);
		current.twist.linear.y = estimate(KFNav::yc);

		const KFNav::matrix& covariance = nav.getStateCovariance();
		state.position_variance.north = covariance(KFNav::xp, KFNav::xp);
		state.position_variance.east = covariance(KFNav::yp, KFNav::yp);
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
			state.origin.latitude = transformDeg.getOrigin().y();
			state.origin.longitude = transformDeg.getOrigin().x();
		}
		catch(tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
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

