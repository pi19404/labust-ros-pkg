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
#include <labust/tools/DynamicsLoader.hpp>

#include <auv_msgs/NavSts.h>
#include <nav_msgs/Odometry.h>

#include <sstream>

using namespace labust::simulation;

void labust::simulation::configureModel(const ros::NodeHandle& nh, RBModel& model)
{
	labust::tools::loadDynamicsParams(nh, model);

	nh.param("sampling_time",model.dT, model.dT);
	nh.param("coupled",model.isCoupled,model.isCoupled);
	Eigen::Vector3d bdg;
	labust::tools::getMatrixParam(nh,"bounding_ellipsoid",bdg);
	model.ae = bdg(0);
	model.be = bdg(1);
	model.ce = bdg(2);
	labust::tools::getMatrixParam(nh,"eta0",model.eta0);
	labust::tools::getMatrixParam(nh,"nu0",model.nu0);
	labust::tools::getMatrixParam(nh,"current",model.current);

	typedef Eigen::Matrix<double,6,1> NoiseVec;
	NoiseVec pn(NoiseVec::Zero()),mn(NoiseVec::Zero());
	labust::tools::getMatrixParam(nh,"process_noise",pn);
	labust::tools::getMatrixParam(nh,"measurement_noise",mn);
	model.noise.setNoiseParams(pn,mn);

	model.init();
}

SimCore::SimCore():
		tau(vector::Zero()),
		rate(10),
		wrap(1)
{
	this->onInit();
}

void SimCore::onInit()
{
	ros::NodeHandle nh,ph("~");
	configureModel(nh, model);
	modelReport();

	tauIn = nh.subscribe<auv_msgs::BodyForceReq>("tauIn", 1, &SimCore::onTau<auv_msgs::BodyForceReq>, this);

	tauInWrench = nh.subscribe<geometry_msgs::WrenchStamped>("tauInWrench", 1, &SimCore::onTau<geometry_msgs::WrenchStamped>, this);

	currentsSub = nh.subscribe<geometry_msgs::TwistStamped>("currents", 1, &SimCore::onCurrents, this);

	//Publishers
	meas = nh.advertise<auv_msgs::NavSts>("meas_ideal",1);
	measn = nh.advertise<auv_msgs::NavSts>("meas_noisy",1);
	tauAch = nh.advertise<auv_msgs::BodyForceReq>("tauAch",1);

	odom = nh.advertise<nav_msgs::Odometry>("meas_odom",1);
	odomn = nh.advertise<nav_msgs::Odometry>("meas_odom_noisy",1);
  tauAchWrench = nh.advertise<geometry_msgs::WrenchStamped>("tauAchWrench",1);

	double fs(10);
	ph.param("Rate",fs,fs);
	ph.param("ModelWrap",wrap,wrap);
	model.dT = 1/(fs*wrap);
	rate = ros::Rate(fs);

	runner = boost::thread(boost::bind(&SimCore::start, this));
}

void SimCore::onCurrents(const geometry_msgs::TwistStamped::ConstPtr& currents)
{
	boost::mutex::scoped_lock l(model_mux);
	labust::tools::pointToVector(currents->twist.linear, model.current);
}

void SimCore::start()
{
	while (ros::ok())
	{
		for (size_t i=0; i<wrap;++i) model.step(tau);

		for (std::vector<SimSensorInterface::Ptr>::iterator it=sensors.begin();
				it != sensors.end(); ++it)
		{
			(*it)->step(model, broadcast, listener);
		}

		rate.sleep();
	}
		/*while (ros::ok())
			{

				using namespace labust::simulation;
				Eigen::Vector3f tauXYN,tauXYNsc;
				tauXYN<<tau(VehicleModel6DOF::X),tau(VehicleModel6DOF::Y),tau(VehicleModel6DOF::N);
				double scale = allocator.scale(tauXYN,&tauXYNsc);

		//		tau(VehicleModel6DOF::X) = labust::math::coerce(tau(VehicleModel6DOF::X), minThrust, maxThrust);
		//		tau(VehicleModel6DOF::N) = labust::math::coerce(tau(VehicleModel6DOF::N), minThrust, maxThrust);
		//
		//		//Differential allocation
		//		double t1 = (tau(VehicleModel6DOF::X) + tau(VehicleModel6DOF::N))/2;
		//		double t2 = (tau(VehicleModel6DOF::X) - tau(VehicleModel6DOF::N))/2;
		//
		//		t1 = labust::math::coerce(t1, minThrust, maxThrust);
		//		t2 = labust::math::coerce(t2, minThrust, maxThrust);

				auv_msgs::BodyForceReq t;
				//tau(VehicleModel6DOF::X) = t.wrench.force.x = t1+t2;
				tau(VehicleModel6DOF::X) = t.wrench.force.x = tauXYNsc(0);
				tau(VehicleModel6DOF::Y) = t.wrench.force.y = tauXYNsc(1);
				t.wrench.force.z = tau(VehicleModel6DOF::Z);
				t.wrench.torque.x = tau(VehicleModel6DOF::K);
				t.wrench.torque.y = tau(VehicleModel6DOF::M);
				tau(VehicleModel6DOF::N) = t.wrench.torque.z = tauXYNsc(2);
				//tau(VehicleModel6DOF::N) = t.wrench.torque.z = t1-t2;
				t.header.stamp = ros::Time::now();

				//t.disable_axis.x = tau(VehicleModel6DOF::X) != tauXYN(0);
				//t.disable_axis.yaw = tau(VehicleModel6DOF::N) != tauXYN(2);

				//scale = 1;

				//Publish the scaled values if scaling occured
				if (scale>1)
				{
					//Signal windup occured
					t.disable_axis.x = t.disable_axis.y = t.disable_axis.yaw = 1;
				}

				tauAch.publish(t);

				model.setCurrent(current);
				//Perform simulation with smaller sampling type if wrap>1
				for (size_t i=0; i<wrap;++i) model.step(tau);

				//uwsim.publish(*mapToUWSimOdometry(model.Eta(),model.Nu(),&odom, lisWorld));
				state.publish(*mapToNavSts(model.Eta(),model.Nu(),&nav));
				stateNoisy.publish(*mapToNavSts(model.EtaNoisy(),model.NuNoisy(),&navNoisy));

				const vector& Nu = (useNoisy?model.NuNoisy():model.Nu());
				const vector& Eta = (useNoisy?model.EtaNoisy():model.Eta());
				if ((ros::Time::now()-lastGps).sec >= gpsTime)
				{
					mapToNavSatFix(Eta,Nu,&fix,utmzone,lisWorld,localFrame);
					if (fix.altitude >= 0)
					{
						gpsFix.publish(fix);
					}
					lastGps = ros::Time::now();
				}

				imuMeas.publish(*mapToImu(Eta,Nu,model.NuAcc(),&imu,localFrame));
				dvlMeas.publish(*mapToDvl(Eta,Nu,&dvl,localFrame));
				depth.fluid_pressure = model.getPressure(Eta(VehicleModel6DOF::z));
				depth.header.frame_id = "local";
				pressureMeas.publish(depth);

				tf::Transform transform;
				transform.setOrigin(tf::Vector3(originLon, originLat, 0));
				transform.setRotation(tf::createQuaternionFromRPY(0,0,0));
				Eigen::Quaternion<float> q;
				labust::tools::quaternionFromEulerZYX(M_PI,0,M_PI/2,q);
				if (publishWorld)
				{
					localFrame.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/worldLatLon", "/world"));
					transform.setOrigin(tf::Vector3(0, 0, 0));
					transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
					localFrame.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "local"));
				}

				tf::Transform transform3;
				transform3.setOrigin(tf::Vector3(0, 0, 0));
				q = q.conjugate();
				transform3.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
				localFrame.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "local", "uwsim_frame"));

				const vector& eta = model.Eta();
				//tf::Transform transform;
				transform.setOrigin(tf::Vector3(eta(VehicleModel6DOF::x),
						eta(VehicleModel6DOF::y),
						eta(VehicleModel6DOF::z)));
				labust::tools::quaternionFromEulerZYX(eta(VehicleModel6DOF::phi),
						eta(VehicleModel6DOF::theta),
						eta(VehicleModel6DOF::psi), q);
				transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
				localFrame.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local", "base_link_sim"));

				tf::Transform transform2;
				transform2.setOrigin(tf::Vector3(0, 0, -0.25));
				transform2.setRotation(tf::createQuaternionFromRPY(0,0,0));
				localFrame.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "gps_frame"));

				ros::spinOnce();
	}
	*/
}

void SimCore::modelReport()
{
	ROS_INFO("Loaded the model:");
	ROS_INFO(" sampling-time: %f, mass: %f, gravity: %f, density: %f",
			model.dT, model.m, model.g_acc, model.rho);
	std::ostringstream str;
	str<<"["<<model.eta0.transpose()<<"], ["<<model.nu0.transpose()<<"]"<<std::endl;
	ROS_INFO(" (Eta0,Nu0): %s",str.str().c_str());
	str.str("");
	str<<"["<<model.current.transpose()<<"]";
	ROS_INFO(" Current: %s",str.str().c_str());
	ROS_INFO(" Bounding ellipsoid: (%f,%f,%f)",model.ae, model.be, model.ce);
	ROS_INFO(" Is coupled model: %d",model.isCoupled);
	str.str("");
	str<<model.Io<<"\n"<<model.Ma<<"\n"<<model.Dlin<<"\n"<<model.Dquad;
	ROS_INFO("(Io,Ma,Dlin,Dquad):\n%s",str.str().c_str());
}
