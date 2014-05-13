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
#include <labust/navigation/EKF3D.hpp>
#include <labust/navigation/LDTravModelExtended.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/tools/DynamicsLoader.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/simulation/DynamicsParams.hpp>
#include <labust/navigation/KFModelLoader.hpp>


#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyForceReq.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>

#include <boost/bind.hpp>

using namespace labust::navigation;

Estimator3D::Estimator3D():
		tauIn(KFNav::vector::Zero(KFNav::inputSize)),
		measurements(KFNav::vector::Zero(KFNav::stateNum)),
		newMeas(KFNav::vector::Zero(KFNav::stateNum)),
		alt(0),
		useYawRate(false),
		dvl_model(0),
		compassVariance(0.3),
		gyroVariance(0.003),
		absoluteEKF(false){this->onInit();};

void Estimator3D::onInit()
{
	ros::NodeHandle nh, ph("~");
	//Configure the navigation
	configureNav(nav,nh);
	//Publishers
	stateHat = nh.advertise<auv_msgs::NavSts>("stateHat",1);
	stateMeas = nh.advertise<auv_msgs::NavSts>("meas",1);
	currentsHat = nh.advertise<geometry_msgs::TwistStamped>("currentsHat",1);
	buoyancyHat = nh.advertise<std_msgs::Float32>("buoyancy",1);
	//Subscribers
	tauAch = nh.subscribe<auv_msgs::BodyForceReq>("tauAch", 1, &Estimator3D::onTau,this);
	depth = nh.subscribe<std_msgs::Float32>("depth", 1,	&Estimator3D::onDepth, this);
	altitude = nh.subscribe<std_msgs::Float32>("altitude", 1, &Estimator3D::onAltitude, this);
	modelUpdate = nh.subscribe<navcon_msgs::ModelParamsUpdate>("model_update", 1, &Estimator3D::onModelUpdate,this);
	resetTopic = nh.subscribe<std_msgs::Bool>("reset_nav_covariance", 1, &Estimator3D::onReset,this);
	useGyro = nh.subscribe<std_msgs::Bool>("use_gyro", 1, &Estimator3D::onUseGyro,this);

	KFmode = quadMeasAvailable = false;
	sub = nh.subscribe<auv_msgs::NED>("quad_delta_pos", 1, &Estimator3D::deltaPosCallback,this);
	subKFmode = nh.subscribe<std_msgs::Bool>("KFmode", 1, &Estimator3D::KFmodeCallback, this);

	//Get DVL model
	ph.param("dvl_model",dvl_model, dvl_model);
	nav.useDvlModel(dvl_model);
	ph.param("imu_with_yaw_rate",useYawRate,useYawRate);
	ph.param("compass_variance",compassVariance,compassVariance);
	ph.param("gyro_variance",gyroVariance,gyroVariance);

	ph.param("absoluteEKF", absoluteEKF,absoluteEKF);

	//Configure handlers.
	gps.configure(nh);
	dvl.configure(nh);
	imu.configure(nh);

   Pstart = nav.getStateCovariance();
//Rstart = nav.R;
 // ROS_ERROR("NAVIGATION");

}

void Estimator3D::onReset(const std_msgs::Bool::ConstPtr& reset)
{
   if (reset->data)
   {
      nav.setStateCovariance(10000*KFNav::matrix::Identity(KFNav::stateNum, KFNav::stateNum));
   }
}

void Estimator3D::onUseGyro(const std_msgs::Bool::ConstPtr& use_gyro)
{
   if (use_gyro->data)
   {
      nav.R0(KFNav::psi, KFNav::psi) = gyroVariance;
      ROS_INFO("Switch to using gyro measurements.");
   }
   else
   {
      nav.R0(KFNav::psi, KFNav::psi) = compassVariance;
      ROS_INFO("Switch to using compass measurements.");
   }
}

void Estimator3D::configureNav(KFNav& nav, ros::NodeHandle& nh)
{
	ROS_INFO("Configure navigation.");

	labust::simulation::DynamicsParams params;
	labust::tools::loadDynamicsParams(nh, params);

	ROS_INFO("Loaded dynamics params.");

	this->params[X].alpha = params.m + params.Ma(0,0);
	this->params[X].beta = params.Dlin(0,0);
	this->params[X].betaa = params.Dquad(0,0);

	this->params[Y].alpha = params.m + params.Ma(1,1);
	this->params[Y].beta = params.Dlin(1,1);
	this->params[Y].betaa = params.Dquad(1,1);

	this->params[Z].alpha = params.m + params.Ma(2,2);
	this->params[Z].beta = params.Dlin(2,2);
	this->params[Z].betaa = params.Dquad(2,2);

	this->params[K].alpha = params.Io(0,0) + params.Ma(3,3);
	this->params[K].beta = params.Dlin(3,3);
	this->params[K].betaa = params.Dquad(3,3);

	this->params[M].alpha = params.Io(1,1) + params.Ma(4,4);
	this->params[M].beta = params.Dlin(4,4);
	this->params[M].betaa = params.Dquad(4,4);

	this->params[N].alpha = params.Io(2,2) + params.Ma(5,5);
	this->params[N].beta = params.Dlin(5,5);
	this->params[N].betaa = params.Dquad(5,5);

	nav.setParameters(this->params[X], this->params[Y],
			this->params[Z], this->params[K],
			this->params[M], this->params[N]);

	nav.initModel();
	labust::navigation::kfModelLoader(nav, nh, "ekfnav");
}

void Estimator3D::onModelUpdate(const navcon_msgs::ModelParamsUpdate::ConstPtr& update)
{
	ROS_INFO("Updating the model parameters for %d DoF.",update->dof);
	params[update->dof].alpha = update->alpha;
	if (update->use_linear)
	{
		params[update->dof].beta = update->beta;
		params[update->dof].betaa = 0;
	}
	else
	{
		params[update->dof].beta = 0;
		params[update->dof].betaa = update->betaa;
	}
	nav.setParameters(this->params[X],this->params[Y],
			this->params[Z], this->params[K],
			this->params[M],this->params[N]);
}

void Estimator3D::onTau(const auv_msgs::BodyForceReq::ConstPtr& tau)
{
	tauIn(KFNav::X) = tau->wrench.force.x;
	tauIn(KFNav::Y) = tau->wrench.force.y;
	tauIn(KFNav::Z) = tau->wrench.force.z;
	tauIn(KFNav::Kroll) = tau->wrench.torque.x;
	tauIn(KFNav::M) = tau->wrench.torque.y;
	tauIn(KFNav::N) = tau->wrench.torque.z;
};

void Estimator3D::onDepth(const std_msgs::Float32::ConstPtr& data)
{
	measurements(KFNav::zp) = data->data;
	newMeas(KFNav::zp) = 1;
};

void Estimator3D::onAltitude(const std_msgs::Float32::ConstPtr& data)
{
	measurements(KFNav::altitude) = data->data;
	//Dismiss false altitude
	if (fabs(data->data-nav.getState()(KFNav::altitude)) < 10*nav.calculateAltInovationVariance(nav.getStateCovariance())) 
	{
		newMeas(KFNav::altitude) = 1;
		alt = data->data;
		ROS_INFO("Accepted altitude: meas=%f, estimate=%f, variance=%f",
			data->data, nav.getState()(KFNav::altitude), 10* nav.calculateAltInovationVariance(nav.getStateCovariance()));
	}
	else
	{
		ROS_INFO("Dissmissed altitude: meas=%f, estimate=%f, variance=%f",
			data->data, nav.getState()(KFNav::altitude), 10* nav.calculateAltInovationVariance(nav.getStateCovariance()));
	}
};

void Estimator3D::deltaPosCallback(const auv_msgs::NED::ConstPtr& msg){

	quadMeasAvailable = true;
	deltaXpos = msg->north;
	deltaYpos = msg->east;
	//ROS_ERROR("PRimio topic");
}

void Estimator3D::KFmodeCallback(const std_msgs::Bool::ConstPtr& msg){

	if(!absoluteEKF){
		KFmode = msg->data;

		if(KFmode && (KFmodePast xor KFmode)){

			KFmodePast = KFmode;
			KFNav::matrix P = nav.getStateCovariance();
			P(KFNav::xp,KFNav::xp) = 10000;
			P(KFNav::yp,KFNav::yp) = 10000;
			nav.setStateCovariance(P);

			//nav.R0(KFNav::xp,KFNav::xp) = 1;
			//nav.R0(KFNav::xp,KFNav::xp) = 1;
			//nav.R(7,7) = 0.001;


		} else if(!KFmode && (KFmodePast xor KFmode)){

		KFmodePast = KFmode;
			KFNav::matrix P = nav.getStateCovariance();
			P(KFNav::xp,KFNav::xp) = 10000;
			P(KFNav::yp,KFNav::yp) = 10000;
			nav.setStateCovariance(P);

			//nav.setStateCovariance(Pstart);

			//nav.R0 = Rstart;

			//Rstart(KFNav::xp,KFNav::xp)
		}
	}
}

void Estimator3D::processMeasurements()
{
	if(KFmode == true && absoluteEKF == false)
		{
			if ((newMeas(KFNav::xp) = newMeas(KFNav::yp) = quadMeasAvailable)){

				quadMeasAvailable = false;
				measurements(KFNav::xp) = deltaXpos;
				measurements(KFNav::yp) = deltaYpos;
		    }
		} else {

			//GPS measurements
			if ((newMeas(KFNav::xp) = newMeas(KFNav::yp) = gps.newArrived()))
			{
				measurements(KFNav::xp) = gps.position().first;
				measurements(KFNav::yp) = gps.position().second;
			}
		}

		//ROS_ERROR("xp: %4.2f, yp: %4.2f, MODE: %d",measurements(KFNav::xp),measurements(KFNav::yp),KFmode );

	//Imu measurements
	if ((/*newMeas(KFNav::phi) = newMeas(KFNav::theta) = */newMeas(KFNav::psi) = imu.newArrived()))
	{
		//measurements(KFNav::phi) = imu.orientation()[ImuHandler::roll];
		//measurements(KFNav::theta) = imu.orientation()[ImuHandler::pitch];
		measurements(KFNav::psi) = imu.orientation()[ImuHandler::yaw];

		ROS_INFO("NEW IMU: r=%f, p=%f, y=%f", imu.orientation()[ImuHandler::roll],
				imu.orientation()[ImuHandler::pitch],
				imu.orientation()[ImuHandler::yaw]);

		if ((newMeas(KFNav::r) = useYawRate))
		{
			measurements(KFNav::r) = imu.rate()[ImuHandler::r];
		}
	}
	//DVL measurements
	//if ((newMeas(KFNav::u) = newMeas(KFNav::v) = newMeas(KFNav::w) = dvl.NewArrived()))
	if ((newMeas(KFNav::u) = newMeas(KFNav::v) = dvl.newArrived()))
	{
		double vx = dvl.body_speeds()[DvlHandler::u];
		double vy = dvl.body_speeds()[DvlHandler::v];
		double vxe = nav.getState()(KFNav::u); 
		double vye = nav.getState()(KFNav::v); 

		double rvx(10),rvy(10);
		//Calculate the measured value
		//This depends on the DVL model actually, but lets assume
		nav.calculateUVInovationVariance(nav.getStateCovariance(), rvx, rvy);

		double cpsi = cos(nav.getState()(KFNav::psi));
		double spsi = sin(nav.getState()(KFNav::psi));
		double xc = nav.getState()(KFNav::xc);
		double yc = nav.getState()(KFNav::yc);
		switch (dvl_model)
		{
		  case 1:
		    vxe += xc*cpsi + yc*spsi;
		    vye += -xc*spsi + yc*cpsi;
		    break;
		default: break;
		}

		if (fabs((vx - vxe)) > fabs(rvx))
		{
			ROS_INFO("Outlier rejected: meas=%f, est=%f, tolerance=%f", vx, nav.getState()(KFNav::u), fabs(rvx));
			newMeas(KFNav::u) = false;
		}
		measurements(KFNav::u) = vx;


		if (fabs((vy - vye)) > fabs(rvy))
		{
			ROS_INFO("Outlier rejected: meas=%f, est=%f, tolerance=%f", vy, nav.getState()(KFNav::v), fabs(rvy));
			newMeas(KFNav::v) = false;
		}
		measurements(KFNav::v) = vy;

		//measurements(KFNav::w) = dvl.body_speeds()[DvlHandler::w];
	}

	//Publish measurements
	auv_msgs::NavSts::Ptr meas(new auv_msgs::NavSts());
	meas->body_velocity.x = measurements(KFNav::u);
	meas->body_velocity.y = measurements(KFNav::v);
	meas->body_velocity.z = measurements(KFNav::w);

	meas->position.north = measurements(KFNav::xp);
	meas->position.east = measurements(KFNav::yp);
	meas->position.depth = measurements(KFNav::zp);
	meas->altitude = measurements(KFNav::altitude);

	meas->orientation.roll = measurements(KFNav::phi);
	meas->orientation.pitch = measurements(KFNav::theta);
	meas->orientation.yaw = labust::math::wrapRad(measurements(KFNav::psi));
	if (useYawRate)	meas->orientation_rate.yaw = measurements(KFNav::r);

	meas->origin.latitude = gps.origin().first;
	meas->origin.longitude = gps.origin().second;
	meas->global_position.latitude = gps.latlon().first;
	meas->global_position.longitude = gps.latlon().second;

	meas->header.stamp = ros::Time::now();
	meas->header.frame_id = "local";
	stateMeas.publish(meas);
}

void Estimator3D::publishState()
{
	auv_msgs::NavSts::Ptr state(new auv_msgs::NavSts());
	const KFNav::vector& estimate = nav.getState();
	state->body_velocity.x = estimate(KFNav::u);
	state->body_velocity.y = estimate(KFNav::v);
	state->body_velocity.z = estimate(KFNav::w);

	state->orientation_rate.roll = estimate(KFNav::p);
	state->orientation_rate.pitch = estimate(KFNav::q);
	state->orientation_rate.yaw = estimate(KFNav::r);

	state->position.north = estimate(KFNav::xp);
	state->position.east = estimate(KFNav::yp);
	state->position.depth = estimate(KFNav::zp);
	state->altitude = estimate(KFNav::altitude);

	state->orientation.roll = estimate(KFNav::phi);
	state->orientation.pitch = estimate(KFNav::theta);
	state->orientation.yaw = labust::math::wrapRad(estimate(KFNav::psi));

	state->origin.latitude = gps.origin().first;
    state->origin.longitude = gps.origin().second;
	std::pair<double, double> diffAngle = labust::tools::meter2deg(state->position.north,
			state->position.east,
			//The latitude angle
			state->origin.latitude);
	state->global_position.latitude = state->origin.latitude + diffAngle.first;
	state->global_position.longitude = state->origin.longitude + diffAngle.second;

	const KFNav::matrix& covariance = nav.getStateCovariance();
	state->position_variance.north = covariance(KFNav::xp, KFNav::xp);
	state->position_variance.east = covariance(KFNav::yp, KFNav::yp);
	state->position_variance.depth = covariance(KFNav::zp,KFNav::zp);
	state->orientation_variance.roll =  covariance(KFNav::phi, KFNav::phi);
	state->orientation_variance.pitch =  covariance(KFNav::theta, KFNav::theta);
	state->orientation_variance.yaw =  covariance(KFNav::psi, KFNav::psi);

	state->header.stamp = ros::Time::now();
	state->header.frame_id = "local";
	stateHat.publish(state);

	geometry_msgs::TwistStamped::Ptr current(new geometry_msgs::TwistStamped());
	current->twist.linear.x = estimate(KFNav::xc);
	current->twist.linear.y = estimate(KFNav::yc);
	current->header.stamp = ros::Time::now();
	current->header.frame_id = "local";
	currentsHat.publish(current);

	std_msgs::Float32::Ptr buoyancy(new std_msgs::Float32());
	buoyancy->data = estimate(KFNav::buoyancy);
	buoyancyHat.publish(buoyancy);
}

void Estimator3D::start()
{
	ros::NodeHandle ph("~");
	double Ts(0.1);
	ph.param("Ts",Ts,Ts);
	ros::Rate rate(1/Ts);
	nav.setTs(Ts);

	while (ros::ok())
	{
		nav.predict(tauIn);
		processMeasurements();
		bool newArrived(false);
		for(size_t i=0; i<newMeas.size(); ++i)	if ((newArrived = newMeas(i))) break;
		if (newArrived)	nav.correct(nav.update(measurements, newMeas));
		//if (newArrived)	nav.correct(measurements, newMeas);

		publishState();

		//Send the base-link transform
		geometry_msgs::TransformStamped transform;
		transform.transform.translation.x = nav.getState()(KFNav::xp);
		transform.transform.translation.y = nav.getState()(KFNav::yp);
		transform.transform.translation.z = nav.getState()(KFNav::zp);
		labust::tools::quaternionFromEulerZYX(nav.getState()(KFNav::phi),
				nav.getState()(KFNav::theta),
				nav.getState()(KFNav::psi),
				transform.transform.rotation);
		transform.child_frame_id = "base_link";
		transform.header.frame_id = "local";
		transform.header.stamp = ros::Time::now();
		broadcaster.sendTransform(transform);

		rate.sleep();
		ros::spinOnce();
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"nav_3d");
	Estimator3D nav;
	nav.start();
	return 0;
}


