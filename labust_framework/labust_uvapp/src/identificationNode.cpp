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
*********************************************************************/
#include <labust/control/SOIdentification.hpp>
#include <labust/vehicles/vehiclesfwd.hpp>
#include <labust/xml/GyrosReader.hpp>
#include <labust/xml/GyrosWriter.hpp>
#include <labust/tools/TimingTools.hpp>
#include <labust/control/PIDController.hpp>

#include <std_msgs/String.h>
#include <ros/ros.h>

#include <map>

double yaw(0);
double lastTime(labust::tools::unix_time());

/**
 * The controller tuning data structure.
 */
struct TuningParameters
{
	/**
	 * Model parameters.
	 */
	double alpha,beta,betaa;
	/**
	 * Model function frequency.
	 */
	double w;
	/**
	 * Output saturation.
	 */
	double max;
};

template <class PID>
void tuneController(const TuningParameters& param, PID* pid)
{
  double a3 = 1/(param.w*param.w*param.w), a2 = 3/(param.w*param.w), a1 = 3/(param.w);

  double K_Ipsi = param.alpha/a3;
  double K_Ppsi = param.alpha*a1/a3;
  double K_Dpsi = (param.alpha*a2/a3 - param.beta);

  /*K_Ipsi = 1.55139;
  K_Ppsi = 6.64881;
  K_Dpsi = 8.0083;*/

  //pid->setGains(a1*param.alpha/a3,param.alpha*a3,a2/a3*param.alpha - param.beta,0);
  pid->setGains(K_Ppsi,K_Ipsi,K_Dpsi,K_Dpsi/10);

  std::cout<<"Param:"<<param.alpha<<","<<param.beta<<","<<param.w<<std::endl;
  std::cout<<"K_Ppsi:"<<K_Ppsi<<",K_Ipsi:"<<K_Ipsi<<",K_Dpsi:"<<K_Dpsi<<std::endl;

  labust::math::Limit<double> limit(-param.max,param.max);
  pid->setLimits(limit);

  //pid->setGains(10,0,0,0);
}

void onState(const std_msgs::String::ConstPtr& msg)
{
	labust::xml::GyrosReader reader(msg->data);
	labust::vehicles::stateMap state;
	reader.dictionary(state);

	yaw = state[labust::vehicles::state::yaw];
	lastTime = reader.GetTimeStamp();

	std::cout<<"Received."<<std::endl;
}

void sendTau(double torque,
		const ros::Publisher& tau,  double lat = 0)
{
  //send all zeros at the end.
  std::map<int,double> tauM;
  for (int i=labust::vehicles::tau::X; i<=labust::vehicles::tau::N; ++i)
  {
  	tauM[i] = 0;
  }

  tauM[labust::vehicles::tau::N] = torque;
  tauM[labust::vehicles::tau::Y] = lat;
  tauM[labust::vehicles::tau::Z] = 0;

  labust::xml::GyrosWriter writer(tauM.begin(),tauM.end());
  writer.SetTimeStamp(true);
  std_msgs::String str;
  str.data = writer.GyrosXML();
  tau.publish(str);
}

int main(int argc, char* argv[])
try
{
	ros::init(argc, argv, "vehicleNode");

	ros::NodeHandle n;


	ros::Publisher tau = n.advertise<std_msgs::String>("tau", 10);
	ros::Subscriber state = n.subscribe("state",10,&onState);

  ros::Rate loop_rate(10);
  labust::control::SOIdentification ident;

 ident.setRelay(50,15*M_PI/180);
  ident.reset();

	
 labust::control::IPD ipd;
 //ipd.setStructure(0,1,0);

  double yaw_ref = 0;
 double ditter = 0*2.5;
  double lat = 0*80;

  //sendTau(0,tau);
  int i=0;

  while (ros::ok())
  {
  	++i;
  	if (!ident.isFinished())
  	{
  		sendTau(ident.step(yaw_ref-yaw,0.1),tau);
  		//sendTau(60,tau);

  		std::cout<<yaw<<std::endl;
  	}
  	else
  	{
  		labust::control::SOIdentification::ParameterContainer param;
  		ident.parameters(&param);

  		std::cout<<"Alpha:"<<param[ident.alpha];
  		std::cout<<", beta:"<<param[ident.kx];
  		std::cout<<", betaa:"<<param[ident.kxx];
  		std::cout<<", delta:"<<param[ident.delta]<<std::endl;

  		TuningParameters tparam;

  		tparam.w = 0.4;
  		tparam.alpha = param[ident.alpha];//128;
  		tparam.beta = param[ident.kx];//41.8;
  		tparam.betaa = param[ident.kxx];
  		tparam.max = 80;

  		tuneController(tparam,&ipd);

  	  //send all zeros at the end.
  		//sendTau(0,tau);
  		//Do control.
  		if ((i%100) == 0) lat = -lat;
  	  sendTau(ipd.step(yaw_ref, yaw) + ditter,tau, lat);
  	}

    ros::spinOnce();
    //std::cout<<"Timing:"<<labust::tools::unix_time() - lastTime<<std::endl;
    loop_rate.sleep();

  }

  //send all zeros at the end.
  sendTau(0,tau);

	return 0;
}
catch (std::exception& e)
{
	std::cerr<<e.what()<<std::endl;
}



