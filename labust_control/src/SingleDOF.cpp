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
#include <labust/control/HLControl.hpp>
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/WindupPolicy.hpp>
#include <labust/control/PIFFController.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/simulation/DynamicsParams.hpp>
#include <labust/tools/DynamicsLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <Eigen/Dense>
#include <auv_msgs/BodyForceReq.h>
#include <ros/ros.h>

namespace labust
{
	namespace control{
		///The single DOF controller
		struct SingleDOF : DisableAxis
		{
			int DOF;

			SingleDOF():DOF(0),Ts(0.1){};

			void init()
			{
				initialize_controller();
			}

  		void windup(const auv_msgs::BodyForceReq& tauAch)
			{
				//Copy into controller
  			///\todo Improve on this.
  			double temp[] =
  			{
  					tauAch.disable_axis.x,
  					tauAch.disable_axis.y,
  					tauAch.disable_axis.z,
  					tauAch.disable_axis.roll,
  					tauAch.disable_axis.pitch,
  					tauAch.disable_axis.yaw
  			};

  			con.windup = temp[DOF];
			};

  		void reset(const auv_msgs::BodyVelocityReq& ref, const auv_msgs::NavSts& state)
  		{
  			con.internalState = 0;
  		};

			auv_msgs::BodyForceReqPtr step(const auv_msgs::BodyVelocityReq& ref,
					const auv_msgs::NavSts& state)
			{
  			double tref[6], tstate[6];
  			//Copy linear speeds
  			labust::tools::pointToVector(ref.twist.linear,tref);
  			labust::tools::pointToVector(state.body_velocity,tstate);
  			//Copy angular speeds
  			labust::tools::pointToVector(ref.twist.angular,tref,3);
  			labust::tools::rpyToVector(state.orientation_rate,tstate,3);

				con.desired = tref[DOF];
				con.state = tstate[DOF];
				PIFF_step(&con,Ts);

				auv_msgs::BodyForceReqPtr tau(new auv_msgs::BodyForceReq());
				tau->header.stamp = ros::Time::now();
				tau->goal.requester = "sdof_controller";
				labust::tools::vectorToDisableAxis(disable_axis, tau->disable_axis);

				double ttau[6]={0};
				ttau[DOF] = con.output;
				labust::tools::vectorToPoint(tref, tau->wrench.force);
				labust::tools::vectorToPoint(tref, tau->wrench.torque, 3);

				return tau;
			}

			void initialize_controller()
			{
				ros::NodeHandle nh;

				typedef Eigen::Matrix<double,6,1> Vector6d;
				using labust::simulation::vector;
				vector closedLoopFreq(vector::Ones());
				labust::tools::getMatrixParam(nh,"velocity_controller/closed_loop_freq", closedLoopFreq);
				vector outputLimit(vector::Zero());
				labust::tools::getMatrixParam(nh,"velocity_controller/output_limits", outputLimit);
				vector autoTracking(vector::Zero());
				labust::tools::getMatrixParam(nh,"velocity_controller/auto_tracking", autoTracking);

				labust::simulation::DynamicsParams model;
				labust::tools::loadDynamicsParams(nh,model);
				vector alphas(model.Ma.diagonal());
				vector alpha_mass;
				alpha_mass<<model.m,model.m,model.m,model.Io.diagonal();
				alphas += alpha_mass;

				nh.param("velocity_controller/Ts",Ts,Ts);

				PIDBase_init(&con);
				PT1Model modelParams;
				modelParams.alpha = alphas(DOF);
				modelParams.beta = model.Dlin(DOF,DOF);
				modelParams.betaa = model.Dquad(DOF,DOF);
				con.outputLimit = outputLimit(DOF);
				con.autoWindup = autoTracking(DOF);

				disable_axis[DOF] = 0;

				PIDBase_init(&con);
				PIFF_modelTune(&con, &modelParams, float(closedLoopFreq(DOF)));
			}

		private:
			PIDBase con;
			double Ts;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"sdof_control");

	ros::NodeHandle ph("~");
	std::string type("speed");
	int dof(0);
	ph.param("controller_type",type,type);
	ph.param("dof",dof,dof);

	labust::control::HLControl<labust::control::SingleDOF,
	labust::control::EnableServicePolicy,
	labust::control::WindupPolicy<auv_msgs::BodyForceReq>,
	auv_msgs::BodyForceReq,
	auv_msgs::NavSts,
	auv_msgs::BodyVelocityReq> controller;
	//Initialize for desired DOF
	controller.DOF = dof;
	controller.init();

	ros::spin();

	return 0;
}



