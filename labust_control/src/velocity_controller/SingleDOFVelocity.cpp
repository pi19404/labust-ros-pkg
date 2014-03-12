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
#include <labust/control/ROSController.hpp>
#include <labust/control/PIFFController.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/simulation/DynamicsParams.hpp>
#include <labust/tools/DynamicsLoader.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/control/PIDInfoPublisher.hpp>

#include <Eigen/Dense>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>

namespace labust
{
	namespace control{
		struct SingleDOFVelocity;

		struct VelConTypes
		{
			typedef auv_msgs::BodyForceReq OutputType;
			typedef auv_msgs::BodyVelocityReq ReferenceType;
			typedef auv_msgs::NavSts StateType;
		};

		struct SingleDOFVelocity : VelConTypes
		{
			typedef boost::shared_ptr<SingleDOFVelocity> Ptr;
			enum {numDofs = 6};
			SingleDOFVelocity():
				Ts(0.1),
				idx(0),
				info_pub(ros::NodeHandle()){};

			void init(ros::NodeHandle& nh, ros::NodeHandle& ph)
			{
				ph.param("dof",idx,idx);

				if ((idx < 0) || (idx > 5))
				{
					throw std::invalid_argument("Wrong degree of freedom selected.[0,5]");
				}

				configure(nh);
			}

			void idle(const auv_msgs::BodyVelocityReq& ref,
					const auv_msgs::NavSts& state,
					const auv_msgs::BodyForceReq& track)
			{
				convertInput(ref,state,track);
				double tauOut[6]={0};
				//Calculate step
				con.extTrack = 1;
				con.desired = nuRef[idx];
				con.state = nu[idx];
				con.track = tauAch[idx];
				PIFF_step(&con, Ts);
			}

			auv_msgs::BodyForceReqPtr step(const auv_msgs::BodyVelocityReq& ref,
					const auv_msgs::NavSts& state,
					const auv_msgs::BodyForceReq& track)
			{
				convertInput(ref,state,track);
				double tau[6]={0};
				//Calculate step
				con.extTrack = 0;
				con.extWindup = windup[idx];
				con.desired = nuRef[idx];
				con.state = nu[idx];
				con.track = tauAch[idx];
				PIFF_step(&con, Ts);
				tau[idx] = con.output;
				//Setup output
				auv_msgs::BodyForceReqPtr tauOut(new auv_msgs::BodyForceReq());
				labust::tools::vectorToPoint(tau, tauOut->wrench.force);
				labust::tools::vectorToPoint(tau, tauOut->wrench.torque, 3);
				tauOut->header.stamp = ros::Time::now();

				ROS_INFO("Last command: r=%f,y=%f,w=%d,t=%f,o=%f", con.desired, con.state, con.extWindup, con.track, tau[idx]);

				return tauOut;
			}

			void info()
			{
				info_pub(con);
			}

		private:

			void convertInput(const auv_msgs::BodyVelocityReq& ref,
					const auv_msgs::NavSts& state,
					const auv_msgs::BodyForceReq& track)
			{
				labust::tools::pointToVector(ref.twist.linear, nuRef);
				labust::tools::pointToVector(ref.twist.angular,nuRef, 3);

				labust::tools::pointToVector(state.body_velocity, nu);
				labust::tools::rpyToVector(state.orientation_rate, nu, 3);

				labust::tools::pointToVector(track.wrench.force,tauAch);
				labust::tools::pointToVector(track.wrench.torque,tauAch, 3);

				labust::tools::disableAxisToVector(track.disable_axis,windup);
			}

			void configure(ros::NodeHandle& nh)
			{
				typedef Eigen::Matrix<double,6,1> vector;
				vector closedLoopFreq(vector::Ones());
				labust::tools::getMatrixParam(nh,"velocity_controller/closed_loop_freq", closedLoopFreq);
				vector autoWindup(vector::Zero());
				labust::tools::getMatrixParam(nh,"velocity_controller/auto_windup", autoWindup);
				vector outputLimit(vector::Zero());
				labust::tools::getMatrixParam(nh,"velocity_controller/output_limits", outputLimit);
				vector useBackward(vector::Zero());
				labust::tools::getMatrixParam(nh,"velocity_controller/use_recalc", useBackward);

				labust::simulation::DynamicsParams model;
				labust::tools::loadDynamicsParams(nh,model);
				vector alphas(model.Ma.diagonal());
				vector alpha_mass;
				alpha_mass<<model.m,model.m,model.m,model.Io.diagonal();
				alphas += alpha_mass;

				PT1Model pt1;
				pt1.alpha = alphas(idx);
				pt1.beta = model.Dlin(idx,idx);
				pt1.betaa = model.Dquad(idx,idx);

				nh.param("velocity_controller/Ts",this->Ts,this->Ts);
				PIDBase_init(&this->con);
				PIFF_modelTune(&this->con, &pt1, float(closedLoopFreq(idx)));
				con.autoWindup = autoWindup(idx);
				con.outputLimit = outputLimit(idx);
				con.useBackward = useBackward(idx);
			}

		private:
			int idx;
			double nuRef[numDofs], nu[numDofs], tauAch[numDofs], windup[numDofs];
			PIDBase con;
			double Ts;
			PIDInfoPublisher info_pub;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"velcon");

	labust::control::SingleDOFVelocity::Ptr velcon(
			new labust::control::SingleDOFVelocity());

	labust::control::ROSController<
		labust::control::SingleDOFVelocity> controller(velcon);
	controller.onInit(ros::NodeHandle(), ros::NodeHandle("~"));
	ros::spin();

	return 0;
}



