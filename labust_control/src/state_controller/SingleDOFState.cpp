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
#include <labust/control/HLControl.hpp>
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
			typedef auv_msgs::BodyVelocityReq OutputType;
			typedef auv_msgs::NavSts ReferenceType;
			typedef auv_msgs::NavSts StateType;
		};

		struct SingleDOFState : VelConTypes, DisableAxis
		{
			typedef boost::shared_ptr<SingleDOFState> Ptr;
			enum {z=2,roll,pitch,yaw,alt, numStates};

			SingleDOFState():
				Ts(0.1),
				idx(0),
				info_pub(ros::NodeHandle()){};

			void init(ros::NodeHandle& nh, ros::NodeHandle& ph)
			{
				ph.param("dof",idx,idx);

				if ((idx < z) || (idx > alt))
				{
					throw std::invalid_argument("Wrong degree of freedom selected.[2,6]");
				}

				windupSub = nh.subscribe<auv_msgs::BodyForceReq>("windup", 1,
						&SingleDOFState::onWindup,this);

				configure(nh);
			}

			void onWindup(const auv_msgs::BodyForceReq::ConstPtr& tauAch)
			{
				labust::tools::disableAxisToVector(tauAch->disable_axis,windup);
				windup[numStates-1] = tauAch->disable_axis.z;
			}

			void idle(const auv_msgs::NavSts& ref,
					const auv_msgs::NavSts& state,
					const auv_msgs::BodyVelocityReq& track)
			{
				convertInput(ref,state,track);
				double tauOut[6]={0};
				//Calculate step
				con.extTrack = 1;
				con.desired = stateRef[idx];
				con.state = stateHat[idx];
				con.track = nuAch[idx];

				if ((idx >= roll) && (idx <= yaw))
				{
					con.desired = labust::math::wrapRad(con.desired);
					con.state = labust::math::wrapRad(con.state);
					float errorWrap = labust::math::wrapRad(
						con.desired - con.state);
					PIFF_wffStep(&con, Ts, errorWrap, 0);
				}
				else
				{
					PIFF_ffStep(&con, Ts, 0);
				}
			}

			auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
					const auv_msgs::NavSts& state,
					const auv_msgs::BodyVelocityReq& track)
			{
				convertInput(ref,state,track);
				double nu[6]={0};
				//Calculate step
				con.extTrack = 0;
				con.extWindup = windup[idx];
				con.desired = stateRef[idx];
				con.state = stateHat[idx];
				con.track = nuHat[idx];
				if ((idx >= roll) && (idx <= yaw))
				{
					con.desired = labust::math::wrapRad(con.desired);
					con.state = labust::math::wrapRad(con.state);
					float errorWrap = labust::math::wrapRad(
						con.desired - con.state);
					PIFF_wffStep(&con, Ts, errorWrap, 0);
				}
				else
				{
					PIFF_ffStep(&con, Ts, 0);
				}
				nu[idx] = con.output;
				//Setup output
				auv_msgs::BodyVelocityReqPtr nuOut(new auv_msgs::BodyVelocityReq());
				labust::tools::vectorToPoint(nu, nuOut->twist.linear);
				labust::tools::vectorToPoint(nu, nuOut->twist.angular, 3);
				nuOut->header.stamp = ros::Time::now();
				nuOut->goal.requester = ros::this_node::getName();
				labust::tools::vectorToDisableAxis(disable_axis, nuOut->disable_axis);

				ROS_INFO("Last command: r=%f,y=%f,w=%d,t=%f,o=%f", con.desired, con.state, con.extWindup, con.track, nu[idx]);

				return nuOut;
			}

			void info()
			{
				info_pub(con);
			}

		private:

			void convertInput(const auv_msgs::NavSts& ref,
					const auv_msgs::NavSts& state,
					const auv_msgs::BodyVelocityReq& track)
			{
				labust::tools::nedToVector(ref.position, stateRef);
				labust::tools::rpyToVector(ref.orientation, stateRef, 3);
				stateRef[numStates-1] = -ref.altitude;

				labust::tools::nedToVector(state.position, stateHat);
				labust::tools::rpyToVector(state.orientation, stateHat, 3);
				stateHat[numStates-1] = -state.altitude;

				labust::tools::pointToVector(state.body_velocity, nuHat);
				labust::tools::rpyToVector(state.orientation_rate, nuHat, 3);
				stateHat[numStates-1] = -state.body_velocity.z;

				labust::tools::pointToVector(track.twist.linear,nuAch);
				labust::tools::pointToVector(track.twist.angular,nuAch, 3);
				nuAch[numStates-1] = -track.twist.linear.z;
			}

			void configure(ros::NodeHandle& nh)
			{
				typedef Eigen::Matrix<double,7,1> vector;
				vector closedLoopFreq(vector::Ones());
				labust::tools::getMatrixParam(nh,"state_controller/closed_loop_freq", closedLoopFreq);
				vector autoWindup(vector::Zero());
				labust::tools::getMatrixParam(nh,"state_controller/auto_windup", autoWindup);
				vector outputLimit(vector::Zero());
				labust::tools::getMatrixParam(nh,"state_controller/output_limits", outputLimit);
				vector useIP(vector::Zero());
				labust::tools::getMatrixParam(nh, "state_controller/use_ip",useIP);
				vector useBackward(vector::Zero());
				labust::tools::getMatrixParam(nh,"state_controller/use_recalc", useBackward);
				nh.param("state_controller/Ts",this->Ts,this->Ts);

				disable_axis[idx] = 0;

				PIDBase_init(&con);
				PIFF_tune(&con, float(closedLoopFreq(idx)));
				con.autoWindup = autoWindup(idx);
				con.outputLimit = outputLimit(idx);
				con.useBackward = useBackward(idx);
			}

		private:
			int idx;
			double stateRef[numStates], stateHat[numStates], nuHat[numStates], nuAch[numStates], windup[numStates];
			PIDBase con;
			double Ts;
			PIDInfoPublisher info_pub;
			ros::Subscriber windupSub;
		};
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"statecon");

	labust::control::SingleDOFState::Ptr velcon(
			new labust::control::SingleDOFState());

	labust::control::ROSController<
		labust::control::SingleDOFState> controller(velcon);
	controller.onInit(ros::NodeHandle(), ros::NodeHandle("~"));
	ros::spin();

	return 0;
}



