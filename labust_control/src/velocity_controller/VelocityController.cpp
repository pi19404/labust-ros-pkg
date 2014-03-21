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
#include <labust/control/PIFFController.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/simulation/DynamicsParams.hpp>
#include <labust/tools/DynamicsLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <Eigen/Dense>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/NavSts.h>
#include <ros/ros.h>

namespace labust
{
	namespace control{
		///The heading controller

		struct VelConTypes
		{
			typedef auv_msgs::BodyForceReq OutputType;
			typedef auv_msgs::BodyVelocityReq ReferenceType;
			typedef auv_msgs::NavSts StateType;
		};


		struct SinglePID
		{
			SinglePID(){};

			float step(float Ts, float ref, float state, float track,
					bool extWindup = false,
					bool extTrack = false)
			{
				con.extTrack = extTrack;
				con.extWindup = extWindup;
				con.desired = ref;
				con.state = state;
				con.track = track;
				PIFF_step(&con, Ts);
				return con.output;
			}

			void tune(float w, double outputLimit, bool autoWindup, const PT1Model& pt1)
			{
				this->tune(w, outputLimit, autoWindup);
				PIFF_modelTune(&this->con, &pt1, w);
			}

			void tune(float w, double outputLimit, bool autoWindup)
			{
				PIDBase_init(&this->con);
				con.autoWindup = autoWindup;
				con.outputLimit = outputLimit;
				PIFF_tune(&con, w);
			}

			PIDBase con;
		};

		struct VelocityController
		{
			enum {numDofs = 6};
			enum {disabled = 0, manual = 1, external = 2};
			typedef boost::shared_ptr<VelocityController> Ptr;

			VelocityController():Ts(0.1){};

			void init(ros::NodeHandle& nh, ros::NodeHandle& ph)
			{
				//Read the required dofs.
				configure(nh);
			}

			void setState();

			auv_msgs::BodyForceReqPtr step(const auv_msgs::BodyVelocityReq& ref,
					const auv_msgs::NavSts& state, const auv_msgs::BodyForceReq& track)
			{
				double nuRef[6], nu[6], tauAch[6], tau[6]={0};
				labust::tools::pointToVector(ref.twist.linear, nuRef);
				labust::tools::pointToVector(ref.twist.angular,nuRef, 3);

				labust::tools::pointToVector(state.body_velocity, nu);
				labust::tools::rpyToVector(state.orientation_rate, nu, 3);

				labust::tools::pointToVector(track.wrench.force, tauAch);
				labust::tools::pointToVector(track.wrench.torque, tauAch, 3);

				bool anyActive = false;
				for (int i=0; i<numDofs; ++i)
				{
					switch (this->state[i])
					{
					//Track
					case disabled:
						controllers[i].internalState = tauAch[i];
						break;
					case external:
						controllers[i].desired = nuRef[i];
						controllers[i].state = nu[i];
						PIFF_step(&controllers[i], Ts);
						tau[i] = controllers[i].output;
						anyActive = true;
						break;
					default: break;
					}

				}

				auv_msgs::BodyForceReqPtr tauOut(new auv_msgs::BodyForceReq());
				labust::tools::vectorToPoint(tau,tauOut->wrench.force);
				labust::tools::vectorToPoint(tau, tauOut->wrench.torque, 3);

				return tauOut;
			}

		private:

			void configure(ros::NodeHandle& nh)
			{
				typedef Eigen::Matrix<double,6,1> Vector6d;
				using labust::simulation::vector;
				vector closedLoopFreq(vector::Ones());
				labust::tools::getMatrixParam(nh,"velocity_controller/closed_loop_freq", closedLoopFreq);
				vector autoTracking(vector::Zero());
				labust::tools::getMatrixParam(nh,"velocity_controller/auto_tracking", autoTracking);
				nh.param("velocity_controller/Ts",Ts,Ts);

				labust::simulation::DynamicsParams model;
				labust::tools::loadDynamicsParams(nh,model);
				vector alphas(model.Ma.diagonal());
				vector alpha_mass;
				alpha_mass<<model.m,model.m,model.m,model.Io.diagonal();
				alphas += alpha_mass;

				for (int i=0; i<numDofs; ++i)
				{
					PT1Model pt1;
					pt1.alpha = alphas(i);
					pt1.beta = model.Dlin(i,i);
					pt1.betaa = model.Dquad(i,i);

					/*controllers[numDofs].tune(float(closedLoopFreq(i), )

					PIDBase_init(&controllers[i]);
					PIFF_modelTune(&controllers[i], &pt1, );
					controllers[i].autoWindup = autoTracking(i);*/
				}
			}

		private:
			SinglePID controllers[numDofs];
			int state[numDofs];
			double Ts;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"velcon");
	ros::spin();

	return 0;
}



