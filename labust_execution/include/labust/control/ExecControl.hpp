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
#ifndef EXECCONTROL_HPP_
#define EXECCONTROL_HPP_
#include <labust/control/ExecDepGraph.hpp>
#include <labust/control/ExecPNGraph.hpp>
#include <labust/control/PNController.hpp>
#include <navcon_msgs/RegisterController.h>

#include <std_msgs/String.h>
#include <ros/ros.h>

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>
#include <map>
#include <list>

namespace labust
{
	namespace control
	{
		/**
		 * The class contains the implementation of the execution controller that manages different
		 * controller dependencies and tracks variable publishing for safety applications.
		 *
		 * \todo Add full dependency check.
		 * \todo Add unregister controller option.
		 * \todo Remove the named maps form helper classes and pass only the controller id number to classes.
		 * \todo When everythin is working, build dep and pn graphs directly from the petri-net matrices
		 */
		class ExecControl
		{
			enum {u=0,v,w,p,q,r};
			enum {X=0,Y,Z,K,M,N};
			enum {alpha=0,beta,betaa};
			enum {Kp=0,Ki,Kd,Kt};

			typedef
			struct ControllerInfo
			{
				navcon_msgs::RegisterControllerRequest info;
				int graph_idx;
				int en_idx;
				int dis_idx;

				int place_num;
				int en_t_num;
				int dis_t_num;
			} ControllerInfo;

			typedef std::map<std::string, ControllerInfo> ControllerMap;
		  //typedef boost::property<boost::vertex_name_t, std::string> VertexProperty;
			struct VertexProperty
			{
				VertexProperty():
					name("uninitialized"),
					type(0),
					marked(false){};

				VertexProperty(const std::string& name, int type):
					name(name),
					type(type),
					marked(false){};

				enum {p=0, t=1};
				std::string name;
				int type;
				bool marked;
			};

			struct RVertexProperty
			{
				Eigen::VectorXi marking;
			};

			struct REdgeProperty
			{
				int t;
				int weight;
			};

		  typedef boost::adjacency_list<boost::vecS, boost::vecS,
		  		boost::directedS, VertexProperty,
		  		boost::property < boost::edge_weight_t, int > > GraphType;

		  typedef boost::adjacency_list<boost::vecS, boost::vecS,
		  		boost::directedS, RVertexProperty,
		  		REdgeProperty > RGraphType;

		public:
			/**
			 * Main constructor
			 */
			ExecControl();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();
//			/**
//			 * Performs one iteration.
//			 */
//			void step();
//			/**
//			 * Start the controller loop.
//			 */
//			void start();
//
//		private:
//			/**
//			 * Handle incoming reference message.
//			 */
//			void handleReference(const auv_msgs::BodyVelocityReq::ConstPtr& ref);
//			/**
//			 * Handle incoming estimates message.
//			 */
//			void handleEstimates(const auv_msgs::NavSts::ConstPtr& estimate);
//			/**
//			 * Handle incoming estimates message.
//			 */
//			void handleMeasurement(const auv_msgs::NavSts::ConstPtr& meas);
//			/**
//			 * Handle incoming estimates message.
//			 */
//			void handleWindup(const auv_msgs::BodyForceReq::ConstPtr& tau);
//			/**
//			 * Handle incoming estimates message.
//			 */
//			void handleManual(const sensor_msgs::Joy::ConstPtr& joy);
			/**
			 * Handle controller registration.
			 */
			bool onRegisterController(navcon_msgs::RegisterController::Request& req,
					navcon_msgs::RegisterController::Response& resp);
			/**
			 * Activate controller tester.
			 */
			void onActivateController(const std_msgs::String::ConstPtr& name);
			/**
			 * Builds the petri net graph.
			 */
			void addToGraph(const std::string& name = "");
			/**
			 * Helper method for ensuring PetriNet connections.
			 * It is always i->j
			 */
			void add_pn_edge(int i, int j, GraphType& graph);
			/**
			 * The helper function to find graph transitions.
			 */
			void find_path();
			/**
			 * Calculates the reachability graph.
			 */
			void reachability();
			/**
			 * Calculates the firing sequence to enable a desired controller.
			 */
			void get_firing(const std::string& name);
			void get_firing2(const std::string& name);
			/**
			 * Add to PN matrix.
			 */
			void addToMatrix(const std::string& name);
			/**
			 * Helper recursion function
			 */
			bool firing_rec(int des_place, std::vector<int>& skip_transitions, std::vector<int>& visited_places);
			bool firing_rec2(int des_place, std::vector<int>& skip_transitions, std::vector<int>& visited_places);
//			/**
//			 * Handle the enable control request.
//			 */
//			bool handleEnableControl(labust_uvapp::EnableControl::Request& req,
//					labust_uvapp::EnableControl::Response& resp);
//			/**
//			 * Dynamic reconfigure callback.
//			 */
//			void dynrec_cb(labust_uvapp::VelConConfig& config, uint32_t level);
//			/**
//			 * The safety test.
//			 */
//			void safetyTest();
//			/**
//			 * Update the dynamic reconfiguration settings.
//			 */
//			void updateDynRecConfig();
//			/**
//			 * Perform the identification step.
//			 */
//			double doIdentification(int i);
//			/**
//			 * The ROS node handles.
//			 */
//			ros::NodeHandle nh,ph;
//			/**
//			 * Last message times.
//			 */
//			ros::Time lastRef, lastMan, lastEst, lastMeas;
//			/**
//			 * Timeout
//			 */
//			double timeout;
//
//			/**
//			 * Initialize the controller parameters etc.
//			 */
//			void initialize_controller();
//			/**
//			 * The velocity controllers.
//			 */
//			PIDController controller[r+1];
//			/**
//			 * The identification controllers.
//			 */
//			boost::shared_ptr<SOIdentification> ident[r+1];
//			/**
//			 * The mesurement needed for identification.
//			 */
//			double measurement[r+1];
//			/**
//			 * Joystick message.
//			 */
//			float tauManual[N+1];
//			/**
//			 * Joystick scaling.
//			 */
//			double joy_scale, Ts;
//			/**
//			 * Enable/disable controllers, external windup flag.
//			 */
//			boost::array<int32_t,r+1> axis_control;
//			/**
//			 * Timeout management.
//			 */
//			bool suspend_axis[r+1];
//			/**
//			 * Dynamic reconfigure server.
//			 */
//			boost::recursive_mutex serverMux;
//
//			/**
//			 * The publisher of the TAU message.
//			 */
			ros::Publisher depGraphPub, pnGraphPub;
//			/**
//			 * The subscribed topics.
//			 */
			ros::Subscriber activateController;
			/**
			 * High level controller service.
			 */
			ros::ServiceServer registerController;
//			/**
//			 * The dynamic reconfigure parameters.
//			 */
//			labust_uvapp::VelConConfig config;
//			/**
//			 * The dynamic reconfigure server.
//			 */
//		  dynamic_reconfigure::Server<labust_uvapp::VelConConfig> server;
//		  /**
//		   * Variable access helper mass.
//		   */
//		  const static std::string dofName[r+1];

			/**
			 * The controller map.
			 */
			ControllerMap controllers;
			/**
			 * The controller names list.
			 */
			std::vector<std::string> names;
			/**
			 * Active dofs.
			 */
			std::string active_dof[6];
			/**
			 * Active controllers.
			 */
			std::list<std::string> active_cnt;
			/**
			 * The controller dependency graph.
			 */
			GraphType graph;
			/**
			 * The reachability graph
			 */
			RGraphType rgraph;
			/**
			 * The transition number.
			 */
			int tnum;
			/**
			 * The place num.
			 */
			int pnum;
			/**
			 * PN matrices.
			 */
			Eigen::MatrixXi Dm,Dp;
			/*
			 * The current marking.
			 */
			Eigen::VectorXi marking;
			/**
			 * Map placenum to name.
			 */
			std::map<int, std::string> pname;
			/**
			 * The last firing sequence.
			 */
			std::vector<int> firing_seq;
			/**
			 * The dependency graph tool.
			 */
			ExecDepGraph depGraph;
			/**
			 * The petri-net graph tool.
			 */
			ExecPNGraph pnGraph;
			/**
			 * The Petri-Net controller.
			 */
			PNController pnCon;
		};
	}
}

/* EXECCONTROL_HPP_ */
#endif
