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
 *  Created: 30.10.2013.
 *********************************************************************/
#include <labust/control/ExecControl.hpp>

#include <boost/graph/graphviz.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <std_msgs/String.h>

#include <fstream>
#include <iostream>

using namespace labust::control;

ExecControl::ExecControl():
		tnum(0),
		pnum(6),
		marking(pnum)
{
	this->onInit();
}

void ExecControl::onInit()
{
	ros::NodeHandle nh,ph("~");
	depGraphPub = nh.advertise<std_msgs::String>("dependency_graph",1);
	pnGraphPub = nh.advertise<std_msgs::String>("petri_net_graph",1);

	registerController = nh.advertiseService("register_controller",
			&ExecControl::onRegisterController, this);

	activateController = nh.subscribe<std_msgs::String>("activate_cnt",1,
			&ExecControl::onActivateController, this);

	//Import main vertex names
	std::string dofs[]={"X","Y","Z","K","M","N"};
	//Add DOFs to the name list
	for (int i=X; i<=N;++i)
	{
		boost::add_vertex(VertexProperty(dofs[i],VertexProperty::p),graph);
		marking(i) = 1;
		pname[i] = dofs[i];
	}
}

bool ExecControl::onRegisterController(
		navcon_msgs::RegisterController::Request& req,
		navcon_msgs::RegisterController::Response& resp)
{
	//Check if controller with same name exists.
	if (controllers.find(req.name) != controllers.end())
	{
		ROS_ERROR("The %s controller is already registered.", req.name.c_str());
		resp.accepted = false;
		return true;
	}

	//Check if other dependencies are satisfied
	for (int i=0; i<req.used_cnt.size(); ++i)
	{
		if (controllers.find(req.used_cnt[i]) == controllers.end())
		{
			ROS_ERROR("The controller %s is missing dependency %s.",
					req.name.c_str(),
					req.used_cnt[i].c_str());
			resp.unmet_cnt.push_back(req.used_cnt[i]);
		}
	}

	if (resp.unmet_cnt.size())
	{
		resp.accepted = false;
		return true;
	}

	//Add the controller
	ROS_INFO("Adding controller %s.",req.name.c_str());
	resp.accepted = true;
	controllers[req.name].info = req;
	depGraph.addToGraph(req);
	//pnGraph.addToGraph(req);
	//pnCon.addToPNGraph(req);

	ros::Time now = ros::Time::now();
	pnCon.addToGraph(req);
	double addgraph_dT = (ros::Time::now() - now).toSec();
	now = ros::Time::now();
	pnCon.addToPNGraph(req);
	double addpngraph_dT = (ros::Time::now() - now).toSec();
	//pnCon.reachability();
	double classic_dT = (ros::Time::now() - now).toSec();
	now = ros::Time::now();
	//pnCon.addToRGraph2(req.name);
	pnCon.addToRGraph(req.name);
	double incremental_dT = (ros::Time::now() - now).toSec();

	//pnCon.addToRGraph();
	//addToMatrix(req.name);
	names.push_back(req.name);

	std_msgs::String out;
	std::fstream dep_file("dep_graph.dot",std::ios::out);
	std::fstream pn_file("pn_graph.dot",std::ios::out);
	std::fstream r_file("r_graph.dot",std::ios::out);
	std::string temp;
	depGraph.getDotDesc(temp);
	dep_file<<temp;
	out.data = temp;
	depGraphPub.publish(out);
	pnCon.getDotDesc2(temp);
	pn_file<<temp;
	out.data = temp;
	pnGraphPub.publish(out);
	pnCon.getDotDesc(temp);
	r_file<<temp;

	std::ofstream prof_file("profile.csv", std::ios::app);
	prof_file<<addgraph_dT<<","<<addpngraph_dT<<","<<classic_dT<<","<<incremental_dT<<std::endl;

	return true;
}

void ExecControl::addToMatrix(const std::string& name)
{
	//Build from scratch
	using namespace boost;
	ControllerInfo& newcon = controllers[name];
	newcon.place_num = pnum++;
	pname[newcon.place_num] = name;
	marking.conservativeResize(pnum);
	marking(newcon.place_num) = 0;
	newcon.en_t_num = tnum++;
	newcon.dis_t_num = tnum++;
	std::cout<<"Resize matrix 2."<<std::endl;
	Dm.conservativeResize(pnum, tnum);
	Dp.conservativeResize(pnum, tnum);
	Dm.row(newcon.place_num) = Eigen::VectorXi::Zero(tnum);
	Dp.row(newcon.place_num) = Eigen::VectorXi::Zero(tnum);
	Dm.col(newcon.en_t_num)= Eigen::VectorXi::Zero(pnum);
	Dp.col(newcon.en_t_num)= Eigen::VectorXi::Zero(pnum);
	Dm.col(newcon.dis_t_num)= Eigen::VectorXi::Zero(pnum);
	Dp.col(newcon.dis_t_num)= Eigen::VectorXi::Zero(pnum);

	//Add local connection
	Dm(newcon.place_num, newcon.dis_t_num) = 1;
	Dp(newcon.place_num, newcon.en_t_num) = 1;
	//Add basic dependencies.
	for (int i=0; i<newcon.info.used_dofs.size(); ++i)
	{
		if (newcon.info.used_dofs[i])
		{
			Dm(i, newcon.en_t_num) = 1;
			Dp(i, newcon.dis_t_num) = 1;
		}
	}

	//Add basic dependencies.
	for (int i=0; i<newcon.info.used_cnt.size(); ++i)
	{
		const std::string& dep = newcon.info.used_cnt[i];
		ControllerMap::iterator it = controllers.end();
		if ((it = controllers.find(dep)) != controllers.end())
		{
			//ROS_INFO("Add edge %d -> %d.",it->second.graph_idx,newcon.graph_idx);
			//boost::add_edge(newcon.graph_idx, it->second.graph_idx, graph);
			Dm(it->second.place_num, newcon.en_t_num) = 1;
			Dp(it->second.place_num, newcon.dis_t_num) = 1;
		}
	}

	std::cout<<"Current marking:"<<marking<<std::endl;
	std::cout<<"Current Dm matrix:"<<Dm<<std::endl;
	std::cout<<"Current Dp matrix:"<<Dp<<std::endl;
	std::cout<<"Current I matrix:"<<Dp-Dm<<std::endl;

	reachability();
}

void ExecControl::onActivateController(const std_msgs::String::ConstPtr& name)
{
	if (controllers.find(name->data) != controllers.end())
	{
		//firing_seq.clear();
		//this->get_firing2(name->data);
		//pnCon.get_firing(name->data);
		//pnCon.get_firing_r(name->data);
		pnCon.get_firing_pn(name->data);
	}
	else
	{
		//A testing example, we can try setting a PN place directly.
		std::string dofs[]={"X","Y","Z","K","M","N"};
		//Add DOFs to the name list
		for (int i=X; i<=N;++i)
		{
			if (name->data == dofs[i])
			{
				//pnCon.get_firing_r(name->data);
				pnCon.get_firing_pn(name->data);
				break;
			}
		}
	}
}

bool ExecControl::firing_rec2(int des_place, std::vector<int>& skip_transitions, std::vector<int>& visited_places)
{
	visited_places.push_back(des_place);

	//Find all possible transition activators for this place
	Eigen::VectorXi transitions = Dp.row(des_place);
	std::vector<int> activators;
	for (int i=0; i<transitions.size(); ++i)
	{
		if (transitions(i))
		{
			bool skipped = false;
			//Filter skipped transitions
			for (int j=0; j< skip_transitions.size(); ++j)
			{
				if ((skipped = (skip_transitions[j] == i))) break;
			}

			if (!skipped) activators.push_back(i);
		}
	}

	std::cout<<"Place "<<pname[des_place]<<" depends on transitions:";
	for (int i=0; i<activators.size(); ++i)
	{
		std::cout<<activators[i]<<", ";
	}
	std::cout<<std::endl;

	//If no activators this is a dead-end.
	if (activators.empty())
	{
		std::cout<<"Dead-end."<<std::endl;
		return false;
	}

	//For each activator find places
	bool add_seq = false;
	for (int i=0; i<activators.size(); ++i)
	{
		Eigen::VectorXi t_en = Dm.col(activators[i]);
		std::vector<int> places;
		for (int j=0; j<t_en.size(); ++j)
		{
			if (t_en(j))
			{

				bool skipped = false;
				//Filter skipped transitions
				for (int k=0; k< visited_places.size(); ++k)
				{
					if ((skipped = (visited_places[k] == j))) break;
				}

				if (!skipped) places.push_back(j);
			}
		}

		std::cout<<"Transition "<<activators[i]<<" depends on places:";
		for (int j=0; j<places.size(); ++j)
		{
			std::cout<<pname[places[j]]<<", ";
		}
		std::cout<<std::endl;

		bool add_cur_seq=true;
		for (int j=0; j<places.size(); ++j)
		{
			//If place is active add
			if (!marking(places[j]))
			{
				if (!firing_rec2(places[j], skip_transitions, visited_places))
				{
					add_cur_seq = false;
					break;
				}
			}
		}

		if (add_cur_seq && !places.empty())
		{
			bool add = true;
			for (int j=0; j<firing_seq.size(); ++j)
			{
				if (firing_seq[j] == activators[i])
				{
					add = false;
					break;
				}
			}
			if (add)
			{
				std::cout<<"Adding to firing sequence:"<<activators[i]<<std::endl;
				firing_seq.push_back(activators[i]);
				add_seq = true;
				if (activators[i]%2 == 0)
				{
					skip_transitions.push_back(activators[i]+1);
				}
				else
				{
					skip_transitions.push_back(activators[i]-1);
				}
			}
		}
	}
	return add_seq;
}

bool ExecControl::firing_rec(int des_place, std::vector<int>& skip_transitions, std::vector<int>& visited_places){}

void ExecControl::get_firing(const std::string& name)
{}

void ExecControl::get_firing2(const std::string& name)
{
	std::vector<int> skip_transitions;
	std::vector<int> visited_places;

	//Desired place to activate
	int des_place = controllers[name].place_num;
	firing_rec2(des_place, skip_transitions, visited_places);

	std::cout<<"The firing sequence is:";
	for (int i=0; i<firing_seq.size(); ++i)
	{
		std::cout<<firing_seq[i]<<" ,";
	}
	std::cout<<std::endl;

	//Do the firing
	//Add testing "simulation" of the firing before applying.
	//Add enable/disable service calls for these.
	for(int i=0; i<firing_seq.size(); ++i)
	{
		Eigen::VectorXi fire = Eigen::VectorXi::Zero(Dm.cols());
		fire(firing_seq[i]) = 1;
		marking = marking + (Dp-Dm)*fire;
		std::cout<<"Transition "<<firing_seq[i]<<" fired, new marking:"<<marking<<std::endl;
		if (marking(des_place)) break;
	}
}


void ExecControl::reachability()
{
	rgraph.clear();
	Eigen::VectorXi trans = marking.transpose()*Dm;

	std::cout<<"Transition matrix:"<<trans<<std::endl;

	int last_m = boost::add_vertex(rgraph);
	rgraph[last_m].marking = marking;

	Eigen::VectorXi fire = Eigen::VectorXi::Zero(trans.size());
	int tnum = -1;
	for (int i=0; i<trans.size(); ++i)
	{
		if (trans(i)>0)
		{
			std::cout<<"Transition "<<i<<" can fire."<<std::endl;
			fire(i) = 1;
			tnum = i;
			break;
		}
	}

	//marking = marking + (Dp-Dm)*fire;

	int new_m = boost::add_vertex(rgraph);
	rgraph[new_m].marking = marking;
	REdgeProperty prop;
	prop.t = tnum;
	prop.weight = 1;
	//Enable edge
	boost::add_edge(last_m, new_m, prop, rgraph);
	prop.t+=1;
	//Disable edge added so we can skip it in next iteration.
	boost::add_edge(new_m, last_m, prop, rgraph);


	std::cout<<"Transition fired, new marking:"<<marking<<std::endl;
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"exec_control");
	labust::control::ExecControl exec;
	ros::spin();
	return 0;
}
