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

#include <fstream>
#include <iostream>

using namespace labust::control;

ExecControl::ExecControl()
{
	this->onInit();
}

void ExecControl::onInit()
{
	ros::NodeHandle nh,ph("~");
	registerController = nh.advertiseService("register_controller",
			&ExecControl::onRegisterController, this);

	//Import main vertex names
	std::string dofs[]={"X","Y","Z","K","M","N"};
	//Add DOFs to the name list
	for (int i=X; i<=N;++i)
	{
		boost::add_vertex(VertexProperty(dofs[i],VertexProperty::p),graph);
	}
}

bool ExecControl::onRegisterController(
		labust_control::RegisterController::Request& req,
		labust_control::RegisterController::Response& resp)
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
	addToGraph(req.name);
	names.push_back(req.name);

	return true;
}

void ExecControl::add_pn_edge(int i, int j, GraphType& graph)
{
	assert(((graph[i].type == VertexProperty::p) && (graph[j].type == VertexProperty::t)) ||
			((graph[i].type == VertexProperty::t)	&& (graph[j].type == VertexProperty::p)) &&
						"You can only connect places and transitions.");
	boost::add_edge(i,j, graph);
}

template <class GraphType>
class pn_writer {
public:
  pn_writer(GraphType& graph):graph(graph){}
  template <class Vertex>
  void operator()(std::ostream &out, const Vertex& e) const
  {

    out << "[label="<< graph[e].name;
    out << ((graph[e].type == GraphType::vertex_property_type::value_type::t)?", shape=rectangle":"") << "]";
  }
private:
  GraphType& graph;
};

template <class GraphType>
pn_writer<GraphType> make_pn_writer(GraphType& graph){return pn_writer<GraphType>(graph);};

void ExecControl::addToGraph(const std::string& name)
{
	//Build from scratch
	using namespace boost;
	ControllerInfo& newcon = controllers[name];
	newcon.en_idx = add_vertex(VertexProperty(name+"_en", VertexProperty::t), graph);
	newcon.graph_idx = add_vertex(VertexProperty(name, VertexProperty::p), graph);
	newcon.dis_idx = add_vertex(VertexProperty(name+"_dis", VertexProperty::t), graph);
	//Add local connections
	add_pn_edge(newcon.en_idx, newcon.graph_idx, graph);
	add_pn_edge(newcon.graph_idx, newcon.dis_idx, graph);
	//Add basic dependencies.
	for (int i=0; i<newcon.info.used_dofs.size(); ++i)
	{
		if (newcon.info.used_dofs[i])
		{
			//ROS_INFO("Add edge %d -> %d.",i,newcon.graph_idx);
			//add_pn_edge(i,newcon.graph_idx, graph);
			add_pn_edge(i,newcon.en_idx, graph);
			add_pn_edge(newcon.dis_idx, i, graph);
		}
	}

	for (int i=0; i<newcon.info.used_cnt.size(); ++i)
	{
		const std::string& dep = newcon.info.used_cnt[i];
		ControllerMap::iterator it = controllers.end();
		if ((it = controllers.find(dep)) != controllers.end())
		{
			//ROS_INFO("Add edge %d -> %d.",it->second.graph_idx,newcon.graph_idx);
			//boost::add_edge(newcon.graph_idx, it->second.graph_idx, graph);
			add_pn_edge(it->second.graph_idx,newcon.en_idx, graph);
			add_pn_edge(newcon.dis_idx, it->second.graph_idx, graph);
		}
	}

	std::fstream file("graph.dot",std::ios::out);
	//write_graphviz(file, graph, boost::make_label_writer(boost::get(vertex_name_t(), graph)));
	write_graphviz(file, graph, make_pn_writer(graph));
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"exec_control");
	labust::control::ExecControl exec;
	ros::spin();
	return 0;
}
