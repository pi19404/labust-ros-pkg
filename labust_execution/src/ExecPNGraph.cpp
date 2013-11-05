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
#include <labust/control/ExecPNGraph.hpp>

#include <boost/graph/graphviz.hpp>

#include <sstream>

using namespace labust::control;

ExecPNGraph::ExecPNGraph()
{
	//Add the basic vertices.
	std::string dofs[]={"X","Y","Z","K","M","N"};
	//Add DOFs to the name list
	for (int i=0; i<6;++i)	nameMap[dofs[i]].place_num = add_place(dofs[i]);
}

void ExecPNGraph::addToGraph(const navcon_msgs::RegisterControllerRequest& info)
{
	//Build from scratch
	using namespace boost;
	GraphType::vertex_descriptor curr_place = add_place(info.name),
			curr_en_t = add_transition(info.name+"_enable"),
			curr_dis_t = add_transition(info.name+"_disable");

	nameMap[info.name].place_num = curr_place;
	nameMap[info.name].enable_t = curr_en_t;
	nameMap[info.name].disable_t = curr_dis_t;

	//Add local connections.
	add_edge(curr_en_t, curr_place);
	add_edge(curr_place, curr_dis_t);

	//Add basic dependencies.
	for (int i=0; i<info.used_dofs.size(); ++i)
	{
		if (info.used_dofs[i])
		{
			add_edge(i,curr_en_t);
			add_edge(curr_dis_t, i);
		}
	}

	//Add advanced dependencies
	for (int i=0; i<info.used_cnt.size(); ++i)
	{
		GraphType::vertex_descriptor place_num =
				nameMap[info.used_cnt[i]].place_num;
		add_edge(place_num,curr_en_t);
		add_edge(curr_dis_t, place_num);
	}
}

void ExecPNGraph::getDotDesc(std::string& desc)
{
	using namespace boost;
	//Construct a label writer.
	std::ostringstream out;
	write_graphviz(out, graph,
			pn_writer(graph));
	desc = out.str();
}

void ExecPNGraph::findPath(const std::string& start, const std::string& end,
		std::list<std::string>& path){}
