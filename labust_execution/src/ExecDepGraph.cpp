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
#include <labust/control/ExecDepGraph.hpp>

#include <boost/graph/graphviz.hpp>

#include <sstream>

using namespace labust::control;

ExecDepGraph::ExecDepGraph()
{
	//Add the basic vertices.
	std::string dofs[]={"X","Y","Z","K","M","N"};
	//Add DOFs to the name list
	for (int i=0; i<6;++i) nameMap[dofs[i]] = boost::add_vertex(dofs[i], graph);
}

void ExecDepGraph::addToGraph(const navcon_msgs::RegisterControllerRequest& info)
{
	//Build from scratch
	using namespace boost;
	GraphType::vertex_descriptor curr_vert = boost::add_vertex(info.name,graph);
	nameMap[info.name] = curr_vert;
	//Add basic dependencies.
	for (int i=0; i<info.used_dofs.size(); ++i)
		if (info.used_dofs[i]) boost::add_edge(curr_vert,i, 1, graph);

	//Add advanced dependencies
	for (int i=0; i<info.used_cnt.size(); ++i)
		boost::add_edge(curr_vert,nameMap[info.used_cnt[i]], 1, graph);
}

void ExecDepGraph::getDotDesc(std::string& desc)
{
	using namespace boost;
	//Construct a label writer.
	std::ostringstream out;
	write_graphviz(out, graph,
			make_label_writer(get(vertex_name_t(), graph)));
	desc = out.str();
}

void ExecDepGraph::findPath(const std::string& start, const std::string& end,
		std::list<std::string>& path){}
