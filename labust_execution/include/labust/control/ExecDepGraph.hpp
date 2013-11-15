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
#ifndef EXECDEPGRAPH_HPP_
#define EXECDEPGRAPH_HPP_
#include <navcon_msgs/RegisterController.h>

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <list>
#include <string>
#include <map>
#include <iosfwd>

namespace labust
{
	namespace control
	{
		/**
		 * The class contains implementation of the mission execution dependency graph.
		 */
		class ExecDepGraph
		{
			typedef boost::property<boost::vertex_name_t, std::string> VertexProperty;
			typedef boost::property < boost::edge_weight_t, int > EdgeProperty;
			typedef boost::adjacency_list<boost::vecS, boost::vecS,
		  		boost::directedS, VertexProperty,
		  		EdgeProperty > GraphType;
		public:
			/**
			 * Main constructor
			 */
			ExecDepGraph();

			/**
			 * Add controller to the dependency graph.
			 */
			void addToGraph(const navcon_msgs::RegisterControllerRequest& info);
			/**
			 * Find the path between to controllers.
			 */
			void findPath(const std::string& start, const std::string& end,
					std::list<std::string>& path);
			/**
			 * Get the dependency graph DOT description.
			 */
			void getDotDesc(std::string& desc);

		private:
			/**
			 * The controller dependency graph.
			 */
			GraphType graph;
			/**
			 * The name to vertice map.
			 */
			std::map<std::string,
				GraphType::vertex_descriptor> nameMap;
		};
	}
}

/* EXECDEPGRAPH_HPP_ */
#endif
