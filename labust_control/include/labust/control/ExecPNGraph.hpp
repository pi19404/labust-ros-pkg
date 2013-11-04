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
#ifndef EXECPNGRAPH_HPP_
#define EXECPNGRAPH_HPP_
#include <labust_control/RegisterController.h>

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
		 * The class contains implementation of the mission execution Petri Net graph.
		 */
		class ExecPNGraph
		{
			struct VertexProperty
			{
				enum {p=0, t=1};

				VertexProperty():
					name("uninitialized"),
					type(0),
					marked(false){};

				VertexProperty(const std::string& name, int type):
					name(name),
					type(type),
					marked(false){};

				std::string name;
				int type;
				bool marked;
			};

			typedef boost::property < boost::edge_weight_t, int > EdgeProperty;
			typedef boost::adjacency_list<boost::vecS, boost::vecS,
		  		boost::directedS, VertexProperty,
		  		EdgeProperty > GraphType;

			struct PlaceInfo
			{
				PlaceInfo():
					place_num(-1),
					enable_t(-1),
					disable_t(-1){};

				GraphType::vertex_descriptor
				place_num,
				enable_t,
				disable_t;
			};

		public:
			/**
			 * Main constructor
			 */
			ExecPNGraph();

			/**
			 * Add controller to the dependency graph.
			 */
			void addToGraph(const labust_control::RegisterControllerRequest& info);
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
			 * Adds a place to the petri-net graph.
			 */
			inline GraphType::vertex_descriptor
			add_place(const std::string& name)
			{
				return boost::add_vertex(
						VertexProperty(name,VertexProperty::p),graph);
			}
			/**
			 * Adds a transition to the petri-net graph.
			 */
			inline GraphType::vertex_descriptor
			add_transition(const std::string& name)
			{
				return boost::add_vertex(
						VertexProperty(name,VertexProperty::t),graph);
			}
			/**
			 * Adds a checked edge to the petri-net graph.
			 */
			inline std::pair<GraphType::edge_descriptor, bool>
			add_edge(GraphType::vertex_descriptor from,
					GraphType::vertex_descriptor to, int weight = 1)
			{
				assert(((graph[from].type == VertexProperty::p) &&
						(graph[to].type == VertexProperty::t)) ||
						((graph[from].type == VertexProperty::t)
						&& (graph[to].type == VertexProperty::p)) &&
						"You can only connect places and transitions.");
				return boost::add_edge(from,to, weight, graph);
			}
			/**
			 * The graphviz writer class for the PN graph.
			 */
			struct pn_writer {
				pn_writer(GraphType& graph):graph(graph){}
				template <class Vertex>
				void operator()(std::ostream &out, const Vertex& e) const
				{

					out << "[label="<< graph[e].name;
					out << ((graph[e].type == GraphType::vertex_property_type::value_type::t)?", shape=rectangle":"") << "]";
				}
				GraphType& graph;
			};
			/**
			 * The controller dependency graph.
			 */
			GraphType graph;
			/**
			 * The name to vertice map.
			 */
			std::map<std::string,
				PlaceInfo> nameMap;
		};
	}
}

/* EXECPNGRAPH_HPP_ */
#endif
