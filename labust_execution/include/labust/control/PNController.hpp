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
#ifndef PNCONTROLLER_HPP_
#define PNCONTROLLER_HPP_
#include <navcon_msgs/RegisterController.h>

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <Eigen/Dense>

#include <string>
#include <map>
#include <set>

namespace labust
{
	namespace control
	{
		/**
		 * The class contains implementation of Petri-Net builder and controller.
		 * \todo Add multiple desired places setup
		 * \todo Add place turn-off option
		 * \todo Add additional state enabled place in parallel with the control place.
		 * \todo Add weighted transitions for multi DOF controllers ?
		 * \todo Add simulation step for activation to detect wrong firing sequences.
		 * \todo Extract reachability graph building and ploting
		 * \todo Extract debugging information and structure it in a debug class
		 * \todo Possible optimizations: Incremental graph building
		 * \todo Possible optimizations: BFS only on a subset of the graph ?
		 * \todo Possible optimizations: Spin-off BFS for the new state in a thread to have it ready in advance
		 * \todo Recommended optimizations: Reachability calculation memory/time/copying/std::find usage
		 * \todo Add detection of faulty controller registrations or setups.
		 *       -analyze if they have indirect dependencies to DOFs
		 * \todo Possible optimizations: Find sequences that can fire simultaneously
		 */
		class PNController
		{
			struct PlaceInfo
			{
				PlaceInfo():
					place_num(-1),
					enable_t(-1),
					disable_t(-1){};

				int	place_num,
				enable_t,
				disable_t;
			};

		public:
			/**
			 * Main constructor
			 */
			PNController();

			/**
			 * Add controller to the petri-net.
			 */
			void addToGraph(const navcon_msgs::RegisterControllerRequest& info);
			/**
			 * Get the firing sequence for the named controller.
			 */
			void get_firing(const std::string& name);
			/**
			 * Get the firing sequence for the named controller.
			 */
			void get_firing_r(const std::string& name);
			/**
			 * Calculates the reachability graph.
			 */
			void reachability();
			/**
			 * Get the reachability graph DOT description.
			 */
			void getDotDesc(std::string& desc);

		private:
			/**
			 * Firing sequence recursive calculation.
			 */
			bool firing_rec(int des_place,
					std::set<int>& skip_transitions,
					std::set<int>& visited_places);

			/**
			 * The place and transition number.
			 */
			int pnum, tnum;
			/**
			 * PN matrices.
			 */
			Eigen::MatrixXi Dm,Dp,I;
			/*
			 * The current marking.
			 */
			Eigen::VectorXi marking;
			/**
			 * Helper maps for debugging.
			 */
			std::map<int, std::string> placeMap, transitionMap;
			/**
			 * Helper name to p/t mapping.
			 */
			std::map<std::string, PlaceInfo> nameMap;
			/**
			 * The last firing sequence.
			 */
			std::vector<int> firing_seq;

			//Reachability graph stuff
			struct VertexProperty
			{
				Eigen::VectorXi marking;
			};

			struct EdgeProperty
			{
			  typedef boost::edge_property_tag kind;
				EdgeProperty():
					t_num(-1),
					weight(1){};
				EdgeProperty(int t_num):
					t_num(t_num),
					weight(1){};

				int t_num;
				int weight;
			};

			typedef boost::adjacency_list<boost::vecS, boost::vecS,
		  		boost::directedS, VertexProperty,
		  		boost::property<boost::edge_name_t, int> > GraphType;

			struct pn_writer
			{
				pn_writer(GraphType& graph):graph(graph){}
				template <class Vertex>
				void operator()(std::ostream &out, const Vertex& e) const
				{
					out << "[label= \"(";
					for(int i=0; i<graph[e].marking.size();++i)
					{
						out<<graph[e].marking[i]<<",";
					}
					out<< ")\"]";
				}
				GraphType& graph;
			};

			template<class PropertyMap, class NameMap>
			struct edge_writer {
			  edge_writer(PropertyMap edge_map, NameMap map):
			  	edge_map(edge_map),
			  	map(map){};
			  template <class Edge>
			  void operator()(std::ostream &out, const Edge& e) const
			  {
			    out << "[label=\"" << map.at(edge_map[e]) << "\"]";
			  }
			  PropertyMap edge_map;
			  NameMap map;
			};

			template<class PropertyMap, class NameMap>
			edge_writer<PropertyMap, NameMap> make_edge_writer(PropertyMap pmap, NameMap map)
			{
				return edge_writer<PropertyMap, NameMap>(pmap,map);
			}

			GraphType rgraph;

			std::vector<Eigen::VectorXi> all_markings;
			std::vector<GraphType::vertex_descriptor> all_idx;
		};
	}
}

/* PNCONTROLLER_HPP_ */
#endif
