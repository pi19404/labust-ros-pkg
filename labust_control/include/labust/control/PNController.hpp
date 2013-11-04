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
#include <labust_control/RegisterController.h>

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
			void addToGraph(const labust_control::RegisterControllerRequest& info);
			/**
			 * Get the firing sequence for the named controller.
			 */
			void get_firing(const std::string& name);
			/**
			 * Get the firing sequence for the named controller.
			 */
			void get_firing_bfs(const std::string& name);

		private:
			/**
			 * Firing sequence recursive calculation.
			 */
			bool firing_rec(int des_place,
					std::set<int>& skip_transitions,
					std::set<int>& visited_places);
			/**
			 * Firing sequence recursive calculation.
			 */
			bool firing_rec_bfs(int des_place,
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
		};
	}
}

/* PNCONTROLLER_HPP_ */
#endif
