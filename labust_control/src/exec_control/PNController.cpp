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
#include <labust/control/PNController.hpp>

using namespace labust::control;

PNController::PNController():
					pnum(6),
					tnum(0),
					marking(pnum)
{
	//Add the basic vertices.
	std::string dofs[]={"X","Y","Z","K","M","N"};
	//Initial marking
	for (int i=0; i<6;++i)
	{
		placeMap[i]=dofs[i];
		nameMap[dofs[i]].place_num = i;
		marking(i) = 1;
	}
}

void PNController::addToGraph(const labust_control::RegisterControllerRequest& info)
{
	PlaceInfo& newcon=nameMap[info.name];
	newcon.place_num = pnum++;

	marking.conservativeResize(pnum);
	marking(newcon.place_num) = 0;
	newcon.enable_t = tnum++;
	newcon.disable_t = tnum++;

	//Debug info
	placeMap[newcon.place_num] = info.name;
	transitionMap[newcon.enable_t] = info.name+"_enable";
	transitionMap[newcon.disable_t] = info.name+"_disable";

	Dm.conservativeResize(pnum, tnum);
	Dp.conservativeResize(pnum, tnum);
	Dm.row(newcon.place_num) = Eigen::VectorXi::Zero(tnum);
	Dp.row(newcon.place_num) = Eigen::VectorXi::Zero(tnum);
	Dm.col(newcon.enable_t)= Eigen::VectorXi::Zero(pnum);
	Dp.col(newcon.enable_t)= Eigen::VectorXi::Zero(pnum);
	Dm.col(newcon.disable_t)= Eigen::VectorXi::Zero(pnum);
	Dp.col(newcon.disable_t)= Eigen::VectorXi::Zero(pnum);

	//Add local connection
	Dm(newcon.place_num, newcon.disable_t) = 1;
	Dp(newcon.place_num, newcon.enable_t) = 1;
	//Add basic dependencies.
	for (int i=0; i<info.used_dofs.size(); ++i)
	{
		if (info.used_dofs[i])
		{
			Dm(i, newcon.enable_t) = 1;
			Dp(i, newcon.disable_t) = 1;
		}
	}

	//Add advanced dependencies.
	for (int i=0; i<info.used_cnt.size(); ++i)
	{
		int place_num = nameMap[info.used_cnt[i]].place_num;
		Dm(place_num, newcon.enable_t) = 1;
		Dp(place_num, newcon.disable_t) = 1;
	}

	I=Dp-Dm;
}

bool PNController::firing_rec(int des_place,
		std::set<int>& skip_transitions,
		std::set<int>& visited_places)
{
	//Add place to visited places.
	visited_places.insert(des_place);

	//Find all possible transition activators for this place
	Eigen::VectorXi transitions = Dp.row(des_place);
	std::vector<int> activators;
	for (int i=0; i<transitions.size(); ++i)
	{
		//If connected (!=0) AND not in skipped transitions.
		if ((transitions(i) &&
				(skip_transitions.find(i) == skip_transitions.end())))
		{
			activators.push_back(i);
		}
	}

	//If no activators this is a dead-end.
	if (activators.empty())
	{
		std::cout<<"Place "<<placeMap[des_place]<<" is a dead-end."<<std::endl;
		return false;
	}

	//Output debugging information for the curret
	std::cout<<"Place "<<placeMap[des_place]<<" depends on transitions:";
	for (int i=0; i<activators.size(); ++i) std::cout<<transitionMap[activators[i]]<<", ";
	std::cout<<std::endl;

	//For each activator test if there exists a posibility to trigger it.
	bool trigerable = false;
	for (int i=0; i<activators.size(); ++i)
	{
		Eigen::VectorXi t_en = Dm.col(activators[i]);
		std::vector<int> places;
		for (int j=0; j<t_en.size(); ++j)
		{
			//If connected (!=0) AND not in visited places.
			if ((t_en(j) &&
					(visited_places.find(j) == visited_places.end())))
			{
				places.push_back(j);
			}
		}

		if (places.size() == 0)
		{
			std::cout<<"Activator candidate transition "<<transitionMap[activators[i]];
			std::cout<<" is a dead-end."<<std::endl;
			continue;
		}

		//Debugging information for the activator places.
		std::cout<<"Activator candidate transition "<<transitionMap[activators[i]];
		std::cout<<" depends on:";
		for (int j=0; j<places.size(); ++j) std::cout<<placeMap[places[j]]<<", ";
		std::cout<<std::endl;

		bool add_cur_seq=true;
		//Places that are already marked add to visited places to avoid backlash.
		for (int j=0; j<places.size(); ++j)
			if (marking(places[j])) visited_places.insert(places[j]);

		for (int j=0; j<places.size(); ++j)
		{
			//Recurse to search a lever higher for unmarked places
			if (!marking(places[j]))
			{
				if (!firing_rec(places[j], skip_transitions, visited_places))
				{
					add_cur_seq = false;
					break;
				}
			}
		}

		if (add_cur_seq)
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

			//Before adding check if still not skipped
			if (add)
			{
				std::cout<<"Adding to firing sequence:"<<transitionMap[activators[i]]<<std::endl;
				firing_seq.push_back(activators[i]);
				trigerable = true;

				//Skip other pair. There is no change in the marking if we trigger
				//a pair in the same sequence.
				if (activators[i]%2 == 0)
				{
					std::cout<<"Adding to skip sequence:"<<transitionMap[activators[i]+1]<<std::endl;
					skip_transitions.insert(activators[i]+1);
				}
				else
				{
					std::cout<<"Adding to skip sequence:"<<transitionMap[activators[i]-1]<<std::endl;
					skip_transitions.insert(activators[i]-1);
				}
			}
			else
			{
				std::cout<<"Adding to skip sequence:"<<transitionMap[activators[i]]<<std::endl;
				skip_transitions.insert(activators[i]);
			}
		}
	}
	return trigerable;
}

bool PNController::firing_rec_bfs(int des_place,
		std::set<int>& skip_transitions,
		std::set<int>& visited_places)
{
	//Add place to visited places.
	visited_places.insert(des_place);

	//Find all possible transition activators for this place
	Eigen::VectorXi transitions = Dp.row(des_place);
	std::vector<int> activators;
	for (int i=0; i<transitions.size(); ++i)
	{
		//If connected (!=0) AND not in skipped transitions.
		if ((transitions(i) &&
				(skip_transitions.find(i) == skip_transitions.end())))
		{
			activators.push_back(i);
		}
	}

	//If no activators this is a dead-end.
	if (activators.empty())
	{
		std::cout<<"Place "<<placeMap[des_place]<<" is a dead-end."<<std::endl;
		return false;
	}

	//Output debugging information for the curret
	std::cout<<"Place "<<placeMap[des_place]<<" depends on transitions:";
	for (int i=0; i<activators.size(); ++i) std::cout<<transitionMap[activators[i]]<<", ";
	std::cout<<std::endl;

	//Add trigerable transitions to firing_seq
	Eigen::VectorXi enabled_trans = marking.transpose()*Dm;
	std::vector<int> un_activators;
	for (int i=0; i<activators.size(); ++i)
	{
		if (enabled_trans(activators[i]) == Dm.col(firing_seq[i]).sum())
		{
			std::cout<<"Adding to firing sequence:"<<transitionMap[activators[i]]<<std::endl;
			firing_seq.push_back(activators[i]);
			skip_transitions.insert(activators[i]);
			//Skip other pair. There is no change in the marking if we trigger
			//a pair in the same sequence.
			if (activators[i]%2 == 0)
			{
				std::cout<<"Adding to skip sequence:"<<transitionMap[activators[i]+1]<<std::endl;
				skip_transitions.insert(activators[i]+1);
			}
			else
			{
				std::cout<<"Adding to skip sequence:"<<transitionMap[activators[i]-1]<<std::endl;
				skip_transitions.insert(activators[i]-1);
			}
		}
		else
		{
			un_activators.push_back(activators[i]);
		}
	};

	for (int i=0; i<un_activators.size(); ++i)
	{
		Eigen::VectorXi t_en = Dm.col(un_activators[i]);
		std::vector<int> places;
		for (int j=0; j<t_en.size(); ++j)
		{
			//If connected (!=0) AND not in visited places.
			if ((t_en(j) &&
					(visited_places.find(j) == visited_places.end())))
			{
				places.push_back(j);
			}
		}

		if (places.size() == 0)
		{
			std::cout<<"Activator candidate transition "<<transitionMap[un_activators[i]];
			std::cout<<" is a dead-end."<<std::endl;
			continue;
		}

		//Debugging information for the activator places.
		std::cout<<"Activator candidate transition "<<transitionMap[un_activators[i]];
		std::cout<<" depends on:";
		for (int j=0; j<places.size(); ++j) std::cout<<placeMap[places[j]]<<", ";
		std::cout<<std::endl;

		bool add_cur_seq=true;
		//Places that are already marked add to visited places to avoid backlash.
		for (int j=0; j<places.size(); ++j)
			if (marking(places[j])) visited_places.insert(places[j]);

		for (int j=0; j<places.size(); ++j)
		{
			//Recurse to search a lever higher for unmarked places
			if (!marking(places[j]))
			{
				firing_rec(places[j], skip_transitions, visited_places);
			}
		}
	};
}

void PNController::get_firing(const std::string& name)
{
	//Desired place to activate
	int des_place = nameMap[name].place_num;

	while (marking(des_place) != 1)
	{
		firing_seq.clear();
		std::set<int> skip_transitions;
		std::set<int> visited_places;
		firing_rec(des_place, skip_transitions, visited_places);

		std::cout<<"The final firing sequence is:";
		for (int i=0; i<firing_seq.size(); ++i)
		{
			std::cout<<transitionMap[firing_seq[i]]<<" -> ";
		}
		std::cout<<"end"<<std::endl;

		//Do the firing
		//Add testing "simulation" of the firing before applying.
		//Add enable/disable service calls for these.
		for(int i=0; i<firing_seq.size(); ++i)
		{
			Eigen::VectorXi fire = Eigen::VectorXi::Zero(Dm.cols());
			fire(firing_seq[i]) = 1;

			Eigen::VectorXi enabled_trans = marking.transpose()*Dm;
			if (enabled_trans(firing_seq[i]) == Dm.col(firing_seq[i]).sum())
			{
				std::cout<<"Transition "<<transitionMap[firing_seq[i]]<<" is enabled."<<std::endl;
			}
			else
			{
				std::cout<<"Transition "<<transitionMap[firing_seq[i]]<<" is not enabled."<<std::endl;
				std::cerr<<"Error - cannot fire transition."<<std::endl;
				break;
			}

			marking = marking + I*fire;
			std::cout<<"Transition "<<transitionMap[firing_seq[i]]<<" fired, new marking:"<<marking<<std::endl;
			if (marking(des_place))
			{
				std::cout<<"Reached desired update in the marking."<<std::endl;
				break;
			}
		}

		usleep(1000*1000);
	}
}

void PNController::get_firing_bfs(const std::string& name)
{
	//Desired place to activate
	int des_place = nameMap[name].place_num;

	firing_seq.clear();
	std::set<int> skip_transitions;
	std::set<int> visited_places;
	firing_rec_bfs(des_place, skip_transitions, visited_places);

	std::cout<<"The final firing sequence is:";
	for (int i=0; i<firing_seq.size(); ++i)
	{
		std::cout<<transitionMap[firing_seq[i]]<<" -> ";
	}
	std::cout<<"end"<<std::endl;

	//Do the firing
	//Add testing "simulation" of the firing before applying.
	//Add enable/disable service calls for these.
	for(int i=0; i<firing_seq.size(); ++i)
	{
		Eigen::VectorXi fire = Eigen::VectorXi::Zero(Dm.cols());
		fire(firing_seq[i]) = 1;

		Eigen::VectorXi enabled_trans = marking.transpose()*Dm;
		if (enabled_trans(firing_seq[i]) == Dm.col(firing_seq[i]).sum())
		{
			std::cout<<"Transition "<<transitionMap[firing_seq[i]]<<" is enabled."<<std::endl;
		}
		else
		{
			std::cout<<"Transition "<<transitionMap[firing_seq[i]]<<" is not enabled."<<std::endl;
			std::cerr<<"Error - cannot fire transition."<<std::endl;
			break;
		}

		marking = marking + I*fire;
		std::cout<<"Transition "<<transitionMap[firing_seq[i]]<<" fired, new marking:"<<marking<<std::endl;
		if (marking(des_place))
		{
			std::cout<<"Reached desired update in the marking."<<std::endl;
			break;
		}
	}
}

