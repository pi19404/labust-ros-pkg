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
#include <boost/graph/graphviz.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <queue>
#include <list>
#include <algorithm>
#include <sstream>

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

void PNController::addToGraph(const navcon_msgs::RegisterControllerRequest& info)
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

template <class Vector>
void print_vec(const Vector& vec)
{
	std::cout<<"(";
	int i;
	for(i=0; i<vec.size()-1;++i)
	{
		std::cout<<vec(i)<<",";
	}
	std::cout<<vec(i)<<")";
}

template <typename DistanceMap, typename PredecessorMap>
class bacon_number_recorder : public boost::default_bfs_visitor
{
public:
	bacon_number_recorder(DistanceMap dist, PredecessorMap p) : d(dist), p(p) { }

	template <typename Edge, typename Graph>
	void tree_edge(Edge e, const Graph& g) const
	{
		typename boost::graph_traits<Graph>::vertex_descriptor
		u = source(e, g), v = target(e, g);
		d[v] = d[u] + 1;
		p[v] = u;
	}
private:
	DistanceMap d;
	PredecessorMap p;
};
// Convenience function
template <typename DistanceMap, typename PredecessorMap>
bacon_number_recorder<DistanceMap, PredecessorMap>
record_bacon_number(DistanceMap d, PredecessorMap p)
{
	return bacon_number_recorder<DistanceMap, PredecessorMap>(d,p);
}

void PNController::get_firing_r(const std::string& name)
{
	//Desired place to activate
	int des_place = nameMap[name].place_num;

	std::cout<<"Desired place to activate is: "<<placeMap[des_place]<<" == "<<des_place<<std::endl;

	std::vector<Eigen::VectorXi> av_markings;
	std::vector<int> av_diff, av_diff2;
	for(int i=0; i<all_markings.size(); ++i)
	{
		if (all_markings[i](des_place))
		{
			av_markings.push_back(all_markings[i]);
			Eigen::VectorXi diff = (all_markings[i] - marking);
			diff = diff.array().abs();
			av_diff.push_back(diff.sum());
			av_diff2.push_back(diff.sum());
		}
	}

	std::sort(av_diff2.begin(),av_diff2.end());
	std::vector<int>::iterator it=std::find(av_diff.begin(), av_diff.end(),av_diff2[0]);

	std::cout<<"From "<<all_markings.size();
	std::cout<<" markings, "<<av_markings.size()<<" have the needed place enabled."<<std::endl;
	std::cout<<"The smallest difference marking is:";
	print_vec(av_markings[it-av_diff.begin()]);
	std::cout<<std::endl;
	std::cout<<"The current marking is:";
	print_vec(marking);
	std::cout<<std::endl;
	//marking = av_markings[it-av_diff.begin()];

	std::vector<Eigen::VectorXi>::iterator it2 = std::find(all_markings.begin(), all_markings.end(),marking);

	std::vector <int> bacon_number(boost::num_vertices(rgraph));
	std::vector <int> pred_map(boost::num_vertices(rgraph));
	//Create the distance map
	GraphType::vertex_descriptor curr_vx = all_idx[it2-all_markings.begin()];
	std::cout<<"The current marking index is: "<<it2-all_markings.begin()<<std::endl;
  boost::breadth_first_search(rgraph, all_idx[it2-all_markings.begin()],
                       boost::visitor(record_bacon_number(&bacon_number[0],&pred_map[0])));

  std::vector<int> av_idx;
  av_markings.clear();
  av_diff.clear();
  av_diff2.clear();
	for(int i=0; i<all_markings.size(); ++i)
	{
		if (all_markings[i](des_place))
		{
			av_markings.push_back(all_markings[i]);
			av_idx.push_back(all_idx[i]);
			av_diff.push_back(bacon_number[all_idx[i]]);
			av_diff2.push_back(bacon_number[all_idx[i]]);
		}
	}

	std::sort(av_diff2.begin(),av_diff2.end());
	it=std::find(av_diff.begin(), av_diff.end(),av_diff2[0]);

  std::cout<<"The closest marking(s) is(are):"<<std::endl;
  bool run=true;
  std::vector<int>::iterator it_h = av_diff.begin();
  while (it_h != av_diff.end())
  {
  	it_h=std::find(it_h, av_diff.end(),av_diff2[0]);
  	if (it_h != av_diff.end())
  	{
  		std::cout<<"\t";
  		print_vec(av_markings[it_h-av_diff.begin()]);
  		std::cout<<std::endl;
  		++it_h;
  	}
  }
  std::cout<<std::endl;
  std::cout<<"The current marking is:";
  print_vec(marking);
  std::cout<<std::endl;
  Eigen::VectorXi m_new = av_markings[it-av_diff.begin()];

  std::cout<<"Calculating firing seq."<<std::endl;
  //Firing sequence to closest based on predecessor map.
  GraphType::vertex_descriptor closest_vx = av_idx[it-av_diff.begin()];
  GraphType::vertex_descriptor curr_pred = closest_vx;

  boost::property_map<GraphType, boost::edge_name_t>::type edgeMap =
  		boost::get(boost::edge_name_t(), rgraph);
  std::cout<<"Get the edge map."<<std::endl;
//  if (boost::edge(curr_pred, closest_vx, rgraph).second)
//  {
//  	std::cout<<"Get edge connection between "<<curr_pred<<" and "<<closest_vx<<std::endl;
//    firing_seq.push_back(edgeMap[boost::edge(curr_pred, closest_vx, rgraph).first]);
//  }
//  else
//  {
//  	std::cout<<"No edge connection between "<<curr_pred<<" and "<<closest_vx<<std::endl;
//  }

  std::cout<<"Loop across edges. The curr_pred = closest_vx = "<<closest_vx<<std::endl;
  //Go back...
  std::vector<int> firing_seq;
  while (curr_pred != curr_vx)
  {
    if (boost::edge(pred_map[curr_pred], curr_pred, rgraph).second)
    {
    	std::cout<<"Get edge connection between "<<pred_map[curr_pred]<<" and "<<curr_pred<<std::endl;
    	firing_seq.push_back(edgeMap[boost::edge(pred_map[curr_pred], curr_pred, rgraph).first]);
    }
    else
    {
    	std::cout<<"No edge connection between "<<pred_map[curr_pred]<<" and "<<curr_pred<<std::endl;
    }
    curr_pred = pred_map[curr_pred];
  }

  std::cout<<"Triggering the following sequence:";
  for (int i=firing_seq.size()-1; i>=0; --i)
  {
  	std::cout<<transitionMap[firing_seq[i]]<<" -> ";
  }
  std::cout<<"end"<<std::endl;

  marking = m_new;
}

void PNController::reachability()
{
	rgraph.clear();
	Eigen::VectorXi m0=Eigen::VectorXi::Zero(marking.size());
	for (int i=0; i<6;++i) m0(i) = 1;

	typedef GraphType::vertex_descriptor IntType;
	std::queue<Eigen::VectorXi> new_markings;
	std::queue<IntType> new_idx;

	std::vector<Eigen::VectorXi> visited;
	std::vector<Eigen::VectorXi> all_markings;
	std::vector<IntType> all_markings_idx;
	std::vector<IntType> visited_idx;
	std::list<Eigen::VectorXi> dead_end;

	new_markings.push(m0);
	new_idx.push(boost::add_vertex(rgraph));
	rgraph[new_idx.back()].marking = m0;

	std::cout<<"Building reachability graph:"<<std::endl<<std::endl;

	while (!new_markings.empty())
	{
		Eigen::VectorXi m;
		IntType idx;
		bool foundNew = false;
		do
		{
			m = new_markings.front();
			idx = new_idx.front();
			new_markings.pop();
			new_idx.pop();
			foundNew = !(std::find(visited.begin(),visited.end(),m) != visited.end());
		}
		while ((!foundNew) && (!new_markings.empty()));

		if (!foundNew) break;
		std::cout<<"Selected new marking:"<<m<<std::endl;
		visited.push_back(m);
		visited_idx.push_back(idx);
		if (std::find(all_markings.begin(),all_markings.end(),m) == all_markings.end())
		{
			all_markings.push_back(m);
			all_markings_idx.push_back(idx);
		}

		//Check for enabled transitions
		Eigen::VectorXi all_trans = m.transpose()*Dm;
		for (int i=0; i<all_trans.size(); ++i)
		{
			if (all_trans(i) == Dm.col(i).sum())
			{
				std::cout<<"Transition "<<transitionMap[i]<<" is enabled.";
				std::cout<<"("<<all_trans(i)<<", "<<Dm.col(i).sum()<<")"<<std::endl;
				Eigen::VectorXi fire = Eigen::VectorXi::Zero(Dm.cols());
				fire(i) = 1;
				Eigen::VectorXi newmarking = m + I*fire;

				//Check if we already found this marking.
				std::vector<Eigen::VectorXi>::iterator it;
				it = std::find(all_markings.begin(),all_markings.end(),newmarking);

				if (it == all_markings.end())
				{
					//New marking
					IntType nidx = boost::add_vertex(rgraph);
					new_markings.push(newmarking);
					new_idx.push(nidx);
					all_markings.push_back(newmarking);
					all_markings_idx.push_back(nidx);
					rgraph[nidx].marking = newmarking;
					boost::add_edge(idx, nidx,
							i,rgraph);
				}
				else
				{
					//Old marking
					//Possibly suboptimal
					int old_idx = it - all_markings.begin();
					boost::add_edge(idx, all_markings_idx[old_idx],
							i,rgraph);
				}
			}
		}
	}

	std::cout<<"Finished building graph."<<std::endl;
	this->all_markings.swap(all_markings);
	this->all_idx.swap(all_markings_idx);
}

void PNController::getDotDesc(std::string& desc)
{
	using namespace boost;
	//Construct a label writer.
	std::ostringstream out;
	write_graphviz(out, rgraph,
			pn_writer(rgraph),
			make_edge_writer(
					boost::get(boost::edge_name_t(),rgraph),
					transitionMap));
	desc = out.str();
}

