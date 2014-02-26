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
*  Created: 01.02.2013.
*********************************************************************/
#include <ros/ros.h>
#include <navcon_msgs/RegisterController.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_ident");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<navcon_msgs::RegisterController>("register_controller");
  navcon_msgs::RegisterController srv;

  //Add manual
  std::string manualPos[]={"manual_x","manual_y","manual_z","manual_roll", "manual_pitch", "manual_yaw"};
  std::string manualNu[]={"manual_u","manual_v","manual_w","manual_p", "manual_q", "manual_r"};
  std::string manuals[]={"manualX","manualY","manualZ","manualK", "manualM", "manualN"};
  //Add surge, sway, heave, yaw_rate controllers.
  std::string basics[]={"surge","sway","heave","roll_rate","pitch_rate","yaw_rate"};
  std::string ident[]={"identX","identY","identZ","identK","identM","identN"};
  std::string pose_cnt[]={"fadp","fadp","depth","roll","pitch","heading"};
  int basics_dofs[]={0,1,2,3,4,5};

  for (int i=0; i<6; ++i)
  {
  	//velocity controllers
	  srv.request.used_dofs.assign(0);
	  srv.request.name = basics[i];
	  srv.request.used_dofs[basics_dofs[i]] = 1;
	  client.call(srv);

	  //tau manual
	  srv.request.used_dofs.assign(0);
	  srv.request.name = manuals[i];
	  srv.request.used_dofs[basics_dofs[i]] = 1;
	  client.call(srv);

	  //nu manual
	  navcon_msgs::RegisterController srv2;
	  srv2.request.name = manualNu[i];
	  srv2.request.used_cnt.push_back(basics[i]);
	  client.call(srv2);

	  //identification
	  navcon_msgs::RegisterController srv3;
	  srv3.request.name = ident[i];
	  //srv3.request.used_dofs.assign(1);
	  srv3.request.used_dofs[2] = 0;
	  srv3.request.used_dofs[5] = 0;
	  srv3.request.used_dofs[i] = 1;
	  client.call(srv3);

	  //Pose controllers
	  navcon_msgs::RegisterController srv4;
	  srv4.request.name = pose_cnt[i];
	  srv4.request.used_cnt.push_back(basics[i]);
	  if (i == 1) srv4.request.used_cnt.push_back(basics[i-1]);
	  if (i != 0) client.call(srv4);

	  std::cout<<"finished no:"<<i<<std::endl;
  }

  //Underactuated dp
  srv.request.used_dofs.assign(0);
  srv.request.used_cnt.clear();
  srv.request.name = "uadp";
  srv.request.used_cnt.push_back("surge");
  srv.request.used_cnt.push_back("heading");
  client.call(srv);

  std::cout<<"Added dp."<<std::endl;

  //Virtual target
  srv.request.used_dofs.assign(0);
  srv.request.used_cnt.clear();
  srv.request.name = "vt";
  srv.request.used_cnt.push_back("surge");
  srv.request.used_cnt.push_back("yaw_rate");
  client.call(srv);
  std::cout<<"Added dp."<<std::endl;
  //LF target
  srv.request.used_dofs.assign(0);
  srv.request.used_cnt.clear();
  srv.request.name = "falf";
  srv.request.used_cnt.push_back("surge");
  srv.request.used_cnt.push_back("sway");
  srv.request.used_cnt.push_back("heading");
  client.call(srv);
  //LF underactuated
  srv.request.used_dofs.assign(0);
  srv.request.used_cnt.clear();
  srv.request.name = "ualf";
  srv.request.used_cnt.push_back("surge");
  srv.request.used_cnt.push_back("yaw_rate");
  client.call(srv);

  //exit
  return 0;
}



