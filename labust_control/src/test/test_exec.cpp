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
#include <labust_control/RegisterController.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_ident");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<labust_control::RegisterController>("register_controller");
  labust_control::RegisterController srv;

  //Add manual
  std::string manuals[]={"manualX","manualY","manualZ","manualN"};
  //Add surge, sway, heave, yaw_rate controllers.
  std::string basics[]={"surge","sway","heave","yaw_rate"};
  int basics_dofs[]={0,1,2,5};

  for (int i=0; i<4; ++i)
  {
	  srv.request.used_dofs.assign(0);
	  srv.request.name = basics[i];
	  srv.request.used_dofs[basics_dofs[i]] = 1;
	  client.call(srv);

	  srv.request.used_dofs.assign(0);
	  srv.request.name = manuals[i];
	  srv.request.used_dofs[basics_dofs[i]] = 1;
	  client.call(srv);
  }

  //Add identifications: surge, sway, heave, yaw
  srv.request.name = "identX";
  srv.request.used_dofs.assign(1);
  //Let heading DOF be free ?
  srv.request.used_dofs[5] = 0;
  client.call(srv);

  srv.request.name = "identY";
  srv.request.used_dofs.assign(1);
  //Let heading DOF be free ?
  srv.request.used_dofs[5] = 0;
  client.call(srv);

  srv.request.name = "identZ";
  srv.request.used_dofs.assign(1);
  client.call(srv);

  srv.request.name = "identN";
  srv.request.used_dofs.assign(1);
  client.call(srv);

  srv.request.used_dofs.assign(0);
  //Add complex controllers: dp, hdg, depth, dp+depth, dp+hdg
  srv.request.name = "dp";
  srv.request.used_cnt.resize(2);
  srv.request.used_cnt[0] = "surge";
  srv.request.used_cnt[1] = "sway";
  client.call(srv);

  srv.request.name = "hdg";
  srv.request.used_cnt.resize(1);
  srv.request.used_cnt[0] = "yaw_rate";
  client.call(srv);

  srv.request.name = "depth";
  srv.request.used_cnt.resize(1);
  srv.request.used_cnt[0] = "heave";
  client.call(srv);

  srv.request.name = "dpDepth";
  srv.request.used_cnt.resize(3);
  srv.request.used_cnt[0] = "surge";
  srv.request.used_cnt[1] = "sway";
  srv.request.used_cnt[2] = "depth";
  client.call(srv);

  srv.request.name = "dpHdg";
  srv.request.used_cnt.resize(3);
  srv.request.used_cnt[0] = "surge";
  srv.request.used_cnt[1] = "sway";
  srv.request.used_cnt[2] = "hdg";
  client.call(srv);

  srv.request.name = "dpHdgDepth";
  srv.request.used_cnt.resize(4);
  srv.request.used_cnt[0] = "surge";
  srv.request.used_cnt[1] = "sway";
  srv.request.used_cnt[2] = "hdg";
  srv.request.used_cnt[3] = "depth";
  client.call(srv);

  //exit
  return 0;
}



