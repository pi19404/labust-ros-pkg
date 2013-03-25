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
#include <labust/vehicles/PlaDyPosNode.hpp>
#include <labust/vehicles/Allocation.hpp>

#include <string>
#include <sstream>

using namespace labust::vehicles;

PlaDyPosNode::PlaDyPosNode():
	io(),
	port(io){}

PlaDyPosNode::~PlaDyPosNode(){}

void PlaDyPosNode::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Initialize subscribers and publishers
	tau = nh.subscribe("tauIn", 1, &PlaDyPosNode::onTau,this);
	tauAch = nh.advertise<auv_msgs::BodyForceReq>("tauAch",1);

	//Initialize max/min tau
	double maxThrust(1),minThrust(-1);
	ph.param("maxThrust",maxThrust,maxThrust);
	ph.param("minThrust",minThrust,minThrust);

	//Initialize the allocation matrix
	Eigen::Matrix<float, 3,4> B;
	float cp(cos(M_PI/4)),sp(sin(M_PI/4));
	B<<cp,cp,-cp,-cp,
		 sp,-sp,sp,-sp,
		 1,-1,-1,1;

	//Scaling allocation only for XYN
	allocator.configure(B,maxThrust,minThrust);

	//Setup serial port
	std::string portName("/dev/ttyUSB0");
	int baud(9600);
	ph.param("PortName",portName,portName);
	ph.param("Baud",baud,baud);
  port.open(portName);

  if (port.is_open())
  {
    port.set_option(boost::asio::serial_port::baud_rate(baud));
    //port.set_option(serial_port::flow_control(config->flowControl));
    //port.set_option(serial_port::parity(config->parity));
    //port.set_option(serial_port::stop_bits(config->stopBits));
    //port.set_option(serial_port::character_size(config->dataBits));
  }
  else
  {
  	ROS_WARN("Port %s on open.", portName.c_str());
  }
}


void PlaDyPosNode::onTau(const auv_msgs::BodyForceReq::ConstPtr tau)
{
	//Perform allocation
	Eigen::Vector3f tauXYN,tauXYNsc;
	Eigen::Vector4f tauI;
	tauXYN<<tau->wrench.force.x,tau->wrench.force.y,tau->wrench.torque.z;
	double scale = allocator.scaleII(tauXYN,&tauXYNsc,&tauI);

	//Publish the scaled values if scaling occured
	if (scale>1)
	{
		auv_msgs::BodyForceReq t;
		t.wrench.force.x = tauXYNsc(0);
		t.wrench.force.y = tauXYNsc(1);
		t.wrench.force.z = 0;
		t.wrench.torque.x = 0;
		t.wrench.torque.y = 0;
		t.wrench.torque.z = tauXYNsc(2);
		t.header.stamp = ros::Time::now();
		tauAch.publish(t);
	}

	//Tau to Revs
	int n[4];
	std::ostringstream out("");

	for (int i=0; i<4;++i)
	{
		n[i] = labust::vehicles::AffineThruster::getRevs(tauI(i),1.0/(255*255),1.0/(255*255));
		out<<"(P"<<i<<","<<abs(n[i])<<","<<((n[i]>0)?0:1)<<")";
	}

	std::string todriver(out.str());
	ROS_INFO("Revolutions output:%s\n",todriver.c_str());
	ROS_INFO("Revs: %d %d %d %d\n",n[0],n[1],n[2],n[3]);

	boost::asio::write(port,boost::asio::buffer(todriver));
}
