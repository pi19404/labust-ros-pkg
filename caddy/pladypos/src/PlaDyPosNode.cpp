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
#include <labust/math/NumberManipulation.hpp>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <std_msgs/Float32MultiArray.h>

#include <boost/regex.hpp>

#include <string>
#include <sstream>

using namespace labust::vehicles;

const float PlaDyPosNode::sscale[6]={0.00926,
		0.00926,
		0.00926,
		0.00926,
		0.0152738,
		0.02795
};

PlaDyPosNode::PlaDyPosNode():
	io(),
	port(io),
	lastTau(ros::Time::now()),
	timeout(0.5),
	revControl(false){}

PlaDyPosNode::~PlaDyPosNode()
{
	io.stop();
	safety.join();
	iorunner.join();
}

void PlaDyPosNode::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Initialize subscribers and publishers
	tau = nh.subscribe("tauIn", 1, &PlaDyPosNode::onTau,this);
	tauAch = nh.advertise<auv_msgs::BodyForceReq>("tauAch",1);
	diag = nh.advertise<diagnostic_msgs::DiagnosticStatus>("diagnostics",1);
	info = nh.advertise<std_msgs::Float32MultiArray>("pladypos_info",1);
	
	//Initialize max/min tau
	double maxThrust(1),minThrust(-1);
	ph.param("maxThrust",maxThrust,maxThrust);
	ph.param("minThrust",minThrust,minThrust);

	//Initialize the allocation matrix
	Eigen::Matrix<float, 3,4> B;
	float cp(cos(M_PI/4)),sp(sin(M_PI/4));
	B<<-cp,cp,cp,-cp,
	   -sp,-sp,sp,sp,
	    1,-1,1,-1;

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
  	ROS_INFO("Port %s on is open.", portName.c_str());
    port.set_option(boost::asio::serial_port::baud_rate(baud));
    //port.set_option(serial_port::flow_control(config->flowControl));
    //port.set_option(serial_port::parity(config->parity));
    //port.set_option(serial_port::stop_bits(config->stopBits));
    //port.set_option(serial_port::character_size(config->dataBits));
  }
  else
  {
  	ROS_WARN("Port %s on is not open.", portName.c_str());
  }

	//Configure the dynamic reconfigure server
  server.setCallback(boost::bind(&PlaDyPosNode::dynrec, this, _1, _2));

  this->start_receive();
  safety = boost::thread(boost::bind(&PlaDyPosNode::safetyTest, this));
  iorunner = boost::thread(boost::bind(&boost::asio::io_service::run, &this->io));
}

void PlaDyPosNode::start_receive()
{
	boost::asio::async_read_until(port, sbuffer, boost::regex("\\)|!|\\?"),
			boost::bind(&PlaDyPosNode::onReply,this,_1,_2));
}

void PlaDyPosNode::onReply(const boost::system::error_code& error, const size_t& transferred)
{
	static int trackC=0;
	if (!error)
	{
		std::istream is(&sbuffer);
		char c;
		is>>c;

		char ret, junk, delimit;
		int idx, value;
		switch (c)
		{
		case '!':
			//ROS_INFO("Message acknowledged.");
			break;
		case '?':
			//ROS_INFO("Communication error - received '?'");
			break;
		case '(':
			is>>ret>>idx>>junk>>value>>delimit;
			if (ret == 'c')
			{
				++trackC;
				sensors[idx] = value*sscale[idx];
				//ROS_INFO("Received data: type=%c, idx=%d, value=%d", ret,idx,value);

				if (trackC == sensors.size()-1)
				{
					trackC = 0;
				  	pubDiagnostics();
				}
			}
			break;
		default:
			ROS_ERROR("Unknown start character.");
		}
	}
	start_receive();
}

void PlaDyPosNode::pubDiagnostics()
{
	diagnostic_msgs::DiagnosticStatusPtr status(new diagnostic_msgs::DiagnosticStatus());

	status->level = status->OK;
	status->name = "PlaDyPos Driver";
	status->message = "Voltage and current report";
	status->hardware_id = "None";

	std::string names[6]={"C0","C1","C2","C3","CP","Voltage"};
	std::ostringstream out;
	std_msgs::Float32MultiArrayPtr data(new std_msgs::Float32MultiArray());
	data->data.resize(sensors.size());
	for (size_t i=0; i<sensors.size(); ++i)
	{
		out.str("");
		out<<sensors[i];
		diagnostic_msgs::KeyValue ddata;
		ddata.key = names[i];
		ddata.value = out.str();
		status->values.push_back(ddata);
		data->data[i]=sensors[i];
	}

	for (size_t i=0; i<4; ++i) data->data.push_back(lastRevs[i]);

	diag.publish(status);
	info.publish(data);
}

void PlaDyPosNode::dynrec(pladypos::ThrusterMappingConfig& config, uint32_t level)
{
	revControl = config.enableRevs;

	if (revControl)
	{
		int n[4]={config.rev0, config.rev1, config.rev2, config.rev3};
		driverMsg(n);
		//ROS_INFO("Revoultion control enabled.");
	}
}

void PlaDyPosNode::safetyTest()
{
	ros::Rate rate(5);

	while (ros::ok())
	{
		if (((ros::Time::now() - lastTau).toSec() > timeout) && !revControl)
		{
			ROS_WARN("Timeout triggered.");
			int n[4]={0,0,0,0};
			driverMsg(n);
		}
		rate.sleep();
	}
}

void PlaDyPosNode::driverMsg(const int n[4])
{
	std::ostringstream out;

	//Here we set the thrust
	for (int i=0; i<4;++i)
	{
		lastRevs[i]=n[i];
		out<<"(P"<<i<<","<<abs(n[i])<<","<<((n[i]>0)?1:0)<<")";
	}
	//Here we request the currents and voltages.
	for (int i=0; i<6;++i)	out<<"(C"<<i<<")";

	boost::mutex::scoped_lock lock(serialMux);
	boost::asio::write(port,boost::asio::buffer(out.str()));
}

void PlaDyPosNode::onTau(const auv_msgs::BodyForceReq::ConstPtr tau)
{
	lastTau = ros::Time::now();
	if (revControl) return;
	//Perform allocation
	Eigen::Vector3f tauXYN,tauXYNsc;
	Eigen::Vector4f tauI;
	tauXYN<<tau->wrench.force.x,tau->wrench.force.y,tau->wrench.torque.z;
	double scale = allocator.scaleII(tauXYN,&tauXYNsc,&tauI);

	//Publish the scaled values if scaling occured
	auv_msgs::BodyForceReq t;
	t.wrench.force.x = tauXYNsc(0);
	t.wrench.force.y = tauXYNsc(1);
	t.wrench.force.z = 0;
	t.wrench.torque.x = 0;
	t.wrench.torque.y = 0;
	t.wrench.torque.z = tauXYNsc(2);
	t.header.stamp = ros::Time::now();

	if (scale>1)
	{
	  t.disable_axis.x = 1;
	  t.disable_axis.y = 1;
	  t.disable_axis.yaw = 1;
	}

	tauAch.publish(t);

	//Tau to Revs
	int n[4];
	//Here we map the thrusts
	for (int i=0; i<4;++i)
	{
		//This is the quadratic allocation.
		//n[i] = labust::vehicles::AffineThruster::getRevs(tauI(i),1.0/(255*255),1.0/(255*255));
		//This is the linear allocation.
		//n[i]=tauI(i)*255; 
		//This is the compensated quadratic+linear allocation.
		if (fabs(tauI(i)) < 0.23184)
		{
		  n[i] = 255*labust::vehicles::AffineThruster::getRevsD(tauI(i),1.894,1.894);
		}
		else
		{
		  double a(1.21), b(-0.1915);
		  n[i]=255*labust::math::coerce((tauI(i)-fabs(tauI(i))/tauI(i)*b)/a, -1,1);
		}
	}

	//Manual corection
	//if (n[3] < 0)
	//{ 
	//	n[3]*= 1.15;
	//	n[3] = labust::math::coerce(n[3],-255,255);
	//}
	//n[1] *= 0.95;

	
	driverMsg(n);

	//ROS_INFO("Revs: %d %d %d %d\n",n[0],n[1],n[2],n[3]);
}
