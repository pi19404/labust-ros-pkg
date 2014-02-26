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
 *  Author : Dula Nad
 *  Created: 23.01.2013.
 *********************************************************************/
#include <labust/navigation/SpatialNode.hpp>
#include <labust/navigation/SpatialMessages.hpp>
#include <labust/preprocessor/clean_serializator.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>

#include <boost/bind.hpp>
#include <boost/crc.hpp>

#include <iosfwd>

PP_LABUST_CLEAN_ARRAY_ISERIALIZATOR_IMPL(boost::archive::binary_iarchive)
PP_LABUST_CLEAN_ARRAY_OSERIALIZATOR_IMPL(boost::archive::binary_oarchive)

using namespace labust::navigation;
using namespace labust::spatial;

SpatialNode::SpatialNode():
					io(),
					port(io),
					ringBuffer(headerSize,0)
{
	this->onInit();
}

SpatialNode::~SpatialNode()
{
	io.stop();
	runner.join();
}

void SpatialNode::onInit()
{
	ros::NodeHandle nh, ph("~");
	ros::Rate r(1);
	bool setupOk(false);
	while (!(setupOk = this->setup_port()) && ros::ok())
	{
		ROS_ERROR("SpatialNode::Failed to open port.");
		r.sleep();
	}

	//Setup publisher
	imu = nh.advertise<sensor_msgs::Imu>("imu",1);
	gps = nh.advertise<sensor_msgs::NavSatFix>("fix",1);
	vel = nh.advertise<geometry_msgs::TwistStamped>("ned_vel",1);

	if (setupOk)
	{
		using namespace labust::spatial;
		//Setup handlers
		handler[ID::AckPacket] = boost::bind(&SpatialNode::onAckPacket,this,_1);
		handler[ID::SystemState] = boost::bind(&SpatialNode::onSystemStatePacket,this,_1);
		handler[ID::EulerStdDev] = boost::bind(&SpatialNode::onVec3fPacket,
				this, boost::ref(eulerCov),_1);
		handler[ID::VelocityStdDev] = boost::bind(&SpatialNode::onVec3fPacket,
				this, boost::ref(velCov),_1);

		//Configure the device
		this->configureSpatial();
		//Start the receive cycle
		this->start_receive();
		runner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
	}
}

void SpatialNode::configureSpatial()
{
	using namespace labust::spatial;
	std::ostringstream out;
	boost::archive::binary_oarchive dataSer(out, boost::archive::no_header);

	enum {fs = 10};

	//uint32_t rebootSeq = 0x21057A7E;
	//dataSer << rebootSeq;
	//this->sendToDevice(ID::RebootPacket, LEN::RebootPacket, out.str());

	//Set the packet timer period
	PacketTimerPeriod p;
	p.UTCSync = 100;
	p.packetTimerPeriod = 10000;
	dataSer << p;
	this->sendToDevice(ID::PacketTimerPeriod, LEN::PacketTimerPeriod, out.str());

	//Set the packet timer period
	out.str("");
	PacketsPeriod period;
	period.clear = 1;
	period.packetID = ID::SystemState;
	//1000000/(p.packetTimerPeriod * fs)
	period.period = 1000000/(p.packetTimerPeriod * fs);
	dataSer << period;
	//Append the euler and velocity stddev packet (num_appended = 2)
	uint8_t id = ID::EulerStdDev;	dataSer << id << period.period;
	id = ID::VelocityStdDev; dataSer << id << period.period;
	//len = LEN::PacketsPeriod + 5*num_appended
	this->sendToDevice(ID::PacketsPeriod, LEN::PacketsPeriod + 5*2, out.str());

	//Set the sensor range
	out.str("");
	SensorRanges range;
	range.acc = range.gyro = range.mag = 0;
	dataSer << range;
  this->sendToDevice(ID::SensorRanges, LEN::SensorRanges, out.str());
}

void SpatialNode::sendToDevice(uint8_t id, uint8_t len, const std::string& data)
{
	boost::asio::streambuf output;
	boost::archive::binary_oarchive dataSer(output, boost::archive::no_header);
	//Calculate the header
	boost::crc_ccitt_type result;
	result.process_bytes(data.data(),data.size());
	uint16_t crc = 	result.checksum();
	uint8_t crc0 = result.checksum()%256;
	uint8_t crc1 = result.checksum()/256;
	//LRC
	uint8_t lrc = ((id + len + crc0 + crc1)^0xFF) + 1;
	dataSer << lrc;
	//Header info
	dataSer << id << len << crc;

	//write header
	boost::asio::write(port, output.data());
	//write data
	boost::asio::write(port, boost::asio::buffer(data));
}

void SpatialNode::start_receive()
{
	using namespace boost::asio;
	async_read(port, buffer.prepare(headerSize),
			boost::bind(&SpatialNode::onHeader, this, _1,_2));
}

bool SpatialNode::setup_port()
{
	ros::NodeHandle ph("~");
	std::string portName("/dev/ttyUSB0");
	int baud(115200);

	ph.param("PortName",portName,portName);
	ph.param("Baud",baud,baud);

	using namespace boost::asio;
	port.open(portName);
	port.set_option(serial_port::baud_rate(baud));
	port.set_option(serial_port::flow_control(
			serial_port::flow_control::none));

	return port.is_open();
}

void SpatialNode::onHeader(const boost::system::error_code& e,
		std::size_t size)
{
	if (!e)
	{
		//std::cout<<"Header read size:"<<size<<std::endl;
		buffer.commit(size);
		if (size == 1)
		{
			//Put the new byte on the end of the ring buffer
			ringBuffer.push_back(buffer.sbumpc());
		}
		else
		{
			//Copy all data into the buffer
			buffer.sgetn(reinterpret_cast<char*>(ringBuffer.data()),size);
		}

		//Check LRC
		uint8_t result = (std::accumulate(ringBuffer.begin()+1, ringBuffer.end(),0)^0xFF) + 1;
		if ((ringBuffer[lrc] == result) && (ringBuffer[packet_length] != 0))
		{
			//std::cout<<"Received LRC checked message."<<std::endl;
			std::cout<<"ID:"<<int(ringBuffer[packet_id])<<", len:"<<int(ringBuffer[packet_length])<<std::endl;

			boost::asio::async_read(port, buffer.prepare(ringBuffer[packet_length]),
						boost::bind(&SpatialNode::onData,this,_1,_2));
			return;
		}
		else
		{
			//std::cout<<"LRC check failed."<<std::endl;
			ringBuffer.erase(ringBuffer.begin());
			boost::asio::async_read(port,
					buffer.prepare(1),
					boost::bind(&SpatialNode::onHeader,this,_1,_2));
			return;
		}
	}
	else
	{
		ROS_ERROR("SpatialNode: %s",e.message().c_str());
	}
	this->start_receive();
}

void SpatialNode::onData(const boost::system::error_code& e,
		std::size_t size)
{
	//std::cout<<"Received data size:"<<size<<std::endl;
	if (!e)
	{
		buffer.commit(size);
		std::istream is(&buffer);
		std::string data(size,'\0');
		is.read(&data[0],size);

		boost::crc_ccitt_type result;
		result.process_bytes(data.data(),data.size());
		if (result.checksum() == (256*ringBuffer[crc1]+ringBuffer[crc0]))
		{
			//Process tested data
			HandlerMap::iterator it(handler.find(ringBuffer[packet_id]));
			if (it != handler.end())
			{
			  std::istringstream is;
				is.rdbuf()->pubsetbuf(&data[0],data.size());
				boost::archive::binary_iarchive dataSer(is, boost::archive::no_header);
				it->second(dataSer);
			}
		}
	}
	this->start_receive();
}

void SpatialNode::statusUpdate(uint16_t systemStatus, uint16_t filterStatus)
{

}

void SpatialNode::onSystemStatePacket(boost::archive::binary_iarchive& data)
{
	labust::spatial::SystemState state;
	data >> state;
	statusUpdate(state.systemStatus, state.filterStatus);

	using namespace labust::tools;
	////////////////////////////////////////////////////////////////////////////////////
	//Setup the imu data
	enum {roll=0,pitch,yaw};
	enum {rollStdDev=0, pitchStdDev=4, yawStdDev=8};
	sensor_msgs::Imu::Ptr imuOut(new sensor_msgs::Imu());
	vectorToPoint(state.angularVelocity, imuOut->angular_velocity);

	vectorToPoint(state.bodyAcc, imuOut->linear_acceleration);

	labust::tools::quaternionFromEulerZYX(state.orientation[roll],
			state.orientation[pitch],
			labust::math::wrapRad(state.orientation[yaw]), imuOut->orientation);
	imuOut->orientation_covariance[rollStdDev] = std::pow(eulerCov[roll],2);
	imuOut->orientation_covariance[pitchStdDev] = std::pow(eulerCov[pitch],2);
	imuOut->orientation_covariance[yawStdDev] = std::pow(eulerCov[yaw],2);

	imuOut->header.frame_id = "imu_frame";
	imuOut->header.stamp = ros::Time::now();
	imu.publish(imuOut);

	/////////////////////////////////////////////////////////////////////////////////////
	//Setup the imu data
	enum {latitude=0, longitude, altitude, latStdDev=0, lonStdDev=4,altStdDev=8};
	sensor_msgs::NavSatFix::Ptr gpsOut(new sensor_msgs::NavSatFix());
	gpsOut->altitude = state.latLonHeight[altitude];
	gpsOut->latitude = state.latLonHeight[latitude];
	gpsOut->longitude = state.latLonHeight[longitude];
	gpsOut->position_covariance.elems[latStdDev] = std::pow(state.latLonHeightStdDev[latitude],2);
	gpsOut->position_covariance.elems[lonStdDev] = std::pow(state.latLonHeightStdDev[longitude],2);
	gpsOut->position_covariance.elems[altStdDev] = std::pow(state.latLonHeightStdDev[altitude],2);
	gpsOut->header.frame_id = "gps_frame";
	gpsOut->header.stamp = ros::Time::now();
	gps.publish(gpsOut);

	////////////////////////////////////////////////////////////////////////////////////
	//Setup the velocity data
	geometry_msgs::TwistStamped::Ptr velOut(new geometry_msgs::TwistStamped());
	vectorToPoint(state.linearVelocity, velOut->twist.linear);
	vectorToPoint(state.angularVelocity, velOut->twist.angular);
	velOut->header.frame_id = "local";
	velOut->header.stamp = ros::Time::now();
	vel.publish(velOut);
}

void SpatialNode::onVec3fPacket(vec3f& vec, boost::archive::binary_iarchive& data)
{
	data >> vec;
}

void SpatialNode::onAckPacket(boost::archive::binary_iarchive& data)
{
	labust::spatial::AckPacket ack;
	data >> ack;

  if (ack.res != 0)
	{
		std::cout<<"Ack result: "<<int(ack.res)<<std::endl;
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"navquest_node");
	SpatialNode node;
	ros::spin();

	return 0;
}


