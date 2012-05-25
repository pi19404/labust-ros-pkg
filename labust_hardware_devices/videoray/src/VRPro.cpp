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
#include <labust/vehicles/VRPro.hpp>
#include <labust/vehicles/Allocation.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/comms/SerialConfig.hpp>
#include <labust/xml/XMLReader.hpp>

#include <labust/vehicles/VR3Details.hpp>
#include <labust/vehicles/VR4Details.hpp>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace labust::vehicles;


VRPro::VRPro(const labust::xml::ReaderPtr reader, const std::string& id):
		io(),
		port(io),
		states(new stateMap())
{
	this->configure(reader,id);
}

VRPro::~VRPro()
{
	io.stop();
}

void VRPro::configure(const labust::xml::ReaderPtr& reader, const std::string& id)
{
	//Configure the serial port.
	labust::comms::serial_configure(*reader,port);

	//Read rev and light limits
	revLimit.min = reader->value<double>("revLimit/@min");
	revLimit.max = reader->value<double>("revLimit/@max");
	lightLimit.min = reader->value<double>("lightLimit/@min");;
	lightLimit.max = reader->value<double>("lightLimit/@min");;

	//Read params
	Tnn = reader->value<double>("thruster-param/@an");
	_Tnn = reader->value<double>("thruster-param/@bn");

	//Read depth-mappings
	this->depthScale(reader->value<int>("depth-map/@low"),
			reader->value<int>("depth-map/@high"),
			reader->value<int>("depth-map/@psiHigh"));

	//Identify protocol
	if (reader->try_expression("vr4"))
	{
		comms.reset(new VRFutaba());
	}
	else
	{
		comms.reset(new VRSerialComms());
	}

	//Send initial stop message
	labust::vehicles::tauMap tau;
	this->setTAU(tau);
}

void VRPro::depthScale(int low, int high, double psiHigh, double psiLow)
{
	xDepth = (psiHigh - psiLow)/(high-low);
	yDepth = psiLow - (low*xDepth);
}

void VRPro::start_receive()
{
	boost::asio::async_read(port, boost::asio::buffer(comms->inputBuffer),
			boost::asio::transfer_all(),boost::bind(&VRPro::handleInput,this,_1,_2));
	io.poll();
}

void VRPro::setTAU(const labust::vehicles::tauMapRef tau)
{
	using namespace labust::vehicles::tau;
	using labust::math::coerce;
	using labust::vehicles::AffineThruster;

	try
	{
		thrusters[VRComms::port] = coerce(AffineThruster::getRevs((tau.at(X) + tau.at(N))/2, Tnn, _Tnn),revLimit);
		thrusters[VRComms::stbd] = coerce(AffineThruster::getRevs((tau.at(X) - tau.at(N))/2, Tnn, _Tnn),revLimit);
		thrusters[VRComms::vert] = coerce(AffineThruster::getRevs(tau.at(Z), Tnn, _Tnn),revLimit);
		thrusters[VRComms::light] = coerce(AffineThruster::getRevs(tau.at(Z), Tnn, _Tnn),lightLimit);
	}
	catch (std::exception& e)
	{
		thrusters[VRComms::port] = thrusters[VRComms::stbd] =
				thrusters[VRComms::vert] = thrusters[VRComms::light] = 0;
	}

	comms->encode(thrusters);
	boost::asio::write(port, boost::asio::buffer(comms->outputBuffer));
	//std::cout<<"Data:";
	//for (size_t i=0; i<comms->outputBuffer.size(); ++i) std::cout<<int(comms->outputBuffer[i])<<",";
	//std::cout<<std::endl;
	this->start_receive();
}


void VRPro::getState(labust::vehicles::stateMapPtr state)
{
	//Recalculate depth from pressure ?.
	using namespace labust::vehicles::state;
	(*state) = (*this->states);
	(*state)[z] = ((*states)[depthPressure]*xDepth + yDepth)/1.46;
	(*state)[yaw] = unwrap((*states)[yaw]);
}

void VRPro::setGuidance(const labust::vehicles::guidanceMapRef guidance)
{
	throw std::runtime_error("VRPro::setGuidance not implemented.");
}

void VRPro::setCommand(const labust::vehicles::stringRef commands)
{
	//throw std::runtime_error("VRPro::setCommand not implemented.");
}

void VRPro::getData(const labust::vehicles::stringPtr data)
{
	throw std::runtime_error("VRPro::getData not implemented.");
}

void VRPro::handleInput(const boost::system::error_code& error, const size_t transferred)
{
	if (!error && (transferred == comms->inputBuffer.size()))
	{
		comms->decode(states);
	}
	else
	{
		std::cerr<<"Communication error. Received:"<<transferred<<", expected"
				<<comms->inputBuffer.size()<<std::endl;
	}
};

LABUST_EXTERN
{
	LABUST_EXPORT VehicleFactoryPtr createVehicleFactory()
  {
		return VehicleFactoryPtr(new VehicleFactory::Impl<VRPro>());
  }
};
//const LABUST::MATHUTILS::Limit VideoRayPro3::rev_limit(-220,220);
//const LABUST::MATHUTILS::Limit VideoRayPro3::light_limit(0,200);
//const double VideoRayPro3::Tnn(0.0003);
//const double VideoRayPro3::_Tnn(0.0001389);
//
//VideoRayPro3::VideoRayPro3(const LABUST::XML::Reader& reader, const std::string& ID):
//  serial(io),
//  buffer(Protocol::return_length,0),
//  //watchdog(boost::bind(&VideoRayPro3::stopAll,this),1000),
//  doWork(true)
//{
//  //configure serial port
//  LABUST::COMMUNICATION::serial_configure(reader,serial);
//  stopAll();
//  //configure depth calibration, etc.
//  //Configure message header
//  //output[header] = header_byte;
//  //output[id] = id_byte;
//  boost::asio::async_read(serial, boost::asio::buffer(buffer),
//      boost::bind(&VideoRayPro3::handleInput,this,_1,_2));
//
//  serveIO.reset(new boost::thread(boost::bind(&boost::asio::io_service::run,&io)));
//  outT.reset(new boost::thread(boost::bind(&VideoRayPro3::writeOut,this)));
//};
//
//VideoRayPro3::~VideoRayPro3()
//{
//  std::cout<<"VideoRay destructor."<<std::endl;
//  doWork = false;
//  stopAll();
//  io.stop();
//  std::cout<<"Stopping IO."<<std::endl;
//  serveIO->join();
//  outT->join();
//};
//
//void VideoRayPro3::stopAll()
//{
//  boost::mutex::scoped_lock lock(controlMutex);
//  //Stop all thrusters
//  set_revs(port,0);
//  set_revs(stbd,0);
//  set_revs(vert,0);
//  //Turn off lights
//  thrust[light] = 0;
//  //Stop all camera motion
//  control_bits.reset(tilt_enable);
//  control_bits.reset(focus_enable);
//  //Stop manipulator motion
//  control_bits.reset(manipulator_enable);
//  //Camera selection is left on.
//  //LABUST::TYPES::ByteVectorPtr msg(Protocol::encode(thrust,control_bits));
//  std::cout<<"Stopping operation."<<std::endl;
//  //std::cout<<"Sending safety:";
//
//  //boost::mutex::scoped_lock l(condition_mutex);
//  //outcondition.notify_one();
//
//
//  /*for (int i=0; i<msg->size(); ++i)
//  {
//	  std::cout<<int(msg->at(i))<<",";
//  };
//  std::cout<<std::endl;*/
//  //boost::asio::async_write(serial, boost::asio::buffer(*msg), boost::bind(&writeHandle,_1,_2));
//
//  //LABUST::TOOLS::sleep(50);
//
//
//}
//
//void VideoRayPro3::setTAU(const LABUST::VEHICLES::COREDATASET::tauMap& tau)
//{
//  boost::mutex::scoped_lock lock(controlMutex);
//  //watchdog.reset();
//
//  using namespace LABUST::VEHICLES::COREDATASET;
//  tauMap::const_iterator end = tau.end();
//  tauMap::const_iterator it = tau.find(X);
//  double tauX = (it != end) ? it->second : 0;
//  double tauN = ((it = tau.find(N)) != end) ? it->second : 0;
//  double tauZ = ((it = tau.find(Z)) != end) ? it->second : 0;
//
//
//  control_bits[port_direction] = !set_revs(port, LABUST::ALLOCATION::Affine::get_revolution((tauX + tauN)/2, Tnn, _Tnn));
//  control_bits[stbd_direction] = set_revs(stbd, LABUST::ALLOCATION::Affine::get_revolution((tauX - tauN)/2, Tnn, _Tnn));
//  control_bits[vert_direction] = set_revs(vert, 200*tauZ);
//  //LABUST::TYPES::ByteVectorPtr msg(Protocol::encode(thrust,control_bits));
//  std::cout<<"Sending tau:";
//  /*for (int i=0; i<msg->size(); ++i)
//  {
//	  std::cout<<int(msg->at(i))<<",";
//  };
//  std::cout<<std::endl;
//
//  std::vector<unsigned char> data(8,0);
//
//  data[0] = 35;
//  data[1] = 49;*/
//
//  //boost::asio::async_write(serial, boost::asio::buffer(&data[0],data.size()),boost::bind(&writeHandle,_1,_2));
//
//
//  std::cout<<"thrust:"<<int(thrust[port])<<", "<<int(thrust[stbd])<<", "<<int(thrust[vert])<<","<<int(thrust[light])<<std::endl;
//  //Reset the camera for next step
//  //camera(0,0);
//
//    //LABUST::TOOLS::sleep(50);
//  //boost::mutex::scoped_lock l(condition_mutex);
//  //outcondition.notify_one();
//};
//
//void handle(const boost::system::error_code& e, size_t t){};
//
//void VideoRayPro3::writeOut()
//{
//	//Should be put into setTau, and the watchdog should be placed above
//	//Adding abort or stopAll to the interface could be considered.
//	//LABUST::TOOLS::wait_until_ms delay(100);
//	while (doWork)
//	{
//		//delay();
//		LABUST::TOOLS::sleep(100);
//		boost::mutex::scoped_lock l(controlMutex);
//		LABUST::TYPES::ByteVectorPtr msg(Protocol::encode(thrust,control_bits));
//		boost::asio::async_write(serial, boost::asio::buffer(*msg),&handle);
//	}
//};
//
//
//void VideoRayPro3::setOptional(const LABUST::VEHICLES::EXDATASET::dataMap& commands)
//{
//  using namespace LABUST::VEHICLES::EXDATASET;
//  //Camera tilt
//  dataMap::const_iterator end(commands.end());
//  dataMap::const_iterator it(commands.find("Tilt"));
//  if (it != end)
//  {
//	boost::mutex::scoped_lock l(controlMutex);
//    //THIS SHOULD BE ALL IN ONE OBJECT
//    //std::cout<<"Got something."<<std::endl;
//    LABUST::TYPES::uint8 tiltState = atoi(it->second.c_str());
//    LABUST::TYPES::uint8 focusState = 0;
//
//    if ((it=commands.find("Focus")) != end)
//    {
//      focusState = atoi(it->second.c_str());
//    }
//
//    if ((it=commands.find("CameraNo")) != end)
//    {
//      camera(tiltState, focusState, atoi(it->second.c_str()));
//    }
//    else
//    {
//      camera(tiltState, focusState);
//    }
//  }
//
//  //Change to accomodate the pJoystick problem
//  if ((it=commands.find("lights+")) != end)
//  {
//	  boost::mutex::scoped_lock l(controlMutex);
//	  thrust[light] = LABUST::MATHUTILS::coerce(thrust[light] + 10*atoi(it->second.c_str()),light_limit);
//  }
//
//  if ((it=commands.find("lights-")) != end)
//  {
//	boost::mutex::scoped_lock l(controlMutex);
//	thrust[light] = LABUST::MATHUTILS::coerce(thrust[light] - 10*atoi(it->second.c_str()),light_limit);
//  };
//}
//
//
//void VideoRayPro3::camera(LABUST::TYPES::int8 tilt, LABUST::TYPES::int8 focus, bool rear_cam)
//{
//  control_bits[this->tilt] = tilt>0;
//  control_bits[tilt_enable] = tilt;
//
//  control_bits[this->focus] = focus>0;
//  control_bits[focus_enable] = focus;
//
//  control_bits[camera_rear] = rear_cam;
//};








