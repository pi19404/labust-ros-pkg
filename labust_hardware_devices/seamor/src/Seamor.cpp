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
#include <labust/vehicles/Seamor.hpp>
#include <labust/vehicles/SeamorComms.hpp>
#include <labust/comms/SerialConfig.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/xml/GyrosReader.hpp>
#include <labust/vehicles/Allocation.hpp>

#include <boost/asio.hpp>

using namespace labust::vehicles;

Seamor::Seamor(const labust::xml::ReaderPtr reader, const std::string& id):
		io(),
		port(io),
		nextMsg(Sync),
		numToRead(1),
		inputBuffer(20,0),
		outputBuffer(20,0),
		Tnn(0.06873),
		_Tnn(0.01668),
		revLimit(-128,127)
{
	this->configure(reader,id);
}

Seamor::~Seamor()
{
	io.stop();
	comms->join();
}

void Seamor::configure(const labust::xml::ReaderPtr reader, const std::string& id)
{
	_xmlNode* org_node = reader->currentNode();
	//Find the desired configuration.
	reader->useNode(reader->value<_xmlNode*>("VehicleDriver[@name='Seamor'" + (id.empty()?"]":("and @id='" + id + "']"))));

	//Configure the serial port.
	labust::comms::serial_configure(*reader,port);
	this->start_receive();
	comms.reset(new boost::thread(boost::bind(&boost::asio::io_service::run,&this->io)));

	reader->useNode(org_node);
}

void Seamor::start_receive()
{
	//Save the first byte in buffer during synchronization procedure.
	boost::asio::async_read(port, boost::asio::buffer(inputBuffer.data() + (numToRead == 1),numToRead),
			boost::bind(&Seamor::handleInput, this, _1 , _2));
}

void Seamor::setTAU(const labust::vehicles::tauMapRef tau)
{
	using namespace labust::math;

	//The compensation has to be carried out in rev space and translated back to Seamor "force" space.
	///\todo check this revLimit here.
	double a = cos(M_PI/6);
	double b = sin(M_PI/6);
	int port = coerce(AffineThruster::getRevs((tau[labust::vehicles::tau::X] + tau[labust::vehicles::tau::N])/2, Tnn, _Tnn),revLimit);
	int stbd = coerce(AffineThruster::getRevs((tau[labust::vehicles::tau::X] - tau[labust::vehicles::tau::N])/2, Tnn, _Tnn),revLimit);
	int vport = coerce(AffineThruster::getRevs((tau[labust::vehicles::tau::Z]/a + tau[labust::vehicles::tau::Y]/b)/2, Tnn, _Tnn),revLimit);
	int vstbd = coerce(AffineThruster::getRevs((tau[labust::vehicles::tau::Z]/a - tau[labust::vehicles::tau::Y]/b)/2, Tnn, _Tnn),revLimit);

	boost::mutex::scoped_lock lock(dataMux);
	std::cerr<<"Port:"<<port<<", stbd:"<<stbd<<", vport:"<<vport<<", vstbd:"<<vstbd<<std::endl;
	rovCommand.tau[tau::X] = coerce(port + stbd,revLimit);//tau[labust::vehicles::tau::N];coerce(tau[labust::vehicles::tau::X], limit);
	rovCommand.tau[tau::Y] = coerce(vport - vstbd,revLimit);//coerce(tau[labust::vehicles::tau::Y], revLimit);
	rovCommand.tau[tau::Z] = coerce(vport + vstbd,revLimit);//coerce(tau[labust::vehicles::tau::Z], revLimit);
	rovCommand.tau[tau::N] = coerce(port - stbd,revLimit);//tau[labust::vehicles::tau::N];coerce(tau[labust::vehicles::tau::N], limit);
	SeamorCommand::encode(outputBuffer,rovCommand);

	lock.unlock();

	boost::asio::write(this->port,boost::asio::buffer(outputBuffer));
	//I prefer sync writing when no hardware flow control exists.
	//boost::asio::async_write(port,boost::asio::buffer(outputBuffer),boost::bind(&Seamor::handleOutput,this,_1,_2));
};

void Seamor::getState(labust::vehicles::stateMapPtr states)
{
	boost::mutex::scoped_lock lock(dataMux);
	using namespace labust::vehicles;
	(*states)[state::X_e] = rovCommand.tau[tau::X];
	(*states)[state::Y_e] = rovCommand.tau[tau::Y];
	(*states)[state::Z_e] = rovCommand.tau[tau::Z];
	(*states)[state::N_e] = rovCommand.tau[tau::N];
	(*states)[state::yaw] = labust::math::wrapDeg(rovStatus.heading) * M_PI / 180;
	(*states)[state::z] = rovStatus.depth;
	(*states)[state::roll] = rovStatus.roll * M_PI / 180;
	(*states)[state::pitch] = rovStatus.pitch * M_PI / 180;
};

void Seamor::setGuidance(const labust::vehicles::guidanceMapRef guidance)
{
	throw std::runtime_error("Seamor::setGuidance not implemented.");
}
void Seamor::setCommand(const labust::apps::stringRef commands){};

void Seamor::getData(const labust::apps::stringPtr data){};

void Seamor::handleOutput(const boost::system::error_code& error, const size_t transferred){}

void Seamor::handleInput(const boost::system::error_code& error, const size_t transferred)
{
	if (!error && (transferred == numToRead))
	{
		boost::mutex::scoped_lock lock(dataMux);
		switch (nextMsg)
		{
		case Sync:
			checkSync();
			//Queue the next read and return.
			this->start_receive();
			return;
		case CameraStatus:
			std::cout<<"Camera status handle."<<std::endl;
			//handle
			break;
		case Status:
			std::cout<<"Status handle."<<std::endl;
		  SeamorStatus::decode(inputBuffer,rovStatus);
			break;
		case CameraCommand:
			std::cout<<"Camera commands handle."<<std::endl;
			//handle
			break;
		case Command:
	   	std::cout<<"Commands handle."<<std::endl;
			//handle
			break;
		default:break;
		}
	}
	else
	{
		std::cerr<<"Communication error:"<<error.message()<<std::endl;
	}
	//Continue the default sync read.
	numToRead = 2;
	nextMsg = Sync;
	this->start_receive();
}

void Seamor::checkSync()
{
	using namespace labust::vehicles;

	unsigned char firstByte(inputBuffer[0]),secondByte(inputBuffer[1]);

	std::cout<<"First:"<<int(firstByte)<<","<<int(secondByte)<<std::endl;

	if ((firstByte == SeamorCameraStatus::id) &&
			(secondByte == SeamorCameraStatus::length))
	{
		std::cout<<"Camera status message."<<std::endl;
		numToRead = SeamorCameraStatus::length - 2;
		nextMsg = CameraStatus;
	}
	else if ((firstByte == SeamorCameraCommand::id) &&
			(secondByte == SeamorCameraCommand::length))
	{
		std::cout<<"Camera commands message."<<std::endl;
		numToRead = SeamorCameraCommand::length - 2;
		nextMsg = CameraCommand;
	}
	else if ((firstByte == SeamorCommand::id) &&
			(secondByte == SeamorCommand::length))
	{
		std::cout<<"Command message."<<std::endl;
		numToRead = SeamorCommand::length - 2;
		nextMsg = Command;
	}
	else if ((firstByte == SeamorStatus::id) &&
			(secondByte == SeamorStatus::length))
	{
		std::cout<<"Status message."<<std::endl;
		numToRead = SeamorStatus::length - 2;
		nextMsg = Status;
	}
	else
	{
		std::cerr<<"No sync."<<std::endl;
		inputBuffer[0] = inputBuffer[1];
		numToRead = 1;
		nextMsg = Sync;
	}
}

LABUST_EXTERN
{
	LABUST_EXPORT VehicleFactoryPtr createVehicleFactory()
  {
		return VehicleFactoryPtr(new VehicleFactory::Impl<Seamor>());
  }
};

