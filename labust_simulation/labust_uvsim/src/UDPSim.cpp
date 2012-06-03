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
#include <labust/simulation/UDPSim.hpp>
#include <labust/comms/UDPConfig.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/xml/GyrosReader.hpp>
#include <labust/xml/GyrosWriter.hpp>
#include <labust/vehicles/VehicleDriver.hpp>
#include <labust/vehicles/VehicleFactoryName.hpp>
#include <labust/plugins/PluginLoader.hpp>

using namespace labust::simulation;

UDPSim::UDPSim(const labust::xml::ReaderPtr reader, const std::string& id):
		io(),
		socket_(io)
{
	_xmlNode* org_node = reader->currentNode();
		reader->useNode(reader->value<_xmlNode*>("UUVApp" + (id.empty()?"":("[@id='" + id + "']"))));

	//Configure vehicle driver
	plugin.reset(
			new labust::vehicles::VehiclePlugin(reader->value<std::string>("plugin/@name"),
					labust::vehicles::FactoryCreatorName::value));
	uuv.reset((*plugin)(reader));

	labust::communication::udp_configure(*reader,socket_,remote);

	reader->useNode(org_node);
}

UDPSim::~UDPSim()
{
	this->io.stop();
	if (threadPtr != 0)
	{
		threadPtr->join();
	}
};

void UDPSim::start(bool sync)
{
	start_receive();
	if (sync)
	{
		this->io.run();
	}
	else
	{
		threadPtr.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &this->io)));
	}
}

void UDPSim::stop()
{
	this->io.stop();
}

void UDPSim::start_receive()
{
	socket_.async_receive_from(boost::asio::buffer(buffer),remote_sender,
			boost::bind(&UDPSim::handle_receive,this,_1,_2));
}

void UDPSim::handle_receive(const boost::system::error_code& e, size_t length)
{
	if (!e)
	{
		if (remote == remote_sender)
		{
			try
			{
				std::string data(&buffer[0],length);
				labust::xml::GyrosReader reader(data);

				if (boost::iequals(reader.GetLabel(),"tau"))
				{
					labust::vehicles::tauMap tau;
					//Added as workaround for old programs after TAU was changed to a arrary.
					std::map<int,double> tauVal;
					reader.dictionary(tauVal);
					for (size_t i = 0; i < tau.size(); ++i) tau[i] = tauVal[i];
					uuv->setTAU(tau);
				}
				else if (boost::iequals(reader.GetLabel(),"optional"))
				{
					uuv->setCommand(data);
				}
				else if (boost::iequals(reader.GetLabel(),"exit"))
				{
					stop();
				}

				labust::vehicles::stateMapPtr states(new labust::vehicles::stateMap);
				uuv->getState(states);

				labust::xml::GyrosWriter writer(states->begin(),states->end());
				socket_.send_to(boost::asio::buffer(writer.GyrosXML()), remote);
			}
			catch (labust::xml::XMLException& e)
			{
				std::cerr<<"Malformed message: "<<e.what()<<std::endl;
			}
		}
		else
		{
			std::cerr<<"Received message from unknown sender."<<std::endl;
		}
	}
	else
	{
		std::cerr<<e.message()<<std::endl;
	}

	start_receive();
}




