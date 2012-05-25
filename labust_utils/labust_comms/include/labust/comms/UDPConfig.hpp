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
#ifndef UDPCONFIG_HPP_
#define UDPCONFIG_HPP_
#include <labust/xml/XMLReader.hpp>
#include <labust/tools/StringUtilities.hpp>

#include <boost/asio/ip/udp.hpp>
#include <boost/shared_ptr.hpp>

#include <string>

namespace labust
{
  namespace communication
  {
	  /**
	   * The UDP configuration data structure.
	   */
	  struct UDPConfig
	  {
	  	/**
	  	 * Pointer typedef.
	  	 */
	  	typedef boost::shared_ptr<UDPConfig> Ptr;
	  	/**
	  	 * The host and remote address.
	  	 */
		  std::string host_address, remote_address;
		  /**
		   * The host and remote ports.
		   */
		  int host_port, remote_port;
	  };



	  /**
	   * The XML input operator for the UDP configuration.
	   *
	   * \param reader The XML reader object.
	   * \param config The UDP configuration.
	   *
	   * \return Returns the received reader object.
	   */
	  inline const labust::xml::Reader& operator>>(const labust::xml::Reader& reader, UDPConfig& config)
	  {
	  	//Set default values
	  	config.host_address = "localhost";
	  	config.remote_address = "localhost";
	  	config.host_port = 22220;
	  	config.remote_port = 22221;

	  	reader.try_value("param[@name='host']/@address",&config.host_address);
	  	reader.try_value("param[@name='host']/@port",&config.host_port);
	  	reader.try_value("param[@name='remote']/@address",&config.remote_address);
	  	reader.try_value("param[@name='remote']/@port",&config.remote_port);

	  	return reader;
	  }

	  /**
	   * Bind socket based on configuration file. Useful when you need the socket
	   * bound to a certain address and port. Typical for client-server usage of UDP.
	   * Throws a boost::system_error.
	   *
	   * \param reader The XML reader configuration.
	   * \param socket_ The UDP socket object to be configured.
	   *
	   * \return Returns a pointer to the loaded UDP configuration.
	   */
	  inline UDPConfig::Ptr udp_configure(const labust::xml::Reader& reader,
		   boost::asio::ip::udp::socket& socket_)
	  {
	  	//read configuration
	  	UDPConfig::Ptr config(new UDPConfig());
	  	reader>>(*config);

	  	using namespace boost::asio::ip;
	  	//Open my socket
	  	socket_.open(udp::v4());
	  	//Create a name resolver
	  	udp::resolver resolver(socket_.get_io_service());
	  	//Resolve query and get the iterator
	  	udp::resolver::iterator it = resolver.resolve(udp::resolver::query(
	  			config->host_address,labust::tools::to_string(config->host_port)));

	  	udp::resolver::iterator end;
	  	//try to connect to and endpoint
	  	boost::system::error_code error = boost::asio::error::host_not_found;
	  	while (error && it != end)
	  	{
	  		socket_.bind(*it++, error);
	  	}
	  	//If we did not succeed to connect throw an exception otherwise print
	  	//the connection information
	  	if (error) {throw boost::system::system_error(error);};
	  	return config;
	  }
	  /**
	   * Bind socket and get remote sender based on configuration file.
	   * Throws a boost::system_error.
	   *
	   * \param reader The XML reader configuration.
	   * \param socket_ The UDP socket object to be configured.
	   * \param remote Remote endpoint to be configured.
	   *
	   * \return Returns a pointer to the loaded UDP configuration.
	   */
	  inline UDPConfig::Ptr udp_configure(const labust::xml::Reader& reader,
		   boost::asio::ip::udp::socket& socket_,
		   boost::asio::ip::udp::endpoint& remote)
	  {
	  	using namespace boost::asio::ip;
	  	UDPConfig::Ptr config = udp_configure(reader,socket_);
	  	//Create a name resolver
	  	udp::resolver resolver(socket_.get_io_service());
	  	//Configure the remote endpoint
	  	udp::resolver::iterator it = resolver.resolve(udp::resolver::query(
	  		config->remote_address, labust::tools::to_string(config->remote_port)));
	  	remote = it->endpoint();
	  	return config;
	  }
	  /**
	   * Get remote sender only from the confguration file. Usefull when you only
	   * need the remote address specified. Typical for client usage of UDP.
	   * Throws a boost::system_error.
	   */
	  inline boost::asio::ip::udp::endpoint udp_configure(const labust::xml::Reader& reader,
		   boost::asio::io_service& io)
	  {
	  	using namespace boost::asio::ip;
	  	UDPConfig config;
	  	reader>>config;
	  	//Create a name resolver
	  	udp::resolver resolver(io);
	  	//Configure the remote endpoint
	  	udp::resolver::iterator it = resolver.resolve(udp::resolver::query(
	  		config.remote_address, labust::tools::to_string(config.remote_port)));
	  	return it->endpoint();
	  }
  }
}
/* UDPCONFIG_HPP_ */
#endif
