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
#ifndef SERIALCONFIG_HPP_
#define SERIALCONFIG_HPP_
#include <labust/xml/XMLReader.hpp>

#include <boost/asio/serial_port.hpp>
#include <boost/algorithm/string.hpp>

namespace labust
{
  namespace comms
  {
    /**
     * The serial port configuration options.
     */
    struct SerialConfig
    {
    	/**
    	 * Pointer typedef.
    	 */
    	typedef boost::shared_ptr<SerialConfig> Ptr;
    	/**
    	 * Serial port name.
    	 */
      std::string portName;
      /**
       * Baud rate and data bits count.
       */
      int baud, dataBits;
      /**
       * Flow control options.
       */
      boost::asio::serial_port::flow_control::type flowControl;
      /**
       * Serial port parity.
       */
      boost::asio::serial_port::parity::type parity;
      /**
       * Stop bits count.
       */
      boost::asio::serial_port::stop_bits::type stopBits;
    };

    /**
     * The XML input operator for the serial configuration.
     *
     * \
     */
    inline const labust::xml::Reader& operator>>(const labust::xml::Reader& reader, SerialConfig& config)
    {
    	using namespace boost::asio;
      reader.value("param[@name='PortName']/@value",&config.portName);
      reader.value("param[@name='BaudRate']/@value",&config.baud);

      //Set defaults
      config.flowControl = serial_port::flow_control::none;
      config.parity = serial_port::parity::none;
      config.stopBits = boost::asio::serial_port::stop_bits::one;
      config.dataBits = 8;

      std::string temp;
      if (reader.try_value("param[@name='FlowControl']/@value",&temp))
      {
        if (boost::iequals(temp, "software"))
        {
          config.flowControl = boost::asio::serial_port::flow_control::software;
        }
        else if (boost::iequals(temp, "hardware"))
        {
          config.flowControl = boost::asio::serial_port::flow_control::hardware;
        }
      };

      if (reader.try_value("param[@name='StopBits']/@value",&temp))
      {
        if (boost::iequals(temp, "odd"))
        {
          config.parity = boost::asio::serial_port::parity::odd;
        }
        else if (boost::iequals(temp, "even"))
        {
          config.parity = boost::asio::serial_port::parity::even;
        }
      };

      if (reader.try_value("param[@name='StopBits']/@value",&temp))
       {
         if (boost::iequals(temp, "1.5"))
         {
           config.stopBits = boost::asio::serial_port::stop_bits::onepointfive;
         }
         else if (boost::iequals(temp, "2"))
         {
           config.stopBits = boost::asio::serial_port::stop_bits::two;
         }
       };

      if (!reader.try_value("param[@name='DataBits']/@value",&config.dataBits))
       {
         config.dataBits = 8;
       };

    	return reader;
    }

    /**
     * Bind socket based on configuration file.
     */
    inline SerialConfig::Ptr serial_configure(const labust::xml::Reader& reader,
        boost::asio::serial_port& port)
    {
      using namespace boost::asio;
      SerialConfig::Ptr config(new SerialConfig());
      //Read configuration
      (reader)>>*config;
      //Open port
      port.open(config->portName);

      if (port.is_open())
      {
        port.set_option(serial_port::baud_rate(config->baud));
        port.set_option(serial_port::flow_control(config->flowControl));
        port.set_option(serial_port::parity(config->parity));
        port.set_option(serial_port::stop_bits(config->stopBits));
        port.set_option(serial_port::character_size(config->dataBits));
      }

      return config;
    }
  }
}
/* SERIALCONFIG_HPP_ */
#endif
