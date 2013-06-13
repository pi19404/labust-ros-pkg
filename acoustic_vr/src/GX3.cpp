/*
 * GX3.cpp
 *
 *  Created on: May 20, 2011
 *      Author: dnad
 */
#include <microstrain/GX3.hpp>
#include <microstrain/MicrostrainException.hpp>
#include <microstrain/MicrostrainLib.hpp>
#include <iostream>
#include <boost/asio.hpp>

using namespace LABUST::MICROSTRAIN;

const size_t GX3::supportedFirmware(0);

GX3::GX3(const std::string& portName, int baud):
    io(),
    port(io)
{
  connect(portName, baud);
  checkFirmware();
  //readSetup();
}

void GX3::connect(const std::string& portName, int baud)
{
  try
  {
    port.open(portName);

    if (port.is_open())
    {
      std::cout<<"Port "<<portName<<" is open."<<std::endl;
      port.set_option(boost::asio::serial_port::baud_rate(baud));
      port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
      port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
      port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
      port.set_option(boost::asio::serial_port::character_size(8));
    }
    else
    {
      throw MicrostrainException("Unable to open port.");
    }
  }
  catch (std::exception& e)
  {
    std::cout<<"Failed during port setup : "<<e.what()<<std::endl;
    throw e;
  }
}

void GX3::checkFirmware()
{
  using namespace GX3COMMS;
  //Get firmware message
 /* ReadFirmwareVersion firmware;
  boost::asio::write(port,boost::asio::buffer(&firmware.command,1));
  boost::asio::read(port,boost::asio::buffer(firmware.response));

  std::cout<<firmware.response<<std::endl;*/
}

double GX3::heading()
{
  unsigned char req = 0xCE;
  boost::asio::write(port, boost::asio::buffer(&req,1));
  boost::asio::read(port, boost::asio::buffer(&req,1));
  float data[4];
  boost::asio::read(port, boost::asio::buffer(data,sizeof(data)));
  short int chks = 0;
  boost::asio::read(port,boost::asio::buffer(&chks,sizeof(chks)));

  return GX3COMMS::byte_swap(data[2]);
}
