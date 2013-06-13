

#ifndef GX3_HPP_
#define GX3_HPP_
/*
 * GX3.hpp
 *
 *  Created on: May 20, 2011
 *      Author: dnad
 */
#include <string>
#include <boost/asio/serial_port.hpp>

namespace LABUST
{
  namespace MICROSTRAIN
  {
    /**
     * This class implements the class abstraction for the 3DM-GX3-25 AHRS sensor.
     */
    class GX3
    {
      /**
       * Supported firmware version.
       */
      static const size_t supportedFirmware;
    public:
      /**
       * The GX3 operation modes.
       */
      typedef enum {Continuous, Active, Idle} Mode;
      /**
       * Main constructor. Opens the sensor at the desired port.
       *
       * \param portName Name of the serial port for the GX3 device.
       * \param baud Baudrate to use for the communication. Defaults to 115200.
       */
      GX3(const std::string& portName, int baud = 115200);

      /**
       * Returns a the mode of operation.
       */
      inline Mode mode(){return mode_;};
      /**
       * Sets the desired mode of operation.
       */
      inline void mode(Mode mode){this->mode_ = mode;};

      /**
       * Added preliminary for testing
       */
      double heading();

    private:
      /**
       * This performs connection to the device.
       *
       * \param portName Name of the serial port for the GX3 device.
       * \param baud Baudrate to use for the communication. Defaults to 115200.
       */
      void connect(const std::string& portName, int baud);
      /**
       * Read and check the device firmware.
       */
      void checkFirmware();
      /**
       * Read the configuration of the device.
       */
      void readConfiguration();

      /**
       * Input/output service for the serial port.
       */
      boost::asio::io_service io;
      /**
       * Serial port for the device.
       */
      boost::asio::serial_port port;

      /**
       * Mode of operation
       */
      Mode mode_;
    };

    /*
    struct gx3read
    {
      template <class message>
      void operator()(boost::asio::serial_port& port, message& msg)
      {
        boost::asio::write(port,);
      }
    };*/


  }
}

/* GX3_HPP_ */
#endif
