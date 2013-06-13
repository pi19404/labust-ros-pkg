#ifndef MICROSTRAINLIB_HPP_
#define MICROSTRAINLIB_HPP_
/*
 * MicrostrainLib.hpp
 *
 *  Created on: May 20, 2011
 *      Author: dnad
 */
#include <LABUSTTypes.h>
#include <initializer_list>
#include <boost/array.hpp>

namespace LABUST
{
  namespace MICROSTRAIN
  {
    /**
     * Contains the GX3 command structures. The following message types exist:
     *
     * Common command message (12):
     *  Request:
     *    1 command byte
     *  Response:
     *    1 command byte
     *    n float values
     *    timer value (float)
     *    checksum
     */
    namespace GX3COMMS
    {
      /**
       * Swaps the bytes. Little-endian to big-endian conversion.
       */
      template <class Type> inline Type byte_swap(Type value)
      {
        Type retVal;
        char* po = reinterpret_cast<char*>(&value);
        char* ps = reinterpret_cast<char*>(&retVal);

        for(int i=0; i<sizeof(Type); ++i)
        {
          ps[sizeof(Type) - i - 1] = po[i];
        }


        return retVal;
      }

      template <LABUST::TYPES::uint8 cmd_byte, size_t data_count, class data_type = float>
      struct GX3Message
      {
        /**
         * The command.
         */
        static const LABUST::TYPES::uint8 command = cmd_byte;
        /**
         * The response buffer.
         */
        boost::array<data_type, data_count> data;
        /**
         * Timer value.
         */
        float timer;
      };

      typedef GX3Message<0xE9,7> ReadFirmwareVersion;
    }
  }
}
/* MICROSTRAINLIB_HPP_ */
#endif
