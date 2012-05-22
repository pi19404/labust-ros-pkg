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
#ifndef GYROS_HPP_
#define GYROS_HPP_

#include <labust/tools/TimingTools.hpp>

#include <string>

namespace labust
{
  namespace xml
  {
    /**
     * This contains the different available types of the Gyros message.
     */
    namespace GYROS
    {
      typedef enum
      {
        undefined = -1,
            error=0,
            scalar,
            matrix,
            dictionary,
            binary
      }
      Type;

      typedef enum
      {
        raw = 0,
                jpg,
                png,
                text
      }
      Encoding;
    };

    /**
     * This class represents the GYROS standardized data format used for data
     * exchange. This class is used as a base class for the GyrosReader and
     * GyrosWriter objects.
     */
    class Gyros
    {
    public:
      /**
       * Sets (or changes) the label of the Gyros object. If no label is defined
       * the label will not be present in the XML signature of the object.
       *
       * \param label Label of the Gyros object.
       */
      inline void SetLabel(const std::string& label){this->label = label;};
      /**
       * Sets the timestamp of the object to the current time or to not used
       * the timestamp consists of the unix timestamp as the integer part
       * and miliseconds after the decimal point
       *
       * \param use if true, timestamp is set to current time, if false, it is removed
       */
      inline void SetTimeStamp(bool use=true){this->timestamp = (use ? labust::tools::unix_time():-1);};

      /**
       * Gets the label of the Gyros object
       */
      inline const std::string& GetLabel() const {return label;};
      /**
       * Gets the UNIX timestamp of the object the timestamp consists of the unix timestamp as the integer part
       * and miliseconds after the decimal point (ssssssss.mmmmmmm).
       */
      inline double GetTimeStamp() const {return timestamp;};
      /**
       * Returns the Gyros object type.
       */
      inline GYROS::Type Type() const {return type;}

    protected:
      /**
       * Generic constructor. Creates a empty Gyros object.
       */
      Gyros():
        timestamp(-1),
        label(""),
        type(GYROS::undefined){};
	  /**
       * Autofill constructor. Creates a empty Gyros object.
       */
      Gyros(const std::string& label, double timestamp, GYROS::Type type):
        timestamp(timestamp),
        label(label),
        type(type){};
      /**
       * Timestamp of the message. If no timestamp was found we set this to -1.
       */
      double timestamp;
      /**
       * Label of the object. If no label was found this is empty.
       */
      std::string label;
      /**
       * Type of the Gyros object.
       */
      GYROS::Type type;
    };
  };
};

/* GYROS_HPP_ */
#endif
