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
#ifndef TRACKERROI_HPP_
#define TRACKERROI_HPP_
#include <labust/blueview/trackerfwd.hpp>

namespace labust
{
  namespace blueview
  {
    /**
     * The class implements the tracker Region-of-Interest(ROI) used in the NURC sonar
     * image processing applications.
     */
    class TrackerROI
    {
    public:
    	/**
    	 * Generic constructor.
    	 */
    	TrackerROI();
      /**
       * Main constructor. Constructs the TracerROI from the XML data.
       *
       * \param str The XML encoded string.
       */
      TrackerROI(const std::string& str);
      /**
       * Generic destructor.
       */
      ~TrackerROI();

      /**
       * Converts the current Tracker ROI information to a XML encoded version.
       *
       * \param object The Tracker ROI object.
       * \param str The string object address.
       *
       * \return Returns true if serialization is successful, false otherwise.
       */
      static bool serialize(const TrackerROI& object, std::string* str);
      /**
       * Converts the XML string to the Tracker ROI object.
       *
       * \param str String containing XML data.
       * \param object The TrackerROI object address.
       *
       * \return Returns true if serialization is successful, false otherwise.
       */
      static bool deserialize(const std::string& str, TrackerROI* const object);

      /**
       * The ROI size.
       */
      cv::Size size;
      /**
       * The ROI Sonar head information.
       */
      SonarHead headData;
      /**
       * Origin of the coordinate system where the ROI is taken.
       */
      cv::Point origin;
      /**
       * The ROI image.
       */
      cv::Mat roi;

    protected:
      /**
       * The method decodes the pixel information from the string.
       */
      void _decodePixels(std::string& str);
      /**
       * Separate encoding policy
       */
      inline const char* _encodePixels(char* dst);
    };
  }
}

#endif /* TRACKERROI_HPP_ */
