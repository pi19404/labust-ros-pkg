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
#ifndef CCONVERTER_HPP_
#define CCONVERTER_HPP_
#include <labust/blueview/trackerfwd.hpp>
#include <labust/math/Rotation.hpp>

#include <opencv2/core/core.hpp>

namespace labust
{
  namespace blueview
  {
    /**
     * The class contains conversions between frames needed for BlueView image processing.
     *
     * \todo Drop the conversion stuff into hpp for inline.
     */
    class CConverter
    {
    	typedef boost::numeric::ublas::vector<double> vector;
    	typedef labust::math::rotation_matrix::matrix matrix;
    public:
      /**
       * Generic constructor.
       */
      CConverter();
      /**
       * Generic destructor.
       */
      ~CConverter();

      /**
       * This method converts the estimated world lat-lon-z feature coordinates to the
       * in x-y coordianates in the sonar head.
       *
       * \param head The sonar head information.
       * \param tracklet The tracked feature information.
       */
      void llz2xy(const SonarHead& head, const TrackedFeaturePtr tracklet);
      /**
       * This method converts the features x-y coordinates in the sonar head
       * to the world lat-lon-z.
       *
       * \param head The sonar head information
       * \param tracklet The tracked feature information
       */
      void xy2llz(const SonarHead& head, const TrackedFeaturePtr tracklet);

      /**
       * Meters to degree conversion.
       *
       * \param x North distance in meters
       * \param y East distance in meters
       * \param lat Latitude position in decimal degrees
       *
       * \return Returns pair of lat,lon degrees
       */
      static std::pair<double,double> meter2deg(double x, double y, double lat);
      /**
       * Degree to meter conversion.
       *
       * \param difflat North distance in degree
       * \param difflon East distance in degree
       * \param lat Latitude position in decimal degrees
       *
       * \return Returns pair of lat,lon degrees
       */
      static std::pair<double,double> deg2meter(double difflat, double difflon, double lat);

      /**
       * Calculate the pixel position based on ROI information and sonar head X-Y position.
       *
       * \param roi The region of interest.
       * \param tracklet The tracked object.
       */
      static void meter2pixel(const TrackerROI& roi, const TrackedFeaturePtr tracklet);
      /**
       * Calculate the pixel position back to sonar head X-Y position.
       *
       * \param roi The region of interest.
       * \param tracklet The tracked object.
       */
      static void pixel2meter(const TrackerROI& roi, const TrackedFeaturePtr tracklet);

    private:
      /**
       * This method updates the rotation matrix NED->XYZ conversion
       */
      void update(const SonarHead& head);

      /**
       * Rotation matrix NED->XYZ.
       */
      matrix R;
    };
  }
}

#endif /* TRACKERROI_HPP_ */
