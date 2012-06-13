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
#ifndef TRACKERFWD_HPP_
#define TRACKERFWD_HPP_
#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>

namespace labust
{
  namespace blueview
  {
    /**
     * The general sonar head data.
     */
    struct SonarHead
    {
      /**
       * Sonar geographical world position. Latitude/longitude in degrees and depth in meters.
       */
      cv::Point3d latlon;
      /**
       * Sonar platform heading.
       */
      double heading;
      /**
       * Sonar pan and tilt angle.
       */
      double panAngle, tiltAngle;
      /**
       * Sonar range and bearing measurement towards target.
       */
      double range, bearing;
      /**
       * Sonar image resolution.
       */
      double resolution;
    };

    /**
     * The tracked feature parameters.
     */
    struct TrackedFeature
    {
      /**
       * Relative 2D feature position in meters
       */
      cv::Point2d position;
      /**
       * Feature world position.
       */
      cv::Point3d latlon;
      /**
       * Feature pixel position.
       */
      cv::Point pposition;
      /**
       * Feature area and perimeter in the image.
       */
      double area, perimeter;
    };
    /**
     * The TrackedFeature pointer.
     */
    typedef boost::shared_ptr<TrackedFeature> TrackedFeaturePtr;
    /**
     * Tracked feature vector.
     */
    typedef std::vector<TrackedFeature> TrackedFeatureVec;
    /**
     * The tracked feature vector pointer.
     */
    typedef boost::shared_ptr< TrackedFeatureVec > TrackedFeatureVecPtr;


    /**
     * The class implements the tracker Region-of-Interest(ROI).
     */
    class TrackerROI;
    /**
     * The TrackerROI pointer.
     */
    typedef boost::shared_ptr<TrackerROI> TrackerROIPtr;

    /**
     * The image processing class.
     */

    /**
     * This structure contains all the line data interesting to us.
     */
    /*struct LineData
    {
      cv::Point origin;
      cv::Point target;
    };*/

    /**
     * The OpenCV data pointer.
     */
    typedef boost::shared_ptr<cv::Mat> MatPtr;
  }
}
/* TRACKERFWD_HPP_ */
#endif
