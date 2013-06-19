/*
 * TrackerDefs.hpp
 *
 *  Created on: Mar 8, 2011
 *      Author: dnad
 */

#ifndef TRACKERDEFS_HPP_
#define TRACKERDEFS_HPP_

#include <opencv/cv.h>

namespace LABUST
{
  namespace BLUEVIEW
  {
    /**
     * This structure contains sonar head data
     */
    struct SonarHead
    {
      SonarHead():
        heading(0),
        panAngle(0),
        tiltAngle(0)
        {};
      /**
       * Sonar position latitude/longitude in degrees.
       * 3rd coordinate is depth in meters.
       */
      cv::Point3d latlon;
      /**
       * Platform heading
       */
      double heading;
      /**
       * Pan angle
       */
      double panAngle;
      /**
       * Tilt angle
       */
      double tiltAngle;
      /**
       * Sonar range measurement towards target.
       */
      double range;
      /**
       * Sonar bearing measurement towards target.
       */
      double bearing;
      /**
       * Pixel per meter
       */
      double resolution;
    };

    /**
     * This structure contains a image feature definition
     */
    struct TrackedFeature
    {
      /**
       * Realtive feature position in meters
       */
      cv::Point2d position;
      /**
       * Feature world position
       */
      cv::Point3d latlon;
      /**
       * Feature position in pixels
       */
      cv::Point pposition;
      /**
       * Feature area in the image
       */
      double area;
      /**
       * Feature perimeter in the image
       */
      double perimeter;
    };

    /**
     * This structure contains all the line data interesting to us.
     */
    struct LineData
    {
      cv::Point origin;
      cv::Point target;
    };
  }
}

#endif /* TRACKERDEFS_HPP_ */
