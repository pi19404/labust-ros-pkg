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
#ifndef IMAGEPROCESSING_HPP_
#define	IMAGEPROCESSING_HPP_
#include <labust/blueview/trackerfwd.hpp>

namespace labust
{
  namespace blueview
  {
    /**
     * The class incorporates the image processing for the NURC BlueView tracking
     * system.
     */
    class BVImageProcessor
    {
    public:
      /**
       * Generic constructor.
       */
      BVImageProcessor();
      /**
       * Generic destructor.
       */
      ~BVImageProcessor();

      /**
       * Main method for processing an ROI. Currently no automatic detection is
       * supported. You need to set the initial vehicle position.
       *
       * \param roi The ROI object.
       */
      bool processROI(TrackerROI& roi);

      /**
       * Set manually the inital vehicle contact.
       *
       * \param cnt Conntact information in the SonarHead structure
       */
      void setPosition(const SonarHead& cnt);
      /**
       * Set manually the inital vehicle contact.
       *
       * \param cnt Conntact information in the SonarHead structure
       */
      void setTarget(const SonarHead& cnt);

      /**
       * Checks if the tracker has found the vehicle
       */
      inline bool isTracking(){return foundVehicle;};

      /**
       * Return the info about the tracked feature
       */
      inline TrackedFeature getTracklet(){return tracklet;};

    private:
      /**
       * Hysteresis threshold
       *
       * \param img Image to be processed
       * \param min Minimum threshold
       * \param max Maximum threshold
       * \param max_val Value to assign
       */
      cv::Mat histthresh(const cv::Mat& img, float min, float max, uchar max_val);
      /**
       * This method does the prefiltering
       */
      cv::Mat adjust(cv::Mat& original);
      /**
       * This method does the thresholding
       */
      cv::Mat threshold(cv::Mat& adjusted);
      /**
       * This method does the analysis
       */
      boost::shared_ptr<std::vector<TrackedFeature> > label(cv::Mat& binary);
      /**
       * Associate detections with one or more targets
       * \param features Features found in the image
       */
      bool associate(boost::shared_ptr<std::vector<TrackedFeature> > features);
      /**
       * This method updates the rotation matrix NED->XYZ conversion
       */
      void _update(const SonarHead& head);
      /**
       * This method converts the features x-y coordinates in the sonar head
       * to the world lat-lon-z.
       *
       * \param head The sonar head information
       * \param tracklet The tracked feature information
       */
      void xy2llz(const SonarHead& head, TrackedFeature& tracklet);
      /**
       * This method converts the estimated world lat-lon-z feature coordinates to the
       * in x-y coordianates in the sonar head.
       *
       * \param head The sonar head information
       * \param tracklet The tracked feature information
       */
      void llz2xy(const SonarHead& head, TrackedFeature& tracklet);

      /**
       * Meters to degree conversion.
       *
       * \param x North distance in meters
       * \param y East distance in meters
       * \param lat Latitude position in decimal degrees
       *
       * \return Returns pair of lat,lon degrees
       */
      std::pair<double,double> meter2deg(double x, double y, double lat);
      /**
       * Degree to meter conversion.
       *
       * \param difflat North distance in degree
       * \param difflon East distance in degree
       * \param lat Latitude position in decimal degrees
       *
       * \return Returns pair of lat,lon degrees
       */
      std::pair<double,double> deg2meter(double difflat, double difflon, double lat);

      /**
       * Caluclate the pixel position based on ROI information and sonar head X-Y position
       */
      inline void meter2pixel(TrackerROI& roi, TrackedFeature& tracklet);
      /**
       * Caluclate the pixel position back to sonar head X-Y position
       */
      inline void pixel2meter(TrackerROI& roi, TrackedFeature& tracklet);

      /**
       * Tracked feature information
       */
      TrackedFeature tracklet;
      /**
       * Target feature information
       */
      TrackedFeature target;
      /**
       * Internal roi
       */
      cv::Mat roiImg;
      /**
       * Internal roi offset
       */
      cv::Point roiOffset;
      /**
       * Size of the region of interest
       */
      double roi_len;
      /**
       * Meter resolution
       */
      double resolution;
      /**
       * Rotation matrix NED->XYZ
       */
      cv::Mat R;
      /**
       * Flag to indicate vehicle set point
       */
      bool hasVehicle;
      /**
       * Flag to indicate processing found the vehicle
       */
      bool foundVehicle;
      /**
       * Target found indicator.
       */
      bool hasTarget;
    };
  }
}



#endif	/* IMAGEPROCESSING_HPP_ */

