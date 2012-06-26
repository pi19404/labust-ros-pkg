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
#include <labust/blueview/ImageProcessing.hpp>
#include <labust/blueview/TrackerROI.hpp>
#include <labust/math/uBlasOperations.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#define deg2rad M_PI/180;

using namespace labust::blueview;

BVImageProcessor::BVImageProcessor():
  roi_len(1.5),
  resolution(0),
  hasVehicle(false),
  foundVehicle(false),
  hasTarget(false){};

BVImageProcessor::~BVImageProcessor(){};


bool BVImageProcessor::processROI(TrackerROI& roi)
{
  resolution = roi.headData.resolution;
  if (hasVehicle || foundVehicle)
  {
    std::cout<<"ROI h-p:"<<","<<roi.headData.heading<<","<<roi.headData.panAngle<<std::endl;
    //Calculation of the new position
    //Tracklet position update, calculate new x,y,z based on last known position
    //std::cout<<"Before:"<<std::scientific<<tracklet.position.x<<","<<tracklet.position.y<<std::endl;
    //std::cout<<"Before:"<<std::scientific<<tracklet.latlon.x<<","<<tracklet.latlon.y<<std::endl;
    llz2xy(roi.headData,tracklet);
    meter2pixel(roi,tracklet);

    if (hasTarget)
    {
      llz2xy(roi.headData,target);
      meter2pixel(roi,target);
    }
    std::cout<<"debug point 2"<<std::endl;

    //Where it is expected to be
    cv::Mat disp = roi.roi.clone();

    std::cout<<"debug point 3"<<std::endl;
    cv::circle(disp,getTracklet().pposition,10,cv::Scalar(65536));

    cv::imshow("Expected",disp*500);
    //cv::waitKey(10);

    std::cout<<"Before:"<<std::scientific<<tracklet.pposition.x<<","<<tracklet.pposition.y<<std::endl;
    std::cout<<"After:"<<std::scientific<<tracklet.position.x<<","<<tracklet.position.y<<std::endl;
    //std::cout<<"After:"<<std::scientific<<tracklet.latlon.x<<","<<tracklet.latlon.y<<std::endl;

    //From here things are the same as in the old version
    //Calculate the pixel region (3x3) meters
    roi_len = 3;
    int add = int(roi_len/roi.headData.resolution);

    //    if (roi->roi.size().height<200)
    //    {
    //      roiImg = roi->roi;
    //      roiOffset.x = 0;
    //      roiOffset.y = 0;
    //std::cout<<"No selection"<<std::endl;
    //    }
    //    else
    //    {
    roiImg = roi.roi;
    //roiImg = roi.roi(cv::Rect(tracklet.pposition.x - add,tracklet.pposition.y - add,2*add,2*add));
    roiOffset.x = 0;
    //roiOffset.x = tracklet.pposition.x - add;
    roiOffset.y = 0;
    //roiOffset.y = tracklet.pposition.y - add;
    std::cout<<"ROI width:"<<add<<std::endl;
    //    }

    //Do adjustment
    cv::Mat adjusted = adjust(roiImg);
    cv::imshow("Adjusted",adjusted);
    //cv::waitKey(10);
    //Threshold
    cv::Mat binary = threshold(adjusted);
    cv::imshow("Binary",binary);
    //cv::waitKey(10);
    //Label
    boost::shared_ptr<std::vector<TrackedFeature> > features = label(binary);
    //Associate the results
    bool retVal = associate(features);

    pixel2meter(roi,tracklet);
    xy2llz(roi.headData,tracklet);
    //tracklet.latlon.x += 0.00001;
    //llz2xy(roi.headData,tracklet);

    std::cout<<"After:"<<std::scientific<<tracklet.position.x<<","<<tracklet.position.y<<std::endl;
    std::cout<<"After:"<<std::scientific<<tracklet.latlon.x<<","<<tracklet.latlon.y<<std::endl;
    std::cout<<"After:"<<std::scientific<<tracklet.pposition.x<<","<<tracklet.pposition.y<<std::endl;

    return retVal;

    //exit(0);
  }
  else
  {
    //Do automatic detection on the region
  }

  return false;
}

cv::Mat BVImageProcessor::adjust(cv::Mat& original)
{
  cv::Mat adjust(original.size(),CV_32FC1);
  double min = 0, max = 0;
  cv::minMaxLoc(original,&min,&max);
  original.convertTo(adjust,CV_32FC1,1/max);

  return adjust;
}

cv::Mat BVImageProcessor::threshold(cv::Mat& adjusted)
{
  cv::Mat thresholded(adjusted.size(),CV_8UC1);
  cv::threshold(adjusted, thresholded, 0.5, 255, CV_THRESH_BINARY);
  //thresholded = histthresh(adjusted, 0.3,0.5,255);
  thresholded.convertTo(thresholded,CV_8UC1);

  return thresholded;
}

cv::Mat BVImageProcessor::histthresh(const cv::Mat& img, float min,float max, uchar max_val)
{
    //cv::Mat retVal(img.size(),CV_8U,cv::Scalar(0));
    cv::Mat retVal(img);

    for(int i=1; i<img.size().width-1; ++i)
    {
	for(int j=1; j<img.size().height-1; ++j)
	{
	    cv::Mat thehood = img(cv::Range(i-1,i+1),cv::Range(j-1,j+1));
	    cv::Mat rethood = retVal(cv::Range(i-1,i+1),cv::Range(j-1,j+1));

	    bool flag = false;
	    for (int k=0; k<thehood.size().width; ++k)
	    {
		for (int k2=0; k2<thehood.size().height; ++k2)
		{
		    //std::cout<<"Pixel val:"<<thehood.at<float>(k,k2)<<std::endl;
		    if ((flag = thehood.at<float>(k,k2) >= max)) break;
		}
	    }

	    if (flag)
	    {
		for (int k=0; k<thehood.size().width; ++k)
		{
		    for (int k2=0; k2<thehood.size().height; ++k2)
		    {
			if (thehood.at<float>(k,k2) >= min)
			{
			    rethood.at<float>(k,k2) = max_val;
			}
			else
			{
			    rethood.at<float>(k,k2) = 0;
			}
		    }
		}
	    }
	    else
	    {
		cv::Mat(3,3,CV_8U,cv::Scalar(0)).copyTo(rethood);
	    }
	}
    }

    return retVal;
}

boost::shared_ptr<std::vector<TrackedFeature> > BVImageProcessor::label(cv::Mat& binary)
{
	//This will be replaced with a LABELING algorithm to save execution time
	cv::morphologyEx(binary, binary, cv::MORPH_OPEN,cv::Mat());

	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(binary,contours,hierarchy,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	boost::shared_ptr<std::vector<TrackedFeature> > info(new std::vector<TrackedFeature>);

	for (size_t i = 0; i < contours.size(); ++i)
	{
		std::vector<cv::Point> c_new;
		cv::convexHull(cv::Mat(contours[i]),c_new,true);
		contours[i] = c_new;
		cv::Moments moments = cv::moments(cv::Mat(contours[i]),true);

		TrackedFeature tfeature;
		tfeature.perimeter = cv::arcLength(cv::Mat(contours[i]),true);
		tfeature.area = moments.m00;
		tfeature.pposition.x = (moments.m10)/moments.m00 + roiOffset.x;
		tfeature.pposition.y = (moments.m01)/moments.m00 + roiOffset.y;

		info->push_back(tfeature);
	}

	return info;
}

bool BVImageProcessor::associate(boost::shared_ptr<std::vector<TrackedFeature> > features)
{
 int idx = -1;
 double mindiff = 10000;

 //When its hard to detect anything
 if (features->size()>10) return false;

 for (size_t i=0;i<features->size();++i)
 {
  TrackedFeature& tfeature = (*features)[i];

  double tt_dx = tfeature.pposition.x - target.pposition.x;
  double tt_dy = tfeature.pposition.y - target.pposition.y;
  double tt_distance = sqrt(tt_dx*tt_dx + tt_dy*tt_dy);
   
  double dx = (tfeature.pposition.x) - tracklet.pposition.x;
  double dy = (tfeature.pposition.y) - tracklet.pposition.y;
  double distance = sqrt(dx*dx + dy*dy);
  
  if (!hasTarget) tt_distance = 100;

  //if (tt_distance > distance)
  //{
   if (hasTarget) std::cout<<"Distance to target:"<<tt_distance*resolution<<std::endl;

   if (distance<mindiff)
   {
    mindiff = distance;
    idx = i;
   }
  //}
  //else
  //{
    //std::cout<<"Skipped target."<<std::endl;
  //}
 }

 if (idx != -1)
 {
  tracklet.pposition = (*features)[idx].pposition;
  foundVehicle = true;
  hasVehicle = true;
  return true;
 }
 else
 {
	 return false;
 }
}

void BVImageProcessor::setPosition(const SonarHead& cnt)
{
  //Calculate the position in the sonar image

  tracklet.position.x = cnt.range*cos((cnt.bearing - cnt.panAngle)*M_PI/180);
  tracklet.position.y = cnt.range*sin((cnt.bearing - cnt.panAngle)*M_PI/180);

  //Calculate the world LAT-LON-Z position for the tracklet
  xy2llz(cnt,tracklet);

  //Testing code
  //std::cout<<"Meter position: ("<<tracklet.position.x<<","<<tracklet.position.y<<")"<<std::endl;

  std::cout<<"Contact:"<<tracklet.position.x<<","<<tracklet.position.y<<std::endl;
  std::cout<<"R-b-h-p:"<<cnt.range<<","<<cnt.bearing<<","<<cnt.heading<<","<<cnt.panAngle<<std::endl;

  foundVehicle = hasVehicle = true;
}

void BVImageProcessor::setTarget(const SonarHead& cnt)
{
  //Calculate the position in the sonar image

  target.position.x = cnt.range*cos((cnt.bearing - cnt.panAngle)*M_PI/180);
  target.position.y = cnt.range*sin((cnt.bearing - cnt.panAngle)*M_PI/180);

  //Calculate the world LAT-LON-Z position for the tracklet
  xy2llz(cnt,target);

  //Testing code
  //std::cout<<"Meter position: ("<<tracklet.position.x<<","<<tracklet.position.y<<")"<<std::endl;

  std::cout<<"Target contact:"<<target.position.x<<","<<target.position.y<<std::endl;
  std::cout<<"R-b-h-p:"<<cnt.range<<","<<cnt.bearing<<","<<cnt.heading<<","<<cnt.panAngle<<std::endl;

  hasTarget = true;
}

void BVImageProcessor::llz2xy(const SonarHead& head, TrackedFeature& tracklet)
{
  //Remember to update the rotation matrix
  _update(head);

  //Calculate the X-Y offset
  std::pair<double,double>
   value = deg2meter(tracklet.latlon.x - head.latlon.x,tracklet.latlon.y - head.latlon.y,head.latlon.x);

  //Testing code
  //std::cout<<"Move:"<<value.first<<","<<value.second<<std::endl;

  enum {x = 0,y,z};
  cv::Mat pos_n(3,1,CV_64F);
  pos_n.at<double>(x,0) = value.first;
  pos_n.at<double>(y,0) = value.second;
  //We assume its in the middle of the beam (A BIG ASSUMPTION)
  pos_n.at<double>(z,0) = tracklet.latlon.z;

  std::cout<<"debug point 0"<<std::endl;
  //Calculate to head X-Y-Z coordinates
  using namespace boost::numeric;
  ublas::matrix<double> uR(3,3);
  ublas::vector<double> uPos(3);

  uPos(0) = pos_n.at<double>(0,0);
  uPos(1) = pos_n.at<double>(1,0);
  uPos(2) = pos_n.at<double>(2,0);
  
  uR(0,0) = R.at<double>(0,0);
  uR(0,1) = R.at<double>(0,1);
  uR(0,2) = R.at<double>(0,2);
  uR(1,0) = R.at<double>(1,0);
  uR(1,1) = R.at<double>(1,1);
  uR(1,2) = R.at<double>(1,2);
  uR(2,0) = R.at<double>(2,0);
  uR(2,1) = R.at<double>(2,1);
  uR(2,2) = R.at<double>(2,2);

  ublas::matrix<double> invR(ublas::zero_matrix<double>(3));
  labust::math::gjinverse(uR,invR);
  ublas::vector<double> upos = prod(invR,uPos);

  cv::Mat pos(3,1,CV_64F);
  pos.at<double>(0,0) = upos(0);
  pos.at<double>(1,0) = upos(1);
  pos.at<double>(2,0) = upos(2);
  std::cout<<"debug point 1"<<std::endl;

  //Update tracklet X-Y position
  tracklet.position.x = pos.at<double>(x,0);
  tracklet.position.y = pos.at<double>(y,0);
}

void BVImageProcessor::xy2llz(const SonarHead& head, TrackedFeature& tracklet)
{
  //Remember to update the rotation matrix
  _update(head);

  enum {x = 0,y,z};
  cv::Mat pos(3,1,CV_64F);
  pos.at<double>(x,0) = tracklet.position.x;
  pos.at<double>(y,0) = tracklet.position.y;
  //We assume its in the middle of the beam (A BIG ASSUMPTION)
  pos.at<double>(z,0) = 0;

  //Calculate to world X-Y-Z coordinates
  cv::Mat pos_n = R*pos;
  std::pair<double,double>
  value = meter2deg(pos_n.at<double>(x,0), pos_n.at<double>(y,0), head.latlon.x);

  //Testing code
  //std::cout<<"Meter2Degree"<<value.first<<","<<value.second<<std::endl;

  //Convert to latlon
  tracklet.latlon.x = head.latlon.x + value.first;
  tracklet.latlon.y = head.latlon.y + value.second;
  tracklet.latlon.z = head.latlon.z + pos_n.at<double>(z,0);
}

void BVImageProcessor::meter2pixel(TrackerROI& roi, TrackedFeature& tracklet)
{
  //std::cout<<"ROI origin:"<<roi->origin.x<<","<<roi->origin.y<<std::endl;
  std::cout<<"Meter position:"<<(tracklet.position.x)<<","<<(tracklet.position.y)<<std::endl;

  double pxrange = sqrt(tracklet.position.y*tracklet.position.y + tracklet.position.x*tracklet.position.x) / roi.headData.resolution;
  double bearing = atan2(tracklet.position.y,tracklet.position.x);

  tracklet.pposition.y = roi.origin.y - int(pxrange*cos(bearing));
  tracklet.pposition.x = roi.origin.x + int(pxrange*sin(bearing));

  std::cout<<"Pixel position: ("<<tracklet.pposition.x<<","<<tracklet.pposition.y<<")"<<std::endl;
  //std::cout<<"R-b pixel:"<<pxrange<<","<<bearing<<std::endl;
}

inline void BVImageProcessor::pixel2meter(TrackerROI& roi, TrackedFeature& tracklet)
{
  //std::cout<<"ROI origin:"<<roi->origin.x<<","<<roi->origin.y<<std::endl;
  //std::cout<<"Pixel position 2:"<<(tracklet.pposition.x)<<","<<(tracklet.pposition.y)<<std::endl;

  tracklet.position.y = (-roi.origin.x + tracklet.pposition.x)*roi.headData.resolution;
  tracklet.position.x = (roi.origin.y - tracklet.pposition.y)*roi.headData.resolution;

  //std::cout<<"Meter position: ("<<tracklet.position.x<<","<<tracklet.position.y<<")"<<std::endl;

  //std::cout<<"Before:"<<tracklet.position.x<<","<<tracklet.position.y<<std::endl;
}

void BVImageProcessor::_update(const SonarHead& head)
{
  double c1 = 1, s1 = 0;
  double c2 = cos(head.tiltAngle*M_PI/180), s2 = sin(head.tiltAngle*M_PI/180);
  double c3 = cos(head.panAngle*M_PI/180), s3 = sin(head.panAngle*M_PI/180);

  cv::Mat R_hb(3,3,CV_64F);
  R_hb.at<double>(0,0) = c3*c2;
  R_hb.at<double>(0,1) = c3*s2*s1-s3*c1;
  R_hb.at<double>(0,2) = s3*s1+c3*c1*s2;
  R_hb.at<double>(1,0) = s3*c2;
  R_hb.at<double>(1,1) = c1*c3+s1*s2*s3;
  R_hb.at<double>(1,2) = c1*s2*s3-c3*s1;
  R_hb.at<double>(2,0) = -s2;
  R_hb.at<double>(2,1) = c2*s1;
  R_hb.at<double>(2,2) = c1*c2;

  c1 = 1, s1 = 0;
  c2 = 1, s2 = 0;
  c3 = cos(head.heading*M_PI/180), s3 = sin(head.heading*M_PI/180);

  cv::Mat R_bn(3,3,CV_64F);
  R_bn.at<double>(0,0) = c3;
  R_bn.at<double>(0,1) = -s3;
  R_bn.at<double>(0,2) = 0;
  R_bn.at<double>(1,0) = s3;
  R_bn.at<double>(1,1) = c3;
  R_bn.at<double>(1,2) = 0;
  R_bn.at<double>(2,0) = 0;
  R_bn.at<double>(2,1) = 0;
  R_bn.at<double>(2,2) = 1;

  R = R_bn*R_hb;

 // Testing code
 // for (int i=0;i<3;++i)
 //  for (int j=0; j<3;++j)
 //   std::cout<<R_bn.at<float>(i,j)<<",";
}

std::pair<double,double> BVImageProcessor::meter2deg(double x, double y, double lat)
{
  static const double radius = 6378137;
  static const double ratio = 0.99664719;
  /**
   * These conversion where taken of Wikipedia :)
   * http://en.wikipedia.org/wiki/Geographic_coordinate_system#Cartesian_coordinates
   */
  double mpdlat = 111132.954 - 559.822*cos(2*lat*M_PI/180) + 1.175*cos(4*lat*M_PI/180);
  double mpdlon = M_PI*radius*cos(atan(ratio*tan(lat*M_PI/180)))/180;

  return std::pair<double,double>(x/mpdlat,y/mpdlon);
}

std::pair<double,double> BVImageProcessor::deg2meter(double difflat, double difflon, double lat)
{
  static const double radius = 6378137;
  static const double ratio = 0.99664719;
  /**
   * These conversion where taken of Wikipedia :)
   * http://en.wikipedia.org/wiki/Geographic_coordinate_system#Cartesian_coordinates
   */
  double mpdlat = 111132.954 - 559.822*cos(2*lat*M_PI/180) + 1.175*cos(4*lat*M_PI/180);
  double mpdlon = M_PI*radius*cos(atan(ratio*tan(lat*M_PI/180)))/180;

  return std::pair<double,double>(difflat*mpdlat,difflon*mpdlon);
}
