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
#include <labust/blueview/CConverter.hpp>
#include <labust/blueview/TrackerROI.hpp>
#include <labust/math/uBlasOperations.hpp>

using namespace labust::blueview;

CConverter::CConverter(){};

CConverter::~CConverter(){};

void CConverter::llz2xy(const SonarHead& head, const TrackedFeaturePtr tracklet)
{
  //Remember to update the rotation matrix
  this->update(head);

  //Calculate the X-Y offset
  std::pair<double,double>
   value = deg2meter(tracklet->latlon.x - head.latlon.x,tracklet->latlon.y - head.latlon.y,head.latlon.x);

  //Testing code
  //std::cout<<"Move:"<<value.first<<","<<value.second<<std::endl;

  enum {x = 0,y,z};
  vector pos(3);
  pos(x) = value.first;
  pos(y) = value.second;
  //We assume its in the middle of the beam (A BIG ASSUMPTION)
  pos(z) = tracklet->latlon.z;

  //Calculate to head X-Y-Z coordinates
  matrix inverse;
  labust::math::gjinverse(R,inverse);
  pos = prod(inverse,pos);

  //Update tracklet X-Y position
  tracklet->position.x = pos(x);
  tracklet->position.y = pos(y);
}

void CConverter::xy2llz(const SonarHead& head, const TrackedFeaturePtr tracklet)
{
  //Remember to update the rotation matrix
  this->update(head);

  enum {x = 0,y,z};
  vector pos(3);
  pos(x) = tracklet->position.x;
  pos(y) = tracklet->position.y;
  //We assume its in the middle of the beam (A BIG ASSUMPTION)
  pos(z) = 0;

  pos = prod(R,pos);
  std::pair<double,double> value = meter2deg(pos(x), pos(y), head.latlon.x);

  //Convert to latlon
  tracklet->latlon.x = head.latlon.x + value.first;
  tracklet->latlon.y = head.latlon.y + value.second;
  tracklet->latlon.z = head.latlon.z + pos(z);
}

void CConverter::meter2pixel(const TrackerROI& roi, const TrackedFeaturePtr tracklet)
{
  //std::cout<<"ROI origin:"<<roi->origin.x<<","<<roi->origin.y<<std::endl;
  //std::cout<<"Meter position:"<<(tracklet.position.x)<<","<<(tracklet.position.y)<<std::endl;

  double pxrange = std::sqrt(std::pow(tracklet->position.y,2) + std::pow(tracklet->position.x,2))/roi.headData.resolution;
  double bearing = std::atan2(tracklet->position.y,tracklet->position.x);

  tracklet->pposition.y = roi.origin.y - int(pxrange*std::cos(bearing));
  tracklet->pposition.x = roi.origin.x + int(pxrange*std::sin(bearing));

  //std::cout<<"Pixel position: ("<<tracklet.pposition.x<<","<<tracklet.pposition.y<<")"<<std::endl;
}

void CConverter::pixel2meter(const TrackerROI& roi, const TrackedFeaturePtr tracklet)
{
  //std::cout<<"ROI origin:"<<roi->origin.x<<","<<roi->origin.y<<std::endl;
  //std::cout<<"Pixel position 2:"<<(tracklet.pposition.x)<<","<<(tracklet.pposition.y)<<std::endl;

  tracklet->position.y = (-roi.origin.x + tracklet->pposition.x)*roi.headData.resolution;
  tracklet->position.x = (roi.origin.y - tracklet->pposition.y)*roi.headData.resolution;

  //std::cout<<"Meter position: ("<<tracklet.position.x<<","<<tracklet.position.y<<")"<<std::endl;

  //std::cout<<"Before:"<<tracklet.position.x<<","<<tracklet.position.y<<std::endl;
}

void CConverter::update(const SonarHead& head)
{
	labust::math::rotation_matrix R_hb(0,head.tiltAngle*M_PI/180,head.panAngle*M_PI/180);
	labust::math::rotation_matrix R_bn(0,0,head.heading*M_PI/180);

	this->R = prod(R_bn(),R_hb());
}

std::pair<double,double> CConverter::meter2deg(double x, double y, double lat)
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

std::pair<double,double> CConverter::deg2meter(double difflat, double difflon, double lat)
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
