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
#ifndef GEOUTILITIES_HPP_
#define GEOUTILITIES_HPP_
#define _USE_MATH_DEFINES
#include <utility>
#include <math.h>

namespace labust
{
	/**
	 * \todo Add Vincenty formula: http://en.wikipedia.org/wiki/Vincenty's_formulae
	 */
	namespace tools
  {
    static const double deg2rad = M_PI/180;
    static const double rad2deg = 180/M_PI;
    static const double radius = 6378137;
    static const double ratio = 0.99664719;

    /**
     * The function calculates meters per latitude degree.
     * The conversion was taken from:
     * http://en.wikipedia.org/wiki/Geographic_coordinate_system#Cartesian_coordinates
     */
    inline double mpdlat(double lat)
    {
      return (111132.954 - 559.822*cos(2*lat*deg2rad) + 1.175*cos(4*lat*deg2rad));
    };
    /**
     * The function calculates meters per longitude degree.
     * The conversion was taken from:
     * http://en.wikipedia.org/wiki/Geographic_coordinate_system#Cartesian_coordinates
     */
    inline double mpdlon(double lat)
    {
      return (radius*cos(atan(ratio*tan(lat*deg2rad)))*deg2rad);
    };
    /**
     * The function converts degrees to meters.
     *
     * \param difflat North distance in degrees.
     * \param difflon East distance in degrees.
     * \param lat Latitude position in decimal degrees.
     *
     * \return Returns the latitude and longitude distance in relative meters.
     */
    inline std::pair<double,double> deg2meter(double difflat, double difflon, double lat)
    {
      return std::pair<double,double>(difflat*mpdlat(lat),difflon*mpdlon(lat));
    }
    /**
      * The function converts meters to relative degrees.
      *
      * \param x North distance in meters.
      * \param y East distance in meters.
      * \param lat Latitude position in decimal degrees.
      *
      * \return Returns the relative angles of the distances.
      */
    inline std::pair<double,double> meter2deg(double x, double y, double lat)
    {
      return std::pair<double,double>(x/mpdlat(lat),y/mpdlon(lat));
    };
  }
}
/* GEOUTILITIES_HPP_ */
#endif
