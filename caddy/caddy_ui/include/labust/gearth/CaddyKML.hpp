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
 *
 *  Author: Dula Nad
 *  Created: 01.02.2013.
 *********************************************************************/
#ifndef CADDYKML_HPP_
#define CADDYKML_HPP_

#include <fstream>
#include <deque>
#include <labust/gearth/GoogleEarthLib.hpp>
#include <labust/gearth/CustomLineString.hpp>
#include <labust/gearth/VehiclePolygon.hpp>

#include <ros/ros.h>
#include <auv_msgs/NavSts.h>

#include <boost/thread/mutex.hpp>

namespace labust
{
  namespace gearth
  {
    /**
     * This class represents an interactive KML file. It opens and writes
     * into a KML file. It will remember a specified number of coordinates.
     * Around each last coordinate it will draw the vehicle position.
     *
     * The YAML configuration should have a form:
     *
     * filename: myname.kml
     * diver-path/color="0xAABBGGRR"
     * diver-path/length="200"
     * diver-path/segment-length="20"
     * platform-path/color="0xAABBGGRR"
     */
    class CaddyKML
    {
    public:
      /**
       * Generic constructor.
       */
      CaddyKML();
      /**
       * Generic destructor.
       */
      ~CaddyKML();

      /**
       * Add a new coordinate and attitude of the vehicle.
       *
       * \param nav Navigation data.
       */
      void addVehiclePosition(const auv_msgs::NavSts::ConstPtr& nav)
      {
      	kmlbase::Vec3 position(nav->global_position.longitude,
        		nav->global_position.latitude,
        		-nav->position.depth);

      	boost::mutex::scoped_lock l(kml_mux);
        path.addPoint(position);
        vehicle.updatePosition(position,nav->orientation.yaw);
        diver= *nav;
        this->write();
      }
      /**
       * Add a new coordinate of the ship.
       *
       * \param nav Navigation data.
       */
      void addShipPosition(const auv_msgs::NavSts::ConstPtr& nav)
      {
      	boost::mutex::scoped_lock l(kml_mux);
        ship.updatePosition(kmlbase::Vec3(nav->global_position.longitude,
        		nav->global_position.latitude,
        		-nav->position.depth),nav->orientation.yaw);
        platform = *nav;
        this->write();
      }
      /**
       * Update the diver origin.
       */
      void setDiverOrigin(const geometry_msgs::Point::ConstPtr& point);
      /**
       * Write data to the KML file.
       */
      void write();

    private:
      /**
       * Helper function to add the operating region to the KML file.
       */
      void addOpRegionToDocument(kmldom::DocumentPtr document);
      /**
       * Helper function to add the variance region.
       */
      void addVarianceRegion(const auv_msgs::NavSts& nav, kmldom::DocumentPtr document);
      /**
       * Vehicle path
       */
      CustomLineString<TransparentSegments> path;
      /**
       * Vehicle polygon.
       */
      VehiclePolygon vehicle;
      /**
       * Ship polygon.
       */
      VehiclePolygon ship;
      /**
       *	The acoustic KML operating region.
       */
      kmldom::CoordinatesPtr opRegion;
      /**
       * The diver origin.
       */
      kmlbase::Vec3 diverOrigin;
      /**
       * The KML factory for creating KML files.
       */
      kmldom::KmlFactory* factory;
      /**
       * Document that holds all components together.
       */
      kmldom::DocumentPtr folder;
      /**
       * KML filename
       */
      std::string filename;
      /**
       * The KML file.
       */
      std::ofstream kmlFile;
      /**
       * The write mutex.
       */
      boost::mutex kml_mux;
      /**
       * The navigation stats.
       */
      auv_msgs::NavSts diver, platform;
    };
  }
}
/* CADDYKML_HPP_ */
#endif
