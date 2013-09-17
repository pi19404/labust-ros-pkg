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
#include <kml/engine.h>
#include <kml/dom.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>

#include <boost/bind.hpp>

#include <fstream>

// This ParserObserver inhibits adding of any Feature to the DOM.  A bounding
// box of all features is maintained.
class PlacemarkFilter : public kmldom::ParserObserver {
 public:
	PlacemarkFilter():
		msgptr(new std_msgs::Float64MultiArray()){}

  virtual bool NewElement(const kmldom::ElementPtr& element)
  {
  	if (kmldom::PlacemarkPtr placemark = kmldom::AsPlacemark(element))
  		placemarks.push_back(placemark);
    return true;
  }

  inline void processPlacemarks()
  {
  	for(int i=0; i<placemarks.size(); ++i) processPlacemark(placemarks[i]);
  }

  void processPlacemark(kmldom::PlacemarkPtr placemark)
  	{
  		ROS_INFO("Found placemark %s",placemark->get_name().c_str());
  		kmldom::GeometryPtr geom = placemark->get_geometry();
  		kmldom::CoordinatesPtr coordinates;
  		if (kmldom::PointPtr point = kmldom::AsPoint(geom))
  		{
  			coordinates = point->get_coordinates();
  			ROS_INFO("Placemark type is point:");
  		}
  		else if (kmldom::LineStringPtr lstring = kmldom::AsLineString(geom))
  		{
  			ROS_INFO("Placemark type is line string.");
  			coordinates = lstring->get_coordinates();
  		}
  		else
  		{
  			ROS_INFO("Placemark type is unknown.");
  		}

  		for (int i=0; i<coordinates->get_coordinates_array_size(); ++i)
  		{
  			kmlbase::Vec3 point = coordinates->get_coordinates_array_at(i);
  			msgptr->data.push_back(point.get_latitude());
  			msgptr->data.push_back(point.get_longitude());
  		}
  }

  std::vector < kmldom::PlacemarkPtr > placemarks;
  std_msgs::Float64MultiArrayPtr msgptr;
};

void readKml(std::ifstream& input_file, std::string& kmlfile)
{
  int n = 0;
  do
  {
  	char buffer[100];
  	n = input_file.readsome(buffer,sizeof(buffer));
  	kmlfile.append(buffer,n);
  }
  while ((n != 0) && (!input_file.eof()));
}

void onUploadRequest(ros::Publisher& kmlout, const std_msgs::String::ConstPtr& path)
{
  std::ifstream input_file(path->data.c_str(), std::ios_base::in|std::ios_base::binary);
  if (!input_file.is_open() || !input_file.good())
  {
  	ROS_ERROR("Failed to open file: %s", path->data.c_str());
  	return;
  }

  std::string kmlfile;
  readKml(input_file,kmlfile);
  ROS_INFO("Loaded kml file %s",path->data.c_str());

  //Parse the kml file
  kmldom::Parser parser;
  PlacemarkFilter filter;
  parser.AddObserver(&filter);
  std::string errors;
  kmldom::ElementPtr root = parser.Parse(kmlfile,&errors);

  if (!root)
  {
    ROS_ERROR("Kml parse errors: %s",errors.c_str());
    return;
  }
  //Iterate over kml points.
  filter.processPlacemarks();
  //Publish the multiarray.
  kmlout.publish(filter.msgptr);
}

///\todo Edit the class loading to be loaded from the rosparam server.
int main(int argc, char* argv[])
{
	ros::init(argc,argv,"kml_upload");
	ros::NodeHandle nh;

	ros::Publisher kmlout = nh.advertise<std_msgs::Float64MultiArray>("kml_array",1);
	ros::Subscriber platform = nh.subscribe<std_msgs::String>("kml_file",
			1, boost::bind(&onUploadRequest,boost::ref(kmlout),_1));

	ros::spin();
	return 0;
}





