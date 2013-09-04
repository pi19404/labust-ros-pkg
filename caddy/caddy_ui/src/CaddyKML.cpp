/*
 * RTKML.cpp
 *
 *  Created on: Feb 10, 2011
 *      Author: dnad
 */
#include <labust/gearth/CaddyKML.hpp>
#include <kml/dom.h>
#include <iostream>
#include <math.h>
#include <labust/tools/GeoUtilities.hpp>
#include <kml/engine/feature_visitor.h>
#include <sstream>
#include <string>
#include <iostream>

#include <ros/ros.h>

using namespace labust::gearth;

CaddyKML::CaddyKML():
  factory(kmldom::KmlFactory::GetFactory()),
  filename("caddy_rt.kml"),
  kmlFile(filename.c_str(),std::fstream::out)
{
	ros::NodeHandle ph("~");
	std::string name("RefreshLink.kml");
	ph.param("KMLFileName",filename,filename);
	ph.param("RefreshLinkName",name,name);
  double refreshInterval(1);
  ph.param("RefreshInterval",refreshInterval,refreshInterval);

	labust::gearth::writeLinkKML(name, filename,refreshInterval);
	ROS_INFO("Create the Google Network File.");

	kmlFile.open(filename.c_str(),std::fstream::out);
  kmlFile.close();
  ROS_INFO("Create the KML file.");

  std::string vcolor("0xFF0000FF");
  ph.param("diver/path/color",vcolor,vcolor);
  //accomodate google earth format
  vcolor.erase(0,2);
  vehicle.setColor(vcolor);
  path.setColor(vcolor);

  int max_elements(200);
  ph.param("diver/path/length",max_elements,max_elements);
  path.setMaxElements(max_elements);

  int segments(20);
  ph.param("diver/path/segment_length",segments,segments);
  path.setSegmentSize(segments);
  path.setTransparencyDecrement(uint8_t(256/(max_elements/segments)));
  //std::cout<<"Length: "<<max_elements<<" Segment "<<segments<<std::endl;

  std::string scolor("0xFFFF0000");
  ph.param("platform/path/color",scolor,scolor);
  //accomodate google earth format
  scolor.erase(0,2);
  ship.setColor(scolor);

  double len(10);
  ph.param("platform/length",len,len);
  vehicle.setId("Vehicle");
  vehicle.setLength(len);
  ph.param("platform/length",len,10.0);
  ship.setId("Ship");
  ship.setLength(len);
}

CaddyKML::~CaddyKML(){};

void CaddyKML::write()
{
  using namespace kmldom;
  //Create empty folder
  folder = factory->CreateDocument();
  vehicle.addToDocument(folder);
  ship.addToDocument(folder);
  path.addToDocument(folder);

  folder->set_name("Real-Time tracking");
  KmlPtr kml(factory->CreateKml());
  kml->set_feature(folder);

  kmlFile.open(filename.c_str(),std::fstream::out);
  if (kmlFile.is_open())
  {
    kmlFile<<kmldom::SerializePretty(kml);
    kmlFile.close();
  }
  else
  {
    std::cerr<<"Cannot open kmlfile."<<std::endl;
  }
}

