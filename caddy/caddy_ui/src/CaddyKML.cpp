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
  if (opRegion) addOpRegionToDocument(folder);

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

void CaddyKML::addOpRegionToDocument(kmldom::DocumentPtr document)
{
  //Create the polygon
  kmldom::LinearRingPtr ring(factory->CreateLinearRing());
  kmldom::CoordinatesPtr coords(factory->CreateCoordinates());
  //Copy coordinates
  for (int i=0; i<opRegion->get_coordinates_array_size();++i)
  	coords->add_vec3(opRegion->get_coordinates_array_at(i));

  ring->set_coordinates(coords);
  kmldom::OuterBoundaryIsPtr boundary(factory->CreateOuterBoundaryIs());
  boundary->set_linearring(ring);
  kmldom::PolygonPtr poly(factory->CreatePolygon());
  poly->set_outerboundaryis(boundary);

  //Create the polygon
  kmldom::PolyStylePtr poly_style(factory->CreatePolyStyle());
  poly_style->set_fill(false);
  //kmldom::LineStylePtr line_style(factory->CreateLineStyle());
  //line_style->set_color(vehicle_color);
  kmldom::StylePtr style(factory->CreateStyle());
  style->set_polystyle(poly_style);
  //style->set_linestyle(line_style);
  style->set_id("RegionPolygonStyle");

  kmldom::PlacemarkPtr region(factory->CreatePlacemark());
  region->set_name("Operating region");
  region->set_geometry(poly);
  region->set_styleurl("#RegionPolygonStyle");
  document->add_feature(region);
  document->add_styleselector(style);
}

void CaddyKML::setDiverOrigin(const geometry_msgs::Point::ConstPtr& point)
{
	//Do this just on change
	if ((point->x != diverOrigin.get_latitude()) ||
			(point->y != diverOrigin.get_longitude()))
	{
		diverOrigin = kmlbase::Vec3(point->y, point->x, 0);
		opRegion.reset(factory->CreateCoordinates());
		for(double i=0; i<2*M_PI; i+=0.1)
		{
			std::pair<double, double> latlon=
					labust::tools::meter2deg(kml_radius*cos(i),
							kml_radius*sin(i),
							diverOrigin.get_latitude());
			opRegion->add_latlng(point->x + latlon.first,
					point->y + latlon.second);
		}
		//connect the poly
		opRegion->add_vec3(opRegion->get_coordinates_array_at(0));
	}
}

