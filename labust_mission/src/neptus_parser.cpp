/*
 * neptus_parser.cpp
 *
 *  Created on: Apr 3, 2014
 *      Author: Filip Mandic
 */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, LABUST, UNIZG-FER
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

#include <iostream>
#include <string>
#include <cstddef>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TransformStamped.h>
#include <auv_msgs/NED.h>
#include <std_msgs/String.h>
#include <labust/tools/conversions.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <tinyxml2.h>

#include <labust_mission/maneuverGenerator.hpp>

//enum {none = 0, go2point_FA, go2point_UA, dynamic_positioning, course_keeping_FA, course_keeping_UA};


using namespace std;
using namespace tinyxml2;
using namespace utils;


struct LatLon2NED {
	LatLon2NED(){
		//ros::NodeHandle nh;
	}

	void convert(sensor_msgs::NavSatFix LatLon){
		//Calculate to X-Y tangent plane
		//geometry_msgs::TransformStamped transformDeg, transformLocal;
		//try
		//{
//			transformLocal = buffer.lookupTransform("local", "gps_frame", ros::Time(0));
//			transformDeg = buffer.lookupTransform("worldLatLon", "local", ros::Time(0));
//			posxy =	labust::tools::deg2meter(LatLon.latitude - transformDeg.transform.translation.y,
//						LatLon.longitude - transformDeg.transform.translation.x,
//						transformDeg.transform.translation.y);
			posxy =	labust::tools::deg2meter(LatLon.latitude - 44.00,
						LatLon.longitude - 13.00 ,
						13.00);
//
//			originLL.first = transformDeg.transform.translation.y;
//			originLL.second = transformDeg.transform.translation.x;
//
//			posLL.first = LatLon.latitude;
//			posLL.second = LatLon.longitude;



			//return posxy;

			//isNew = true;
	//	}
//		catch(tf2::TransformException& ex)
//		{
//			ROS_WARN("Unable to decode GPS measurement. Missing frame : %s",ex.what());
//		}
	}

	auv_msgs::NED str2NED(string Lat, string Lon){
//		string Dstr = Lat.substr (0,2);
//		string Mstr = Lat.substr (3,2);
//		string Sstr = Lat.substr (5,17);

		//pair<double, double> ;
		auv_msgs::NED position;
		sensor_msgs::NavSatFix LatLon;

		double DLat = atof(Lat.substr(0,2).c_str());
		double MLat = atof(Lat.substr(3,2).c_str());
		double SLat = atof(Lat.substr(6,17).c_str());

		ROS_ERROR("DMS %f, %f, %f", DLat, MLat, SLat);

		double DLon = atof(Lon.substr(0,2).c_str());
		double MLon = atof(Lon.substr(3,2).c_str());
		double SLon = atof(Lon.substr(6,17).c_str());

		ROS_ERROR("DMS %f, %f, %f", DLon, MLon, SLon);

		LatLon.latitude = DLat+MLat/60+SLat/3600;
	    LatLon.longitude = DLon+MLon/60+SLon/3600;

	    convert(LatLon);
	    position.north = posxy.first;
	    position.east = posxy.second;
	    position.depth = 0;

	    return position;
		//return DD;



	}

	//void onGps(const sensor_msgs::NavSatFix::ConstPtr& data);
	//sensor_msgs::NavSatFix LatLon;
	std::pair<double, double> posxy, originLL, posLL;
//	tf2_ros::Buffer buffer;
	//tf2_ros::TransformListener listener;
	//ros::Subscriber gps;
};


int parseNeptus(string xmlFile){

	enum {none = 0, go2point_FA, go2point_UA, dynamic_positioning, course_keeping_FA, course_keeping_UA};


   LatLon2NED LL2NED;
   ManeuverGenerator MG;
   XMLDocument xmlDoc;

   /* Open XML file */
   if(xmlDoc.LoadFile(xmlFile.c_str()) == XML_SUCCESS) {

	   ROS_ERROR("Otvorio .nmis");

	   MG.writeXML.addMission();

	   XMLElement *graph = xmlDoc.FirstChildElement("mission-def")->FirstChildElement("body")
										   ->FirstChildElement("plan")->FirstChildElement("graph");

	   for (XMLElement* node = graph->FirstChildElement(); node != NULL; node = node->NextSiblingElement()){

	   // do something with each child element

		   ROS_ERROR("debug Loop vanjski");

		  if( XMLElement *maneuver = node->FirstChildElement("maneuver")){

			   for (XMLElement* maneuverType = maneuver->FirstChildElement(); maneuverType != NULL; maneuverType = maneuverType->NextSiblingElement()){

				   ROS_ERROR("debug Loop unutarnji");

				   ROS_ERROR("Loop, %s", maneuverType->ToElement()->Name());


				   if(strcmp(maneuverType->ToElement()->Name(),"Goto") == 0){

					   ROS_ERROR("GoTo manevar");

					   //XMLElement *

					   XMLElement *LatLon = maneuverType->FirstChildElement("finalPoint")->FirstChildElement("point")->
							   	   	   	   	   	   	   	   	   FirstChildElement("coordinate")->FirstChildElement("latitude");

					   ROS_ERROR("Lat: %s", LatLon->ToElement()->GetText());
					   string lat = LatLon->ToElement()->GetText();

					   LatLon = LatLon->NextSiblingElement();

					   ROS_ERROR("Lon: %s", LatLon->ToElement()->GetText());
					   string lon = LatLon->ToElement()->GetText();

					   LatLon = LatLon->NextSiblingElement();

					   ROS_ERROR("Height: %s", LatLon->ToElement()->GetText());

					   auv_msgs::NED position;
					   position = LL2NED.str2NED(lat,lon);

					   ROS_ERROR("Preracunato: %f,%f", position.north, position.east);

					   MG.writeXML.addGo2point_UA(position.north, position.east,0.5,1.0);




				   } else  if(strcmp(maneuverType->ToElement()->Name(),"Loiter") == 0){

					   ROS_ERROR("loiter manevar");


				   } else  if(strcmp(maneuverType->ToElement()->Name(),"RowsManeuver") == 0){

					   ROS_ERROR("lawnmower manevar");

					   XMLElement *LatLon = maneuverType->FirstChildElement("basePoint")->FirstChildElement("point")->
					 							   	   	   	   	   	   	   	   	   FirstChildElement("coordinate")->FirstChildElement("latitude");

					   ROS_ERROR("Lat: %s", LatLon->ToElement()->GetText());

					   string lat = LatLon->ToElement()->GetText();

					   LatLon = LatLon->NextSiblingElement();


					   ROS_ERROR("Lon: %s", LatLon->ToElement()->GetText());
					   string lon = LatLon->ToElement()->GetText();

					   LatLon = LatLon->NextSiblingElement();

					   ROS_ERROR("Height%s", LatLon->ToElement()->GetText());

					//  double height = atof(LatLon->ToElement()->GetText().c_str());
					   auv_msgs::NED position;
					   position = LL2NED.str2NED(lat,lon);

					   ROS_ERROR("Preracunato: %f,%f", position.north, position.east);

					   XMLElement *param = maneuverType->FirstChildElement("width");

					   ROS_ERROR("Width: %s", param->ToElement()->GetText());


					   double width = atof(param->ToElement()->GetText());

					   param = param->NextSiblingElement();

					   ROS_ERROR("length: %s", param->ToElement()->GetText());
					   double length = atof(param->ToElement()->GetText());


					   param = param->NextSiblingElement();

					   ROS_ERROR("Hstep: %s", param->ToElement()->GetText());
					   double hstep = atof(param->ToElement()->GetText());


					   param = param->NextSiblingElement();

					   ROS_ERROR("bearing: %s", param->ToElement()->GetText());
					   double angle  = atof(param->ToElement()->GetText());


					   param = param->NextSiblingElement();

					   ROS_ERROR("speed: %s", param->ToElement()->GetText());

					   double speed = atof(param->ToElement()->GetText());

//					   double alternationPercent = 0.5;
//					   double curvOff = 0;
//					   bool squareCurve = true;
//					   double bearingRad = angle;
//					   double crossAngleRadians = 0;
//					   bool invertY = false;
//
//
//					   std::vector<Eigen::Vector4d> tmpPoints;
//					   tmpPoints = MG.calcRowsPoints(width, length, hstep,
//					   	            alternationPercent, curvOff, squareCurve, bearingRad,
//					   	            crossAngleRadians, invertY);
//
//
//					   MG.writePrimitives(go2point_FA, tmpPoints);

				   }
			   }
		  }
	   }

	   MG.writeXML.saveXML();

   } else {
	   ROS_ERROR("Cannot open XML file!");
	   return -1;
   }
}



void startParseCallback(const std_msgs::String::ConstPtr& msg){

	int status = parseNeptus(msg->data);
}

int main(int argc, char** argv){

	ros::init(argc, argv, "neptusParser");
	ros::NodeHandle nh;

	/* Subscribers */
	ros::Subscriber subStartParse = nh.subscribe<std_msgs::String>("startParse",1, startParseCallback);

	ros::spin();

	return 0;

}




