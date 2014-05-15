/*********************************************************************
 * neptus_parser.cpp
 *
 *  Created on: Apr 3, 2014
 *      Author: Filip Mandic
 *
 ********************************************************************/

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

#include <labust_mission/labustMission.hpp>
#include <labust_mission/maneuverGenerator.hpp>

#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TransformStamped.h>
#include <labust/tools/conversions.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <tinyxml2.h>
#include <misc_msgs/StartParser.h>

using namespace std;
using namespace tinyxml2;
using namespace labust::utils;

/*********************************************************************
 *** NeptusParser Class definition
 ********************************************************************/

class NeptusParser{

public:

	/*********************************************************************
	 *** Class functions
	 ********************************************************************/

	NeptusParser():startPointSet(false),
					 startRelative(true){

		offset.north = offset.east = 0;
		xmlSavePath = "";
	}

	void neptusParserReset(){

		startPointSet = false;
		startRelative = true;
		offset.north = offset.east = 0;
		xmlSavePath = "";

	}

	int parseNeptus(string xmlFile){

	   /* Open XML file */
	   if(xmlDoc.LoadFile(xmlFile.c_str()) == XML_SUCCESS) {

		   ROS_INFO("*.nmis mission file successfully loaded.");
		   /* Write mission tag */
		   MG.writeXML.addMission();

		   /* Go to graph node and loop through it's children */
		   XMLElement *graph = xmlDoc.FirstChildElement("mission-def")->FirstChildElement("body")
											   ->FirstChildElement("plan")->FirstChildElement("graph");

		   for (XMLElement* node = graph->FirstChildElement(); node != NULL; node = node->NextSiblingElement()){

			   /* Check if child element is maneuver node */
			   if( XMLElement *maneuver = node->FirstChildElement("maneuver")){

				   /* Loop through maneuver child nodes */
				   for (XMLElement* maneuverType = maneuver->FirstChildElement(); maneuverType != NULL; maneuverType = maneuverType->NextSiblingElement()){

					   /* Check maneuver type */
					   if(strcmp(maneuverType->ToElement()->Name(),"Goto") == 0){

						   ROS_ERROR("GoTo manevar");
						   parseGoto(maneuverType);

					   } else  if(strcmp(maneuverType->ToElement()->Name(),"Loiter") == 0){

						   ROS_ERROR("loiter manevar");
						   parseLoiter(maneuverType);

					   } else  if(strcmp(maneuverType->ToElement()->Name(),"RowsManeuver") == 0){

						   ROS_ERROR("lawnmower manevar");
						   parseRows(maneuverType);

					   } else  if(strcmp(maneuverType->ToElement()->Name(),"StationKeeping") == 0){

						   ROS_ERROR("DP manevar");
						   parseStationKeeping(maneuverType);
					   }
				   }
			   }
		   }

		   /* Save XML file */
		   MG.writeXML.saveXML(xmlSavePath);
		   return 1;
	   } else {
		   ROS_ERROR("Cannot open XML file!");
		   return -1;
	   }
	}


	void parseGoto(XMLElement *maneuverType){

		ROS_INFO("Parsing goto maneuver...");

		XMLElement *LatLonPoint = maneuverType->FirstChildElement("finalPoint")->FirstChildElement("point")->
									   	   	   	   	   	   	   	   	   FirstChildElement("coordinate")->FirstChildElement("latitude");
		/* Read maneuver parameters */
		ROS_ERROR("Lat: %s", LatLonPoint->ToElement()->GetText());
		string lat = LatLonPoint->ToElement()->GetText();

		LatLonPoint = LatLonPoint->NextSiblingElement();

		ROS_ERROR("Lon: %s", LatLonPoint->ToElement()->GetText());
		string lon = LatLonPoint->ToElement()->GetText();

		auv_msgs::NED position;
		position = str2NED(lat,lon);
		ROS_ERROR("Preracunato: %f,%f", position.north, position.east);

		/* Set offset if in relative mode */
		if(!startPointSet){
			offset.north += position.north;
			offset.east += position.east;
			startPointSet = true;
		}

		/* Set default values */
		double duration = 0;

		/* Loop through rows parameters */
		for (XMLElement* param = maneuverType->FirstChildElement("duration"); param != NULL; param = param->NextSiblingElement()){

			/* Check parameter name */
			if( strcmp(param->ToElement()->Name(),"duration") == 0){

				duration = atof(param->ToElement()->GetText());
				ROS_ERROR("Duration: %s", param->ToElement()->GetText());

			}
		}

		/* Write point to XML file */
		MG.writeXML.addGo2point_FA(position.north-offset.north, position.east-offset.east,0,0.5,1.0);
	}

	void parseRows(XMLElement *maneuverType){

		ROS_INFO("Parsing rows maneuver...");

		XMLElement *LatLonPoint = maneuverType->FirstChildElement("basePoint")->FirstChildElement("point")->
																   FirstChildElement("coordinate")->FirstChildElement("latitude");
		/* Read rows maneuver start point parameters */
		ROS_ERROR("Lat: %s", LatLonPoint->ToElement()->GetText());
		string lat = LatLonPoint->ToElement()->GetText();

		LatLonPoint = LatLonPoint->NextSiblingElement();

		ROS_ERROR("Lon: %s", LatLonPoint->ToElement()->GetText());
		string lon = LatLonPoint->ToElement()->GetText();

		auv_msgs::NED position;
		position = str2NED(lat,lon);
		ROS_ERROR("Preracunato: %f,%f", position.north, position.east);

		/* Set offset if in relative mode */
		if(!startPointSet){
			offset.north += position.north;
			offset.east += position.east;
			startPointSet = true;
		}

		/* Set default values */
		double width = 100;
		double length = 200;
		double hstep = 27;
		double bearing = 0;
		double alternationPercent = 100;
		double curvOff = 15;
		double crossAngle = 0;
		double speed = 0.5; /* Speed in m/s */
		bool squareCurve = true;
		bool invertY = false;

		/* Loop through rows parameters */
		for (XMLElement* param = maneuverType->FirstChildElement("width"); param != NULL; param = param->NextSiblingElement()){

			/* Check parameter name */
			if( strcmp(param->ToElement()->Name(),"width") == 0){

				width = atof(param->ToElement()->GetText());
				ROS_ERROR("Width: %s", param->ToElement()->GetText());

			} else if(strcmp(param->ToElement()->Name(),"length") == 0){

				length = atof(param->ToElement()->GetText());
				ROS_ERROR("length: %s", param->ToElement()->GetText());

			} else if(strcmp(param->ToElement()->Name(),"hstep") == 0){

				hstep = atof(param->ToElement()->GetText());
				ROS_ERROR("Hstep: %s", param->ToElement()->GetText());

			} else if(strcmp(param->ToElement()->Name(),"bearing") == 0){

				bearing = atof(param->ToElement()->GetText());
				ROS_ERROR("bearing: %s", param->ToElement()->GetText());


			} else if(strcmp(param->ToElement()->Name(),"crossAngle") == 0){

				crossAngle = atof(param->ToElement()->GetText());
				ROS_ERROR("crossAngle: %s", param->ToElement()->GetText());

			} else if(strcmp(param->ToElement()->Name(),"alternationPrecentage") == 0){

				alternationPercent = atof(param->ToElement()->GetText());
				ROS_ERROR("alternationPercent: %s", param->ToElement()->GetText());

			} else if(strcmp(param->ToElement()->Name(),"squareCurve") == 0){

				if(strcmp(param->ToElement()->GetText(),"false")==0){
					squareCurve = false;
				} else {
					squareCurve = true;
				}
				ROS_ERROR("squareCurve: %s", param->ToElement()->GetText());

			} else if(strcmp(param->ToElement()->Name(),"firstCurveRight") == 0){

				if(strcmp(param->ToElement()->GetText(),"false")==0){
					invertY = true;
				} else {
					invertY = false;
				}
				ROS_ERROR("firstCurveRight: %s", param->ToElement()->GetText());

			} else if(strcmp(param->ToElement()->Name(),"curveOffset") == 0){

				curvOff = atof(param->ToElement()->GetText());
				ROS_ERROR("curveoffset: %s", param->ToElement()->GetText());

			} else if(strcmp(param->ToElement()->Name(),"speed") == 0){

				if(strcmp(param->ToElement()->Attribute("unit"),"RPM") == 0) {

				ROS_ERROR("Speed units RPM. Leaving default speed: %f m/s", speed);

				} else {
					speed = atof(param->ToElement()->GetText());
					ROS_ERROR("speed: %s", param->ToElement()->GetText());
				}
			}
		}

ROS_ERROR("width: %f, length: %f, hstep: %f, bearing: %f, alternationPercent: %f, curvOff: %f, crossAngle: %f, speed: %f, squareCurve: %d, invertY: %d", width, length, hstep, bearing, alternationPercent,curvOff, crossAngle,speed, squareCurve, invertY);


		/* Generate maneuver points */
		std::vector<Eigen::Vector4d> tmpPoints;
		tmpPoints = MG.calcRowsPoints(width, length, hstep,
					alternationPercent/100, curvOff, squareCurve, bearing*M_PI/180,
					crossAngle*M_PI/180, invertY);

		/* For each point subtract offset and add start point */
		for(std::vector<Eigen::Vector4d>::iterator it = tmpPoints.begin(); it != tmpPoints.end(); ++it){

			Eigen::Vector4d vTmp = *it;
			vTmp[X] += -offset.north + position.north;
			vTmp[Y] += -offset.east + position.east;

			*it = vTmp;
		}

		/* Write maneuver points to XML */
		MG.writePrimitives(go2point_FA, tmpPoints,speed,1.0); /* speed, victoryRadius */
	}

	void parseStationKeeping(XMLElement *maneuverType){

		ROS_INFO("Parsing StationKeeping maneuver...");

		XMLElement *LatLonPoint = maneuverType->FirstChildElement("basePoint")->FirstChildElement("point")->
																	   FirstChildElement("coordinate")->FirstChildElement("latitude");
		/* Read maneuver parameters */
		ROS_ERROR("Lat: %s", LatLonPoint->ToElement()->GetText());
		string lat = LatLonPoint->ToElement()->GetText();

		LatLonPoint = LatLonPoint->NextSiblingElement();

		ROS_ERROR("Lon: %s", LatLonPoint->ToElement()->GetText());
		string lon = LatLonPoint->ToElement()->GetText();

		auv_msgs::NED position;
		position = str2NED(lat,lon);
		ROS_ERROR("Preracunato: %f,%f", position.north, position.east);

		/* Set offset if in relative mode */
		if(!startPointSet){
			offset.north += position.north;
			offset.east += position.east;
			startPointSet = true;
		}

		/* Write point to XML file */
		MG.writeXML.addDynamic_positioning(position.north-offset.north, position.east-offset.east,0);


	}

	void parseLoiter(XMLElement *maneuverType){

	}

	/*********************************************************************
	 *** Class helper functions
	 ********************************************************************/

	auv_msgs::NED str2NED(string Lat, string Lon){

		auv_msgs::NED position;
		sensor_msgs::NavSatFix LatLon;
		std::pair<double, double> posxy;

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

		posxy =	labust::tools::deg2meter(LatLon.latitude - startPoint.latitude, LatLon.longitude - startPoint.longitude, startPoint.longitude);

	    position.north = posxy.first;
	    position.east = posxy.second;
	    position.depth = 0;

	    return position;
	}

	/*********************************************************************
	 *** Class variables
	 ********************************************************************/

	labust::maneuver::ManeuverGenerator MG;
	XMLDocument xmlDoc;
	sensor_msgs::NavSatFix startPoint;
	auv_msgs::NED offset;
	bool startPointSet;
	bool startRelative;
	string xmlSavePath;
};

void startParseCallback(ros::Publisher &pubStartDispatcher, const misc_msgs::StartParser::ConstPtr& msg){

	ros::NodeHandle ph("~");
	NeptusParser NP;
	ph.param("xml_save_path", NP.xmlSavePath, NP.xmlSavePath);
	ROS_ERROR("%s",NP.xmlSavePath.c_str());

	NP.startRelative = msg->relative;
	NP.startPoint.latitude = msg->lat;
	NP.startPoint.longitude = msg->lon;

	if(NP.startRelative){
		NP.offset.north = -NP.startPoint.latitude;
		NP.offset.east = -NP.startPoint.longitude;
	} else {
		NP.offset.north = NP.offset.east = 0;
	}

	int status = NP.parseNeptus(msg->fileName);

	if(status == 1){
		std_msgs::String tmp;
		tmp.data = "/START_DISPATCHER";
		pubStartDispatcher.publish(tmp);
		NP.neptusParserReset();
	}
}

int main(int argc, char** argv){

	ros::init(argc, argv, "neptusParser");
	ros::NodeHandle nh;

	/* Publishers */
	ros::Publisher pubStartDispatcher = nh.advertise<std_msgs::String>("eventString",1);

	/* Subscribers */
	ros::Subscriber subStartParse = nh.subscribe<misc_msgs::StartParser>("startParse",1, boost::bind(&startParseCallback, boost::ref(pubStartDispatcher), _1));

	ros::spin();
	return 0;
}




