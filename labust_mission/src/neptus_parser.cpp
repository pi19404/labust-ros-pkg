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

#include <tinyxml2.h>

#include <labust_mission/maneuverGenerator.hpp>


using namespace std;
using namespace tinyxml2;
using namespace utils;

int parseNeptus(string xmlFile){

   XMLDocument xmlDoc;

   /* Open XML file */
   if(xmlDoc.LoadFile(xmlFile.c_str()) == XML_SUCCESS) {

	   ROS_ERROR("Otvorio .nmis");

	   XMLElement *graph = xmlDoc.FirstChildElement("mission-def")->FirstChildElement("body")
										   ->FirstChildElement("plan")->FirstChildElement("graph");

	   for (XMLElement* node = graph->FirstChildElement(); node != NULL; node = node->NextSiblingElement()){

	   // do something with each child element

		   XMLElement *maneuver = node->FirstChildElement("maneuver");

		   for (XMLElement* maneuverType = maneuver->FirstChildElement(); maneuverType != NULL; maneuverType = maneuverType->NextSiblingElement()){



			   ROS_ERROR("Loop, %s", maneuverType->ToElement()->Name());


			   if(strcmp(maneuverType->ToElement()->Name(),"Goto") == 0){

				   ROS_ERROR("GoTo manevar");

				   //XMLElement *


				  // maneuver::lawnmower();

			   }
		   }
	   }
   } else {
	   ROS_ERROR("Cannot open XML file!");
	   return -1;
   }
}

int main(){

	string xmlFile = "/home/filip/mission.nmis";

	int status = parseNeptus(xmlFile);

	return 0;

}




