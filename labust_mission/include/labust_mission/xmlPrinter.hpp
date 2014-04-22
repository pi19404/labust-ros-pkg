//\todo Moguce napravit funkciju(template) koja ubacuje novi node (smanjenje velicine koda)
//\todo dodati funkciju klase saveAsString

/*********************************************************************
 * xmlPrinter.hpp
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

#ifndef XMLPRINTER_HPP_
#define XMLPRINTER_HPP_

#include <labust_mission/labustMission.hpp>
#include <tinyxml2.h>

using namespace tinyxml2;
using namespace std;

namespace utils {

	/*****************************************************************
	 *** WriteXML Class definition
	 ****************************************************************/

	class WriteXML{

	public:

		/************************************************************
		 *** Class functions
		 ************************************************************/

		WriteXML();

		void addMission();

		void saveXML(string fileName);

		void saveAsString();

		void addGo2point_FA(double north, double east, double heading, double speed, double victoryRadius);

		void addGo2point_UA(double north, double east, double speed, double victoryRadius);

		void addDynamic_positioning(double north, double east, double heading);

		void addCourse_keeping_FA(double course, double speed, double heading);

		void addCourse_keeping_UA(double course, double speed);


		/************************************************************
		 *** Class variables
		 ************************************************************/

		XMLDocument doc;

		XMLNode *mission;
		XMLNode *primitive;
		XMLNode *param;
		XMLNode *idNode;

		string textString;
		int id;

	};


	WriteXML::WriteXML():id(0){

	}

	void WriteXML::addMission(){

		mission = doc.NewElement("mission");
		doc.InsertEndChild(mission);
		id = 0;
	}

	void WriteXML::saveXML(string fileName){

		//string fileName = "/home/filip/catkin_ws/src/test_izlaz.xml";
		doc.SaveFile( fileName.c_str() );
		ROS_ERROR("Mission generated.");
	}

	void saveAsString(){

	}

	void WriteXML::addGo2point_UA(double north, double east, double speed, double victoryRadius){

		id++;

		primitive = doc.NewElement("primitive");
		primitive->ToElement()->SetAttribute("name","go2point_UA");

		idNode = doc.NewElement("id");

		string id_string = static_cast<ostringstream*>( &(ostringstream() << id) )->str();
		idNode->InsertEndChild(doc.NewText(id_string.c_str()));
		primitive->InsertEndChild(idNode);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","north");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << north) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","east");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << east) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","speed");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << speed) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","victory_radius");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << victoryRadius) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		mission->InsertEndChild(primitive);
	}

	void WriteXML::addGo2point_FA(double north, double east, double heading, double speed, double victoryRadius){

		id++;

		primitive = doc.NewElement("primitive");
		primitive->ToElement()->SetAttribute("name","go2point_FA");

		idNode = doc.NewElement("id");

		string id_string = static_cast<ostringstream*>( &(ostringstream() << id) )->str();
		idNode->InsertEndChild(doc.NewText(id_string.c_str()));
		primitive->InsertEndChild(idNode);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","north");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << north) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","east");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << east) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","heading");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << heading) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","speed");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << speed) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);


		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","victory_radius");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << victoryRadius) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		mission->InsertEndChild(primitive);
	}

	void WriteXML::addDynamic_positioning(double north, double east, double heading){

		id++;

		primitive = doc.NewElement("primitive");
		primitive->ToElement()->SetAttribute("name","dynamic_positioning");

		idNode = doc.NewElement("id");

		string id_string = static_cast<ostringstream*>( &(ostringstream() << id) )->str();
		idNode->InsertEndChild(doc.NewText(id_string.c_str()));
		primitive->InsertEndChild(idNode);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","north");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << north) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","east");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << east) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","heading");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << heading) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		mission->InsertEndChild(primitive);
	}

	void WriteXML::addCourse_keeping_FA(double course, double speed, double heading){

		id++;

		primitive = doc.NewElement("primitive");
		primitive->ToElement()->SetAttribute("name","course_keeping_FA");

		idNode = doc.NewElement("id");

		string id_string = static_cast<ostringstream*>( &(ostringstream() << id) )->str();
		idNode->InsertEndChild(doc.NewText(id_string.c_str()));
		primitive->InsertEndChild(idNode);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","course");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << course) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","speed");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << speed) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","heading");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << heading) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		mission->InsertEndChild(primitive);
	}

	void WriteXML::addCourse_keeping_UA(double course, double speed){

		id++;

		primitive = doc.NewElement("primitive");
		primitive->ToElement()->SetAttribute("name","course_keeping_UA");

		idNode = doc.NewElement("id");

		string id_string = static_cast<ostringstream*>( &(ostringstream() << id) )->str();
		idNode->InsertEndChild(doc.NewText(id_string.c_str()));
		primitive->InsertEndChild(idNode);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","course");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << course) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		param = doc.NewElement("param");
		param->ToElement()->SetAttribute("name","speed");

		textString.assign(static_cast<ostringstream*>( &(ostringstream() << speed) )->str());
		param->InsertEndChild(doc.NewText(textString.c_str()));
		primitive->InsertEndChild(param);

		mission->InsertEndChild(primitive);
	}
}

#endif /* XMLPRINTER_HPP_ */
