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

namespace labust{
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

			void addEvent();

			void saveXML(string fileName);

			void addXMLNode(XMLNode* parentNode, string nodeName, string attrName, string attrValue, double value);

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
			XMLNode *events;

			int id;

		};

		WriteXML::WriteXML():id(0){

		}

		void WriteXML::addMission(){

			mission = doc.NewElement("mission");
			doc.InsertEndChild(mission);
			id = 0;
		}

		void WriteXML::addEvent(){

			events = doc.NewElement("events");
			doc.InsertEndChild(events);
		}

		void WriteXML::saveXML(string fileName){

			doc.SaveFile( fileName.c_str() );
			ROS_ERROR("Mission generated.");
		}


		void WriteXML::addXMLNode(XMLNode* parentNode, string nodeName, string attrName, string attrValue, double value){

			XMLNode *node;
			string text;
			node = doc.NewElement(nodeName.c_str());
			if(attrName.empty() == 0)
				node->ToElement()->SetAttribute(attrName.c_str(),attrValue.c_str());

			text.assign(static_cast<ostringstream*>( &(ostringstream() << value) )->str());
			node->InsertEndChild(doc.NewText(text.c_str()));
			parentNode->InsertEndChild(node);
		}

		void WriteXML::addGo2point_UA(double north, double east, double speed, double victoryRadius){

			id++;

			primitive = doc.NewElement("primitive");
			primitive->ToElement()->SetAttribute("name","go2point_UA");

			addXMLNode(primitive,"id","","",id);

			addXMLNode(primitive,"param","name","north",north);
			addXMLNode(primitive,"param","name","east",east);
			addXMLNode(primitive,"param","name","speed",speed);
			addXMLNode(primitive,"param","name","victory_radius",victoryRadius);

			mission->InsertEndChild(primitive);
		}

		void WriteXML::addGo2point_FA(double north, double east, double heading, double speed, double victoryRadius){

			id++;

			primitive = doc.NewElement("primitive");
			primitive->ToElement()->SetAttribute("name","go2point_FA");

			addXMLNode(primitive,"id","","",id);

			addXMLNode(primitive,"param","name","north",north);
			addXMLNode(primitive,"param","name","east",east);
			addXMLNode(primitive,"param","name","heading",heading);
			addXMLNode(primitive,"param","name","speed",speed);
			addXMLNode(primitive,"param","name","victory_radius",victoryRadius);

			mission->InsertEndChild(primitive);
		}

		void WriteXML::addDynamic_positioning(double north, double east, double heading){

			id++;

			primitive = doc.NewElement("primitive");
			primitive->ToElement()->SetAttribute("name","dynamic_positioning");

			addXMLNode(primitive,"id","","",id);

			addXMLNode(primitive,"param","name","north",north);
			addXMLNode(primitive,"param","name","east",east);
			addXMLNode(primitive,"param","name","heading",heading);
			addXMLNode(primitive,"param","name","timeout",10);

			mission->InsertEndChild(primitive);
		}

		void WriteXML::addCourse_keeping_FA(double course, double speed, double heading){

			id++;

			primitive = doc.NewElement("primitive");
			primitive->ToElement()->SetAttribute("name","course_keeping_FA");

			addXMLNode(primitive,"id","","",id);

			addXMLNode(primitive,"param","name","course",course);
			addXMLNode(primitive,"param","name","speed",speed);
			addXMLNode(primitive,"param","name","heading",heading);
			addXMLNode(primitive,"param","name","timeout",10);

			mission->InsertEndChild(primitive);
		}

		void WriteXML::addCourse_keeping_UA(double course, double speed){

			id++;

			primitive = doc.NewElement("primitive");
			primitive->ToElement()->SetAttribute("name","course_keeping_UA");

			addXMLNode(primitive,"id","","",id);

			addXMLNode(primitive,"param","name","course",course);
			addXMLNode(primitive,"param","name","speed",speed);
			addXMLNode(primitive,"param","name","timeout",10);

			mission->InsertEndChild(primitive);
		}
	}
}

#endif /* XMLPRINTER_HPP_ */
