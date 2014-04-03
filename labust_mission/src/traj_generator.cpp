///\todo Prebaci klase za xml citanje i pisanje u include file

/*********************************************************************
 * traj_generator.cpp
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

#include <tinyxml2.h>
#include <string>
#include <iostream>
#include <cmath>

#include <cstddef>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>



using namespace tinyxml2;
using namespace std;



class WriteXML{

public:

	WriteXML();

	void addMission();

	void saveXML(string fileName);

	void addGo2point_FA(double north, double east, double heading, double speed, double victoryRadius);

	void addGo2point_UA(double north, double east, double speed, double victoryRadius);


	/**************************************************
	 *** Class variables
	 ************************************/

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

	doc.SaveFile( fileName.c_str() );
	ROS_ERROR("Mission generated.");
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


void populateLtg(double x, double y, double width, double length, double angle, double step)
{

	/////////////////
	WriteXML writeXML;
	/////////////////

	writeXML.addMission();
    /* Odredi koliko traka stane u širinu lawn-a */
	int stepN = floor(width/step);


  //Dodaj startnu točku

	writeXML.addGo2point_UA(x,y,0.5,1.0);

	double xc = x;
    double yc = y;


	//Dok ima traka radi
	for (int i=0;i<stepN;++i){

		//Svaka neparna ide prema dolje (brojimo od nulte)
		double side = 1;
		if ((i%2) != 0){
		  side = -1;
		}

		//Od trenutne točke imaš novu točku na duljini pod kutem lawn-a

		//Add to next point
		xc += side * length * cos(angle);
		yc += side * length * sin(angle);

		writeXML.addGo2point_UA(xc,yc,0.5,1.0);

		//Do sljedećeg lawn-a imaš mali komadić pod 90
		//Add new turn
		xc += step * cos(angle - M_PI/2);
		yc += step * sin(angle - M_PI/2);

		//Zadnju točku ne dodaš da ostane ficlek
		//Do not add last point
		if (i != (stepN-1)){
			//ltg.appendPoint(*(new ControlUtils::PointC(xc,yc)));
			writeXML.addGo2point_UA(xc,yc,0.5,1.0);
		}
	}

	writeXML.saveXML("src/izlaz_test.xml");
}



int main(){

	populateLtg(5, 5, 100, 15, 0, 8);

	return 0;
}




