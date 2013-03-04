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
*********************************************************************/
#include <labust/blueview/TrackerROI.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/xml/XMLWriter.hpp>

#include <modp_b85.h>

#include <sstream>

using namespace labust::blueview;

TrackerROI::TrackerROI(){};

TrackerROI::TrackerROI(const std::string& str)
{
  this->deserialize(str,this);
}

TrackerROI::~TrackerROI(){};

bool TrackerROI::deserialize(const std::string& str, TrackerROI* const object)
try
{
  //Punch the stuff into the querier
	labust::xml::Reader reader(str);

	reader.useNode(reader.value<_xmlNode*>("/tracker-roi"));

	reader.value("width",&object->size.width);
	reader.value("height",&object->size.height);
	reader.value("meters-per-pixel",&object->headData.resolution);

	//Initialize the image and decode the information.
	object->roi = cv::Mat(object->size, CV_16U);
	std::string pixels;
	reader.value("pixels",&pixels);
	modp_b85_decode(reinterpret_cast<char*>(object->roi.data), str.c_str(), str.length());

	reader.useNode(reader.value<_xmlNode*>("sonar-head"));
	reader.value("latitude",&object->headData.latlon.x);
	reader.value("longitude",&object->headData.latlon.y);
	reader.value("heading",&object->headData.heading);
	reader.value("pan-angle",&object->headData.panAngle);
	reader.value("tilt-angle",&object->headData.tiltAngle);
	reader.value("origin-x",&object->origin.x);
	reader.value("origin-y",&object->origin.y);

	return true;
}
catch (labust::xml::XMLException& e)
{
	std::cerr<<"Failed to deserialize:"<<e.what()<<std::endl;
	return false;
}

bool TrackerROI::serialize(const TrackerROI& object, std::string* str)
try
{
  //Initialize the XMLWriter.
	labust::xml::Writer writer;
	writer.startElement("tracker-roi");
	 writer.addElement("width",object.size.width);
	 writer.addElement("height",object.size.height);
	 writer.addElement("meters-per-pixel",object.headData.resolution);
	 writer.startElement("sonar-head");
	  writer.addElement("latitude",object.headData.latlon.x);
	  writer.addElement("longitude",object.headData.latlon.y);
	  writer.addElement("heading",object.headData.heading);
	  writer.addElement("pan-angle",object.headData.panAngle);
    writer.addElement("tilt-angle",object.headData.tiltAngle);
    writer.addElement("origin-x",object.origin.x);
    writer.addElement("origin-y",object.origin.y);
   writer.endElement();

   boost::shared_ptr<char> buffer(new char[modp_b85_encode_strlen(object.roi.rows*object.roi.step)]);
   modp_b85_encode(buffer.get(),reinterpret_cast<const char*>(object.roi.data),object.roi.rows*object.roi.step);
   writer.addElement("pixels",buffer.get());
  writer.endDocument();

  (*str) = writer.toString();

	return true;
}
catch (labust::xml::XMLException& e)
{
	std::cerr<<"Failed to serialize:"<<e.what()<<std::endl;
	return false;
}
