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
#include <labust/xml/DefineXMLStruct.hpp>
#include <labust/xml/XMLException.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/xml/XMLWriter.hpp>
#include <labust/tools/TimingTools.hpp>

#include <iostream>

bool test_xml_reader(int argc, char* argv[])
try
{
	if (argc > 1)
	{
		labust::xml::Reader reader(argv[1],true);

		double stra; reader.value("//textparam",&stra);

		double start=labust::tools::unix_time();
		enum ent {t=0,k=1};
		for (int i = 0; i<1000000; ++i)
		{
			//ent Ts = ent(reader.value<int>("//sampling-time"));
			//double Ts(0); reader.value("//sampling-time",&Ts);
			//std::string str; reader.value("//textparam",&str);
			//std::string str = reader.value<std::string>("//textparam");
		}

		std::cout<<"Timing:"<<labust::tools::unix_time() - start<<std::endl;

		//ent a = ent(reader.value<int>("//sampling-time"));

		std::string str;
		reader.value("//textparam",&str);
		std::cout<<str<<std::endl;

		return true;
	}
	else
	{
		std::cout<<"Not enough arguments."<<std::endl;
	}

	return false;
}
catch (labust::xml::XMLException& e)
{
	std::cout<<e.what()<<std::endl;
	return false;
};

bool test_xml_writer(int argc, char* argv[])
try
{
	labust::xml::Writer writer;

	writer.addElement("Test", 10);
	writer.startElement("TestAttribute");
	writer.addAttribute("key","Franc");
	writer.addValue("Attribute test");
	writer.endElement();
	writer.endDocument();

	std::map<std::string, float> mp;
	mp["T1"] = labust::tools::unix_time();
	mp["T2"] = 12;

	std::vector<int> a;
	a.push_back(10);
	a.push_back(12);

	labust::xml::Writer writer2;
	writer2.addElementPairs(mp.begin(),mp.end());
	writer2.startElement("Test3");
	writer2.addAttributePairs(mp.begin(),mp.end());
	writer2.addElement("XML1",writer.toString());
	writer2.addB64Value(a.begin(),a.end());
	writer2.endDocument();

	std::cout<<"XML string:"<<writer.toString()<<std::endl;
	std::cout<<"XML string2:"<<writer2<<std::endl;

	double start=labust::tools::unix_time();
	for (int i = 0; i<1000000; ++i)
	{
		labust::xml::Writer writer;
		writer.addElement("Test", 10);
		writer.endDocument();
	}
	std::cout<<"Timing:"<<labust::tools::unix_time() - start<<std::endl;
	return true;
}
catch (labust::xml::XMLException& e)
{
	std::cout<<e.what()<<std::endl;
	return false;
};

PP_LABUST_MAKE_XML_STRUCTURE(
		(demo), employee,
		(std::string, name)
		(int, age));

PP_LABUST_MAKE_XML_STRUCTURE(
		(demo), office,
		(int, room)
		(demo::employee, ivo)
		(demo::employee, jozo))

struct XMLStruct{};

namespace demo
{
	namespace update
	{
		class employee : XMLStruct
		{
			typedef boost::shared_ptr<std::string> StringPtr;
		public:

			typedef enum {nameUpdate, ageUpdate, kidsUpdate} keys;

			employee():
				name(),
				age(){};

			StringPtr write(const std::string& id  = "", bool sendType = false)
			{
				labust::xml::Writer writer;

				writer.startElement("employee");
			  if (!id.empty()) writer.addAttribute("id",id);
			  if (sendType) writer.addAttribute("type","employee");

			  //ADD PARAMETER
			  writer.startElement("param");
			   writer.addAttribute("name","name");
			   writer.addAttribute("value",this->name);
			   if (sendType) writer.addAttribute("type","std::string");
			  writer.endElement();
			  writer.startElement("param");
			   writer.addAttribute("name","age");
			   writer.addAttribute("value",this->age);
			   if (sendType) writer.addAttribute("type","int");
			  writer.endElement();

				writer.endDocument();
				return writer.toStringPt();
			}

			StringPtr writeUpdate(const std::string& id  = "", bool sendType = false)
			{
				labust::xml::Writer writer;

				writer.startElement("employee");
			  if (!id.empty()) writer.addAttribute("id",id);
			  if (sendType) writer.addAttribute("type","employee");

			  //ADD PARAMETER
			  if (updateMap[nameUpdate])
			  {
			  	if (boost::is_base_of<XMLStruct, std::string>::value)
			  	{
			  		writer.startElement("param");
			  	   writer.addAttribute("name","name");
			       writer.addAttribute("value",this->name);
			       if (sendType) writer.addAttribute("type","std::string");
  			    writer.endElement();
		   	    updateMap[nameUpdate] = false;
			  	}
			  	else
			  	{
			  	}
			  }

			  if (updateMap[ageUpdate])
			  {
			    writer.startElement("param");
			     writer.addAttribute("name","age");
			     writer.addAttribute("value",this->age);
			     if (sendType) writer.addAttribute("type","int");
			    writer.endElement();
			    updateMap[ageUpdate] = false;
			  }

				writer.endDocument();
				return writer.toStringPt();
			}

			void read(const std::string& str, const std::string& id = "")
			{
				labust::xml::Reader reader(str);
				this->read(reader,id);
			}

			void read(labust::xml::Reader& reader, const std::string& id = "")
			{
				_xmlNode* org_node = reader.currentNode();
				std::string head("/employee");
				if (reader.try_expression("employee")) head = "employee";
				reader.useNode(reader.value<_xmlNode*>(id.empty()?head:head + "[@id='" + id + "']"));

				reader.try_value("param[@name='age']/@value",&this->age);

				reader.useNode(org_node);
			}

			//Testers
			inline bool wasUpdated(keys key)
			{
				return updateMap[key];
			}

			//Accessors
			inline const std::string& Name(){updateMap[nameUpdate] = false;return name;};
			inline void Name(const std::string& name){this->name = name;updateMap[nameUpdate] = true;};
			inline const int& Age(){updateMap[ageUpdate] = false;return age;};
			inline void Age(int age){this->age = age;updateMap[ageUpdate] = true;};

		private:

			std::string name;
			int age;

			//Update map
			std::map<keys, bool> updateMap;
		};

		labust::xml::Reader& operator>>(labust::xml::Reader& reader, employee& emp)
		{
			std::cout<<"Select this."<<std::endl;
			return reader;
		}

		class office : XMLStruct
		{
		public:
			typedef enum {countUpdate, ivoUpdate, mirkoUpdate} keys;

			office():
				count(2),
				Mirko(),
				Ivo(){};

			void read(const std::string& str, const std::string& id = "")
			{
				labust::xml::Reader reader(str);
				this->read(reader,id);
			}

			void read(labust::xml::Reader& reader, const std::string& id = "")
			{
				_xmlNode* org_node = reader.currentNode();
				std::string head("/office");
				if (reader.try_expression("office")) head = "office";
				reader.useNode(reader.value<_xmlNode*>(id.empty()?head:head + "[@id='" + id + "']"));

				reader.try_value("param[@name='count']/@value",&this->count);
				//reader>>Mirko;

				reader.useNode(org_node);
			}

			//Testers
			inline bool wasUpdated(keys key)
			{
				return updateMap[key];
			}

		private:
			int count;
			employee Mirko,Ivo;

			//Update map
			std::map<keys, bool> updateMap;
		};
	}
}


int main(int argc, char* argv[])
try
{
	bool flag(true);
	double time = labust::tools::unix_time();
	for (int i=0; i<10000; ++i)
	{
		demo::employee ivo, jozo;
		ivo.name = "ivo";
		ivo.age = 10;

		//std::cout<<*ivo.write()<<std::endl;
		jozo.read(*ivo.write());
		//std::cout<<*jozo.write()<<std::endl;
		jozo.name = "Jozo";
		jozo.age = 22;

		demo::office lab1;
		lab1.ivo = ivo;
		lab1.jozo = jozo;
		lab1.room = 213;

		//std::cout<<*lab1.write()<<std::endl;

		demo::office lab2;
		labust::xml::Reader reader(*lab1.write());
		lab2.read(reader);

		//std::cout<<*lab2.ivo.write()<<std::endl;
	}

	std::cout<<"Timesum:"<<(labust::tools::unix_time() - time)/10000.<<std::endl;

	/*
	demo::update::employee test;

	test.Name("Ivo");
	//test.Age(10);

	labust::xml::Reader reader("<stuff><param /></stuff>");
	reader>>test;

	boost::shared_ptr<std::string> writer(test.writeUpdate("Ivo",false));
	std::cout<<*writer<<std::endl;

	demo::update::employee test2;

	test2.read(*writer);

	boost::shared_ptr<std::string> writer2(test2.write());

	std::cout<<*writer2<<std::endl;

	/*

	if ((flag = test_xml_reader(argc, argv)))
	{
		std::cout<<"XML reader ... OK"<<std::endl;
	};

	if ((flag = flag && test_xml_writer(argc, argv)))
	{
		std::cout<<"XML writer ... OK"<<std::endl;
	};
	if (flag) std::cout<<"liblabust_xml passed test."<<std::endl;*/

	return 0;
}
catch (std::exception& e)
{
	std::cout<<e.what()<<std::endl;
}
catch (...)
{
	std::cout<<"Unknown error."<<std::endl;
}


