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
#include <labust/xml/XMLException.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/xml/XMLWriter.hpp>
#include <labust/tools/TimingTools.hpp>
#include <labust/xml/xmlfwd.hpp>

#include <boost/type_traits.hpp>

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
			double Ts(0); reader.value("//sampling-time",&Ts);
			//std::cout<<Ts<<std::endl;
			std::string str; reader.value("//textparam",&str);
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

int main(int argc, char* argv[])
try
{
	bool flag(true);

	if ((flag = test_xml_reader(argc, argv)))
	{
		std::cout<<"XML reader ... OK"<<std::endl;
	};

	if ((flag = flag && test_xml_writer(argc, argv)))
	{
		std::cout<<"XML writer ... OK"<<std::endl;
	};
	if (flag) std::cout<<"liblabust_xml passed test."<<std::endl;

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


