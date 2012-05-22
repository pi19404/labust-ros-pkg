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
#include <labust/xml/XMLWriter.hpp>

#include <libxml/xmlwriter.h>

using namespace labust::xml;

const std::string Writer::encoding("ISO-8859-1");

Writer::Writer():
    xml_buffer(xmlBufferCreate(),std::ptr_fun(xmlBufferFree))
{
  //Test XML version
  LIBXML_TEST_VERSION;
  if (!xml_buffer) throw XMLException("Failed instantiating the XML buffer object."); 
writer.reset(xmlNewTextWriterMemory(xml_buffer.get(), false), std::ptr_fun(xmlFreeTextWriter));
  if (!writer) throw XMLException("Failed to construct XML writer object.");
}

Writer::Writer(const std::string& path):
		xml_buffer(xmlBufferCreate(),std::ptr_fun(xmlBufferFree))
{
  //Test XML version
  LIBXML_TEST_VERSION;

  if (!xml_buffer) throw XMLException("Failed instantiating the XML buffer object.");
  writer.reset(xmlNewTextWriterFilename(path.c_str(), 0),std::ptr_fun(xmlFreeTextWriter));
  if (!writer) throw XMLException("Failed to construct XML file writer object.");

  this->startDocument();
}

Writer::~Writer()
{
}

void Writer::startDocument()
{
	assert(writer.get() && "Null-pointer at labust::xml::Writer.");

  if (xmlTextWriterStartDocument(writer.get(), NULL, encoding.c_str(), NULL) < 0)
  {
    throw XMLException("Failed to write the XML document header.");
  }
}

void Writer::endDocument()
{
	assert(writer.get() && "Null-pointer at labust::xml::Writer.");

	if (xmlTextWriterEndDocument(writer.get()) < 0)
	{
		throw XMLException("Failed to end XML document.");
  }

	xml_final.reset(new std::string(reinterpret_cast<const char*>(xml_buffer->content)));
	writer.reset();
}

void Writer::startElement(const std::string& name)
{
	assert(writer.get() && "Null-pointer at labust::xml::Writer.");

	if (xmlTextWriterStartElement(writer.get(), reinterpret_cast<const xmlChar*>(name.c_str())) < 0)
		throw XMLException("Failed starting element: " + name);
}

void Writer::endElement()
{
	assert(writer.get() && "Null-pointer at labust::xml::Writer.");

	if (xmlTextWriterEndElement(writer.get()) < 0)
		throw XMLException("Failed to end element.");
}

void Writer::addXML(const std::string& xml)//added to correctly handle insertion of strings containing xml (not escaping)
{
  if (xmlTextWriterWriteRaw(writer.get(),reinterpret_cast<const xmlChar*>(xml.c_str())) < 0) throw XMLException("When writting text: " + xml);
}

void Writer::writeAttribute(const std::string& name, const std::string& value)
{
	assert(writer.get() && "Null-pointer at labust::xml::Writer.");

	if (xmlTextWriterWriteAttribute(writer.get(),
			reinterpret_cast<const xmlChar*>(name.c_str()),
			reinterpret_cast<const xmlChar*>(value.c_str())) < 0)
		throw XMLException("Failed to write attribute: " + name);
}

void Writer::writeElement(const std::string& name, const std::string& value)
{
	assert(writer.get() && "Null-pointer at labust::xml::Writer.");

	if (xmlTextWriterWriteElement(writer.get(),
			reinterpret_cast<const xmlChar*>(name.c_str()),
			reinterpret_cast<const xmlChar*>(value.c_str())) < 0)
		throw XMLException("Failed to write element: " + name);
}

void Writer::writeString(const std::string& value)
{
	assert(writer.get() && "Null-pointer at labust::xml::Writer.");

	if (xmlTextWriterWriteString(writer.get(), reinterpret_cast<const xmlChar*>(value.c_str())) < 0)
		throw XMLException("Failed to write value: " + value);
}

void Writer::encodeB64(const char* data, size_t size)
{
	assert(writer.get() && "Null-pointer at labust::xml::Writer.");

	if(xmlTextWriterWriteBase64(writer.get(),data,0,size) < 0)
		throw XMLException("Failed encoding to base64.");
}

std::ostream& labust::xml::operator<<(std::ostream& out, const Writer& object)
{
	assert (!object.writer.get() && "Can not read string until you call endDocument().");

  out<<*object.xml_final;
  return out;
}

