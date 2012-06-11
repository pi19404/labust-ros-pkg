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
#include <labust/xml/XMLReader.hpp>
#include <labust/xml/XMLException.hpp>

#include <libxml/xpath.h>
#include <sstream>


using namespace labust::xml;

Reader::Reader(const std::string& xmlData, bool isFile)
{
  //Create document
  initDoc(xmlData,isFile);
  root_node = xpath_context->node;
}

Reader::Reader(const std::string& xmlData, const std::string& subNode, bool isFile)
{
  initDoc(xmlData,isFile);
  root_node = xpath_context->node = value<_xmlNode*>(subNode);
}

Reader::~Reader()
{}

void Reader::initDoc(const std::string& xmlData, bool isFile)
{
  xmlInitParser();
  LIBXML_TEST_VERSION;
  //Create document
  if (isFile)
  {
  	document.reset(xmlParseFile(xmlData.c_str()), std::ptr_fun(xmlFreeDoc));
  }
  else
  {//causes memory leak but prevents fault
  	document.reset(xmlReadMemory(xmlData.c_str(), xmlData.length(), "reader.xml", NULL, 0),std::ptr_fun(xmlFreeDoc));
  }

  if (!document)
  {
    throw XMLException("Error initializing XML document.");
  };

  xpath_context.reset(xmlXPathNewContext(document.get()), std::ptr_fun(xmlXPathFreeContext));
  if (!xpath_context)
  {
    throw XMLException("Error creating XPath context.");
  }
}


_xmlNode* Reader::useNode(_xmlNode* node)
{
	xmlNode* cnode = xpath_context->node;
	xpath_context->node = node;
	return cnode;
}

void Reader::useRootNode()
{
	xpath_context->node = root_node;
};

_xmlNode* Reader::currentNode() const
{
	return xpath_context->node;
}

const char* Reader::evaluate(const char* xpath_expression) const
{
	const char* content;
	if (this->evaluate(xpath_expression, &content))
	{
		return content;
	}
	else
	{
		throw XMLException(lastErrorMessage);
	}
}

bool Reader::evaluate(const char* xpath_expression, const char** content) const
{
  xmlXPathObjectPtr xpath_object(
  		xmlXPathEvalExpression(BAD_CAST xpath_expression, this->xpath_context.get()),
  		std::ptr_fun(xmlXPathFreeObject));

  //If the XPath evaluation return false.
  if (!xpath_object)
  {
  	lastErrorMessage = std::string("XPATH expression: '") +	xpath_expression + std::string("' failed evaluation.");
  	return false;
  }

  //Using switch for better readability
  switch (xpath_object->type)
  {
  case XPATH_NODESET:
    if (xpath_object->nodesetval && (xpath_object->nodesetval->nodeNr > 0))
    {
    	return this->content(xpath_object->nodesetval->nodeTab[0], content);
    }
    else
    {
   		lastErrorMessage = std::string("XPATH node set is empty or undefined. When evaluating:'") +	xpath_expression + std::string("'");
      return false;
    }
  case XPATH_STRING: *content = reinterpret_cast<const char*>(xpath_object->stringval); return true;
  default:
  		lastErrorMessage = std::string("Unknown XPATH expression type. When evaluating:'") + xpath_expression + std::string("'. We cover only the XPATH_NODESET and XPATH_STRING types.");
  	return false;
  }
}

bool Reader::evaluate2nodeset(const char* xpath_expression, NodeCollectionPtr& nodeCollection) const
{
  xmlXPathObjectPtr xpath_object(
  		xmlXPathEvalExpression(BAD_CAST xpath_expression, this->xpath_context.get()),
  		std::ptr_fun(xmlXPathFreeObject));

  //If the XPath evaluation failed throw.
  if (!xpath_object)
  {
  	lastErrorMessage = (std::string("XPATH expression: '") + xpath_expression + std::string("' failed evaluation."));
  	return false;
  }

  if (xpath_object->type == XPATH_NODESET)
  {
  	if (xpath_object->nodesetval && (xpath_object->nodesetval->nodeNr > 0))
    {
  		_xmlNode** begin = &xpath_object->nodesetval->nodeTab[0];
      _xmlNode** end = &xpath_object->nodesetval->nodeTab[xpath_object->nodesetval->nodeNr];
      nodeCollection.reset(new std::vector<_xmlNode*>(begin,end));
      return true;
    }
    else
    {
    	lastErrorMessage = std::string("XPATH node set is empty or undefined. When evaluating:'") +
    			xpath_expression + std::string("'");
    	return false;
    }
  }
  else
  {
   lastErrorMessage = std::string("XPATH expression: '") + xpath_expression + std::string("' did not evaluate to a node set.");
   return false;
  }
}

const char* Reader::content(const _xmlNode* const xml_node) const
{
	const char* content(0);
	if (this->content(xml_node, &content))
	{
		return content;
	}
	else
	{
		throw XMLException(lastErrorMessage);
	}
}

bool Reader::content(const _xmlNode* const xml_node, const char** content) const
{
	//Using switch for better readability
	switch (xml_node->type)
	{
	case XML_ELEMENT_NODE:
    if (xml_node->children && (xml_node->children[0].type == XML_TEXT_NODE))
    {
      //Try to return a child content text
    	*content = reinterpret_cast<const char*>(xml_node->children[0].content);
      return true;
    }
    else
    {
    	lastErrorMessage = "Can not return content of node that has no text.";
      return false;
    }
	case XML_ATTRIBUTE_NODE: *content = reinterpret_cast<const char*>(xml_node->children[0].content); return true;
	case XML_TEXT_NODE: *content = reinterpret_cast<const char*>(xml_node->content); return true;
	default:
		lastErrorMessage = "Can not return content to unhandled type.";
		return false;
	}
}
