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
#ifndef DEFINEXMLSTRUCT_HPP_
#define DEFINEXMLSTRUCT_HPP_
#include <labust/xml/XMLStructureFwd.hpp>

#define PP_LABUST_WRITE_NAMED_ELEM_TO_XML(R, ATTRIBUTE_TUPEL_SIZE, ATTRIBUTE) \
	pwriter.setParamName(PP_LABUST_MAKE_STRING_LVL2(BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,1,ATTRIBUTE))); \
	pwriter.setParamType( PP_LABUST_MAKE_STRING_LVL2(BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,0,ATTRIBUTE))); \
	pwriter<<this->BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,1,ATTRIBUTE); \

#define PP_LABUST_READ_NAMED_ELEM_FROM_XML(R, ATTRIBUTE_TUPEL_SIZE, ATTRIBUTE) \
	preader.setParamName(PP_LABUST_MAKE_STRING_LVL2(BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,1,ATTRIBUTE))); \
	preader>>&this->BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,1,ATTRIBUTE); \

#define PP_LABUST_MAKE_XML_STRUCTURE(NAMESPACE_SEQ, NAME, ATTRIBUTES) \
		PP_LABUST_NAMESPACE_DEFINITIONS_BEGIN((0)NAMESPACE_SEQ) \
		\
		class NAME \
		{\
			public:\
			labust::xml::StringPtr write(const std::string& id = "", bool sendType = false)\
			{\
				labust::xml::Writer writer; \
				writer.startElement(#NAME); \
				if (!id.empty()) writer.addAttribute("id",id); \
			  if (sendType) writer.addAttribute("type",#NAME); \
				labust::xml::ParamWriter pwriter(writer);\
  			PP_LABUST_MACRO_ON_ATTRIBUTES(PP_LABUST_WRITE_NAMED_ELEM_TO_XML,ATTRIBUTES)\
  			return pwriter.getStringPtr(); \
			}\
			\
			void read(const std::string& str, const std::string& id = "") \
			{ \
				labust::xml::Reader reader(str); \
				this->read(reader,id); \
			} \
			void read(labust::xml::Reader& reader, const std::string& id = "") \
			{ \
				_xmlNode* org_node = reader.currentNode(); \
				_xmlNode* head(0); \
				std::string headStr("/"#NAME); \
				try \
				{ \
					head = reader.value<_xmlNode*>(headStr); \
				} \
				catch (std::exception& e){} \
				\
				if (!head) headStr = #NAME; \
				reader.useNode(reader.value<_xmlNode*>(id.empty()?headStr:headStr + "[@id='" + id + "']")); \
				labust::xml::ParamReader preader(reader); \
				PP_LABUST_MACRO_ON_ATTRIBUTES(PP_LABUST_READ_NAMED_ELEM_FROM_XML,ATTRIBUTES) \
				reader.useNode(org_node);\
			} \
			PP_LABUST_MACRO_ON_ATTRIBUTES(PP_LABUST_ATTRIBUTE_EXPAND,ATTRIBUTES)\
 	  };\
 	  \
		labust::xml::ParamReader& operator>>(labust::xml::ParamReader& reader, NAME* object) \
		{ \
 	  	object->read(reader.getReader(), reader.getParamName()); \
 	  	return reader; \
		} \
		labust::xml::ParamWriter& operator<<(labust::xml::ParamWriter& writer, NAME& object) \
		{ \
			writer.appendXML(*object.write(writer.getParamName())); \
			return writer; \
		} \
	  PP_LABUST_NAMESPACE_DEFINITIONS_END((0)NAMESPACE_SEQ)

/* DEFINEXMLSTRUCT_HPP_ */
#endif
