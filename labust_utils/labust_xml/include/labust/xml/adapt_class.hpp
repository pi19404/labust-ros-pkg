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
#ifndef ADAPT_CLASS_HPP_
#define ADAPT_CLASS_HPP_
#include <labust/xml/xmltools.hpp>
#include <labust/preprocessor/class_adaptor.hpp>

#include <boost/mpl/has_xxx.hpp>
BOOST_MPL_HAS_XXX_TRAIT_DEF(use_xml_operator)

#define PP_LABUST_SELECT_WRITE_TO_XML(R, ATTRIBUTE_TUPEL_SIZE, ATTRIBUTE) \
	labust::xml::wrapInXml(writer, object.BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,1,ATTRIBUTE),\
	PP_LABUST_MAKE_STRING_LVL2(BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,1,ATTRIBUTE)),\
	PP_LABUST_MAKE_STRING_LVL2(BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,0,ATTRIBUTE)),\
	has_use_xml_operator< BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,0,ATTRIBUTE) >::type());

#define PP_LABUST_SELECT_READ_TO_XML(R, ATTRIBUTE_TUPEL_SIZE, ATTRIBUTE) \
	labust::xml::unwrapFromXml(reader, object.BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,1,ATTRIBUTE),\
	PP_LABUST_MAKE_STRING_LVL2(BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,1,ATTRIBUTE)),\
	PP_LABUST_MAKE_STRING_LVL2(BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,0,ATTRIBUTE)),\
	has_use_xml_operator< BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,0,ATTRIBUTE) >::type());

#define PP_LABUST_MAKE_CLASS_XML_OPERATORS(NAMESPACE_SEQ, NAME, ATTRIBUTES) \
	PP_LABUST_NAMESPACE_DEFINITIONS_BEGIN((0)NAMESPACE_SEQ)\
	\
	labust::xml::Writer& operator<<(labust::xml::Writer& writer, const NAME& object)\
	{\
	  PP_LABUST_MACRO_ON_ATTRIBUTES(PP_LABUST_SELECT_WRITE_TO_XML,ATTRIBUTES)\
		return writer;\
	}\
	\
	labust::xml::Reader& operator>>(labust::xml::Reader& reader, NAME& object)\
	{\
		PP_LABUST_MACRO_ON_ATTRIBUTES(PP_LABUST_SELECT_READ_TO_XML,ATTRIBUTES)\
		return reader;\
	}\
	PP_LABUST_NAMESPACE_DEFINITIONS_END((0)NAMESPACE_SEQ)

#define PP_LABUST_IN_CLASS_ADD_FRIENDS(NAME)\
	friend labust::xml::Writer& operator<<(labust::xml::Writer& writer, const NAME& object);\
	friend labust::xml::Reader& operator>>(labust::xml::Reader& reader, NAME& object);\

#define PP_LABUST_IN_CLASS_ADAPT_TO_XML(NAME) \
	PP_LABUST_IN_CLASS_ADD_FRIENDS(NAME)\
	struct use_xml_operator; \
	inline labust::xml::StringPtr wrapInXml(const std::string& id = "")\
	{\
		labust::xml::Writer writer;\
		labust::xml::wrapInXml(writer,*this,id,#NAME);\
		writer.endDocument();\
		return writer.toStringPt();\
	}\
	\
	inline void unwrapFromXml(const std::string& data, const std::string& id = "")\
	{\
		labust::xml::Reader reader(data);\
		bool isAbsolute(reader.try_expression("/"#NAME));\
		labust::xml::unwrapFromXml(reader,*this,id,(isAbsolute?"/"#NAME:"//"#NAME));\
	}\

#define PP_LABUST_MAKE_STRUCT_WITH_XML(NAMESPACE_SEQ, NAME, ATTRIBUTES)\
		PP_LABUST_NAMESPACE_DEFINITIONS_BEGIN((0)NAMESPACE_SEQ)\
			struct NAME\
			{\
				PP_LABUST_IN_CLASS_ADAPT_TO_XML(NAME)\
				PP_LABUST_MACRO_ON_ATTRIBUTES(PP_LABUST_ATTRIBUTE_EXPAND,ATTRIBUTES)\
			};\
		PP_LABUST_NAMESPACE_DEFINITIONS_END((0)NAMESPACE_SEQ)\
		PP_LABUST_MAKE_CLASS_XML_OPERATORS(NAMESPACE_SEQ, NAME, ATTRIBUTES)

/* ADAPT_CLASS_HPP_ */
#endif
