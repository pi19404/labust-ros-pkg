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
#ifndef XMLSTRUCTUREFWD_HPP_
#define XMLSTRUCTUREFWD_HPP_
#include <boost/preprocessor/tuple/elem.hpp>
#include <boost/preprocessor/seq/for_each_i.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/dec.hpp>
#include <boost/preprocessor/seq/size.hpp>
#include <boost/preprocessor/repetition/repeat.hpp>
#include <boost/preprocessor/control/if.hpp>
#include <labust/xml/XMLWriter.hpp>
#include <labust/xml/XMLReader.hpp>

#define PP_LABUST_NAMESPACE_BEGIN(R,DATA,ELEM) \
	namespace ELEM{

#define PP_LABUST_NAMESPACE_END(Z,I,DATA) }

#define PP_LABUST_MAKE_STRING_LVL1(string) #string
#define PP_LABUST_MAKE_STRING_LVL2(string) PP_LABUST_MAKE_STRING_LVL1(string)

#define PP_LABUST_FILLER_0(X, Y) \
	((X, Y)) PP_LABUST_FILLER_1
#define PP_LABUST_FILLER_1(X, Y) \
	((X, Y)) PP_LABUST_FILLER_0
#define PP_LABUST_FILLER_0_END
#define PP_LABUST_FILLER_1_END

#define PP_LABUST_ATTRIBUTE_EXPAND(R, ATTRIBUTE_TUPEL_SIZE, ATTRIBUTE)   \
  BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,0,ATTRIBUTE) \
	BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,1,ATTRIBUTE);

#define PP_LABUST_NAMESPACE_DEFINITIONS_BEGIN(NAMESPACE_SEQ) \
	BOOST_PP_IF(																			 	\
		BOOST_PP_DEC(BOOST_PP_SEQ_SIZE(NAMESPACE_SEQ)),		\
    BOOST_PP_SEQ_FOR_EACH_R,													\
    BOOST_PP_TUPLE_EAT(4))(														\
    	1, PP_LABUST_NAMESPACE_BEGIN,													\
      _, BOOST_PP_SEQ_TAIL(NAMESPACE_SEQ))

#define PP_LABUST_NAMESPACE_DEFINITIONS_END(NAMESPACE_SEQ)									\
	BOOST_PP_REPEAT_1(         												\
		BOOST_PP_DEC(BOOST_PP_SEQ_SIZE(NAMESPACE_SEQ)),	\
    PP_LABUST_NAMESPACE_END, _)

#define PP_LABUST_MACRO_ON_ATTRIBUTES(MACRO,ATTRIBUTES) \
	BOOST_PP_SEQ_FOR_EACH_R(1, MACRO, \
		2, BOOST_PP_CAT(PP_LABUST_FILLER_0 ATTRIBUTES, _END))

namespace labust
{
	namespace xml
	{
		///Deprecated
		class ParamReader
		{
		public:
			ParamReader(labust::xml::Reader& reader):
				reader(reader){};

			void setParamName(const std::string& name)
			{
				this->paramName=name;
			}

			const std::string& getParamName()
			{
				return this->paramName;
			}

			labust::xml::Reader& getReader(){return this->reader;};

			const std::string& getParamType(){return this->paramType;};

			template<class ValueType>
			inline ParamReader& operator>>(ValueType* const value)
			{
				reader.try_value("param[@name='" + paramName + "']/@value",value);
				reader.try_value("param[@name='" + paramName + "']/@type",&paramType);
				return *this;
			}

		private:
			labust::xml::Reader& reader;
			std::string paramName;
			std::string paramType;
		};

		//Deprecated
		class ParamWriter
		{
		public:
			ParamWriter(labust::xml::Writer& writer):
				writer(writer){};

			inline void setParamName(const std::string& name)
			{
				this->paramName=name;
			}

			inline const std::string& getParamName()
			{
				return this->paramName;
			}

			inline void setParamType(const std::string& paramType){this->paramType=paramType;};

			template<class ValueType>
			inline ParamWriter& operator<<(const ValueType&  value)
			{
				writer.startElement("param");
				 writer.addAttribute("name",paramName);
				 writer.addAttribute("value",value);
				 writer.addAttribute("type",paramType);
				writer.endElement();
				return *this;
			}

			inline labust::xml::StringPtr getStringPtr()
			{
				writer.endDocument();
				return writer.toStringPt();
			}

			inline void appendXML(const std::string& xml)
			{
				writer.addXML(xml);
			}

		private:
			labust::xml::Writer& writer;
			std::string paramName;
			std::string paramType;
		};
	}
}

/* XMLSTRUCTUREFWD_HPP_ */
#endif
