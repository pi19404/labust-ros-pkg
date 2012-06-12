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
#ifndef XMLTOOLS_HPP_
#define XMLTOOLS_HPP_

#include <labust/xml/xmlfwd.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/xml/XMLWriter.hpp>

#include <boost/foreach.hpp>
#include <boost/mpl/bool.hpp>

namespace labust
{
	namespace xml
	{
		/**
		 * The function converts a parameter oriented list into std::map.
		 *
		 * \param reader Pointer to the used XML reader.
		 * \param value The address of the std::map object.
		 * \param param Name of the key-value pair object.
		 * \param keyName Name of the key parameter.
		 * \param valueName Name of the value parameter.
		 *
		 * \tparam Type of the std::map. Allows polymorphism.
		 */
		template <class ReturnType>
		void param2map(labust::xml::ReaderPtr reader, ReturnType* value,
				const std::string& param = "param",
				const std::string& keyName = "@key",
				const std::string& valueName = "@value")
		{
			typedef typename ReturnType::key_type key_type;
			typedef typename ReturnType::mapped_type mapped_type;
			_xmlNode* cnode = reader->currentNode();

			NodeCollectionPtr nodes = reader->value<NodeCollectionPtr >(param);

			BOOST_FOREACH(_xmlNode* pt, *nodes)
			{
				reader->useNode(pt);
				value->insert( std::make_pair(
						reader->value< key_type >(keyName),
						reader->value< mapped_type >(valueName)));
			}

			reader->useNode(cnode);
		};
		/**
		 * Override to enable testing before parameter reading.
		 *
		 * \param reader Pointer to the used XML reader.
		 * \param value The address of the std::map object.
		 * \param param Name of the key-value pair object.
		 * \param keyName Name of the key parameter.
		 * \param valueName Name of the value parameter.
		 *
		 * \tparam Type of the std::map. Allows polymorphism.
		 */
		template <class ReturnType>
		bool param2map(labust::xml::ReaderPtr reader,
				const std::string& nodeName,
				ReturnType* value,
				const std::string& param = "param",
				const std::string& keyName = "@key",
				const std::string& valueName = "@value")
		try
		{
			typedef typename ReturnType::key_type key_type;
			typedef typename ReturnType::mapped_type mapped_type;
			_xmlNode* cnode = reader->currentNode();
			reader->useNode(reader->value<_xmlNode*>(nodeName));

			NodeCollectionPtr nodes = reader->value<NodeCollectionPtr >(param);

			BOOST_FOREACH(_xmlNode* pt, *nodes)
			{
				reader->useNode(pt);
				value->insert( std::make_pair(
						reader->value< key_type >(keyName),
						reader->value< mapped_type >(valueName)));
			}

			reader->useNode(cnode);
		}
		catch (std::exception& e){return false;};

		/**
		 * The struct wraps the object into the XML writer. With a specified packaging.
		 *
		 * \param writer The XML writer which to use.
		 * \param object The object that can will be added to the XML.
		 * \param type The XML header is determined from the type.
		 * \param id The identification name of the wrapping node.
		 *
		 * \tparam Value The value of the type.
		 */
		template <class Value>
		labust::xml::Writer& wrapInXml(labust::xml::Writer& writer, const Value& object,
			const std::string& id, const std::string& type,	boost::mpl::true_ eval= boost::mpl::true_())
		{
			writer.startElement(type);
			if (!id.empty()) writer.addAttribute("id",id);
			writer<<object;
			writer.endElement();
			return writer;
		};
		/**
		 * Specialization for types that do not have a XMLWriter operator<< defined.
		 */
		template <class Value>
		inline labust::xml::Writer& wrapInXml(labust::xml::Writer& writer, const Value& object,
				const std::string& id, const std::string& type,	boost::mpl::false_)
		{
			writer.setParamName(id).setParamType(type)<<object;
			return writer;
		}
		/**
		 * The function unwraps the object from the specified XML packaging.
		 *
		 * \param writer The XML writer which to use.
		 * \param object The object that can will be added to the XML.
		 * \param header The header name of the wrapping node.
		 * \param id The identification name of the wrapping node.
		 *
		 * \tparam Value The value of the type.
		 */
		template <class Value>
		bool unwrapFromXml(labust::xml::Reader& reader, Value& object,
				const std::string& id, const std::string& type, boost::mpl::true_ eval=boost::mpl::true_())
		{
			_xmlNode* org_node(reader.currentNode());
			_xmlNode* node(0);
			if (reader.try_value(id.empty()?type:type+"[@id='"+id+"']", &node))
			{
				reader.useNode(node);
				reader>>object;
				reader.useNode(org_node);
				return true;
			}
			return false;
		};
		/**
		 * Specialization for types that do not have a XMLReader operator>> defined.
		 */
		template <class Value>
		inline bool unwrapFromXml(labust::xml::Reader& reader, Value& object,
				const std::string& name, const std::string& type, boost::mpl::false_)
		{
			return reader.try_value("param[@name='"+name+"']/@value",&object);
		};
	}
}


/* XMLTOOLS_HPP_ */
#endif
