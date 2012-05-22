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
#ifndef XMLWRITER_HPP_
#define XMLWRITER_HPP_
#include <labust/xml/xmlfwd.hpp>
#include <labust/xml/XMLException.hpp>

#include <boost/type_traits/is_float.hpp>
#include <boost/type_traits/is_convertible.hpp>
#include <boost/static_assert.hpp>
#include <boost/noncopyable.hpp>

#include <string>
#include <sstream>
#include <map>

namespace labust
{
	namespace xml
	{
		/**
		 * The labust::xml::Writer creates and writes to a XML string. Data can be written to memory or
		 * a file. When the XML object is assembled you can output and close it. The object is reusable
		 * but the data is lost after the endDocument call.
		 *
		 * \todo Document methods and variables.
		 * \todo Check the B64 encoding.
		 */
		class Writer : boost::noncopyable
		{
		public:
			/**
			 * Main constructor that creates a XML memory writer. If the memory cannot be instantiated
			 * the constructor throws a labust::xml::XMLException.
			 */
			Writer();
			/**
			 * This constructor opens a document with a specified filename and instantiates the XML text writer.
			 * If the contruction fails the throws a labust::xml::XMLException.
			 *
			 * \param path Path of the desired XML document.
			 */
			Writer(const std::string& path);
			/**
			 * Generic destructor.
			 */
			~Writer();

			/**
			 * The method writes the XML document header. This is only used for XML files. The XML memory
			 * objects does not get this header for efficiency reasons. If the header write fails the
			 * method throws a labust::xml::XMLException.
			 */
			void startDocument();
			/**
			 * The method ends the document. Data is flushed from the buffer into the string. The flushed data
			 * can not be reused. If the method fails it throws a labust::xml::XMLException.
			 */
			void endDocument();

			/**
			 * The method opens a new element with given name. If the method fails it throws a labust::xml::XMLException.
			 *
			 * \param name Name of the new element.
			 */
			void startElement(const std::string& name);
			/**
			 * The method ends the currently open element. If the method fails it throws a labust::xml::XMLException.
			 */
			void endElement();

		 	/**
			 * Insert an XML value. When inserting XML as a simple string the
			 * XML writer will replace the XML specific characters.
			 */
			 void addXML(const std::string& value);

			/**
			 * The methods adds an attribute to the currently open element.
			 * If the method fails it throws a labust::xml::XMLException.
			 *
			 * \param name Name of the element attribute.
			 * \param value Desired value of the attribute.
			 *
			 * \tparam ValueType Type of the attribute value.
			 */
			template<class ValueType>
			inline void addAttribute(const std::string& name, const ValueType& value)
			{
				std::ostringstream str;

				if (boost::is_float<ValueType>::value)
				{
					str.precision(6);
					str<<std::fixed;
				}

				str<<value;
				this->writeAttribute(name, str.str());
			}
			/**
			 * The method adds a list of attributes to the current element. The attributes should consist of
			 * name-value pairs. If the method fails it throws a labust::xml::XMLException.
			 *
			 * \param begin First element of the attribute list.
			 * \param end Last element of the attribute list.
			 */
			template <class IteratorType>
			inline void addAttributePairs(const IteratorType begin, const IteratorType end)
			{
				for (IteratorType it = begin; it != end; ++it)
				{
					this->addAttribute(it->first, it->second);
				}
			};

			/**
			 * The methods adds an subelement to the currently open element.
			 * If the method fails it throws a labust::xml::XMLException.
			 *
			 * \param name Name of the element.
			 * \param value Desired value of element.
			 *
			 * \tparam ValueType Type of the attribute value.
			 */
			template <class ValueType>
			inline void addElement(const std::string& name, const ValueType& value)
			{
				std::ostringstream str;

				if (boost::is_float<ValueType>::value)
				{
					str.precision(6);
					str << std::fixed;
				}

				str << value;
				this->writeElement(name,str.str());
			}
			/**
			 * The method adds a list of subelements to the current element. The subelements should consist of
			 * name-value pairs. If the method fails it throws a labust::xml::XMLException.
			 *
			 * \param begin First element of the attribute list.
			 * \param end Last element of the attribute list.
			 */
			template <class IteratorType>
			inline void addElementPairs(IteratorType begin, IteratorType end)
			{
				for (IteratorType it = begin; it != end; ++it)
				{
					this->addElement(it->first, it->second);
				}
			};

			/**
			 * The methods adds a value to the currently open element.
			 * If the method fails it throws a labust::xml::XMLException.
			 *
			 * \param value Desired value to be added to the element.
			 *
			 * \tparam ValueType Type of the attribute value.
			 */
			template <class ValueType>
			inline void addValue(const ValueType& value)
			{
				std::ostringstream str;

				if (boost::is_float<ValueType>::value)
				{
					str.precision(6);
					str << std::fixed;
				}

				str << value;
				this->writeString(str.str());
			}
			/**
			 * The method adds a list of values to the current element. The values should be a
			 * simple list. If the method fails it throws a labust::xml::XMLException.
			 *
			 * \param begin First element of the attribute list.
			 * \param end Last element of the attribute list.
			 */
			template <class IteratorType>
			inline void addValue(const IteratorType begin, const IteratorType end)
			{
				for (IteratorType it = begin; it != end; ++it)
				{
					this->addValue(*it);
				}
			};
			/**
			 * The method encodes the data to a base64 string. If the method fails it throws a
			 * labust::xml::XMLException.
			 *
			 * \param begin First element of the attribute list.
			 * \param end Last element of the attribute list.
			 */
			template <class IteratorType>
			inline void addB64Value(const IteratorType begin, const IteratorType end)
			{
				//We allow only random access iterators
				typedef typename std::iterator_traits<IteratorType>::iterator_category racat;
				BOOST_STATIC_ASSERT((boost::is_convertible<racat, const std::random_access_iterator_tag&>::value));

				//Get byte size
				int size = std::distance(begin,end) * sizeof(*begin);
				//Do some unsafe stuff
				const char* pbegin(reinterpret_cast<const char*>(&(*begin)));

				this->encodeB64(pbegin,size);
			}

			/**
			 * The method flushes the buffer into a XML string and returns the reference to it.
			 * The method ends the XML document and repeated calls will return the same XML string.
			 *
			 * \return Pointer to string with the XML encoded data.
			 */
			inline boost::shared_ptr<std::string> toStringPt() const
			{
				assert(!writer.get() && "Can not read string until you call endDocument().");
				return xml_final;
			}
			/**
			 * The method returns the XML encoded data in a string.
			 *
			 * \return String with the XML encoded data.
			 */
			inline const std::string& toString() const
			{
				assert(!writer.get() && "Can not read string until you call endDocument().");
				return *xml_final;
			}

		private:
			/**
			 * The method writes a new XML attribute with the given name and data. If the method fails it throws a
			 * labust::xml::XMLException.
			 *
			 * \param name Name of the XML element.
			 * \param value Value of the XML element.
			 */
			void writeAttribute(const std::string& name, const std::string& value);
			/**
			 * The method writes a new XML element with the given name and data. If the method fails it throws a
			 * labust::xml::XMLException.
			 *
			 * \param name Name of the XML element.
			 * \param value Value of the XML element.
			 */
			void writeElement(const std::string& name, const std::string& value);
			/**
			 * The method writes a value to the open XML element. If the method fails it throws a
			 * labust::xml::XMLException.
			 *
			 * \param value Value to be written to the element.
			 */
			void writeString(const std::string& value);
			/**
			 * The method encodes a character array to a Base64 string and writes it to
			 * the open XML element. If the method fails it throws a labust::xml::XMLException.
			 *
			 * \param data The pointer to the data.
			 * \param size Binary length of the data.
			 */
			void encodeB64(const char* data, size_t size);

			/**
			 * Pointer to the text writer object.
			 */
			xmlTextWriterPtr writer;
			/**
			 * Pointer to the XML buffer object.
			 */
			xmlBufferPtr xml_buffer;
			/**
			 * The global encoding used in the XML file.
			 */
			static const std::string encoding;
			/**
			 * The container for the XML encoded data.
			 */
			boost::shared_ptr<std::string> xml_final;

			/**
			 * Output operator for convenience.
			 */
			friend std::ostream& operator<<(std::ostream& out, const Writer& object);
		};

		/**
		 * Outputs the contents of the Writer into an output stream.
		 */
		std::ostream& operator<<(std::ostream& out, const Writer& object);
	}
}
/* XMLWRITER_HPP_ */
#endif
