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
#ifndef GYROSWRITER_HPP_
#define GYROSWRITER_HPP_

#include <labust/xml/Gyros.hpp>
#include <labust/xml/GyrosMatrix.hpp>
#include <labust/xml/XMLWriter.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>

namespace labust
{
    namespace xml
    {
        /**
         * The XML::GyrosWriter class helps in constructing a Gyros string object.
         * The Gyros string object is a subset of the XML language used for gadget
         * communication.
         */
        class GyrosWriter : public Gyros
        {
        public:

            /**
             * This constructor creates a Gyros matrix object from the supplied parameter.
             * Supplied elements have to be of GyrosMatrix helper type
             *
             * \param matrix matrix to use
             */
            GyrosWriter(GyrosMatrix &matrix);

            /**
             * This constructor creates a scalar Gyros object from the supplied parameter.
             *
             * \param scalar The value that will be inserted into the Gyros object.
             * \param error if set, will generate an error object and use scalar as its description, optional, default false
             */
            template <class Scalar>
            GyrosWriter(const Scalar& scalar, bool error = false) :
            xml_data(new Writer())
            {
                if (error)
                {
                    this->type = GYROS::error;
                    xml_data->addElement("error", scalar);
                }
                else
                {
                    this->type = GYROS::scalar;
                    xml_data->addElement("scalar", scalar);
                }
                xml_data->endDocument();
            }

            /**
             * This constructor creates a dictionary Gyros object from the supplied parameters.
             * Supplied elements have to be <Key, Value> pairs.
             *
             * \param begin Pointer to the starting value
             * \param end Pointer to the one beyond the end element.
             */
            template <class Iterator>
            GyrosWriter(Iterator begin, Iterator end) :
            xml_data(new Writer())
            {
                this->type = GYROS::dictionary;

                xml_data->startElement("dictionary");

                for (Iterator it = begin; it != end; ++it)
                {
                    xml_data->startElement("element");
                    xml_data->addAttribute("key", it->first);
                    xml_data->addValue(it->second);
                    xml_data->endElement();
                }

                xml_data->endDocument();
            }

            /**
             * This constructor creates a Gyros binary object from the supplied parameters.
             * Supplied elements have to be <Key, Value> pairs.
             *
             * \param begin Pointer to the starting value
             * \param end Pointer to the one beyond the end element.
             * \param encoding Binary encoding method that should be used.
             * \param descriptor descriptor of the data type, optional, default "raw" dataType element of gyros is set to "base64/[descriptor]
             * \param metadataMap of string-string params to describe the data, optional
             */
            template <class Iterator>
            GyrosWriter(Iterator begin, Iterator end, GYROS::Encoding encoder, std::map<std::string, std::string> metadata = std::map<std::string, std::string>()) :
            xml_data(new Writer())
            {
                xml_data->startElement("binary");

                if (!metadata.empty())
                {
                    xml_data->startElement("metadata");
                    for (std::map<std::string, std::string>::iterator metadataElement = metadata.begin(); metadataElement != metadata.end(); metadataElement++)
                    {
                        xml_data->startElement("var");
                        xml_data->addAttribute("name", metadataElement->first);
                        xml_data->addValue(metadataElement->second);
                        xml_data->endElement();

                    }
                    xml_data->endElement();
                }

                switch (encoder)
                {//do encoding stuff and add
                    case GYROS::raw:
                    {
                        xml_data->addElement("datatype", "base64/raw");
                    }
                        break;
                    case GYROS::jpg:
                    {
                        xml_data->addElement("datatype", "base64/jpg");
                    }
                        break;
                    case GYROS::png:
                    {
                        xml_data->addElement("datatype", "base64/png");
                    }
                        break;
                    case GYROS::text:
                    {
                        xml_data->addElement("datatype", "base64/text");
                    }
                        break;
                    default:
                    {
                        xml_data->addElement("datatype", "base64/raw");
                    }
                        break;
                }
                xml_data->startElement("data");
                xml_data->addB64Value(begin, end);
                xml_data->endElement();
                xml_data->endDocument();
            }

            /**
             * The destructor frees the memory used by the XML writer structure
             */
            ~GyrosWriter()
            {
            };

            /**
             * This returns the XML encoding of the Gyros object.
             */
            inline const std::string& GyrosXML() const
            {
                return ( ((lastHeader.first != label) || (lastHeader.second != timestamp)) ? toGyros() : gyros_xml);
            };

        private:
            /**
             * Encode to Gyros object.
             */
            const std::string& toGyros() const;

            /**
             * Data contained in the Gyros object. When we request a Gyros object the
             * whole XML encoding will be saved under gyros_xml to enable reuse.
             */
            boost::shared_ptr<Writer> xml_data;
            /**
             * Data that contains the last GyrosXML construct. We set it mutable since
             * getting a gyros object from a constant should be possible. On the other
             * hand we do XML encoding on request or change of the document.
             */
            mutable std::string gyros_xml;
            /**
             * Last timestamp and label. Same reason for this to be mutable as for the
             * gyros_xml object.
             */
            mutable std::pair<std::string, double> lastHeader;
        };
    }
}
/* GYROSWRITER_HPP_ */
#endif
