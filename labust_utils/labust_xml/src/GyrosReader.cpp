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
#include <labust/xml/Base64Decoder.hpp>
#include <labust/xml/GyrosError.hpp>
#include <labust/xml/GyrosMatrix.hpp>
#include <labust/tools/StringUtilities.hpp>
#include <labust/xml/GyrosReader.hpp>
#include <algorithm>

#include <vector>

using namespace labust::xml;

GyrosReader::GyrosReader(const std::string& xmlEncoded) :
reader(new Reader(xmlEncoded, std::string("//gyros")))
{
    reader->try_value("@label", &this->label);
    reader->try_value("@timestamp", &this->timestamp);

    _xmlNode* gyros_type = 0;

    if (reader->try_value("scalar", &gyros_type))
    {
        this->type = GYROS::scalar;
    }
    else if (reader->try_value("dictionary", &gyros_type))
    {
        this->type = GYROS::dictionary;
        reader->useNode(gyros_type);
    }
    else if (reader->try_value("binary", &gyros_type))
    {
        this->type = GYROS::binary;
        reader->useNode(gyros_type);
    }
    else if (reader->try_value("matrix", &gyros_type))
    {
        this->type = GYROS::matrix;

    }
    else if (reader->try_value("error", &gyros_type))
    {
        this->type = GYROS::error;
        reader->useNode(gyros_type);
    }
}

GyrosReader::~GyrosReader(){}

void GyrosReader::binary(GyrosBinaryReturn &output)
{
    if (type == GYROS::error)
    {
		_xmlNode* startNode = reader->currentNode();
		  std::string errorMessage = reader->value<std::string>("."); 
		  reader->useNode(startNode);
		  throw GyrosError(errorMessage,this->label, this->timestamp);
    }
    else if (type == GYROS::binary)
    {
		_xmlNode* startNode = reader->currentNode();
		
        _xmlNode *datatype = NULL, *data = NULL, *metadata = NULL;
        reader->try_value("metadata", &metadata);

        if (!reader->try_value("datatype", &datatype))
        {
            throw GyrosError("Binary data missing datatype", this->label, this->timestamp);
        }

        if (!reader->try_value("data", &data))
        {
            throw GyrosError("Binary data missing data", this->label, this->timestamp);
        }

        if (metadata != NULL)
        {
            reader->useNode(metadata);
            NodeCollectionPtr vars = reader->value<NodeCollectionPtr > ("var");

            BOOST_FOREACH(_xmlNode* pt, *vars)
            {
            		reader->useNode(pt);
                output.metadata.insert(std::make_pair(
                        reader->value<std::string > ("@name"),
                        reader->value<std::string > ("text()")));
            }
        }

        reader->useNode(datatype);
        std::string typeString = reader->value<std::string > (".");
        if (boost::iequals(typeString, "base64/raw"))
        {
            output.encoding = GYROS::raw;
        }
        else if (boost::iequals(typeString, "base64/jpg"))
        {
            output.encoding = GYROS::jpg;
        }
        else if (boost::iequals(typeString, "base64/png"))
        {
            output.encoding = GYROS::png;
        }
        else if (boost::iequals(typeString, "base64/text"))
        {
            output.encoding = GYROS::text;
        }
        else
        {
            throw GyrosError("Binary data has invalid datatype", this->label, this->timestamp);
        }

        reader->useNode(data);
        std::string encodedData = reader->value<std::string > (".");
        output.data.clear();
	std::remove(encodedData.begin(),encodedData.end(),'\n');
        output.data = Base64_decode(encodedData);  
		reader->useNode(startNode);
    }
    else
    {
        throw GyrosError("Gyros object is not of binary type.", this->label, this->timestamp);
    }
}

template<>
boost::shared_ptr<std::string> GyrosReader::matrix<boost::shared_ptr<std::string> >()
{
    if (type == GYROS::matrix)
    {
		_xmlNode* startNode = reader->currentNode(); 
		  
      boost::shared_ptr<std::string> retVal(new std::string);
      std::string& matrixData = *retVal;

      matrixData += "Gyros matrix\n";

        if (!this->label.empty())
        {
            matrixData += "Title: " + this->label + "\n";
        }
        if ((this->timestamp) != -1)
        {
            matrixData += "Timestamp: " + labust::tools::to_string(this->timestamp) + "\n";
        }
        std::vector<std::vector<std::string> > data;
        GyrosMatrix matrix(*reader);
        matrix.GetData(data);
        for (std::vector<std::vector<std::string> >::iterator row = data.begin(); row != data.end(); row++)
        {
            for (std::vector<std::string>::iterator col = row->begin(); col != row->end(); col++)
            {
                matrixData += *col + " ";
            }
            matrixData += "\n";
        }
		reader->useNode(startNode);
        return retVal;
    }
    else if (type == GYROS::error)
    {
        _xmlNode* startNode = reader->currentNode();
		  std::string errorMessage = reader->value<std::string>("."); 
		  reader->useNode(startNode);
		  throw GyrosError(errorMessage,this->label, this->timestamp);
    }
    else
    {
        throw GyrosError("Gyros object is not of matrix type.", this->label, this->timestamp);
    }
}

template<>
boost::shared_ptr<GyrosMatrix> GyrosReader::matrix<boost::shared_ptr<GyrosMatrix> >()
{
    if (type == GYROS::matrix)
    {
		return boost::shared_ptr<GyrosMatrix>(new GyrosMatrix(*reader));
    }
    else if (type == GYROS::error)
    {
        throw GyrosError(reader->value<std::string > ("."), this->label, this->timestamp);
    }
    else
    {
        throw GyrosError("Gyros object is not of matrix type.", this->label, this->timestamp);
    }
}
