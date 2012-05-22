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
#ifndef GYROSREADER_HPP_
#define GYROSREADER_HPP_

#include "labust/xml/Gyros.hpp"
#include "labust/xml/GyrosError.hpp"
#include "labust/xml/XMLReader.hpp"
#include "labust/xml/xmlfwd.hpp"

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>

namespace labust
{
namespace xml
{
struct GyrosBinaryReturn
{
	std::vector<char> data;
	GYROS::Encoding encoding;
	std::map<std::string, std::string> metadata;
};

/**
 * The XML::GyrosReader class helps in constructing a Gyros string object.
 * The Gyros string object is a subset of the XML language used for gadget
 * communication.
 *
 * \todo Check for const correctness
 */
class GyrosReader : public Gyros
{
public:
	/**
	 * Main constructor that takes the string encoding of a Gyros object
	 * and performs decoding.
	 */
	GyrosReader(const std::string& xmlEncoded);

	~GyrosReader();

      /**
       * This method returns the requested object from a scalar value.
       *
       * \param value Object into which we have to copy the
       */
      template <class ReturnType>
      void scalar(ReturnType& value)
      {

		if (type == GYROS::scalar)
		{
			_xmlNode* startNode = reader->currentNode();
			std::stringstream out(reader->value<std::string > ("scalar"));
			out >> value;
			reader->useNode(startNode);
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
			throw GyrosError("Gyros object is not of scalar type.",this->label, this->timestamp);
		}
	}

	/**
	 * This method returns the requested object from a scalar value.
	 *
	 * \return Value of the Gyros object
	 */
	template <class ReturnType>
	ReturnType scalar()
	{

		if (type == GYROS::scalar)
		{
			_xmlNode* startNode = reader->currentNode();
			ReturnType retVal;
			std::stringstream out(reader->value<std::string > ("scalar"));
			reader->useNode(startNode);
			out >> retVal;
			return retVal;
		}
		else if(type==GYROS::error)
		{
			_xmlNode* startNode = reader->currentNode();
			std::string errorMessage = reader->value<std::string>(".");
			reader->useNode(startNode);
			throw GyrosError(errorMessage,this->label, this->timestamp);
		}
		else
		{
			throw GyrosError("Gyros object is not of scalar type.",this->label, this->timestamp);
		}
	}

	/**
	 * This method returns the requested object from a dictionary value.
	 *
	 * \param value Object into which we have to copy the
	 */
	template <class ReturnType>
	void dictionary(ReturnType& value) const
	{
		typedef typename ReturnType::key_type key_type;
		typedef typename ReturnType::mapped_type mapped_type;
		_xmlNode* startNode = reader->currentNode();
		if (type == GYROS::dictionary)
		{
			_xmlNode* startNode = reader->currentNode();
			NodeCollectionPtr nodes = reader->value<NodeCollectionPtr > ("element");

			BOOST_FOREACH(_xmlNode* pt, *nodes)
			{
				reader->useNode(pt);
				key_type key = reader->value<key_type > ("@key");
				mapped_type data = reader->value<mapped_type > ("text()");
				value.insert(std::make_pair(key,data));
			}
			reader->useNode(startNode);
		}
		else if(type==GYROS::error)
		{
			_xmlNode* startNode = reader->currentNode();
			std::string errorMessage = reader->value<std::string>(".");
			reader->useNode(startNode);
			throw GyrosError(errorMessage,this->label, this->timestamp);
		}
		else
		{
			throw GyrosError("Gyros object is not of dictionary type.",this->label, this->timestamp);
		}
		reader->useNode(startNode);
	}

	void binary(GyrosBinaryReturn &output);

	/**
	 * This method returns the requested object from a matrix value.
	 *
	 * \return Value of the Gyros object
	 */
	template <class ReturnType> ReturnType matrix();

private:
	/**
	 * The XML reader that will contain the data.
	 */
	boost::shared_ptr<Reader> reader;
};

template<>
boost::shared_ptr<GyrosMatrix> GyrosReader::matrix<boost::shared_ptr<GyrosMatrix> >();

template<>
boost::shared_ptr<std::string> GyrosReader::matrix<boost::shared_ptr<std::string> >();
}
}
/* GYROSREADER_HPP_ */
#endif
