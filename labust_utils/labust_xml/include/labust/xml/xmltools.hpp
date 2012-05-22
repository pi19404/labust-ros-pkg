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

#include <boost/foreach.hpp>

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
						reader->value<key_type >(keyName),
						reader->value<mapped_type >(valueName)));
			}

			reader->useNode(cnode);
		};
	}
}


/* XMLTOOLS_HPP_ */
#endif
