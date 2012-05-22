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
#ifndef XMLUBLAS_HPP_
#define XMLUBLAS_HPP_

#include <labust/xml/XMLReader.hpp>

#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/traits.hpp>
#include <boost/numeric/ublas/banded.hpp>

namespace labust
{
	namespace xml
	{
		/**
		 * The operator extracts the matrix value from the XML configuration file and converts it to a
		 * desired uBlas equivalent matrix.
		 *
		 * \param reader The labust::xml::Reader reference from which to read.
		 * \param matrix The matrix name and memory address for parsing and conversion.
		 *
		 * \return The labust::xml::Reader reference for easier call binding.
		 */
  	template <class MatrixType>
  	const Reader& operator>>(const Reader& reader, const std::pair<std::string,MatrixType*>& matrix)
  	{
  		typedef typename boost::numeric::ublas::container_traits<MatrixType>::value_type T;
  		std::string out = reader.evaluate(matrix.first.c_str());

  		//skip spaces
  		size_t offset = out.find_first_not_of(' ');
  		bool negate = out[offset] == '-';
  		if (negate) ++offset;
  		bool isDiagonal = !out.substr(offset,6).compare("diag([");

  		if (isDiagonal)
  		{
  			std::istringstream str(out.substr(offset+5));
  			boost::numeric::ublas::vector<T> vec;
  			str>>vec;
  			(*matrix.second) = boost::numeric::ublas::diagonal_matrix<T>(vec.size(),vec.data());
  		}
  		else
  		{
  			std::istringstream str(out.substr(offset));
  			str>>(*matrix.second);
  		}

  		if (negate)
  		{
  			(*matrix.second)*=-1;
  		}

  		return reader;
  	};
	};
};
/* XMLUBLAS_HPP_ */
#endif
