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
#ifndef GYROSERROR_HPP_
#define GYROSERROR_HPP_

#include "labust/xml/XMLException.hpp"
#include "labust/xml/Gyros.hpp"

namespace labust
{
  namespace xml
  {
    /**
     * Implements the error class of the Gyros protocol. It is an
     * extension of the XMLException.
     */
    class GyrosError : public XMLException, public Gyros
    {
    public:
      /**
       * Main constructor that specifies the error message
		 *
		 * \param error_msg - error message to show
		 * \param label - label of error message, optional
	 	 * \param timestamp - timestamp of error message, optional
       */
      GyrosError(std::string error_msg, std::string label="", double timestamp = -1) throw();
      /**
       * Generic destructor.
       */
      virtual ~GyrosError() throw();

      /** 
	   * Returns a C-style character string describing the general cause of
       * the current error (the same string passed to the ctor).  
	   */
      virtual const char* what() const throw();
	  /**
	   * Return internal error string.
	   */
	  inline std::string getError(){return error_msg;}

	protected:
      /**
       * Gyros encoded message. Declared mutable because we assemble
       * the message on request.
       */
      mutable std::string gyros_msg;
    };
  }
}
/* GYROSERROR_HPP_ */
#endif
