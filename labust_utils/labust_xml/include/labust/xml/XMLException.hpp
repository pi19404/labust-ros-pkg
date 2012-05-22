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
#ifndef XMLEXCEPTION_HPP_
#define XMLEXCEPTION_HPP_

#include <stdexcept>
#include <string>

namespace labust
{
  namespace xml
  {
    /**
     * The class defines a specialized runtime exception for the XML wrapper library.
     * Catch this exception if you wish to handle XML errors that can occur during
     * configuration or data exchange.
     */
    class XMLException : public std::runtime_error
    {
    public:
      /**
       * Main constructor.
       *
       * \param error_msg The supplied error message for exception qualification.
       */
      explicit XMLException(const std::string& error_msg) throw();
      /**
       * Generic destructor
       */
      virtual ~XMLException() throw();

      /**
       * Returns a C-style character string describing the general cause of
       * the current error (the same string passed to the ctor).
       *
       * \return Error message of the exception.
       */
      virtual const char* what() const throw();

      /**
       * The method helps appending new error messages if several throw and catch phases
       * are passed.
       *
       * \param str Message to append to the exception error message.
       */
      void append(const std::string& str);

    protected:
      /**
       * Error message object.
       */
      std::string error_msg;
    };
  };
};
/* XMLEXCEPTION_HPP_ */
#endif
