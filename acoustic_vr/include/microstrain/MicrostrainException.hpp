#ifndef MICROSTRAINEXCEPTION_HPP_
#define MICROSTRAINEXCEPTION_HPP_
/*
 * MicrostrainException.hpp
 *
 *  Created on: May 8, 2011
 *      Author: dnad
 */
#include <string>
#include <stdexcept>

namespace LABUST
{
  namespace MICROSTRAIN
  {
    /**
     * The MicrostrainException class is used for error reporting inside the microstrain library.
     * It also allow for throw-catch mode of error handling and reporting.
     *
     * The method MicrostrainException::append is supplied for extending multiple the
     * error message through multiple throw-catch statements.
     */
    class MicrostrainException : public std::runtime_error
    {
    public:
      /**
       * Alternative constructor. This method never throws.
       * \param error_msg This is the additional error message to help in localization.
       */
      explicit MicrostrainException(const std::string& error_msg) throw();
      /**
       * Generic destructor. This method never throws.
       */
      virtual ~MicrostrainException() throw();

      /**
       * Returns the error message of the object.
       * This method never throws.
       */
      const char* what() const throw();

      /**
       * Append the additional message to the current error_msg.
       * \param msg The message to be appended.
       */
      void append(const std::string& msg){error_msg+=msg;};

    private:
      /**
       * The supplied error message.
       */
      std::string error_msg;
      /**
       * The empty constructor is hidden.
       */
      MicrostrainException();
    };
  };
};

/* MICROSTRAINEXCEPTION_HPP_ */
#endif
