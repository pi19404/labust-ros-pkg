/*
 * MicrostrainException.cpp
 *
 *  Created on: May 8, 2011
 *      Author: dnad
 */
#include <microstrain/MicrostrainException.hpp>

using namespace LABUST::MICROSTRAIN;

MicrostrainException::MicrostrainException(const std::string& error_msg) throw()
   :runtime_error("Microstrain run-time exception."),
   error_msg("Microstrain:" + error_msg){};

MicrostrainException::~MicrostrainException() throw() {};

const char* MicrostrainException::what() const throw()
{
  return error_msg.c_str();
}


