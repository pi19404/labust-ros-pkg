#ifndef LABUSTTYPES_H_
#define LABUSTTYPES_H_
/* 
 * LABUSTTypes.h
 *                           
 *  Created on: Jul 27, 2010
 *      Author: Dula Nad   
 */ 
#include <vector>
#include <boost/shared_ptr.hpp>

namespace LABUST
{
 /**
  * The TYPES namespace contains some specific typedefs to ease coding.
  */    
  namespace TYPES
  {
    typedef signed char  int8;
    typedef signed short int16;
    typedef signed long  int32;

    typedef unsigned char  uint8;
    typedef unsigned short uint16;
    typedef unsigned long  uint32;

    typedef unsigned char uchar;

    typedef std::vector<uint8> ByteVectorType;
    typedef boost::shared_ptr<ByteVectorType> ByteVectorPtr;

    struct PointStruct
    {
      double x,y,z;
    };
  };
};
/* LABUSTTYPES_H_ */
#endif
