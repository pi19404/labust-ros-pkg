/*
 * serialization_test.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: dnad
 */
#include <labust/navigation/SpatialMessages.hpp>
#include <labust/archive/delimited_oarchive.hpp>
#include <labust/archive/delimited_iarchive.hpp>
#include <labust/preprocessor/clean_serializator.hpp>

#include <boost/serialization/string.hpp>

PP_LABUST_CLEAN_ARRAY_OSERIALIZATOR_IMPL(labust::archive::delimited_oarchive)
PP_LABUST_CLEAN_ARRAY_ISERIALIZATOR_IMPL(labust::archive::delimited_iarchive)

int main()
{
	using namespace labust::archive;

  return 0;
}
