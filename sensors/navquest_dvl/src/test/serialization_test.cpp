/*
 * serialization_test.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: dnad
 */
#include <labust/navigation/NavQuestMessages.hpp>
#include <labust/archive/delimited_oarchive.hpp>
#include <labust/archive/delimited_iarchive.hpp>
#include <labust/preprocessor/clean_serializator.hpp>

#include <boost/serialization/string.hpp>

PP_LABUST_CLEAN_ARRAY_OSERIALIZATOR_IMPL(labust::archive::delimited_oarchive)
PP_LABUST_CLEAN_ARRAY_ISERIALIZATOR_IMPL(labust::archive::delimited_iarchive)

std::string nq_data()
{
	return "$#NQ.RES 0X0001 1 2 3 4 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -?13713 -?50937 -17779 -?05940 -313933 ?89842 -?24036 730108 22.40 32.49 325.7 -3.0 17.4 0.005 35.0 1514 9682";
}

int main()
{
	using namespace labust::archive;
	using namespace labust::navigation;

	NQRes dvl_data;
	std::stringstream ss(nq_data());
	std::cout<<"Data:"<<ss.str()<<std::endl;
	delimited_iarchive ia(ss);
	vec4i kp2={1,2,3,4}, kp2r;
	ia>>dvl_data;

	delimited_oarchive oa(std::cout);
  //std::string data("hello"),datar;
  //oa<<kp2<<data;
	std::cout<<"Read:";
	oa<<dvl_data;

  return 0;
}
