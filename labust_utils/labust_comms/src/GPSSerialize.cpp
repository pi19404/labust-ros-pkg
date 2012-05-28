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
#include <labust/comms/gps/GPSSerialize.hpp>
#include <labust/plugins/PlugableDefs.hpp>
#include <labust/comms/NMEASentence.hpp>

using namespace labust::comms;

bool GPSSerialize::decode(const msgqueue& mmsg, std::string* const xmlstr)
{
	for (size_t i=0; i<mmsg.size(); ++i)
	{
		labust::comms::NMEASentence sentence(mmsg[i]);
		mmsg[i]
	}

	return true;
}

bool GPSSerialize::encode(const std::string& xmlstr, std::string* const mmsg)
{
	throw std::runtime_error("GPSSerialize is read-only.");
};

LABUST_EXTERN
{
	LABUST_EXPORT MMSerializeFactoryPtr createSerializer()
	{
		return labust::comms::MMSerializeFactoryPtr(
				new labust::comms::MMSerializeFactory::Impl<labust::comms::GPSSerialize>());
	}
}



