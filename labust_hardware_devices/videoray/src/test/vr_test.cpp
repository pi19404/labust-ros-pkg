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
#include <labust/vehicles/VRPro.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/tools/TimingTools.hpp>

int main(int argc, char* argv[])
{
	labust::xml::ReaderPtr reader(new labust::xml::Reader(argv[1],true));
	reader->useNode(reader->value<_xmlNode*>("//UUVApp"));
	labust::vehicles::VRPro vr(reader,"");

	labust::tools::wait_until_ms delay(100);

	int i = 0;

	while(true)
	{
		labust::vehicles::tauMap tau;
		tau[labust::vehicles::tau::X] = 0;
		vr.setTAU(tau);
		labust::vehicles::stateMapPtr state(new labust::vehicles::stateMap());
		vr.getState(state);

		std::cout<<"Heading:"<<(*state)[labust::vehicles::state::heading]<<std::endl;
		std::cout<<"Pitch:"<<(*state)[labust::vehicles::state::pitch]<<std::endl;
		std::cout<<"Roll:"<<(*state)[labust::vehicles::state::roll]<<std::endl;
		std::cout<<"z:"<<(*state)[labust::vehicles::state::z]<<std::endl;
		std::cout<<"Pressure:"<<(*state)[labust::vehicles::state::depthPressure]<<std::endl;

		std::cout.precision(6);
		std::cout<<"Time:"<<std::fixed<<labust::tools::unix_time()<<std::endl;

		delay();
		++i;
	}

	labust::vehicles::tauMap tau;
	vr.setTAU(tau);

	return 0;
}




