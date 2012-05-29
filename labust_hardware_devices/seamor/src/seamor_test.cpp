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
#include <labust/vehicles/SeamorDriver.h>
#include <labust/vehicles/Seamor.hpp>
#include <labust/tools/TimingTools.hpp>
#include <labust/xml/XMLReader.hpp>

#include <iostream>

int main(int argc, char* argv[])
try
{
	labust::xml::ReaderPtr reader(new labust::xml::Reader(argv[1],true));
	reader->useNode(reader->value<_xmlNode*>("//configurations"));
	//labust::vehicles::SeamorDriver seamor(reader,"");
	labust::vehicles::Seamor seamor(reader,"");

	int i=0;
	labust::tools::wait_until_ms delay(100);

	while (true)
	{
		double lastTime = labust::tools::unix_time();
		++i;

		labust::vehicles::tauMap tau;
		tau[labust::vehicles::tau::X]= 20;
		seamor.setTAU(tau);
		labust::vehicles::stateMapPtr state(new labust::vehicles::stateMap());
		seamor.getState(state);
		labust::vehicles::strMapPtr data(new labust::vehicles::strMap());
		//seamor.getData(data);

		std::cout<<"Heading:"<<(*state)[labust::vehicles::state::yaw]*180/M_PI<<std::endl;

		delay();
		std::cout<<"Loop time:"<<labust::tools::unix_time() - lastTime<<std::endl;
	}
	std::cout<<"Normal exit."<<std::endl;
	return 0;
}
catch (std::exception& e)
{
	std::cout<<e.what()<<std::endl;
}



