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
#include <labust/simulation/NoiseModel.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/xml/XMLuBlas.hpp>
#include <labust/math/uBlasOperations.hpp>

using namespace labust::simulation;

NoiseModel::NoiseModel():
		w(zero_v(wSize)),
		v(zero_v(vSize)),
		ngW(wSize),
		ngV(vSize){};

NoiseModel::NoiseModel(const labust::xml::ReaderPtr reader):
		w(zero_v(6)),
		v(zero_v(6)),
		ngW(wSize),
		ngV(vSize)
{
	this->configure(reader);
}

void NoiseModel::configure(const labust::xml::ReaderPtr reader)
{
	vector pn = zero_v(vSize);
  reader->try_value("process-noise",&pn);

  for (size_t i=0; i<vSize;++i)
  {
  	ngV[i].reset(new noise_generator(rd,boost::normal_distribution<>(0,std::sqrt(pn(i)))));
  }

	vector mn = zero_v(wSize);
  reader->try_value("measurement-noise",&mn);
  for (size_t i=0; i<wSize;++i)
  {
  	ngW[i].reset(new noise_generator(rd,boost::normal_distribution<>(0,std::sqrt(mn(i)))));
  }
}

const vector& NoiseModel::calculateW()
{
	for(size_t i=0; i<wSize; ++i)
	{
		w(i) = (*ngW[i])();
	}

	return w;
}

const vector& NoiseModel::calculateV()
{
	for(size_t i=0; i<vSize; ++i)
	{
		v(i) = (*ngV[i])();
	}
	return v;
}




