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
#include <labust/simulation/VehicleModel6DOF.hpp>
#include <labust/simulation/UVSim.hpp>
#include <labust/vehicles/vehiclesfwd.hpp>
#include <labust/plugins/PlugableDefs.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/math/Rotation.hpp>
#include <labust/math/uBlasOperations.hpp>
#include <labust/xml/XMLuBlas.hpp>

#include <stdexcept>

using namespace labust::simulation;

UVSim::UVSim(const labust::xml::ReaderPtr reader, const std::string& id):
		model(new VehicleModel6DOF(reader,id)),
		currentForce(zero_v(3)),
		current(zero_v(3))
{
	//reader->try_value("current-force",&currentForce);
	std::cout<<*this->model<<std::endl;
};

void UVSim::setTAU(const labust::vehicles::tauMapRef tau)
{
  vector model_tau;

  for (size_t i = labust::vehicles::tau::X; i <= labust::vehicles::tau::N;++i)
  {
    model_tau(i) = tau[i];
  }

  //environment influence here
  vector eta = model->Eta();
  vector3 tauE(prod(labust::math::rotation_matrix()(eta(model->phi),eta(model->theta),eta(model->psi)),currentForce));

  subrange(model_tau, 0,3) += tauE;

  model->step(model_tau);
}

void UVSim::getState(labust::vehicles::stateMapRef state)
{
	int j = 0;
	for(int i = labust::vehicles::state::u; i<= labust::vehicles::state::r; ++i)
	{
		state[labust::vehicles::state::states(i)] = model->Nu()(VehicleModel6DOF::u + j++);
	}

	j= 0;
	for(int i = labust::vehicles::state::x; i<= labust::vehicles::state::yaw; ++i)
	{
		state[labust::vehicles::state::states(i)] = model->Eta()(VehicleModel6DOF::x + j++);
	}
}

void UVSim::setGuidance(const labust::vehicles::guidanceMapRef guidance)
{
  throw std::invalid_argument("Memeber function has no implementation. labust::vehicle::setGuidance.");
}

void UVSim::setCommand(const labust::apps::stringRef commands)
{
	this->unwrapFromXml(commands);

	//Handle updates.
	model->setCurrent(current);
}

void UVSim::getData(labust::apps::stringPtr data)
{
  //throw std::invalid_argument("Memeber function has no implementation. labust::vehicle::getData");
	(*data) = *this->wrapInXml();
	//std::cout<<"Called getData:"<<*data<<std::endl;
}

LABUST_EXTERN
{
  LABUST_EXPORT labust::vehicles::VehicleFactoryPtr createVehicleFactory()
  {
    return labust::vehicles::VehicleFactoryPtr(new labust::vehicles::VehicleFactory::Impl<UVSim>());
  }
}





