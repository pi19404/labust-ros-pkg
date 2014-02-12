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
#ifndef KFMODELLOADER_HPP_
#define KFMODELLOADER_HPP_
#include <labust/navigation/KFBase.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <ros/ros.h>

namespace labust
{
  namespace navigation
  {
  	template <class ModelType>
  	void kfModelLoader(ModelType& model, const ros::NodeHandle& nh, const std::string& group = "")
  	{
  		std::string ns(group);
  		if (!ns.empty() && (ns.at(ns.length()-1) != '/')) ns+="/";

  		labust::tools::getMatrixParam(nh, ns+"Q", model.Q);
  		model.W = ModelType::matrix::Identity(model.Q.rows(), model.Q.cols());
  		labust::tools::getMatrixParam(nh, ns+"R", model.R0);
  		model.V0 = ModelType::matrix::Identity(model.R0.rows(), model.R0.cols());
  		model.H0 = ModelType::matrix::Identity(model.R0.rows(), model.Q.cols());

  		//Optional parameters
  		if (nh.hasParam(ns+"W"))	labust::tools::getMatrixParam(nh, ns+"W", model.W);
  		if (nh.hasParam(ns+"V"))	labust::tools::getMatrixParam(nh, ns+"V", model.V0);
  		if (nh.hasParam(ns+"H"))	labust::tools::getMatrixParam(nh, ns+"H", model.H0);
  		if (nh.hasParam(ns+"P"))
  		{
  			typename ModelType::matrix P;
  			labust::tools::getMatrixParam(nh, ns+"P", P);
  			model.setStateCovariance(P);
  		}
  		if (nh.hasParam(ns+"x0"))
  		{
  			typename ModelType::vector x0;
  			labust::tools::getMatrixParam(nh, ns+"x0", x0);
  			model.setState(x0);
  		}

  		//Set changeable
  		model.R = model.R0;
  		model.V = model.V0;
  		model.H = model.H0;

  		nh.param("sampling_time",model.Ts,0.1);
  	}
  };
}
/* KFMODELLOADER_HPP_ */
#endif
