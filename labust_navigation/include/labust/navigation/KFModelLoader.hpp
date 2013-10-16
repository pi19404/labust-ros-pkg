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
  		labust::tools::getMatrixParam(nh, ns+"W", model.W);
  		labust::tools::getMatrixParam(nh, ns+"V", model.V);
  		labust::tools::getMatrixParam(nh, ns+"R", model.R);

  		typename ModelType::matrix P;
  		std::pair<int, int> size = labust::tools::getMatrixParam(nh, ns+"P", P);
 			if ((size.first != -1) && (size.second != -1)) model.setStateCovariance(P);

  		typename ModelType::vector x0;
  		size = labust::tools::getMatrixParam(nh, ns+"x0", x0);
  		if ((size.first != -1) && (size.second != -1)) model.setState(x0);

  		nh.param("sampling_time",model.Ts,0.1);
  	}
  };
}
/* KFMODELLOADER_HPP_ */
#endif
