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
#ifndef ROSUTILS_HPP_
#define ROSUTILS_HPP_
#include <ros/ros.h>

#include <Eigen/Dense>

#include <string>

namespace Eigen
{
	typedef Eigen::Matrix<double,6,1> Vector6d;
}

namespace labust
{
	namespace tools
	{
		/**
		 * Reads a vector list and populates the matrix.
		 */
		template <class Derived>
		void getMatrixParam(const ::ros::NodeHandle& nh, const std::string& name,
				const Eigen::MatrixBase<Derived>& matrix)
		{
			if (!nh.hasParam(name))
			{
				ROS_INFO("Configuration paramter %s not found",name.c_str());
				return;
			}
			XmlRpc::XmlRpcValue data;
			nh.getParam(name, data);
			ROS_ASSERT(data.getType() == XmlRpc::XmlRpcValue::TypeArray);

			//Check if multi vector description
			if	(data[0].getType() == XmlRpc::XmlRpcValue::TypeArray)
			{
				for(size_t i=0; i<matrix.rows(); ++i)
				{

				}
			}
			size_t col(0),row(0);
			for(size_t i=0; i<data.size(); ++i)
			{
				//Check validity
				bool basicType = data[i].getType() == XmlRpc::XmlRpcValue::TypeDouble;
				basicType = basicType || data[i].getType() == XmlRpc::XmlRpcValue::TypeInt;
				bool vectorType = data[i].getType() == XmlRpc::XmlRpcValue::TypeArray;
				ROS_ASSERT(basicType || vectorType);

				if (basicType)
				{
					const_cast< Eigen::MatrixBase<Derived>& >(matrix)(row,col++) =
							static_cast< typename Eigen::MatrixBase<Derived>::Scalar >(data[i]);
					if (col>=matrix.cols())
					{
						++row;
						col = col%matrix.cols();
					}
				}
				else
				{
					row = i;
					col = 0;
					for(size_t j=0; j<data[i].size(); ++j)
					{
						const_cast< Eigen::MatrixBase<Derived>& >(matrix)(row,col++) =
								static_cast< typename Eigen::MatrixBase<Derived>::Scalar >(data[i][j]);
					}
				}
			}
		}

		/**
		 * The structure populates the dynamic model from the ROS parameter server.
		 */
		struct DynamicsModel
		{
			DynamicsModel():
				Ts(0.1),
				mass(1),
				added_mass(Eigen::Vector6d::Zero()),
				damping(Eigen::Vector6d::Zero()),
				qdamping(Eigen::Vector6d::Zero()),
				inertia_matrix(Eigen::Matrix3d::Zero()){};

			DynamicsModel(const ::ros::NodeHandle& nh):
				Ts(0.1),
				mass(1),
				added_mass(Eigen::Vector6d::Zero()),
				damping(Eigen::Vector6d::Zero()),
				qdamping(Eigen::Vector6d::Zero()),
				inertia_matrix(Eigen::Matrix3d::Zero())
			{
				this->load(nh);
			}

			void load(const ::ros::NodeHandle& nh)
			{
				//Get model name if existing.
				std::string modelName("");
				if (nh.getParam("model_name", modelName))	modelName += "/";

				nh.getParam(modelName+"dynamics/period",Ts);
				nh.getParam(modelName+"dynamics/mass",mass);
				std::cout<<"Searching for:"<<modelName+"dynamics/added_mass"<<std::endl;
				getMatrixParam(nh,modelName+"dynamics/added_mass",added_mass);
				getMatrixParam(nh,modelName+"dynamics/damping",damping);
				getMatrixParam(nh,modelName+"dynamics/quadratic_damping",qdamping);
				getMatrixParam(nh,modelName+"/dynamics/inertia_matrix",inertia_matrix);
			}

			double Ts, mass;
			Eigen::Vector6d added_mass, damping, qdamping;
			Eigen::Matrix3d inertia_matrix;
		};
	}
}

/* ROSUTILS_HPP_ */
#endif
