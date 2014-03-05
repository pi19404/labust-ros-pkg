/*
 * eigen_test.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: dnad
 */
#include <labust/tools/conversions.hpp>
#include <iostream>
#include <sstream>

int main(int argc, char* argv[])
{
	using namespace Eigen;

	if (argc == 4)
	{
		Eigen::Vector3d rpy;
		for (int i=0; i<3; ++i)
		{
			std::istringstream in(argv[i+1]);
			in >> rpy(i);
			rpy(i) *= M_PI/180;
		}
		Quaternion<double> q;
		labust::tools::quaternionFromEulerZYX(rpy(0),rpy(1), rpy(2),q);
		std::cout<<"rpy:"<<rpy(0)<<" "<<rpy(1)<<" "<<rpy(2)<<std::endl;
		std::cout<<"quat:"<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
	}
	else if (argc == 5)
	{
		double roll,pitch,yaw;
		Eigen::Vector4d quat;
		for (int i=0; i<4; ++i)
		{
			std::istringstream in(argv[i+1]);
			in >> quat(i);
		}

		Quaternion<double> q(quat(3),quat(0),quat(1),quat(2));
		labust::tools::eulerZYXFromQuaternion(q, roll,pitch,yaw);
		std::cout<<"rpy:"<<roll<<" "<<pitch<<" "<<yaw<<std::endl;
		std::cout<<"quat:"<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
	}

	return 0;
}





