/*
 * eigen_test.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: dnad
 */
#include <labust/tools/conversions.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
	using namespace Eigen;
	typedef double T;
	double roll = 0;
	double pitch = 0;
	double yaw = M_PI/3;
	Quaternion<double> q;
	labust::tools::quaternionFromEulerZYX(roll,pitch,yaw,q);
	std::cout<<"quat:"<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
	roll = pitch = yaw = 0;
	labust::tools::eulerZYXFromQuaternion(q, roll,pitch,yaw);

	//Tait-Bryan angles (XYZ) local => body
	//The angle axis of Eigen are transposed matrices of what we expect
	//m = (Cz*Cy*Cx).transpose();

	std::cout<<roll<<std::endl;
	std::cout<<pitch<<std::endl;
	std::cout<<yaw<<std::endl;

	//std::cout<<q<<std::endl;

	//q = Quaternion<T>(m);
	return 0;
}





