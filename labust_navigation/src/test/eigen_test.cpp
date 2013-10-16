/*
 * eigen_test.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: dnad
 */
#include <labust/navigation/KinematicModel.hpp>
#include <labust/navigation/KFCore.hpp>
#include <labust/navigation/KFModelLoader.hpp>

#include <ros/ros.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "eigen_test");
	ros::NodeHandle nh;
	using namespace labust::navigation;
	KFCore<KinematicModel> nav;

	kfModelLoader(nav,nh);

	ros::Rate rate(10);

	while (ros::ok())
	{
		nav.predict();
		KinematicModel::vector meas = KinematicModel::vector::Zero(KinematicModel::inputSize);
		nav.correct(meas);
		rate.sleep();
	}

	return 0;
}





