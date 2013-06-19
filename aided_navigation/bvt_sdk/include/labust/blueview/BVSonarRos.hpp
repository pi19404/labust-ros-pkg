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
#ifndef BVSONARROS_HPP_
#define BVSONARROS_HPP_

#include <labust/blueview/blueviewfwd.hpp>

#include <dynamic_reconfigure/server.h>
#include <bvt_sdk/BVSonarConfig.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

namespace labust
{
	namespace blueview
	{
		/**
		 * The BlueView sonar class implementation for the ROS node.
		 * \todo Add life streaming
		 * \todo Add navigation data subscription and handling.
		 * \todo Add logging of sonar pings with navigation data.
		 * \todo Add bzip2 compression instead pure image transmission
		 * \todo Add jpeg compression instead of pure image transmission
		 * \todo Add ROI extraction ?.
		 */
		class BVSonarRos : public nodelet::Nodelet
		{
		public:
			/**
			 * Main constructor.
			 */
			BVSonarRos();
			/**
			 * Generic destructor.
			 */
			~BVSonarRos();

			/**
			 * The main method that performs acquisition.
			 */
			void run();

			/**
			 * Configures the sonar for reading.
			 */
			void onInit();

		private:
			/**
			 * The ROS node handle and private handle.
			 */
			ros::NodeHandle nhandle,phandle;
			/**
			 * The magnitude and color image publishers.
			 */
			ros::Publisher imageTopic, cImageTopic;

			/**
			 * Acquisition when we have a file.
			 */
			void runFileAcquisition();
			/**
			 * Dynamic reconfigure service.
			 */
			void dynrec(bvt_sdk::BVSonarConfig& config, uint32_t level);

			/**
			 * The dynamic reconfigure server.
			 */
		  dynamic_reconfigure::Server<bvt_sdk::BVSonarConfig> server;
			/**
			 * The sonar object.
			 */
			labust::blueview::BVSonarPtr sonar;
			/**
			 * The colormap object.
			 */
			labust::blueview::BVColorMapperPtr mapper;
			/**
			 * The sonar head to read from.
			 */
			BVTHead head;
			/**
			 * The ping rate.
			 */
			int pingRate;
			/**
			 * The maximum range.
			 */
			bvt_sdk::BVSonarConfig config;
			/**
			 * The worker thread.
			 */
			boost::thread worker;
		};
	}
}

/* BVSONARROS_HPP_ */
#endif
