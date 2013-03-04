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
#include <labust/blueview/LabelPolicy.hpp>

#include <opencv2/imgproc/imgproc.hpp>

using namespace labust::blueview;

TrackedFeatureVecPtr SimpleLabel::label(const MatPtr binary)
{
	//This will be replaced with a LABELING algorithm to save execution time
	cv::morphologyEx(*binary, *binary, cv::MORPH_OPEN,cv::Mat());

	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(*binary,contours,hierarchy,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	TrackedFeatureVecPtr info(new TrackedFeatureVec);

	for (size_t i = 0; i < contours.size(); ++i)
	{
		std::vector<cv::Point> c_new;
		cv::convexHull(cv::Mat(contours[i]),c_new,true);
		contours[i] = c_new;
		cv::Moments moments = cv::moments(cv::Mat(contours[i]),true);

		TrackedFeature tfeature;
		tfeature.perimeter = cv::arcLength(cv::Mat(contours[i]),true);
		tfeature.area = moments.m00;
		tfeature.pposition.x = (moments.m10)/moments.m00;
		tfeature.pposition.y = (moments.m01)/moments.m00;

		info->push_back(tfeature);
	}

	return info;
};





