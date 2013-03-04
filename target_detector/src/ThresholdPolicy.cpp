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
#include <labust/blueview/ThresholdPolicy.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace labust::blueview;

MatPtr SimpleThreshold::threshold(const MatPtr prefiltered, float weight)
{
	MatPtr thresholded(new cv::Mat(prefiltered->size(),CV_8UC1));
	cv::threshold(*prefiltered, *thresholded, weight, 255, CV_THRESH_BINARY);
	thresholded->convertTo(*thresholded,CV_8UC1);
	return thresholded;
}

MatPtr HistThreshold::threshold(const MatPtr prefiltered,
		float min,float max, unsigned char maxVal)
{
	enum {thehoodWidth = 1, thehoodHeight = 1};
	MatPtr thresholded(new cv::Mat(prefiltered->size(),CV_8U, cv::Scalar(0)));

	size_t width(prefiltered->size().width), height(prefiltered->size().height);

	for(int i=1; i<(width-1); ++i)
	{
		for(int j=1; j<(height-1); ++j)
		{
			cv::Mat thehood(*prefiltered,
					cv::Range(i-thehoodWidth,i+thehoodWidth),
					cv::Range(j-thehoodHeight,j+thehoodHeight));
			cv::Mat rethood(*prefiltered,
					cv::Range(i-thehoodWidth,i+thehoodWidth),
					cv::Range(j-thehoodHeight,j+thehoodHeight));

			bool flag = false;

			for (int k=0; k<(thehoodWidth*2 + 1); ++k)
			{
				for (int k2=0; k2<(thehoodHeight*2 + 1); ++k2)
				{
					//std::cout<<"Pixel val:"<<thehood.at<float>(k,k2)<<std::endl;
					if ((flag = thehood.at<float>(k,k2) >= max)) break;
				}
			}

			if (flag)
			{
				for (int k=0; k<(thehoodWidth*2 + 1); ++k)
				{
					for (int k2=0; k2<(thehoodHeight*2 + 1); ++k2)
					{
						if (thehood.at<float>(k,k2) >= min)
						{
							rethood.at<float>(k,k2) = maxVal;
						}
						else
						{
							rethood.at<float>(k,k2) = 0;
						}
					}
				}
			}
			else
			{
				cv::Mat(3,3,CV_8U,cv::Scalar(0)).copyTo(rethood);
			}
		}
	}

	return thresholded;
}



