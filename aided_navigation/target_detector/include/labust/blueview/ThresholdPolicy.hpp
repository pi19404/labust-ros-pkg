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
#ifndef THRESHOLDPOLICY_HPP_
#define THRESHOLDPOLICY_HPP_
#include <labust/blueview/trackerfwd.hpp>

namespace labust
{
	namespace blueview
	{
		/**
		 * Empty threshold class.
		 */
		struct NoThreshold
		{
			/**
			 * The method thresholds the prefiltered image.
			 *
			 * \param prefiltered Prefiltered image.
			 *
			 * \return The pointer to the thresholded image.
			 */
			inline static MatPtr threshold(const MatPtr prefiltered){return prefiltered;};
		};

		struct SimpleThreshold
		{
			/**
			 * The method thresholds the prefiltered image with OpenCv binary threshold.
			 *
			 * \param prefiltered Prefiltered image.
			 * \param weight The threshold value.
			 *
			 * \return The pointer to the thresholded image.
			 */
			static MatPtr threshold(const MatPtr prefiltered, float weight = 0.5);
		};

		struct HistThreshold
		{
			/**
			 * The method thresholds the prefiltered image with a hysteresis threshold.
			 *
			 * \param prefiltered Prefiltered image.
			 * \param min The minimum threshold value.
			 * \param max The maximum threshold value.
			 * \param maxVal The value of pixels above the threshold.
			 *
			 * \return The pointer to the thresholded image.
			 */
			static MatPtr threshold(const MatPtr prefiltered,
					float min, float max, unsigned char maxVal = 255);
		};
	}
}


/* THRESHOLDPOLICY_HPP_ */
#endif
