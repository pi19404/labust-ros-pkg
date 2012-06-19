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
#ifndef PROCESSINGCHAIN_HPP_
#define PROCESSINGCHAIN_HPP_
#include <labust/blueview/trackerfwd.hpp>
#include <labust/blueview/CConverter.hpp>
#include <labust/blueview/PrefilterPolicy.hpp>
#include <labust/blueview/ThresholdPolicy.hpp>
#include <labust/blueview/LabelPolicy.hpp>
#include <labust/blueview/AssociatePolicy.hpp>
#include <labust/blueview/TrackerROI.hpp>

#include <opencv2/highgui/highgui.hpp>

namespace labust
{
	namespace blueview
	{
		/**
		 * The template incorporates the image processing chain for the NURC
		 * BlueView sonar image processing algorithm.
		 */
		template<
			class Prefilter = SimpleAdjust,
			class Threshold = SimpleThreshold,
			class Label = SimpleLabel,
			class Associate = DirectAssociate>
		class ProcessingChain
		{
		public:
			/**
			 * Generic constructor.
			 */
			ProcessingChain():
			tracklet(new TrackedFeature),
			hasTarget(false),
			foundVehicle(false){};

			/**
			 * The main processing command. Performs the operation on the specifed
			 * ROI and returns the contanct if any.
			 */
			TrackedFeaturePtr processROI(const TrackerROI& roi)
			{
				if (true)
				{
					//Calculation of the new position
				  //Tracklet position update, calculate new x,y,z based on last known positiom
					this->ccon.llz2xy(roi.headData,this->tracklet);
					CConverter::meter2pixel(roi,this->tracklet);
					//Do the same conversion to the target
					if (this->hasTarget)
			    {
			      this->ccon.llz2xy(roi.headData,target);
			      CConverter::meter2pixel(roi,target);
			    }

			    cv::Mat disp = roi.roi.clone();
			    cv::circle(disp,this->tracklet->pposition,10,cv::Scalar(65536));
			    cv::imshow("Expected",disp*500);

			    //From here things are the similar to the old version
			    //Calculate the pixel region (3x3) meters
			    MatPtr roiImg(new cv::Mat(roi.roi));
			    //Do adjustment
			    MatPtr prefiltered = Prefilter::prefilter(roiImg);
			    MatPtr binary = Threshold::threshold(prefiltered,0.65);
			    TrackedFeatureVecPtr features = Label::label(binary);

			    this->foundVehicle = Associate::associate(features, roi.headData, tracklet.get(), target.get());

			    cv::imshow("Adjusted",*prefiltered);
			    cv::imshow("Binary",*binary);

			    CConverter::pixel2meter(roi,tracklet);
			    this->ccon.xy2llz(roi.headData,tracklet);

			    return TrackedFeaturePtr(new TrackedFeature(*tracklet));
			  }

				return TrackedFeaturePtr();
			}

			inline bool isTracking(){return foundVehicle;};

			void setPosition(const SonarHead& cnt)
			{
			  tracklet->position.x = cnt.range*cos((cnt.bearing - cnt.panAngle)*M_PI/180);
			  tracklet->position.y = cnt.range*sin((cnt.bearing - cnt.panAngle)*M_PI/180);
			  this->ccon.xy2llz(cnt,tracklet);
			  foundVehicle = true;
			}

		private:
			/**
			 * The coordinate converter.
			 */
			CConverter ccon;
			/**
			 * The last tracklet and target position.
			 */
			TrackedFeaturePtr tracklet, target;
			/**
			 * Target and tracklet flags.
			 */
			bool hasTarget, foundVehicle;
		};
	}
}

/* PROCESSINGCHAIN_HPP_ */
#endif
