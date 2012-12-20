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
#ifndef ASSOCIATEPOLICY_HPP_
#define ASSOCIATEPOLICY_HPP_
#include <labust/blueview/trackerfwd.hpp>

namespace labust
{
	namespace blueview
	{
		/**
		 * Empty associate class.
		 */
		struct NoAssociate
		{
			/**
			 * The method associates detected features with the tracklet.
			 *
			 * \param features The vector of detected features.
			 * \param tracklet The old tracklet value.
			 *
			 * \return True if found, false otherwise.
			 */
			inline static bool associate(const TrackedFeatureVecPtr features,
					TrackedFeature* tracklet){return false;};
		};

		/**
		 * Direct associate class.
		 *
		 * \todo Check the target dismissal.
		 */
		struct DirectAssociate
		{
			/**
			 * The method associates detected features with the tracklet.
			 *
			 * \param features The vector of detected features.
			 * \param tracklet The old tracklet value.
			 * \param target The target tracklet to avoid the target lock.
			 *
			 * \return True if found, false otherwise.
			 */
			static bool associate(const TrackedFeatureVecPtr features, const SonarHead& head,
					TrackedFeature* tracklet, TrackedFeature* target);
		};

	}
}

/* ASSOCIATEPOLICY_HPP_ */
#endif
