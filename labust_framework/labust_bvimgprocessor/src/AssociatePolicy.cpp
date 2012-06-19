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
#include <labust/blueview/AssociatePolicy.hpp>

using namespace labust::blueview;

bool DirectAssociate::associate(const TrackedFeatureVecPtr features, const SonarHead& head,
					TrackedFeature* const tracklet, TrackedFeature* const target)
{
	int idx = -1;
	double mindiff = 10000;

	bool hasTarget(target!=0);

	for (size_t i=0;i<features->size();++i)
	{
		TrackedFeature& tfeature = (*features)[i];

		/*double tt_distance(0);
		if (hasTarget)
		{
			double tt_dx = tfeature.pposition.x - target->pposition.x;
			double tt_dy = tfeature.pposition.y - target->pposition.y;
			tt_distance = sqrt(tt_dx*tt_dx + tt_dy*tt_dy);
		}
		else
		{
			tt_distance = 100;
		}
		*/

	  double dx = (tfeature.pposition.x) - tracklet->pposition.x;
	  double dy = (tfeature.pposition.y) - tracklet->pposition.y;
	  double distance = sqrt(dx*dx + dy*dy);

	  /*if (tt_distance > distance)
	  {
	   if (hasTarget) std::cout<<"Distance to target:"<<tt_distance*head.resolution<<std::endl;*/

	   if (distance<mindiff)
	   {
	    mindiff = distance;
	    idx = i;
	   }
	  /*}
	  else
	  {
	    std::cout<<"Skipped target."<<std::endl;
	  }
	  */
	}

	if (idx != -1)
	{
	 tracklet->pposition = (*features)[idx].pposition;
	 return true;
	}

	return false;
};





