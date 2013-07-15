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
#ifndef BLUEVIEWFWD_HPP_
#define BLUEVIEWFWD_HPP_

#include <bvt_sdk.h>

#include <boost/shared_ptr.hpp>

#include <stdexcept>

namespace labust
{
	namespace blueview
	{
		typedef boost::shared_ptr<BVTOpaqueSonar> BVSonarPtr;
		typedef boost::shared_ptr<BVTOpaqueColorMapper> BVColorMapperPtr;
		typedef boost::shared_ptr<BVTOpaquePing> BVPingPtr;
		typedef boost::shared_ptr<BVTOpaqueMagImage> BVMagImagePtr;
		typedef boost::shared_ptr<BVTOpaqueColorImage> BVColorImagePtr;
		typedef boost::shared_ptr<BVTOpaqueNavData> BVNavDataPtr;

		struct BVFactory
		{
			static inline BVSonarPtr makeBVSonar()
			{
				return BVSonarPtr(BVTSonar_Create(), std::ptr_fun(&BVTSonar_Destroy));
			};

			static inline BVNavDataPtr makeBVNavData()
			{
				return BVNavDataPtr(BVTNavData_Create(), std::ptr_fun(&BVTNavData_Destroy));
			};

			static inline BVNavDataPtr makeBVNavData(BVTNavData data)
			{
				return BVNavDataPtr(data, std::ptr_fun(&BVTNavData_Destroy));
			};

			static inline BVColorMapperPtr makeBVColorMapper(const std::string& colormap)
			{
				BVColorMapperPtr mapper(BVTColorMapper_Create(), std::ptr_fun(&BVTColorMapper_Destroy));
				BVTColorMapper_Load(mapper.get(),colormap.c_str());
				return mapper;
			};

			static inline BVPingPtr getBVPing(BVTHead head, int pingNum)
			{
				BVTPing ping(0);
				if (int error = BVTHead_GetPing(head,pingNum,&ping)) throw std::invalid_argument(BVTError_GetString(error));
				return BVPingPtr(ping, std::ptr_fun(&BVTPing_Destroy));
			};

			static inline BVNavDataPtr getBVNavData(BVPingPtr ping)
			{
				BVTNavData navData = BVTNavData_Create();
				if (int error = BVTPing_GetNavDataCopy(ping.get(),&navData)) throw std::invalid_argument(BVTError_GetString(error));
				return BVNavDataPtr(navData,std::ptr_fun(&BVTNavData_Destroy));
			};

			static inline BVMagImagePtr getBVMagImage(BVPingPtr ping)
			{
				BVTMagImage magnitude(0);
				if (int error = BVTPing_GetImage(ping.get(),&magnitude)) throw std::invalid_argument(BVTError_GetString(error));
				return BVMagImagePtr(magnitude, std::ptr_fun(&BVTMagImage_Destroy));
			};

			static inline BVColorImagePtr getBVColorImage(BVMagImagePtr magnitude, BVColorMapperPtr mapper)
			{
				BVTColorImage image(0);
				if (int error = BVTColorMapper_MapImage(mapper.get(),magnitude.get(),&image)) throw std::invalid_argument(BVTError_GetString(error));
				return BVColorImagePtr(image, std::ptr_fun(&BVTColorImage_Destroy));
			};
		};
	}
}

/* BLUEVIEWFWD_HPP_ */
#endif
