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
/*********************************************************************
 * Author: Đula Nađ
 *   Date: 11.12.2012.
 *********************************************************************/
#ifndef TRITECHFWD_HPP_
#define TRITECHFWD_HPP_

#include <boost/shared_ptr.hpp>
#include <boost/asio/streambuf.hpp>

#include <sstream>
#include <vector>
#include <cstdint>

namespace labust
{
	namespace tritech
	{
		namespace Nodes
		{
		enum Node
        {
        ///Scanning sonar ID:2
        Sonar = 2,
        ///Attitude sensor ID:75
        AttitudeSensor = 75,
        ///MasterModem ID:85
        MasterModem = 85,
        ///SlaveModem ID:86
        SlaveModem = 86,
        ///USBL head ID:90
        USBL = 90,
        ///GPS Devices
        GPS = 245,
        ///Main surface unit
        Surface = 255,
         mtAll = 255
		};
		};

		typedef boost::shared_ptr<boost::asio::streambuf> StreamPtr;

		typedef std::vector<uint8_t> ByteVector;

		struct MTMsg;
		typedef boost::shared_ptr<MTMsg> MTMsgPtr;

		struct TCONMsg;
		typedef boost::shared_ptr<TCONMsg> TCONMsgPtr;

		class TCPDevice;
		typedef boost::shared_ptr<TCPDevice> TCPDevicePtr;

		class MTDevice;
		typedef boost::shared_ptr<MTDevice> MTDevicePtr;

		enum AttitudeSensorCmd
		{
			attsen_idle = 0,
			attsen_RunRaw,
			attsen_RunProc,
			attsen_Reset,
			attsen_CalMag,
			attsen_CalGrv,
			attsen_CalRate,
			attsen_CalPress,
			attsen_CalTemp,
			attsen_CalCompass
		};
	}
}

/* TRITECHFWD_HPP_ */
#endif
