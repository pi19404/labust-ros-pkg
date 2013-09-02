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
#ifndef MMCMESSAGES_HPP_
#define MMCMESSAGES_HPP_
#include <labust/preprocessor/mem_serialized_struct.hpp>
#include <boost/array.hpp>

namespace labust
{
	namespace tritech
	{
		/**
		 * This enumeration contains mini modem commands
		 */
		enum mmcMessage
		{
			///mmcGetRangeSync - ID:1 - calculates a range to the desired device
			mmcGetRangeSync = 1,
			///mmcRangeData - ID:2 - returns the range in 4 bytes little endian appended to the header
			mmcRangeData,
			///mmcListenRangeSync - ID:21 - listens for range sync between modems
			mmcListenRangeSync = 21,
			///mmcGetRangeTxRxByte - ID:24 - calculates a range and send/receive a byte V4 > M1 > M2 Ranger
			mmcGetRangeTxRxByte = 24,
			///mmcLsnRangeRxByte - ID:25 - LsnRangeSync + Receive 1 Byte, V4 > M3 RangeLsnr
			mmcLsnRangeRxByte,
			///mmcRangeRxByte - ID:26 - The Range & Rx 'Rep' Byte from Cmds 24 & 25 V4 < M1, [M3] Ranger,[Lsnr]
			mmcRangeRxByte,
			///mmcSetRangeRepByte - ID:27 User Setting of the Ranged Unit Reply Byte V4 > M2 Ranged
			mmcSetRangeRepByte,
			///mmcRangedRepByte - ID:28 The Ranged Unit Reply Byte in response to 24 M2 > M1[M3] Ranged
			mmcRangedRepByte,
			///mmcRangedRcvByte - ID:29 The Ranged Unit received Byte from 24 V4 < M2 Ranged
			mmcRangedRcvByte,
			///MiniModems M1 = 'Ranger', M2 = 'Ranged', M3 = 'RangeLsnr'  V4 = 'User <--> MM Device Comms
			///mmcGetRangeTxRxBits24 - ID:30 - GetRange + Send 24Bits, Receive 0 .. 24 Bits  V4 > M1 > M2     Ranger
			mmcGetRangeTxRxBits24,
			///mmcLsnRangeRxBits - ID:31 - LsnRange + Receive UpTo GetTxRxN NBits V4 >      M3     RangeLsnr
			mmcLsnRangeRxBits,
			///mmcRangeRxBits - ID:32 Range + Rx 'Rep' NBits from Cmds     30 & 31  V4 < M1, [M3]    Ranger,[Lsnr]
			mmcRangeRxBits,
			///mmcSetRepBits - ID:33 - User Setting of the Ranged Unit Reply NBits V4 > M2 Ranged
			mmcSetRepBits,
			///mmcRepBits24 - ID:34 The Ranged Unit Reply NBits (33) reply to 30 M2 > M1[M3] Ranged
			mmcRepBits24,
			///mmcRangedRcvBits - ID:35 The Ranged Unit received NBits from 30 V4 < M2 Ranged
			mmcRangedRcvBits,
			///mmcGetRangeTxRxBits32vc - ID:36 GetRange + Send 24Bits, Receive 0 .. 24 Bits  V4 > M1 > M2 Ranger
			mmcGetRangeTxRxBits32,
			///mmcRepBits32 - ID:37 The Ranged Unit Reply NBits (33) reply to 36 M2 > M1[M3] Ranged
			mmcRepBits32,
			///mmcGetRangeTxRxBits40 - ID:38 GetRange + Send 40Bits, Receive Any RepBitsNN V4 > M1 > M2 Ranger
			mmcGetRangeTxRxBits40,
			///mmcRepBits40 - ID:39 Ranged Unit Reply to Any mmcGetRangeTxRxBits M2 > M1[M3] Ranged
			mmcRepBits40,
			///mmcGetRangeTxRxBits48 - ID:40 GetRange + Send 48Bits, Receive Any RepBitsNN V4 > M1 > M2 Ranger
			mmcGetRangeTxRxBits48,
			///mmcRepBits48 - ID:41 Ranged Unit Reply to Any mmcGetRangeTxRxBits M2 > M1[M3] Ranged
			mmcRepBits48,
			//mmcStatusMsg - ID:42 Sent on Error Detected or Fixed V4 < M1, [M3] Ranger,[Lsnr]
			mmcStatusMsg,
		};

		typedef boost::array<uint8_t,16> vec16u;
		struct MMCMsg
		{
			enum {ranged_payload_size=4,
				ranged_payload=5,
				payload_size=0,
				payload=1};

			MMCMsg():
			 msgType(24),
			 tx(0),
			 rx(2),
			 rxTmo(60000){};

			uint8_t msgType;
			uint16_t tx,rx,rxTmo;
			//For modem replay messages with the first 4 bytes of payload are usually the range.
			vec16u data;
		};
	}
}

BOOST_CLASS_IMPLEMENTATION(labust::tritech::vec16u , boost::serialization::primitive_type)
///\todo Document the Modem message
PP_LABUST_MAKE_BOOST_SERIALIZATOR_CLEAN(labust::tritech::MMCMsg,
		(uint8_t, msgType)
        (uint16_t, tx)
		(uint16_t, rx)
		(uint16_t, rxTmo)
		(vec16u, data))

/* MMCMESSAGES_HPP_ */
#endif
