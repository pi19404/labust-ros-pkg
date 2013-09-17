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
#ifndef REMOTEFWD_HPP_
#define REMOTEFWD_HPP_

namespace labust
{
	namespace tritech
	{
    /**
     * Enumeration of Tritech nodes numbers
     */
    enum Node
    {
      ///Scanning sonar ID:2
      Sonar = 2,
          ///Attitude sensor ID:75
          AttitudeSensor = 75,
          ///MasterModem ID:85
          MasterModem = 85,
          ///USBL head ID:90
          USBL = 90,
          ///GPS Devices
          GPS = 245,
          ///Main surface unit
          Surface = 255,
          mtAll = 255
    };

    /**
     * Command for TCP/IP communication setup
     */
    enum client_server_command {scAttachToNode = 1, scDetachFromNode = 7};
    /**
     * Internal application classes when using TCP/IP communication
     */
    enum app_classes {atNull=0, atGeneric = 15, atAMNAV = 26, atMiniAttSen = 33};
    /**
     * Connection type for TCP/IP communication
     */
    enum con_type {rctHead = 1};

    /**
     * This represents the TCP/IP request object for Tritech
     */
    struct TCPRequest
    {
      /**
       * Node number
       */
      LABUST::TYPES::uint8 node;
      /**
       * Command type
       */
      LABUST::TYPES::uint8 cmd;
      /**
       * Priority of the connection
       */
      LABUST::TYPES::uint8 priority;
      /**
       * Application class
       */
      LABUST::TYPES::uint8 app_class;
      /**
       * Connection type
       */
      LABUST::TYPES::uint8 type;
      /**
       * GUIID
       */
      LABUST::TYPES::uint8 guid[16];
    };
	}
}

/* REMOTEFWD_HPP_ */
#endif
