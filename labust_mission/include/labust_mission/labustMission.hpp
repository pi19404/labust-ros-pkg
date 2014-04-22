/*********************************************************************
 * labustMission.hpp
 *
 *  Created on: Apr 10, 2014
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, LABUST, UNIZG-FER
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

#ifndef LABUSTMISSION_HPP_
#define LABUSTMISSION_HPP_

/*********************************************************************
 *** Common includes
 ********************************************************************/

#include <string>

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <misc_msgs/SendPrimitive.h>
#include <misc_msgs/Go2PointFA.h>
#include <misc_msgs/Go2PointUA.h>
#include <misc_msgs/DynamicPositioning.h>
#include <misc_msgs/CourseKeepingFA.h>
#include <misc_msgs/CourseKeepingUA.h>

#include <auv_msgs/NED.h>

/*********************************************************************
 *** Common global variables
 ********************************************************************/

enum {X = 0, Y, Z, T};

enum {none = 0, go2point_FA, go2point_UA, dynamic_positioning, course_keeping_FA, course_keeping_UA};
const char *primitives[] = {"none", "go2point_FA", "go2point_UA", "dynamic_positioning", "course_keeping_FA", "course_keeping_UA"};


#endif /* LABUSTMISSION_HPP_ */
