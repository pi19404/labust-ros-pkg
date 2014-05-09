/*********************************************************************
 * eventEvaluation.hpp
 *
 *  Created on: May 8, 2014
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

#ifndef EVENTEVALUATION_HPP_
#define EVENTEVALUATION_HPP_

#include <labust_mission/labustMission.hpp>
#include <exprtk/exprtk.hpp>

/*********************************************************************
 ***  EventEvaluation class definition
 ********************************************************************/

namespace labust {
	namespace event {

		class EventEvaluation{

		public:

			typedef exprtk::symbol_table<double> symbol_table_t;
			typedef exprtk::expression<double> expression_t;
			typedef exprtk::parser<double> parser_t;
			typedef exprtk::parser_error::type error_t;

			/*****************************************************************
			 ***  Class functions
			 ****************************************************************/

			EventEvaluation();

			int checkEventState(auv_msgs::NavSts data, std::string expression_str);

			void updateData(auv_msgs::NavSts data, symbol_table_t& symbol_table);

			/*********************************************************************
			 ***  Class variables
			 ********************************************************************/

			ros::NodeHandle nh_;

		};

		EventEvaluation::EventEvaluation(){

		}

		int EventEvaluation::checkEventState(auv_msgs::NavSts data, std::string expression_str){


			//std::string expression_str = "z := 2 [sin(x * pi)^3.3 + cos(pi / y)^4.4] % (2.3/3.2x + 3.4/4.3y)";
			//std::string expression_str = "mand(x < y, y or z)";

			symbol_table_t symbol_table;
			updateData(data, symbol_table);

			expression_t expression;
			expression.register_symbol_table(symbol_table);

			parser_t parser;

			if (!parser.compile(expression_str,expression))
			{
			  ROS_ERROR("Error: %s\tExpression: %s\n",parser.error().c_str(),expression_str.c_str());

			  for (std::size_t i = 0; i < parser.error_count(); ++i)
			  {
				 error_t error = parser.get_error(i);
				 ROS_ERROR("Error: %02d Position: %02d Type: [%s] Msg: %s Expr: %s\n",
						static_cast<int>(i),
						static_cast<int>(error.token.position),
						exprtk::parser_error::to_str(error.mode).c_str(),
						error.diagnostic.c_str(),
						expression_str.c_str());
			  }
			  return 1;
			}

			double result = expression.value();

			ROS_ERROR("Result: %10.5f\n",result);

			return 0;


//			/* Read external states */
//
//			/* Events */
//			if(0){
//				mainEventQueue->riseEvent("/PRIMITIVE_FINSIHED");
//
//			} else if(0){
//				mainEventQueue->riseEvent("/PRIMITIVE_FINSIHED");
//
//			}

		}

		void EventEvaluation::updateData(auv_msgs::NavSts data, symbol_table_t& symbol_table){

			/* Read estimated values */
			double u = data.body_velocity.x;
			double v = data.body_velocity.y;
			double w = data.body_velocity.z;
			double r = data.orientation_rate.yaw;
			double x = data.position.north;
			double y = data.position.east;
			double z = data.position.depth;
			double psi = data.orientation.yaw;

			double x_var = data.position_variance.north;
			double y_var = data.position_variance.east;
			double z_var = data.position_variance.depth;
			double psi_var = data.orientation_variance.yaw;

			double alt = data.altitude;

			symbol_table.add_constants();
			symbol_table.add_variable("u",u);
			symbol_table.add_variable("v",v);
			symbol_table.add_variable("w",w);
			symbol_table.add_variable("r",r);
			symbol_table.add_variable("x",x);
			symbol_table.add_variable("y",y);
			symbol_table.add_variable("z",z);
			symbol_table.add_variable("psi",psi);
			symbol_table.add_variable("x_var",x_var);
			symbol_table.add_variable("y_var",y_var);
			symbol_table.add_variable("z_var",z_var);
			symbol_table.add_variable("psi_var",psi_var);
			symbol_table.add_variable("alt",alt);


		}
	}
}


#endif /* EVENTEVALUATION_HPP_ */
