//\todo Dodati strukturu s raznim parametrima primitiva ??????????

/*********************************************************************
 * maneuverGenerator.hpp
 *
 *  Created on: Apr 3, 2014
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

#ifndef MANEUVERGENERATOR_HPP_
#define MANEUVERGENERATOR_HPP_

#include <labust_mission/labustMission.hpp>
#include <labust_mission/xmlPrinter.hpp>
#include <Eigen/Dense>

using namespace labust::utils;

/*********************************************************************
 *** ManeuverGenerator class definition
 *********************************************************************/

namespace labust {
	namespace maneuver {

		class ManeuverGenerator {

		public:

			/*********************************************************
			 *** Class functions
			 ********************************************************/

			ManeuverGenerator();

			void writePrimitives(int primitiveID, std::vector<Eigen::Vector4d> points, double speed, double victoryRadius);

			std::vector<Eigen::Vector4d> calcRIPatternPoints(double width, double hstep,
								double alternationPercent, double curvOff, bool squareCurve, double bearingRad);

			std::vector<Eigen::Vector4d> calcCrossHatchPatternPoints(double width, double hstep,
								double curvOff, bool squareCurve, double bearingRad);

			std::vector<Eigen::Vector4d> calcRowsPoints(double width, double length, double hstep,
								double alternationPercent, double curvOff, bool squareCurve, double bearingRad,
								double crossAngleRadians);

			std::vector<Eigen::Vector4d> calcRowsPoints(double width, double length, double hstep,
								double alternationPercent, double curvOff, bool squareCurve, double bearingRad,
								double crossAngleRadians, bool invertY);

			std::pair<double,double> rotate(double angleRadians, double x, double y, bool clockwiseRotation);


			/*********************************************************
			 *** Class variables
			 ********************************************************/

			WriteXML writeXML;
		};


		ManeuverGenerator::ManeuverGenerator(){}


		void ManeuverGenerator::writePrimitives(int primitiveID, std::vector<Eigen::Vector4d> points, double speed, double victoryRadius){

			switch(primitiveID){

				case go2point_FA:

					for(std::vector<Eigen::Vector4d>::iterator it = points.begin() ; it != points.end(); ++it){

							Eigen::Vector4d vTmp = *it;

							writeXML.addGo2point_FA(vTmp[X],vTmp[Y],0,speed,victoryRadius);

					}

					//ROS_ERROR("T1 = %f,%f, T2 = %f,%f, Heading = %f, Speed = %f, Victory radius = %f", CM->Xpos, CM->Ypos, newXpos, newYpos, newHeading, newSpeed, newVictoryRadius);
					break;

				case go2point_UA:

					for(std::vector<Eigen::Vector4d>::iterator it = points.begin() ; it != points.end(); ++it){

							Eigen::Vector4d vTmp = *it;

							writeXML.addGo2point_UA(vTmp[X],vTmp[Y],speed,victoryRadius);

					}

					//ROS_ERROR("T1 = %f,%f, T2 = %f,%f, Speed = %f, Victory radius = %f", CM->Xpos, CM->Ypos, newXpos, newYpos, newSpeed, newVictoryRadius);
					break;

				case dynamic_positioning:

					//ROS_ERROR("T2 = %f,%f, Heading = %f", newXpos, newYpos, newHeading);

					break;

				case course_keeping_FA:
					//ROS_ERROR("Course = %f, Heading = %f, Speed = %f", newCourse, newHeading, newSpeed);

					break;

				case course_keeping_UA:

					//ROS_ERROR("Course = %f, Speed = %f", newCourse, newSpeed);

					break;

				case none:

					break;

			}
		}


		std::vector<Eigen::Vector4d> ManeuverGenerator::calcRIPatternPoints(double width, double hstep,
				double alternationPercent, double curvOff, bool squareCurve, double bearingRad) {

			std::vector<Eigen::Vector4d> newPoints;

			double length = width;
			Eigen::Vector4d pointBaseB;
			pointBaseB << -length/2.0, -width/2.0, 0, -1;

			std::pair<double, double> res = rotate(bearingRad, pointBaseB[X], pointBaseB[Y], false);
		   // double[] res = AngleCalc.rotate(bearingRad, pointBaseB[X], pointBaseB[Y], false);
			Eigen::Vector4d pointBase1;
			pointBase1  << res.first, res.second, 0, -1;

			res = rotate(bearingRad+(-60*M_PI/180), pointBaseB[X], pointBaseB[Y], false);

			Eigen::Vector4d pointBase2;
			pointBase2 << res.first, res.second, 0, -1;

			res = rotate(bearingRad+(-120*M_PI/180), pointBaseB[X], pointBaseB[Y], false);

			Eigen::Vector4d pointBase3;
			pointBase3 << res.first, res.second, 0, -1;

			std::vector<Eigen::Vector4d> points1 = calcRowsPoints(width, width, hstep, 2-alternationPercent, curvOff,
					squareCurve, bearingRad, 0);

			for(std::vector<Eigen::Vector4d>::iterator it = points1.begin() ; it != points1.end(); ++it){

				Eigen::Vector4d vTmp = *it;
				vTmp[X] += pointBase1[X];
				vTmp[Y] += pointBase1[Y];

				*it = vTmp;
			}

			std::vector<Eigen::Vector4d> points2 = calcRowsPoints(width, width, hstep, 2-alternationPercent, curvOff,
							squareCurve, bearingRad + (-60*M_PI/180), 0);

			for(std::vector<Eigen::Vector4d>::iterator it = points2.begin() ; it != points2.end(); ++it){

				Eigen::Vector4d vTmp = *it;
				vTmp[X] += pointBase2[X];
				vTmp[Y] += pointBase2[Y];

				*it = vTmp;
			}

			std::vector<Eigen::Vector4d> points3 = calcRowsPoints(width, width, hstep, 2-alternationPercent, curvOff,
							squareCurve, bearingRad + (-120*M_PI/180), 0);

			for(std::vector<Eigen::Vector4d>::iterator it = points3.begin() ; it != points3.end(); ++it){

				Eigen::Vector4d vTmp = *it;
				vTmp[X] += pointBase3[X];
				vTmp[Y] += pointBase3[Y];

				*it = vTmp;
			}

			// Provjeri ovaj dio moguci problemi s pokazivacem !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			std::vector<Eigen::Vector4d>::iterator it;

			it = newPoints.end();
			newPoints.insert(it+1, points1.begin(), points1.end());
			it = newPoints.end();
			newPoints.insert(it+1, points2.begin(), points2.end());
			it = newPoints.end();
			newPoints.insert(it+1, points3.begin(), points3.end());

			return newPoints;
		}


		std::vector<Eigen::Vector4d> ManeuverGenerator::calcCrossHatchPatternPoints(double width, double hstep,
				double curvOff, bool squareCurve, double bearingRad) {

			std::vector<Eigen::Vector4d> newPoints;

			double length = width;

			Eigen::Vector4d pointBase1;
			pointBase1 << -length/2., -width/2., 0, -1;

			Eigen::Vector4d pointBase2;
			pointBase2 << -length/2., width/2., 0, -1;

			std::pair<double, double> res = rotate(bearingRad, pointBase1[X], pointBase1[Y], false);

			pointBase1 << res.first, res.second, 0, -1;

			res = rotate(bearingRad, pointBase2[X], pointBase2[Y], false);

			pointBase2 << res.first, res.second, 0, -1;

			std::vector<Eigen::Vector4d> points1 = calcRowsPoints(width, width, hstep, 1, curvOff,
					squareCurve, bearingRad, 0);

			for(std::vector<Eigen::Vector4d>::iterator it = points1.begin() ; it != points1.end(); ++it){

				Eigen::Vector4d vTmp = *it;
				vTmp[X] += pointBase1[X];
				vTmp[Y] += pointBase1[Y];

				*it = vTmp;
			}

			std::vector<Eigen::Vector4d> points2 = calcRowsPoints(width, width, hstep, 1, curvOff,
							squareCurve, bearingRad + (-90*M_PI/180), 0);

			for(std::vector<Eigen::Vector4d>::iterator it = points2.begin() ; it != points2.end(); ++it){

				Eigen::Vector4d vTmp = *it;
				vTmp[X] += pointBase2[X];
				vTmp[Y] += pointBase2[Y];

				*it = vTmp;
			}

			// Provjeri ovaj dio moguci problemi s pokazivacem !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			std::vector<Eigen::Vector4d>::iterator it;

			it = newPoints.end();
			newPoints.insert(it+1, points1.begin(), points1.end());
			it = newPoints.end();
			newPoints.insert(it+1, points2.begin(), points2.end());

			return newPoints;
		}


		 std::vector<Eigen::Vector4d> ManeuverGenerator::calcRowsPoints(double width, double length, double hstep,
				double alternationPercent, double curvOff, bool squareCurve, double bearingRad,
				double crossAngleRadians) {
			return calcRowsPoints(width, length, hstep, alternationPercent, curvOff, squareCurve,
					bearingRad, crossAngleRadians, false);
		}


		 std::vector<Eigen::Vector4d> ManeuverGenerator::calcRowsPoints(double width, double length, double hstep,
				double alternationPercent, double curvOff, bool squareCurve, double bearingRad,
				double crossAngleRadians, bool invertY) {

			width = fabs(width);
			length = fabs(length);
			hstep = fabs(hstep);

			bool direction = true;

			std::vector<Eigen::Vector4d> newPoints;

			Eigen::Vector4d point;
			point<<-curvOff,0,0,-1;

			newPoints.push_back(point);

	// double x1;
			double x2;
			for (double y = 0; y <= width; y += hstep) {

				if (direction) {
	// x1 = -curvOff;
					x2 = length + curvOff;
				}
				else {
	// x1 = length + curvOff;
					x2 = -curvOff;
				}
				direction = !direction;

				double hstepDelta = 0;
				if (direction)
					hstepDelta = hstep * (1 - alternationPercent);
				//Eigen::Vector4d point;
				point << x2, y - hstepDelta, 0, -1;

				newPoints.push_back(point);

				if (y + hstep <= width) {
					double hstepAlt = hstep;
					if (!direction)
						hstepAlt = hstep * alternationPercent;

					point << x2 + (squareCurve ? 0 : 1) * (direction ? curvOff : -curvOff), y + hstepAlt, 0, -1;
					newPoints.push_back(point);
				}
			}

			for (std::vector<Eigen::Vector4d>::iterator it = newPoints.begin() ; it != newPoints.end(); ++it){


				//std::cout << ' ' << *it;

				Eigen::Vector4d vTmp = *it;
			   // double yTmp = vTmp[0];

				std::pair<double,double> res = rotate(-crossAngleRadians, vTmp[0], 0, false);
				vTmp[X] = res.first;
				vTmp[Y] = vTmp[Y] + res.second;
				if (invertY)
					vTmp[Y] = -vTmp[Y];
				res = rotate(bearingRad + (!invertY ? -1 : 1) * -crossAngleRadians, vTmp[X], vTmp[Y], false);
				vTmp[X] = res.first;
				vTmp[Y] = res.second;

				*it = vTmp;

			}

			return newPoints;
		}

		 /**
			 * XY Coordinate conversion considering a rotation angle.
			 *
			 * @param angleRadians angle
			 * @param x original x value on entry, rotated x value on exit.
			 * @param y original y value on entry, rotated y value on exit.
			 * @param clockwiseRotation clockwiseRotation rotation or not
			 */


		 std::pair<double,double> ManeuverGenerator::rotate(double angleRadians, double x, double y, bool clockwiseRotation) {

			double sina = sin(angleRadians);
			double cosa = cos(angleRadians);

			std::pair<double,double> xy;

			if (clockwiseRotation) {
				xy.first = x * cosa + y * sina;
				xy.second = -x * sina + y * cosa;
			}
			else {
				xy.first = x * cosa - y * sina;
				xy.second = x * sina + y * cosa;
			}
			return xy;
		}
	}
}

#endif /* MANEUVERGENERATOR_HPP_ */
