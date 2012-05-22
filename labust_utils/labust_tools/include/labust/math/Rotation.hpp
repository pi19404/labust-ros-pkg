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
#ifndef ROTATION_HPP_
#define ROTATION_HPP_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

namespace labust
{
  namespace math
  {
    /**
     * The structures construct a rotation matrix object. Based on given angles a 3x3 rotation matrix
     * is constructed.
     */
    struct rotation_matrix
    {
      typedef boost::numeric::ublas::c_matrix<double,3,3> matrix;
      /**
       * Default constructor. Instantiates a 3x3 matrix.
       */
      rotation_matrix(): result(3,3){};
      /**
       * Helper constructor. Instantiates a 3x3 matrix with values.
       *
       * \param phi The roll angle about X-axis.
       * \param theta The pitch angle about Y-axis.
       * \param psi The yaw angle about Z-axis.
       */
      rotation_matrix(double phi, double theta, double psi): result(3,3){this->operator()(phi, theta, psi);};
      /**
       * The operator calculates the general rotation matrix for all euler angels.
       *
       * \param phi The roll angle about X-axis.
       * \param theta The pitch angle about Y-axis.
       * \param psi The yaw angle about Z-axis.
       *
       * \return The calculated rotation matrix.
       */
      const matrix& operator()(double phi, double theta, double psi)
      {
        double c1 = cos(phi), s1 = sin(phi);
        double c2 = cos(theta), s2 = sin(theta);
        double c3 = cos(psi), s3 = sin(psi);

        result(0,0)=c3*c2;
        result(0,1)=c3*s2*s1-s3*c1;
        result(0,2)=s3*s1+c3*c1*s2;
        result(1,0)=s3*c2;
        result(1,1)=c1*c3+s1*s2*s3;
        result(1,2)=c1*s2*s3-c3*s1;
        result(2,0)=-s2;
        result(2,1)=c2*s1;
        result(2,2)=c1*c2;

        return result;
      }

      /**
       * The method calculates the rotation matrix for only the yaw angle.
       *
       * \param angle The yaw angle about the Z-axis.
       *
       * \return The calculated rotation matrix.
       */
      const matrix& rotZ(double angle)
      {
        result(0,0)=cos(angle);
        result(0,1)=-sin(angle);
        result(0,2)=0;
        result(1,0)=sin(angle);
        result(1,1)=cos(angle);
        result(1,2)=0;
        result(2,0)=0;
        result(2,1)=0;
        result(2,2)=1;

        return result;
      }
      /**
       * The method calculates the rotation matrix for only the pitch angle.
       *
       * \param angle The pitch angle about the Y-axis.
       *
       * \return The calculated rotation matrix.
       */
      const matrix& rotY(double angle)
      {
        result(0,0)=cos(angle);
        result(0,1)=0;
        result(0,2)=sin(angle);
        result(1,0)=0;
        result(1,1)=1;
        result(1,2)=0;
        result(2,0)=-sin(angle);
        result(2,1)=0;
        result(2,2)=cos(angle);

        return result;
      }
      /**
       * The method calculates the rotation matrix for only the roll angle.
       *
       * \param angle The roll angle about the X-axis.
       *
       * \return The calculated rotation matrix.
       */
      const matrix& rotX(double angle)
      {
        result(0,0)=1;
        result(0,1)=0;
        result(0,2)=0;
        result(1,0)=0;
        result(1,1)=cos(angle);
        result(1,2)=-sin(angle);
        result(2,0)=0;
        result(2,1)=sin(angle);
        result(2,2)=cos(angle);

        return result;
      }

      /**
       * The operator returns the last calculated rotation matrix.
       * Useful for multiple use of the rotation matrix.
       *
       * \return The calculated rotation matrix.
       */
      const matrix& operator()(){return result;};

    private:
      /**
       * The calculated rotation matrix.
       */
      matrix result;
    };

    /**
     * The structure creates a angular velocity transformation based on the given Euler angles.
     */
    struct ang_vel_trans
    {
      typedef boost::numeric::ublas::c_matrix<double,3,3> matrix;
      /**
       * Default contructor. Instantiates the 3x3 matrix.
       */
      ang_vel_trans() : result(3,3){};

      /**
       * The opeartor returns the angluar velocity transformation.
       *
       * \param phi The roll angle about X-axis.
       * \param theta The pitch angle about Y-axis.
       * \param psi The yaw angle about Z-axis.
       *
       * \return The calculated transformation matrix.
       */
      const matrix& operator()(double phi, double theta, double psi)
      {
        double c1 = cos(phi), s1 = sin(phi);
        double c2 = cos(theta), t2 = tan(theta);

        result(0,0)=1;
        result(0,1)=s1*t2;
        result(0,2)=c1*t2;
        result(1,0)=0;
        result(2,0)=0;
        result(1,1)=c1;
        result(1,2)=-s1;
        result(2,1)=s1/c2;
        result(2,2)=c1/c2;

        return result;
      }

      /**
       * The operator returns the last calculated transformation matrix.
       * Useful for multiple use of the rotation matrix.
       *
       * \return The calculated transformation matrix.
       */
      const matrix& operator()(){return result;};
    private:
      /**
       * The calculated transformation matrix.
       */
      matrix result;
    };
    /**
     * This creates a 6x6 linear and angular velocity transformation matrix.
     * It is a combination of rotation_matrix and ang_vel_trans functors.
     */
    struct transform_matrix
    {
      typedef boost::numeric::ublas::c_matrix<double,6,6> matrix;
      /**
       * Default constructor. Instantiates the 6x6 matrix.
       */
      transform_matrix() : result(6,6){};

      /**
       * The operator calculates the desired transformation matrix.
       *
       * \param phi The roll angle about X-axis
       * \param theta The pitch angle about Y-axis
       * \param psi The yaw angle about Z-axis
       *
       * \return The calculated transformation matrix.
       */
      const matrix& operator()(double phi, double theta, double psi)
      {
        using namespace boost::numeric::ublas;
        subrange(result, 0,3, 0,3 ) = rotation_matrix()(phi,theta,psi);
        subrange(result, 0,3, 0,3 ) = ang_vel_trans()(phi,theta,psi);

        return result;
      }
      /**
       * The operator returns the last calculated transformation matrix.
       * Useful for multiple use of the rotation matrix.
       *
       * \return The calculated transformation matrix.
       */
      const matrix& operator()(){return result;};

    private:
      /**
       * The calculated transformation matrix.
       */
      matrix result;
    };
  }
}
/* ROTATION_HPP_ */
#endif
