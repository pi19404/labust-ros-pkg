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
#ifndef LFMODEL_HPP_
#define LFMODEL_HPP_
#include <labust/navigation/SSModel.hpp>
#include <labust/xml/xmlfwd.hpp>

#include <boost/call_traits.hpp>
#include <boost/array.hpp>

namespace labust
{
  namespace navigation
  {
    /**
     * This class implements Line following model.
     *
     * \todo Add on the fly parameters reconfiguration.
     * \todo Add surge speed model.
     */
    class LFModel : public SSModel<double>
    {
      typedef SSModel<double> Base;
    public:
      typedef vector input_type;
      typedef vector output_type;

      enum {z_m=0,psi_m,dV_m,dH_m};
      enum {Z=0,N=1,X=2};
      enum {u=0,w,r,z,psi,dV,dH,buoyancy,vertCurrent,horzCurrent};
      enum {xp=0,yp,zp};
      enum {stateNum = 10};
      enum {inputSize = 3};

      /**
       * This is a helper class for the line calculations.
       */
      class Line
      {
      public:
        /**
         * Calculates the horizontal distance from the line.
         */
        double calculatedH(double x0, double y0, double z0) const;
        /**
         * Calculate the vertical distance from the line.
         */
        double calculatedV(double x0, double y0, double z0) const;
        /**
         * Set the line parameters.
         *
         * \param T1 Position of the line start.
         * \param T2 Position of the line end.
         */
        void setLine(const vector& T1, const vector& T2);

        /**
         * Get the line elevation angle.
         */
        inline double xi(){return this->Xi;};
        /**
         * Get the line azimuth angle.
         */
        inline double gamma(){return this->Gamma;};

      protected:
        /**
         * Line orientation.
         */
        double Gamma, Xi;
        /**
         * Line points.
         */
        Base::vector T1,T2;
      };

    public:
      /**
       * Generic constructor.
       */
      LFModel();
      /**
       * Main constructor that configures the model based on the supplied XML configuration.
       *
       * \param reader The XML configuration object.
       */
      LFModel(const labust::xml::ReaderPtr reader);

      /**
       * Configure the model based on the XML supplied.
       */
      void configure(const labust::xml::ReaderPtr reader);

      /**
       * Initialize the LineFollowing model.
       */
      void initModel();
      /**
       * Perform a prediction step based on the system input.
       *
       * \param u System input.
       */
      void step(const input_type& input);
      /**
       * Calculates the estimated output of the model.
       *
       * \param y Inserts the estimated output values here.
       */
      void estimate_y(output_type& y);

      /**
       * Assemble the measurement vector.
       *
       * \param yaw Vehicle heading.
       * \param depth Vehicle depth.
       */
      const input_type& measurement(double yaw, double depth);
      /**
       * Assemble the measurement vector.
       *
       * \param yaw Vehicle heading.
       * \param depth Vehicle depth.
       * \param x0 Vehicle X position.
       * \param y0 Vehicle Y position.
       */
      const input_type& measurement(double yaw, double x0, double y0, double z0);

      /**
       * Set the line parameters.
       *
       * \param T1 Position of the line start.
       * \param T2 Position of the line end.
       */
      inline void setLine(const vector& T1, const vector& T2){line.setLine(T1,T2);};
      /**
       * Return the used line. Usefull for debugging purposes.
       *
       * \return Returns the line reference.
       */
      inline const Line& getLine(){return line;};

    protected:
      /**
       * Calculates the matrix derivatives of A and W.
       */
      void derivativeAW();
      /**
       * Calculates the matrix derivatives of H and V.
       *
       * \param numMeas Number of available measurements.
       */
      void derivativeHV(int numMeas);

      /**
       * The line parameters
       */
      Line line;
      /**
       * The measurement vector.
       */
      vector meas;
      /**
       * Heave model parameters.
       */
      double betaW, alphaW;
      /**
       * Yaw rate model parameters
       */
      double betaR, alphaR;
      /**
       * The full measurement covariance.
       */
      matrix R0,V0;
    };
  };
};

/* LFMODEL_HPP_ */
#endif
