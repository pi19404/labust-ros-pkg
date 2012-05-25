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
#ifndef NOISEMODEL_HPP_
#define NOISEMODEL_HPP_
#include <labust/simulation/matrixfwd.hpp>
#include <labust/xml/xmlfwd.hpp>

#include <boost/random/normal_distribution.hpp>
#include <boost/nondet_random.hpp>
#include <boost/random/variate_generator.hpp>

namespace labust
{
	namespace simulation
	{
		/**
		 * The class incorporates the process and measurement noise used for
		 * the UUV simulation.
		 */
		class NoiseModel
		{
			enum {vSize = 6, wSize = 6};

			typedef boost::variate_generator<boost::random_device&, boost::normal_distribution<> > noise_generator;
		public:
			/**
			 * Generic constructor.
			 */
			NoiseModel();
			/**
			 * Main constructor. Configures the noise vectors using a XML configuration.
			 *
			 * \param reader Pointer to the XMLReader object containing the configuration data.
			 */
			NoiseModel(const labust::xml::ReaderPtr reader);

			/**
			 * Configure the class based on the XML configuration file.
			 *
			 * \param reader Pointer to the XMLReader object containing the configuration data.
			 */
			void configure(const labust::xml::ReaderPtr reader);

			/**
			 * The method returns the process noise vector.
			 *
			 * \return The process noise.
			 */
			const vector& calculateW();
			/**
			 * The method returns the measurement noise vector.
			 *
			 * \return The process noise.
			 */
			const vector& calculateV();

		private:
			/**
			 * The process and measurement noise vectors.
			 */
			vector w,v;
			/**
			 * Stochastic noise generator.
			 */
			boost::random_device rd;
			/**
			 * Gaussian distributions for noise.
			 */
			std::vector<boost::shared_ptr<noise_generator > > ngW,ngV;
		};
	}
}


/* NOISEMODEL_HPP_ */
#endif
