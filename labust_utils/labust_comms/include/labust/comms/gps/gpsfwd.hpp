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
#ifndef GPSFWD_HPP_
#define GPSFWD_HPP_
#include <labust/xml/xmlfwd.hpp>

namespace labust
{
	namespace comms
	{
		/**
		 * NMEASentence forward declaration.
		 */
		class NMEASentence;
		/**
		 * The navigation data contained in GPS messages.
		 */
		struct GPSNavData
		{
			/**
			 * The latitude and longitude.
			 */
			float northing, easting;
			/**
			 * The device course.
			 */
			float courseTrue, courseMagnetic;
			/**
			 * Progression speed.
			 */
			float speed;
			/**
			 * Geoidal data.
			 */
			float altitudeMSL, geoidalSeparation;
			/**
			 * Time information.
			 */
			std::string fixTime, fixDate;
			/**
			 * Message validity.
			 */
      bool valid;

      /**
       * Converts from XML to GPS navigation data.
       *
       * \param reader The XML reader containing the encoded data.
       * \param data The GPSNavData to be populated.
       *
       * \return Returns the reader object for chaining.
       */
      friend const labust::xml::Reader& operator>>(const labust::xml::Reader& reader, GPSNavData& data);
		};

		/**
		 * The method parses GGA sentences.
		 *
		 * \param gga The GGA NMEA sentence.
		 * \param nav The navigation data object to be filled.
		 * \param fix The fix data object to be filled.
		 */

    /**
     * Converts directly to XML encoding without type conversion.
     *
     * \param sentence The sentence to convert.
     * \param writer The XML writer object for writing.
     */
    void gga2xml(const NMEASentence& sentence, labust::xml::Writer* writer);
	}
}

/* GPSFWD_HPP_ */
#endif
