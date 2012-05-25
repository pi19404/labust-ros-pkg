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
#ifndef NMEASENTENCE_HPP_
#define NMEASENTENCE_HPP_
#include <labust/tools/StringUtilities.hpp>
#include <string>

namespace labust
{
	namespace comms
	{
		/**
		 *  This class implements an abstract NMEASentence class. User should inherit from this class when writing
		 *  their own NMEASentence parsers.
		 *
		 *  NMEA sentence protocol rules:
		 *   - Each message's starting character is a dollar sign.
		 *   - The next five characters identify the talker (two characters) and the type of message (three characters).
		 *   - All data fields that follow are comma-delimited.
		 *   - Where data is unavailable, the corresponding field contains NUL bytes (e.g., in "123,,456", the second field's data is unavailable).
		 *   - The first character that immediately follows the last data field character is an asterisk.
		 *   - The asterisk is immediately followed by a two-digit checksum representing a hex number. The checksum is the exclusive OR of all characters between the $ and *. According to the official specification, the checksum is optional for most data sentences, but is compulsory for RMA, RMB, and RMC (among others).
		 *   - <CR><LF> ends the message. (THIS IS AVOIDED)
		 */
		class NMEASentence
		{
		public:
			/**
			 * Main constructor. Takes a sentence in a string format and parses it.
			 *
			 * \param sentence The NMEA sentence to be parsed.
			 */
			NMEASentence(const std::string& sentence)
			{
				this->parse(sentence);
			}
			/**
			 * Generic destructor.
			 */
			~NMEASentence(){};

			bool parse(const std::string& sentence)
			{
				//Copy the message into a temporary string
				std::string str(sentence);
				int retVal = -1;
				int firstComma = str.find(',');
				//Test for minimum similarity to a NMEA message
				// -> string should start with the $ sign
				// -> existence of ','
				if (!((str.at(0) == '$') && (firstComma != -1)))
				{
					return false;
				}

				this->id = str.substr(1,2);
				this->name = str.substr(3,3);
				str.erase(0,7);

				dataFields.clear();
				std::string part;

				//Special case: if last element is ',' then the
				//last data field is ""
				bool addOne = (str.length()>0) && (str.find_last_of(',') == (str.length()-1));

				while (str.length()>0)
				{
					part=labust::tools::chomp(str);
					dataFields.push_back(part);
				}

				if (addOne)
				{
					dataFields.push_back("");
				}

				return true;
			}

			/**
			 * The method returns the sentence name.
			 */
			const std::string& getName(){return this->name;}

		private:

			/**
			 * The sentence talker ID and name.
			 */
			std::string id, name;
			/**
			 * The parsed data fields.
			 */
			std::vector<std::string> dataFields;
		};
	}
}


/* NMEASENTENCE_HPP_ */
#endif
