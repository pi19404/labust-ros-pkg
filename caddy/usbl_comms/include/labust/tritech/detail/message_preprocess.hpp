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
 *
 *  Author: Dula Nad
 *  Created: 29.04.2013.
 *********************************************************************/
#ifndef MESSAGE_PREPROCESS_HPP_
#define MESSAGE_PREPROCESS_HPP_
#include <boost/integer/integer_mask.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/tuple/rem.hpp>

#define PP_MSG_FILLER_0(X1,X2,X3) \
	((X1,X2,X3)) PP_MSG_FILLER_1
#define PP_MSG_FILLER_1(X1,X2,X3) \
	((X1,X2,X3)) PP_MSG_FILLER_0
#define PP_MSG_FILLER_0_END
#define PP_MSG_FILLER_1_END

#define DEFINE_MAPPED_MESSAGES(MAPNAME, seq)\
		typedef enum {BOOST_PP_SEQ_FOR_EACH_R(1, GET_MESSAGE_NAME, \
		3, BOOST_PP_CAT(PP_MSG_FILLER_0 seq, _END)) \
		BOOST_PP_CAT(MAPNAME,_enum_last)} BOOST_PP_CAT(MAPNAME,Enum);\
		\
		static void BOOST_PP_CAT(register_,MAPNAME)() \
    { \
			static bool inited(false); \
			if (!inited) \
			{ \
				inited = true; \
				BOOST_PP_SEQ_FOR_EACH_R(1, ADD_TO_MAP, \
						MAPNAME, BOOST_PP_CAT(PP_MSG_FILLER_0 seq, _END))\
			} \
    }; \

#define ADD_MESSAGE_ENUM(r, data, elem) \
	enum {BOOST_PP_TUPLE_ELEM(3,0,elem) = BOOST_PP_TUPLE_ELEM(3,1,elem)};\

#define GET_MESSAGE_NAME(r, data, elem) \
	BOOST_PP_TUPLE_ELEM(3,0,elem) = BOOST_PP_TUPLE_ELEM(3,1,elem),

#define ADD_TO_MAP(r, data, elem) \
	data[BOOST_PP_TUPLE_ELEM(3,1,elem)] = BitMap({4, BOOST_PP_TUPLE_REM_CTOR(11,BOOST_PP_TUPLE_ELEM(3,2,elem))}); \


#define ADD_CADDY_MESSAGE2(r, data, elem) \
	ADD_CADDY_MESSAGE(BOOST_PP_TUPLE_ELEM(3,0,elem), \
			BOOST_PP_TUPLE_ELEM(3,1,elem), \
			BOOST_PP_TUPLE_ELEM(3,2,elem), \
			data) \

#define ADD_CADDY_MESSAGE(NAME, CODE, BITS) \
		struct NAME : public virtual labust::tritech::DiverBitmap\
		{\
			enum{type=CODE};\
			const std::vector<int>& bitmap() \
			{ \
				static const std::vector<int> bitmap({4, BOOST_PP_TUPLE_REM_CTOR(11, BITS)});	\
				return bitmap; \
			} \
		};

/* MESSAGE_PREPROCESS_HPP_ */
#endif



