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
*  Created on: 13.06.2013.
*  Author: Dula Nad
*********************************************************************/
#ifndef AAGUI_HPP_
#define AAGUI_HPP_
#include <boost/shared_ptr.hpp>

#include <GL/glut.h>
#include <string>

namespace labust
{
	namespace gui
	{
		/**
		 * The structure contains the refactored OpenGL code for visual debuging of the augmented
		 * acoustics application by A. Vasilijevic.
		 */
		struct AAGui
		{
			typedef boost::shared_ptr<AAGui> Ptr;

			AAGui();
			/**
			 * Main constructor
			 *
			 * \param path The texture location path.
			 */
			AAGui(const std::string& path);

			/**
			 * Init callbacks.
			 */
			void init();
			/**
			 * Load textures.
			 */
			void loadTextures(const std::string& path);

			void start();

			void drawSkyBox(float x, float y);

			GLuint texture;
			GLuint skyboxTexture[6];
		};
	}
}

/* AAGUI_HPP_ */
#endif
