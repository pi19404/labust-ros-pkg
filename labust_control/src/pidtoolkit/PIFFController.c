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
#include <labust/control/PIFFController.h>
#include <math.h>

void PIFF_modelTune(PIDBase* self,
		const PT1Model* const model,
		float w)
{
	self->Kp = 2*w*model->alpha-model->beta;
	self->Ki = model->alpha*w*w;
	self->Kd = self->Kt = self->Tf = 0;

	self->model.alpha = model->alpha;
	self->model.beta = model->beta;
	self->model.betaa = model->betaa;
}

void PIFF_tune(PIDBase* self, float w)
{
	self->Kp = 2*w;
	self->Ki = w*w;
	self->Kd = self->Kt = self->Tf = 0;
}

void PIFF_wffStep(PIDBase* self, float Ts, float error, float ff)
{
	if (1 || self->extTrack == 0)
	{
		if (self->autoWindup != 0)
		{
			//Report windup and perform anti-windup step
			self->windup = (self->output != self->internalState) &&
					(self->output*error > 0);
		}
		else
		{
			//If externally reported windup perform tracking
			self->windup = (self->extWindup != 0) &&
					(self->output*error > 0);
		}

		//Backward recalculation
		if ((self->lastI != 0) && self->windup && self->useBackward)
		{
			//Calculate the proportional influence
			float diff = self->track - self->internalState + self->lastI;
			//If the proportional part is already in windup remove the whole last integral
			//Otherwise recalculate the integral to be on the edge of windup
			self->internalState -= ((diff*self->track <= 0)?self->lastI:0*(self->lastI - diff));
		}

		//Proportional term
		self->lastP = self->Kp*(error-self->lastError);
		//self->internalState += self->lastP;

		//Integral term
		//This is the equivalent of full tracking anti-windup
		if (self->windup && self->extTrack)
		{
			self->lastI = self->Ki*Ts*error + 2*self->Ki*Ts*(self->track - self->internalState);
		}
		else if (!self->extTrack)
		{
			self->lastI = ((!self->windup)?self->Ki*Ts*error:0);
			//self->internalState += self->lastI;
		}

		//Feed forward term
		self->lastF = ff - self->lastFF;
		//self->internalState += self->lastF;
		//Set initial output
		self->output = self->internalState += self->lastP + self->lastI + self->lastF;

		if (self->autoWindup != 0)
			self->output = sat(self->internalState,-self->outputLimit, self->outputLimit);
	}
	else
	{
		self->internalState = self->output = self->track;
	}

	self->lastError = error;
	self->lastRef = self->desired;
	self->lastFF = ff;
}



