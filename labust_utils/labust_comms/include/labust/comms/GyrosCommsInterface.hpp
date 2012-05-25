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
#ifndef GYROSCOMMSINTERFACE_HPP_
#define GYROSCOMMSINTERFACE_HPP_
#include <labust/xml/GyrosReader.hpp>
#include <labust/xml/GyrosWriter.hpp>
#include <labust/xml/GyrosMatrix.hpp>
#include <boost/smart_ptr.hpp>

#include <string>

namespace labust
{
	namespace comms
	{

		namespace commerrors
		{
			/** 
			 * \enum ComError 
			 * \brief enum that contains error codes that are returned from Send() and Receive() methods
			 */
			enum CommError
			{
				notImplemented = -2, /**< for functions that do not have an implementation (e.g. optional legacy functions) */
				undefined = -1, /**< generic error code */
				noError = 0, /**< code to return on success */
				noData = 1, /**< no data is received - used when wait is false on receive*/
				callbackRegistered, /**< Callback is registered and user tries to receive data using Receive()*/
				communicationFailed, /**< Error occured while communicating */
				commsMisconfigured /**< Trying to use comms which is not configured properly */
			};
		}

		/**
		 * \class CommEntity
		 * \brief This class represents an object which is passed to a commsInterface for performing callbacks.
		 * If a class will rely on callbacks from CommsInterface, it must inherit from CommEntity class.
		 */
		class CommEntity
		{
		public:
			/**
			 * Generic destructor.
			 */
			virtual ~CommEntity(){};
			/**
			 * Application hook te receive data from CommsInterface.
			 * Can be used to parse data, or as a switchboard to pass received data to
			 * the correct member functions.
			 *
			 * \param inputData data received from comms interface, GYROSReader object vector
			 * \returns error code, 0 if OK
			 */
			virtual int AcceptData(std::vector<labust::xml::GyrosReader> &data) = 0;
		private:

		};

		/**
		 * This class represents the communication interface which can be used to
		 * send and receive data with other processes.
		 * It is configured from an xml config file.
		 * The interface uses the GYROS data format (xml based).
		 * The CommsInterface must ensure that any communication protocol can be used in two ways:
		 * PASSIVE: data is sent and received by calling member functions
		 * ACTIVE: data is sent by calling CommsInterface member functions, and received data is passed from CommsInterface through a callback (see CommEntity)
		 */
		class GyrosCommsInterface
		{
		public:

			GyrosCommsInterface():
				connected(false)
			{};

			/*
			 * Destructor, if interface needs to be explicitely terminated, put code here
			 */
			virtual ~GyrosCommsInterface()
			{
			}

			/**
			 * PASSIVE or ACTIVE usage
			 * This method allows user to send data over the underlying connection channel
			 * data is passed as a GYROS writer object \see GyrosWriter.hpp
			 * Must be implemented
			 *
			 * \param data vector of gyroswriters with data to send
			 * \param wait true if function should block until the data is sent, false to return immediately
			 * \return COMERRORS::ComError in case of error, 0 if everything is ok
			 */
			virtual commerrors::CommError Send(const labust::xml::GyrosWriter &data, bool wait = false) = 0;

			/**
			 * PASSIVE or ACTIVE usage
			 * This method allows user to send data over the underlying connection channel
			 * data is passed as a GYROS writer object vector \see GyrosWriter.hpp
			 * Must be implemented
			 *
			 * \param data vector of gyroswriters with data to send
			 * \param wait true if function should block until the data is sent, false to return immediately
			 * \return COMERRORS::ComError in case of error, 0 if everything is ok
			 */
			virtual commerrors::CommError Send(const std::vector<labust::xml::GyrosWriter> &data, bool wait = false) = 0;

			/**
			 * PASSIVE
			 * This methods allows to receive data from the comms buffer.
			 * Stores all received data in the GYROS vector as multiple objects
			 * If lock is set, the call blocks until data is available for receiving, otherwise exits with an empty vector
			 * Must be implemented
			 *
			 * \param data vector of gyrosreaders with received data
			 * \param wait true if the call should block until data is available, false to return error code
			 * \return COMERRORS::ComError in case of error, 0 if everything is ok
			 */
			virtual commerrors::CommError Receive(std::vector<labust::xml::GyrosReader> &data, bool wait = false) = 0;

			/**
			 * This methods allows to access the hidden comms device. This allows
			 * the user to set some specific options. The user is not forced to
			 * implement this function.
			 *
			 * \return void pointer to the underlaying comm object
			 */
			virtual const void* GetCommObject(void)
			{
				return NULL;
			};

			/**
			 * ACTIVE
			 * this method allows user to register an object which will be used as a communications hook
			 * in case the comm channel has to initiate communication
			 * The comm channel should call this function when it has received data
			 * and needs the system to provide or read it
			 *
			 * \param obj pointer to object on which callbacks will be executed
			 */
			virtual void RegisterCallbackObject(CommEntity *entity)
			{
				this->callbackObject = entity;
			};

			/**
			 * sets callbackobject to NULL
			 * Must not delete the object!
			 */
			virtual void UnRegisterCallbackObject()
			{
				this->callbackObject = NULL;
			};

			inline bool Connected() const
			{
				return connected;
			}
		protected:

			/**
			 * In ACTIVE mode the comm channel can call the callback method on this object when
			 * it is ready to communicate
			 */
			CommEntity *callbackObject;

			bool connected;
		private:
			/*
			 * Noncopyable
			 */
			GyrosCommsInterface(const GyrosCommsInterface& orig);
			GyrosCommsInterface & operator =(const GyrosCommsInterface &);
		};
	};
};
#endif /* GYROSCOMMSINTERFACE_HPP_ */
