/*
 * GyrosCommsInterface.hpp
 *
 *  Created on: Feb 14, 2011
 *      Author: Tomislav Lugaric
 *
 * This class represents an abstract communication interface
 * The communication interface is configured from an XML file
 * It uses a standardised xml based data format for communication
 * It can operate in active or passive mode
 * In active mode, the interface carries out operations when data is sent or received
 * In passive mode, an external object polls the interface for data
 * 
 */

#ifndef GYROSCOMMSINTERFACE_HPP_
#define GYROSCOMMSINTERFACE_HPP_

#include <LABUSTTypes.h>
#include <string>
#include <labust/xml/GyrosReader.hpp>
#include <labust/xml/GyrosWriter.hpp>
#include <labust/xml/GyrosMatrix.hpp>
#include <boost/smart_ptr.hpp>

#include <labust/plugins/Factory.hpp>
#include <labust/plugins/DLLoad.hpp>
#include <labust/xml/xmlfwd.hpp>

namespace LABUST
{
    namespace COMMUNICATION
    {

        namespace COMMERRORS
        {

            enum CommError
            {
                notImplemented = -2, // for functions that do not have an implementation (e.g. optional legacy functions)
                undefined = -1, // generic error
                noError = 0,
                noData = 1,
                callbackRegistered,
                communicationFailed, // for send/receive
                commsMisconfigured // if interface is not successfully configured
            };
        }

        /** this represents an object which is passed to a commsInterface for performing callbacks
         * If a class will rely on callbacks from CommsInterface, it must inherit CommEntity class
         */
        class CommEntity
        {
        public:
            /**
             * Application hook te receive data from CommsInterface
             * Can be used to parse data, or as a switchboard to pass received data to
             * the correct member functions
             *
             * \param inputData data received from comms interface, GYROSReader object vector
             * \returns error code, 0 if OK
             */
            virtual int AcceptData(std::vector<labust::xml::GyrosReader> &data) = 0;
        private:

        };

        /**
         * This class represents the communication interface which can be used to
         * send and receive data with other processes
         * It is configured from an XML config file
         * The interface uses the GYROS data format (XML based)
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
	    virtual COMMERRORS::CommError Send(const labust::xml::GyrosWriter &data, bool wait = false) = 0;

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
            virtual COMMERRORS::CommError Send(const std::vector<labust::xml::GyrosWriter> &data, bool wait = false) = 0;

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
            virtual COMMERRORS::CommError Receive(std::vector<labust::xml::GyrosReader> &data, bool wait = false) = 0;

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
            virtual void unRegisterCallbackObject()
            {
                this->callbackObject = NULL;
            };

			inline const bool Connected()
			{
				return connected;
			}
        protected:

            /**
             * The comm channel can call the callback method on this object when
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


	typedef boost::shared_ptr<GyrosCommsInterface> GyrosCommsInterfacePtr;


    //Plugin factory typedef
    typedef labust::plugins::TmplPluginFactory<
      GyrosCommsInterface,
      const labust::xml::Reader&,
      const std::string&> CommsFactory;
    //Factory typedef
    typedef CommsFactory::AbstractFactory* CommsFactoryPtr;

    typedef labust::plugins::DLLoad<LABUST::COMMUNICATION::CommsFactory> CommsPlugin;
    typedef boost::shared_ptr<CommsPlugin> CommsPluginPtr;
 

    };
};
#endif /* GYROSCOMMSINTERFACE_HPP_ */
