/* 
 * File:   GyrosMoosCommsInterface.h
 * Author: tomi
 *
 * Created on February 14, 2011, 3:35 PM
 */

#ifndef _GYROSMOOSCOMMSINTERFACE_H
#define	_GYROSMOOSCOMMSINTERFACE_H

#include <MOOSLIB/MOOSApp.h>
#include <GyrosCommsInterface.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include "MoosConfig.hpp"


namespace LABUST
{
    namespace COMMUNICATION
    {

        class GyrosMoosCommsInterface : public GyrosCommsInterface, CMOOSApp
        {
        public:
            ///
            /// Constructor that reads MOOS configuration from a preloaded XML file /xmlreader)
			/// XML reader must be set to the right <config> node before
            ///
            /// \param reader XML reader with preloaded and preset config file, mandatory
			/// \param configToUse name of configuration to use, if empty, the first one will be used
            ///
            GyrosMoosCommsInterface(const labust::xml::Reader &reader, std::string configToUse = "");

            ///
            /// Constructor that reads MOOS configuration from an XML file on disk
            /// finds the first element <commsConfig type="moos"> with the configToUse name (if provided)
            ///
            /// \param configPath path to config file, mandatory
			/// \param configToUse name of configuration to use, if empty, the first one will be used
            ///
            GyrosMoosCommsInterface(const std::string& configPath, std::string configToUse = "");
            
            virtual ~GyrosMoosCommsInterface();

            /**
             * PASSIVE or ACTIVE usage
             * Sends data through MOOS
             * data is passed as a GYROSWriter object \see GyrosWriter.hpp
             * MOOS variable the data is sent to is determined by the label of gyros object
             * If gyros has no label, output variable is "[process name]_OUT";
             *
             * \param data reference to GYrosWriter with data to send
             * \param wait true if function should block until the data is sent, false to return immediately
             * \return COMERRORS::ComError in case of error, 0 if everything is ok
             */
            COMMERRORS::CommError Send(const labust::xml::GyrosWriter &data, bool wait = false);

            /**
             * PASSIVE or ACTIVE usage
             * Sends data through MOOS
             * data is passed as a GYROSWriter object vector \see GyrosWriter.hpp
             * MOOS variable the data is sent to is determined by the label of gyros object
             * If gyros has no label, output variable is "[process name]_OUT";
             *
             * \param data reference to GYrosWriter vector with data to send
             * \param wait true if function should block until the data is sent, false to return immediately
             * \return COMERRORS::ComError in case of error, 0 if everything is ok
             */
            COMMERRORS::CommError Send(const std::vector<labust::xml::GyrosWriter> &data, bool wait = false);

            /**
             * PASSIVE
             * This methods allows to receive data from the comms buffer.
             * in case the system has to initiate communication
             * Stores all received data in the GYROSreader vector as multiple objects
             * If lock is set, the call blocks until data is available for receiving, otherwise exits with an empty vector
             * NAme of variable data was received from is stored as label of the gyros object
             *
             * \param data vector of gyros objects to hold new data
             * \param wait true if the call should block until data is available, false to return error code
             * \return COMERRORS::ComError in case of error, 0 if everything is ok
             */
            COMMERRORS::CommError Receive(std::vector<labust::xml::GyrosReader> &data, bool wait = false);

            /**
             * Returns pointer to MOOS client
             *
             * \return pointer to MOOS client
             */
            const void* GetCommObject(void);

            /**
             * ACTIVE
             * Allows registration of object which will handle incomming messages from MOOS
             *
             * \param obj pointer to object on which callbacks will be executed
             */
            virtual void RegisterCallbackObject(CommEntity *entity);

            /**
             * sets callbackobject to NULL
             */
            virtual void UnRegisterCallbackObject();

            /**
             * MOOSApp overrides
             * \see MOOSAPP.h
             */
            bool OnNewMail(MOOSMSG_LIST &NewMail);
            bool Iterate();

            bool OnConnectToServer();
            bool OnStartUp();
			bool OnDisconnectFromServer();
            
        private:
            GyrosMoosCommsInterface(const GyrosMoosCommsInterface& orig);
            GyrosMoosCommsInterface & operator =(const GyrosMoosCommsInterface &);

            MoosConfig config;
            std::vector<labust::xml::GyrosWriter> messagesToSend;
            std::vector<labust::xml::GyrosReader> messagesReceived;

            boost::thread commsThread;
            boost::condition_variable dataSentSignal, dataReceivedSignal;
            boost::mutex locker;
        };
    }
}
#endif	/* _GYROSMOOSCOMMSINTERFACE_H */

