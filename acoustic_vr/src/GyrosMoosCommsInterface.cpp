/* 
 * File:   MoosCommsInterface.cpp
 * Author: tomi
 * 
 * Created on February 14, 2011, 3:35 PM
 */

#include <queue>
#include <fstream>
#include <moos/GyrosMoosCommsInterface.h>
#include <boost/system/system_error.hpp>
#include <labust/plugins/PlugableDefs.hpp>

namespace LABUST
{
    namespace COMMUNICATION
    {

        GyrosMoosCommsInterface::GyrosMoosCommsInterface(const labust::xml::Reader &reader, std::string configToUse)
        {
            std::cout << "Configuring MOOS Comms interface" << std::endl;

            std::string configQuery;
            if (configToUse.empty())
            {
                configQuery = "commsConfig[@type='moos']";
            }
            else
            {
                configQuery = "commsConfig[@type='moos' and @name='" + configToUse + "']";
            }

            //setDefaults
            callbackObject = NULL;

            _xmlNode* configNode = NULL;
            try
            {
                if (reader.try_value(configQuery, &configNode))
                {
                   _xmlNode* origin = reader.currentNode();
                    const_cast<labust::xml::Reader&>(reader).useNode(configNode);
                    std::ofstream configFile;
                    config = moos_configure(reader, configFile);
                    //start MOOS
					commsThread = boost::thread(static_cast<bool (GyrosMoosCommsInterface::*)(const char*, const char*)> (&GyrosMoosCommsInterface::Run), this, config.processName.c_str(), "config.moos");
					                
					const_cast<labust::xml::Reader&>(reader).useNode(origin);
                }
            }
            catch (boost::system::system_error exc)
            {
                std::cerr << "Error starting MOOS Comms interface: " << exc.what() << std::endl;
                throw;
            }
            std::cout << "MOOS Comms interface started" << std::endl;
        }

        GyrosMoosCommsInterface::GyrosMoosCommsInterface(const std::string &configPath, std::string configToUse)
        {
            std::ifstream configFile;
            configFile.open(configPath.c_str());
            std::stringstream configXML;
            if (configFile.is_open())
            {
                std::string line;
                while (configFile.good())
                {
                    std::getline(configFile, line);
                    configXML << line << std::endl;
                }
                configFile.close();
            }

            labust::xml::Reader reader(configXML.str());

            std::cout << "Configuring MOOS Comms interface" << std::endl;

            std::string configQuery;
            if (configToUse.empty())
            {
                configQuery = "//commsConfig[@type='moos']";
            }
            else
            {
                configQuery = "//commsConfig[@type='moos' and @name='" + configToUse + "']";
            }

            //setDefaults
            callbackObject = NULL;

            reader.useRootNode();
            _xmlNode* configNode = NULL;
            try
            {
                if (reader.try_value(configQuery, &configNode))
                {
                    reader.useNode(configNode);
                    std::ofstream configFile;
                    config = moos_configure(reader, configFile);
                    //start MOOS
					commsThread = boost::thread(static_cast<bool (GyrosMoosCommsInterface::*)(const char*, const char*)> (&GyrosMoosCommsInterface::Run), this, config.processName.c_str(), "config.moos");
				}
            }
            catch (boost::system::system_error exc)
            {
                std::cerr << "Error starting MOOS Comms interface: " << exc.what() << std::endl;
                throw;
            }
            std::cout << "MOOS Comms interface started" << std::endl;
        }

        GyrosMoosCommsInterface::GyrosMoosCommsInterface(const GyrosMoosCommsInterface& orig)
        {
        }

        GyrosMoosCommsInterface::~GyrosMoosCommsInterface()
        {
            std::cout << "Stopping Moos Comms Interface" << std::endl;
            this->RequestQuit();
            commsThread.join();
            std::cout << "Moos Comms Interface stopped" << std::endl;
        }

		COMMERRORS::CommError GyrosMoosCommsInterface::Send(const labust::xml::GyrosWriter &data, bool wait)
        {
            boost::mutex::scoped_lock lock(locker);
            messagesToSend.push_back(data);
            if (wait)
            { //wait for comms thread to confirm data is sent
                dataSentSignal.wait(lock);
            }
            return COMMERRORS::noError;
        }

        COMMERRORS::CommError GyrosMoosCommsInterface::Send(const std::vector<labust::xml::GyrosWriter> &data, bool wait)
        {
            boost::mutex::scoped_lock lock(locker);
            for (std::vector<labust::xml::GyrosWriter>::const_iterator gyros = data.begin(); gyros != data.end(); gyros++)
            {
                messagesToSend.push_back(*gyros);
            }
            if (wait)
            { //wait for comms thread to confirm data is sent
                dataSentSignal.wait(lock);
            }
            return COMMERRORS::noError;
        }

        COMMERRORS::CommError GyrosMoosCommsInterface::Receive(std::vector<labust::xml::GyrosReader>& gyrosObjects, bool wait)
        {
            COMMERRORS::CommError retVal = COMMERRORS::noError;
            if (callbackObject == NULL)
            {
                boost::mutex::scoped_lock lock(locker);
                while (wait && messagesReceived.empty())
                { //if no data received, wait for comms thread to signal some data is received
                    dataReceivedSignal.wait(lock);
                }

                if (!messagesReceived.empty())
                {
                    gyrosObjects.insert(gyrosObjects.end(), messagesReceived.begin(), messagesReceived.end());
                    messagesReceived.clear();
                }
                else if (!wait)
                {
                    retVal = COMMERRORS::noData;
                }
            }
            else
            {
                std::cout << "Callback object is registered, please unregister it before using Receive function" << std::endl;
                retVal = COMMERRORS::callbackRegistered;
            }
            return retVal;
        }

        const void* GyrosMoosCommsInterface::GetCommObject()
        {
            return &m_Comms;
        }

        void GyrosMoosCommsInterface::RegisterCallbackObject(CommEntity* entity)
        {
            this->callbackObject = entity;
        }

        void GyrosMoosCommsInterface::UnRegisterCallbackObject()
        {
            //mutex to prevent unregistration of callbackObject while it is being used
            boost::mutex::scoped_lock lock(locker);
            this->callbackObject = NULL;
        }

        bool GyrosMoosCommsInterface::OnNewMail(MOOSMSG_LIST& NewMail)
        {
          using namespace labust::xml;
            std::vector<GyrosReader> incomingMessages;
            boost::mutex::scoped_lock lock(locker);
            for (MOOSMSG_LIST::iterator mailIterator = NewMail.begin(); mailIterator != NewMail.end(); mailIterator++)
            {
                CMOOSMsg &message = *mailIterator;
                std::string messageString = message.GetString();
                try
                {
                    GyrosReader reader(message.GetString());
                    incomingMessages.push_back(reader);
                }
                catch (GyrosError error)
                {
                    incomingMessages.push_back(GyrosReader(error.what()));
                }
                catch (XMLException e)
                {
                    std::cerr << e.what() << std::endl;
                }

            }
            if (callbackObject != NULL)
            {
                callbackObject->AcceptData(incomingMessages);
            }
            else
            {
                messagesReceived.insert(messagesReceived.end(), incomingMessages.begin(), incomingMessages.end());
            }
            if (!messagesReceived.empty())
            { //signal data is ready
                dataReceivedSignal.notify_all();
            }
            return true;
        }

        bool GyrosMoosCommsInterface::Iterate()
        {
            boost::mutex::scoped_lock lock(locker);
            std::string outputMessageName;
            for (std::vector<labust::xml::GyrosWriter>::iterator gyros = messagesToSend.begin(); gyros != messagesToSend.end(); gyros++)
            {
                outputMessageName = (*gyros).GetLabel();
                if (outputMessageName.empty())
                {
                    m_Comms.Notify(config.processName + "_OUT", (*gyros).GyrosXML(), MOOSTime());
                }
                else
                {
                    m_Comms.Notify(outputMessageName, (*gyros).GyrosXML(), MOOSTime());
                }
            }
            messagesToSend.clear();

            //signal data is sent
            dataSentSignal.notify_all();
            return true;
        }

        bool GyrosMoosCommsInterface::OnConnectToServer()
        {
			connected = true;
            if (!config.subscriptionVars.empty())
            {
                for (unsigned int i = 0; i < config.subscriptionVars.size(); i++)
                {
                    std::cout << "\nRegistering for '" << config.subscriptionVars[i] << "'";
                    m_Comms.Register(config.subscriptionVars[i], 0);
                }
            }
            else
            {
                std::cerr << "No subscriptions specified" << std::endl;
            }
            return true;
        }

        bool GyrosMoosCommsInterface::OnStartUp()
        {
			remove("config.moos");
			if (!config.subscriptionVars.empty())
            {
                for (unsigned int i = 0; i < config.subscriptionVars.size(); i++)
                {
                    std::cout << "\nRegistering for '" << config.subscriptionVars[i] << "'";
                    m_Comms.Register(config.subscriptionVars[i], 0);
                }
            }
            else
            {
                std::cerr << "Warning: no subscriptions specified" << std::endl;
            }
            return true;
        }

		bool GyrosMoosCommsInterface::OnDisconnectFromServer()
		{
			connected = false;
			return true;
		}

		LABUST_EXTERN
		{
			LABUST_EXPORT LABUST::COMMUNICATION::CommsFactoryPtr createCommsFactory()
			{
				return LABUST::COMMUNICATION::CommsFactoryPtr(new LABUST::COMMUNICATION::CommsFactory::Impl<GyrosMoosCommsInterface>());
			}
		}
    }
}


