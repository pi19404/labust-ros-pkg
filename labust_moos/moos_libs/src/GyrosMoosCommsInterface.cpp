/* 
 * File:   MoosCommsInterface.cpp
 * Author: tomi
 * 
 * Created on February 14, 2011, 3:35 PM
 */
#include <labust/moos/GyrosMoosCommsInterface.h>

#include <boost/system/system_error.hpp>

#include <queue>
#include <fstream>

namespace labust
{
    namespace comms
    {

        GyrosMoosCommsInterface::GyrosMoosCommsInterface(const xml::Reader &readerin, std::string configToUse)
        {
xml::Reader reader = readerin;
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
                    reader.useNode(configNode);
                    std::ofstream configFile;
                    config = moos_configure(reader, configFile);
                    //start MOOS
                    commsThread = boost::thread(static_cast<bool (GyrosMoosCommsInterface::*)(const char*, const char*)> (&GyrosMoosCommsInterface::Run), this, config.processName.c_str(), "config.moos");
                   reader.useNode(origin);
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
            std::stringstream configxml;
            if (configFile.is_open())
            {
                std::string line;
                while (configFile.good())
                {
                    std::getline(configFile, line);
                    configxml << line << std::endl;
                }
                configFile.close();
            }

            labust::xml::Reader reader(configxml.str());

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

        GyrosMoosCommsInterface::~GyrosMoosCommsInterface()
        {
            std::cout << "Stopping Moos Comms Interface" << std::endl;
            this->RequestQuit();
            commsThread.join();
            std::cout << "Moos Comms Interface stopped" << std::endl;
        }

        commerrors::CommError GyrosMoosCommsInterface::Send(const labust::xml::GyrosWriterPtr data, const std::string &messageID, bool wait)
        {
            boost::mutex::scoped_lock lock(locker);
            gyrosMessageToSend message(data,messageID);
            messagesToSend.push_back(message);
            if (wait)
            { //wait for comms thread to confirm data is sent
                dataSentSignal.wait(lock);
            }
            return commerrors::noError;
        }

        commerrors::CommError GyrosMoosCommsInterface::Send(const std::vector<labust::xml::GyrosWriterPtr> &data, const std::string &messageID, bool wait)
        {
            boost::mutex::scoped_lock lock(locker);
            for (std::vector<labust::xml::GyrosWriterPtr>::const_iterator gyros = data.begin(); gyros != data.end(); gyros++)
            {	
					gyrosMessageToSend message(*gyros,messageID);
               messagesToSend.push_back(message);
            }
            if (wait)
            { //wait for comms thread to confirm data is sent
                dataSentSignal.wait(lock);
            }
            return commerrors::noError;
        }

        commerrors::CommError GyrosMoosCommsInterface::Receive(std::vector<receivedGyrosMessage>& data, bool wait)
        {
            commerrors::CommError retVal = commerrors::noError;
            if (callbackObject == NULL)
            {
                boost::mutex::scoped_lock lock(locker);
                while (wait && messagesReceived.empty())
                { //if no data received, wait for comms thread to signal some data is received
                    dataReceivedSignal.wait(lock);
                }

                if (!messagesReceived.empty())
                {
                    data.insert(data.end(), messagesReceived.begin(), messagesReceived.end());
                    messagesReceived.clear();
                }
                else if (!wait)
                {
                    retVal = commerrors::noData;
                }
            }
            else
            {
                std::cout << "Callback object is registered, please unregister it before using Receive function" << std::endl;
                retVal = commerrors::callbackRegistered;
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
            std::vector<receivedGyrosMessage> incomingMessages;
            boost::mutex::scoped_lock lock(locker);
            for (MOOSMSG_LIST::iterator mailIterator = NewMail.begin(); mailIterator != NewMail.end(); mailIterator++)
            {
                CMOOSMsg &message = *mailIterator;
                try
                {
                    	GyrosReaderPtr reader(new GyrosReader(message.GetString()));
							receivedGyrosMessage gyrosMessage(reader,message.GetKey());
                    	incomingMessages.push_back(gyrosMessage);
                }
                catch (GyrosError error)
                {
							GyrosReaderPtr reader(new GyrosReader(error.what()));
                    	receivedGyrosMessage gyrosMessage(reader,message.GetKey());
                    	incomingMessages.push_back(gyrosMessage);
                }
                catch (XMLException e)
                {
                    std::cerr << e.what() << std::endl;
                }

            }
				lock.unlock();
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
            for (std::vector<gyrosMessageToSend>::iterator message = messagesToSend.begin(); message != messagesToSend.end(); message++)
            {
                outputMessageName = message->id.empty() ? message->gyros->GetLabel() : message->id;
                if (!message->id.empty())
                {
                    m_Comms.Notify(message->id, message->gyros->GyrosXML(), MOOSTime());
                }
                else if(!message->gyros->GetLabel().empty())
                {
                    m_Comms.Notify(message->gyros->GetLabel(), message->gyros->GyrosXML(), MOOSTime());
                }
					 else
					 {
						  m_Comms.Notify(config.processName + "_OUT", message->gyros->GyrosXML(), MOOSTime());
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
    }
}


