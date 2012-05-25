/*
 * MoosConfig.cpp
 *
 *  Created on: June 02, 2011
 *      Author: Tomislav Lugaric
 */
#include <labust/xml/XMLReader.hpp>
#include <labust/moos/MoosConfig.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include <fstream>
#include <math.h>

//MOOS configuration

const labust::xml::Reader& labust::comms::operator>>(const labust::xml::Reader& readerin, MoosConfig& config)
{
	labust::xml::Reader reader = readerin;
    reader.value("param[@name='Hostname']/@value", &config.hostname);

    std::string temp;

    if (!reader.try_value("param[@name='Port']/@value", &config.port))
    {
        config.port = 9000;
    }

    if (!reader.try_value("param[@name='ProcessName']/@value", &config.processName))
    {
        config.processName = "MoosCommsInterface";
    }

    if (!reader.try_value("param[@name='AppTick']/@value", &config.appTick))
    {
        config.appTick = 10;
    }

    if (!reader.try_value("param[@name='CommsTick']/@value", &config.commsTick))
    {
        config.commsTick = 10;
    }

    labust::xml::NodeCollectionPtr nodes;
    try
    {
        nodes = reader.value<labust::xml::NodeCollectionPtr > ("param[@name='Subscription']");
        config.subscriptionVars.clear();

        BOOST_FOREACH(_xmlNode* pt, *nodes)
        {
	    reader.useNode(pt);
            config.subscriptionVars.push_back(reader.value<std::string > ("@value"));
        }
    }
    catch (...)
    {

    }


    return readerin;
}

labust::comms::MoosConfig labust::comms::moos_configure(const labust::xml::Reader& reader, std::ofstream &moosConfig)
{
    MoosConfig config;
    //Read configuration
    reader >> config;
    //Start MOOS thread
    moosConfig.open("config.moos", std::ios_base::out | std::ios_base::trunc);
    moosConfig << "ServerPort = " << config.port << std::endl;
    moosConfig << "ServerHost = " << config.hostname << std::endl;
    moosConfig << "ProcessConfig = " << config.processName << std::endl;
    moosConfig << "{" << std::endl;
    moosConfig << "AppTick = " << config.appTick << std::endl;
    moosConfig << "commsTick = " << config.commsTick << std::endl;
    moosConfig << "}" << std::endl;
    moosConfig.close();

    return config;
}
