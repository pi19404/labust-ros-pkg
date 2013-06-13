#ifndef MOOSCONFIG_HPP_
#define MOOSCONFIG_HPP_
/*
 * MoosConfig.hpp
 *
 *  Created on: June 02, 2011
 *      Author: Tomislav Lugaric
 */

#include <boost/thread.hpp>

namespace LABUST
{
    //Forward declaration
    namespace XML
    {
        class Reader;
    };

    namespace COMMUNICATION
    {

        /**
         * This structure represents a full configuration for MOOS comms.
         */
        struct MoosConfig
        {
            int port, appTick, commsTick;
            std::string hostname;
            std::string processName;
            std::vector<std::string> subscriptionVars;
        };

        const labust::xml::Reader & operator>>(const labust::xml::Reader& reader, MoosConfig& config);

        /**
         * Create a moos mission fole from the xml file (can be used to start moos process)
         * \param xml reader with preparsed config file
         * \returns name of file which can be used to start moos comms interface
         */
        MoosConfig moos_configure(const labust::xml::Reader& reader, std::ofstream& configFile);
    }
}
/* MOOSCONFIG_HPP_ */
#endif
