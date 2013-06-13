/* 
 * File:   Joystick.h
 * Author: user
 *
 * Created on July 13, 2010, 12:04 PM
 * Tested and approoved on July 14, 2010, 12:00 PM
 */



#ifndef _JOYSTICKREADER_H
#define _JOYSTICKREADER_H

#ifdef __linux__
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include<linux/joystick.h>
#include <fstream>
#endif

#ifdef WIN32
#include <Windows.h>
#include <basetsd.h>
#include <dinput.h>
#endif

#include <labust/xml/xmlfwd.hpp>
#include <iostream>
#include <boost/noncopyable.hpp>
#include <vector>

namespace LABUST
{
    ///
    ///   Returned from the ReadJoystickData function
    ///   axes vector of vaalues for all axes
    ///   buttons vector of bools for all axes

    struct JoystickData
    {
        std::vector<short> axes;
        std::vector<bool> buttons;
    };

    ///
    ///  Represents a joystick device. Has methods to update state of stick in realtime
    ///  and to extract data in the JoystickData structure
    ///
    ///  Data can be sampled from joystick using ReadJoystickData function at any frequency.

    class JoystickReader : private boost::noncopyable
    {
    public:
        /*
         * Constructor, path based
         * \param path path to file with configuration labust::xml
         * \param configToUse name of configuration to use, optional - if empty, first one will be used
         */
        JoystickReader(const std::string& path, std::string configToUse = "");

        /*
         * Constructor, labust::xmlReader based
         * \param labust::xml reader with the preloaded config file
         * \param configToUse name of configuration to use, optional - if empty, first one will be used
         */
        JoystickReader(const labust::xml::Reader& reader, std::string configToUse = "");
        virtual ~JoystickReader();

        /*
         * Gets data about axes and buttons of the joystick
         *
         * \param data reference to data object to write data to
         */
        JoystickData ReadJoystickData();

#ifdef WIN32
        BOOL CALLBACK enumCallback(const DIDEVICEINSTANCE* instance, VOID* context);
#endif
    private:
        int deviceDescriptor;
        int axes, buttons;
        JoystickData joystickData;
		std::vector<float> axisGains;

#ifdef __linux__
        //std::fstream joystickPort;
        boost::mutex joystickMutex;
        boost::thread joystickThread;
        bool running;
        
        void ThreadFunction();
#endif

#ifdef WIN32
        LPDIRECTINPUT8 di;
        LPDIRECTINPUTDEVICE8 joystick;
#endif

        int deviceCounter;
    };
#ifdef WIN32
    BOOL CALLBACK enumCallback(const DIDEVICEINSTANCE* instance, VOID* context);
#endif
}

#endif	/* _JOYSTICKREADER_H */

