/* 
 * File:   Joystick.cpp
 * Author: tomislav Lugaric
 * 
 * Created on July 13, 2010, 12:04 PM
 * Tested and approved on August 17, 2010, 09:00 AM
 */




#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <fcntl.h>
#include <linux/joystick.h>
#include <cstring>
#include <exception>
#include <iosfwd>
#include <labust/xml/XMLReader.hpp>
#include <boost/foreach.hpp>

#include "JoystickReader.h"

using namespace std;

namespace LABUST
{
    ///
    ///  Creates a new instance of the joystick class
    ///  Opens the specified joystick device and loads
    ///  number of axes and buttons from driver

    JoystickReader::JoystickReader(const std::string& path, std::string configToUse)
    {
        labust::xml::Reader reader(path,true);
        std::string configQuery;
        if (configToUse.empty())
        {
            configQuery = "//peripheralConfig[@type='joystick']";
        }
        else
        {
            configQuery = "//peripheralConfig[@type='joystick' and @name='" + configToUse + "']";
        }

        int deviceNumber;
        bool loadButtonsCount = false, loadAxesCount = false;

        reader.useRootNode();
        _xmlNode* configNode = NULL;
        if (reader.try_value(configQuery, &configNode))
        {
            reader.useNode(configNode);

            if (!reader.try_value("param[@name='DeviceNumber']/@value", &deviceNumber))
            {
                throw std::runtime_error("Missing joystick number");
            }
            if(!reader.try_value("param[@name='Axes']/@value", &axes))
            {
                loadAxesCount = true;
            }
            if(!reader.try_value("param[@name='Buttons']/@value", &buttons))
            {
                loadButtonsCount = true;
            }
        }
        else
        {
            throw std::runtime_error("Unable to open joystick, missing config");
        }

        //generate joystick name
        stringstream joystickPath;
        joystickPath << "/dev/input/js" << deviceNumber;


        //load number of buttons/axes from driver
        if ((deviceDescriptor = open(joystickPath.str().c_str(), O_RDONLY)) >= 0)
        {
            if(loadButtonsCount)
            {
                ioctl(deviceDescriptor, JSIOCGBUTTONS, &buttons);
            }

            if(loadAxesCount)
            {
                ioctl(deviceDescriptor, JSIOCGAXES, &axes);
            }

			axisGains.resize(axes);
			 for(int i=0; i<axes; i++)
			 {
				 axisGains[i] = 1;
			 }
			 using namespace labust::xml;

			 NodeCollectionPtr gains = reader.value<NodeCollectionPtr>("param[@name='AxisGain']");

			 _xmlNode* curr=reader.currentNode();
			 BOOST_FOREACH(_xmlNode* axisGain, *gains)
			 {
				int axisNumber;
				float gain;
				reader.useNode(axisGain);
				std::string axisGainPair = reader.value<std::string>("@value");
				std::replace(axisGainPair.begin(),axisGainPair.end(),':',' ');
				std::stringstream buffer;
				buffer << axisGainPair;
				buffer >> axisNumber >> gain;
				axisGains[axisNumber] = gain;				
			 }

			 reader.useNode(curr);
			 	 	 	 axisGains.resize(axes);
            joystickData.axes.resize(axes);
            joystickData.buttons.resize(buttons);


        }
        else
        {
            std::stringstream buffer;
            buffer<<"Error opening joystick "<<joystickPath;
            throw std::runtime_error(buffer.str());
        }
        running = true;
        joystickThread = boost::thread(boost::bind(&JoystickReader::ThreadFunction,this));

    }

    ///
    ///  Creates a new instance of the joystick class
    ///  Opens the first joystick device and uses
    ///  number of axes and buttons specified by user

    JoystickReader::JoystickReader(const labust::xml::Reader& reader, std::string configToUse)
    {
        std::string configQuery;
        if (configToUse.empty())
        {
            configQuery = "peripheralConfig[@type='joystick']";
        }
        else
        {
            configQuery = "peripheralConfig[@type='joystick' and @name='" + configToUse + "']";
        }

        int deviceNumber;
        bool loadButtonsCount = false, loadAxesCount = false;

        _xmlNode* configNode = NULL;
        if (reader.try_value(configQuery, &configNode))
        {
            const_cast<labust::xml::Reader&>(reader).useNode(configNode);

            if (!reader.try_value("param[@name='DeviceNumber']/@value", &deviceNumber))
            {
                throw std::runtime_error("Missing joystick number");
            }
            if(!reader.try_value("param[@name='Axes']/@value", &axes))
            {
                loadAxesCount = true;
            }
            if(!reader.try_value("param[@name='Buttons']/@value", &buttons))
            {
                loadButtonsCount = true;
            }
        }
        else
        {
            throw std::runtime_error("Unable to open joystick, missing config");
        }


        //generate joystick name
        stringstream joystickPath;
        joystickPath << "/dev/input/js" << deviceNumber;


        //load number of buttons/axes from driver
        if ((deviceDescriptor = open(joystickPath.str().c_str(), O_RDONLY)) >= 0)
        {
            if(loadButtonsCount)
            {
                ioctl(deviceDescriptor, JSIOCGBUTTONS, &buttons);
            }

            if(loadAxesCount)
            {
                ioctl(deviceDescriptor, JSIOCGAXES, &axes);
            }
            joystickData.axes.resize(axes);
            joystickData.buttons.resize(buttons);
        }
        else
        {
            std::stringstream buffer;
            buffer<<"Error opening joystick "<<joystickPath;
            throw std::runtime_error(buffer.str());
        }
        running = true;
        joystickThread = boost::thread(boost::bind(&JoystickReader::ThreadFunction,this));
    }

    JoystickReader::~JoystickReader()
    {
        //std::cout<<"Joystick destructor.";
        running = false;
        //joystickPort.close();
        //OS specific break
        pthread_cancel(joystickThread.native_handle());
        pthread_join(joystickThread.native_handle(),NULL);
        close(deviceDescriptor);
    }

    ///
    ///   Returnd data about the current state of joystick

    JoystickData JoystickReader::ReadJoystickData()
    {
        boost::mutex::scoped_lock lockJoystickData(joystickMutex);
        return joystickData;
    }

    void JoystickReader::ThreadFunction()
    {
        while(running)
        {
            js_event event;
            if (read(deviceDescriptor, &event, sizeof (event)) > 0)
            {
                boost::mutex::scoped_lock lockJoystickData(joystickMutex);
                switch (event.type)
                {
                    case JS_EVENT_AXIS:
                    	  std::cout<<"Event number:"<<int(event.number)<<","<<joystickData.axes.size()<<","<<axisGains.size()<<std::endl;
                        joystickData.axes[event.number] = event.value;// * axisGains[event.number];
                        break;

                    case JS_EVENT_BUTTON:
                        //joystickData.buttons[event.number] = event.value;
                        break;

                    case JS_EVENT_INIT:
                        break;

                    default:
                        break;
                }
            }
        }
    }
}
