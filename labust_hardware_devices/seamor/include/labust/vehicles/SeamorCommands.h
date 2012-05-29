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
#ifndef SEAMORCOMMANDS_H_
#define SEAMORCOMMANDS_H_

#include <labust/vehicles/vehiclesfwd.hpp>

#include <boost/thread/mutex.hpp>

#include <map>

namespace labust
{
  namespace vehicles
  {
    int getSeamorChecksum(const unsigned char* data, int count);
  };

  namespace communication
  {
     namespace seamorrawdata
     {
         enum SeamorRawData
         {
             cameraStatus = 16, cameraStatusLength = 18,
             rovStatus = 32, rovStatusLength = 18,
             cameraControl = 50, cameraControlLength = 20,
             rovControl = 65, rovControlLength = 14
         };
     }


    /**
     * This class represents the seamor vehicle status class.
     */
    class SeamorStatus
    {
    public:
      /**
       * Generic constructor
       */
      SeamorStatus();

      /**
       * Generic destructor
       */
      virtual ~SeamorStatus();

      /**
       * Read binary data obtained from comms interface
       */
      int parseBinary(unsigned char* data, int len);

      /**
       * Message info enumerator
       */
      enum msg_info {id = 32, length = 18};
      /**
       * Message field enumeration
       */
      enum msg_field {
        m_portVI,m_stbdVI,m_portHI,m_stbdHI,
		  m_ballastI,        
		  m_depthLow, m_depthHigh,
        m_manipI, m_pitch, m_roll,
        m_status, m_headingLow, m_headingHigh,
        m_temperature
      };
      /**
       * Vehicle depth in meters
       */
      double depth;
      /**
       * Vehicle heading in degrees.
       */
      double heading;
      /**
       * Vehicle roll angle in degrees
       */
      double roll;
      /**
       * Vehicle pitch angle in degrees
       */
      double pitch;

      /**
       * Port vertical thruster current
       */
      double portVI;
      /**
       * Port vertical thruster current
       */
      double stbdVI;
      /**
       * Port vertical thruster current
       */
      double portHI;
      /**
       * Port vertical thruster current
       */
      double stbdHI;
      /**
       * Ballast motor current (AUX)
       */
      double ballastI;
      /**
       * Manipulator motor current
       */
      double manipI;

      /**
       * Status byte
       */
      int status;
      /*
       * Temperature of electronic can
       */
      double temperature;
      /**
       * Mutex for threading
       */
      boost::mutex mutex;

      std::string timeStamp;
    };

    /**
     * This class represents the seamor vehicle command class.
     */
    class SeamorCommand
    {
    public:
      /**
       * Generic constructor
       */
      SeamorCommand();

      /**
       * Generic destructor
       */
      virtual ~SeamorCommand();

      /**
       * Fill data vector from class variables.
       */
      int parseTau(labust::vehicles::tauMapRef tau);

      int parseDataMap(labust::vehicles::strMapRef dataMap);

      /**
       * Populate the class from a data vector
       */
      int parseBinary(unsigned char* data, int len);


      int getBinary(unsigned char* o_data);

      /**
       * Message field enumeration
       */
      enum msg_field {
        m_Z=2,m_Y,m_X,m_N,m_Ballast,
        m_empty, m_ctrlByte1,m_manipByte,
        m_ctrlByte2,m_lightByte, m_crc1,m_crc2};
      /**
       * Forces and moments vector
       */
      labust::vehicles::tauMap tauC;
      /**
       * Ballast force command
       */
      short ballastF;
      /**
       * Control for autoheading and autodepth
       * 32 = all off, autodepth +16, autoheading +8
       */
      short autopilotCommand;
      /**
       * Control for video selection (0,1,2,3)
       */
      short videoSelector;
      /**
       * Manipulator byte
       */
      short manipByte;
      /**
       * Light byte
       */
      short lightByte;
      /**
       * Mutex for threads
       */
      boost::mutex mutex;
    };

    /**
     * This class represents seamor vehicle camera status,
     */
    class SeamorCameraStatus
    {
    public:
        SeamorCameraStatus();
        virtual ~SeamorCameraStatus();

        int parseBinary(unsigned char* data, int len);

        float tilt; //degrees (-90 - +90)
        int zoom; //percentage - temporary solution
        float temperature; //cegrees celsius
        int lightSetting; //percentage
        std::string gain, redGain, blueGain; //dB
        std::string iris;
        std::string focusDistance; //meters!
        std::string shutterSpeed;
        boost::mutex mutex;

        std::string timeStamp;
    private:

        std::string focusDistances[192];
        std::string irisSettings[18];
        std::string gainSettings[16];
        std::string shutterSettings[20];
    };


    namespace cameracommands
    {
        enum wb {autoWB=0,indoor=1,outdoor=2,atw=4,manualWB=5};
        enum ae {full=0, manual=3,shutterPriority=10, irisPriority=11, gainPriority=12, bright=13, autoShutter=26, autoIris=27, autoGain=28};
        enum laser{laserOFF = 2, laserON = 3};
    }

    class SeamorCameraCommand
    {
    public:
        SeamorCameraCommand();
        virtual ~SeamorCameraCommand();

        int parseBinary(unsigned char* data, int length);

        int getBinary(unsigned char* o_data);

        int parseDataMap(labust::vehicles::strMapRef dataMap);

        bool autoFocus, hiSensitivityFocus;
        int focus;
        int pan, tilt, light, gain;
        int zoom;
        int redGain, blueGain;
        std::string iris, shutterSpeed;
        cameracommands::wb whiteBalance;
        cameracommands::ae autoExposure;
        cameracommands::laser laserState;

        boost::mutex mutex;

    private:
        int irisInt, redGainInt, blueGainInt, shutterSpeedInt;
        std::string irisSettings[18];
        std::string shutterSettings[20];
        std::string gainSettings[16];
    };
  };
};

/* SEAMORCOMMANDS_H_ */
#endif
