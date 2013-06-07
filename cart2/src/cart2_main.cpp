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
 *
 *  Author: Antonio Vasilijevic
 *  Created: 01.02.2013.
 *********************************************************************/
#include <iostream>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <labust/tools/TimingTools.hpp>
#include <std_msgs/Bool.h>
#include <labust/math/NumberManipulation.hpp>
#include <cart2/ImuInfo.h>
#include <labust/control/PIDController.hpp>

#include <ros/ros.h>
#include <auv_msgs/BodyForceReq.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

// Ulazi u funkciju su struktura Tauc (Tau.Frw i Tau.Yaw), Limit thrustera iz strukture TauT, 
//i stanja digitalnih izlaza: int polje StateDO[4]
//
// Izlazi su Supply_Voltage, Motor_Current[2], Vehicle_Current i Temperature i moze biti ScaleThrust iz strukture TauT

struct TauC
{
	float Frw;
	float Yaw;
};

struct TauT
{
	TauT(TauC Tau, float Limit = 1)
	{
		float ScaleThrust;
		if (fabs(Tau.Frw) + fabs(Tau.Yaw) > 1)
		{ScaleThrust = 1 / (fabs(Tau.Frw) + fabs(Tau.Yaw));}
		else
		{ScaleThrust = 1;}
		Port = Limit * ScaleThrust * (Tau.Frw - Tau.Yaw);
		Stb = Limit * ScaleThrust * (Tau.Frw + Tau.Yaw);
	}

	TauC getTauC()
	{
		TauC tau;
		tau.Frw = (Port+Stb)/2;
		tau.Yaw = (-Port+Stb)/2;

		return tau;
	}

	float Port;
	float Stb;
};

float TauFrwold = 0;
float TauYawold = 0;
double integralRPM[2] = {0,0};

int intMax = 32767;
double Supply_Voltage, Vehicle_Current, Temperature, Motor_Current[2], Supply_Voltage_Previous, Vehicle_Current_Previous, Motor_Current_Previous[2], Load_Position[2],
Load_Position_Previous[2], RPM[2]={0};
double currentAvg[2] = {0,0};

//time_t cur_time;
//clock_t start_time;
int DigOutNumber=1; //From 1


using namespace boost::asio;
streambuf inputBuffer, outputBuffer;
std::ostream os(&outputBuffer);
//char a[] = {0x00};
char DriveReply[1], DriveReplyLong[12];

std::string Mode, Mode1;
bool CommsOkFlag = true;
io_service io;
serial_port port(io);//, GPS2port(io,"/dev/ttyS1"), modem(io,"/dev/ttyS0");

char AxisONID1[] = {0x04, 0x00, 0x10,0x01, 0x02, 0x17}; //turn Axis1 ON
char AxisONID2[] = {0x04, 0x00, 0x20,0x01, 0x02, 0x27}; //turn Axis2 ON
char AxisOFFID1[] = {0x04, 0x00, 0x10,0x00, 0x02, 0x16}; //turn Axis1 OFF
char AxisOFFID2[] = {0x04, 0x00, 0x20,0x00, 0x02, 0x26}; //turn Axis2 OFF
char ResetID1[] = {0x04, 0x00, 0x10, 0x04, 0x02, 0x1A}; //Reset Axis 1
char ResetID2[] = {0x04, 0x00, 0x20, 0x04, 0x02, 0x2A}; //Reset Axis 2
char GetVoltage[] = {0x08, 0x00, 0x10, 0xB0, 0x04, 0x00, 0x11, 0x02, 0x40, 0x1F}; //get voltage from Axis1
char GetTemperature[] = {0x08, 0x00, 0x10, 0xB0, 0x04, 0x00, 0x11, 0x02, 0x41, 0x20}; //get temperature from Axis2
char GetVehicleCurrent[] = {0x08, 0x00, 0x10, 0xB0, 0x04, 0x00, 0x11, 0x02, 0x3E, 0x1D}; //get value from AI feedback, address 02 3E
char GetCurrent1[] = {0x08, 0x00, 0x10, 0xB0, 0x04, 0x00, 0x11, 0x02, 0x3C, 0x1B}; //get current from Axis1
char GetCurrent2[] = {0x08, 0x00, 0x20, 0xB0, 0x04, 0x00, 0x11, 0x02, 0x3C, 0x2B}; //get current from Axis2
char ReadISR1[] = {0x08, 0x00, 0x10, 0xB0, 0x04, 0x00, 0x11, 0x03, 0x06, 0xE6};
char ReadISR2[] = {0x08, 0x00, 0x20, 0xB0, 0x04, 0x00, 0x11, 0x03, 0x06, 0xF6};
char GetPosition1[] = {0x08, 0x00, 0x10, 0xB0, 0x04, 0x00, 0x11, 0x02, 0x28, 0x07}; //get current from Axis1
char GetPosition2[] = {0x08, 0x00, 0x20, 0xB0, 0x04, 0x00, 0x11, 0x02, 0x28, 0x17}; //get current from Axis2

int StateDO[] = {1,0,0,0};

float parsData()
{
	char ChSum[1];	
	unsigned char ByteHigh, ByteLow;
	int value;
	ChSum[0] = 0;
	for (int i1=0;i1<11;i1++)
	{ChSum[0] += DriveReply[i1];}
	ByteHigh = DriveReply[9];
	ByteLow = DriveReply[10];
	value = ByteHigh * 256 + ByteLow;
	//Motor_Current[ID-1] = (value-32736) * 2.5 * 4 / 65472;
	if (!(ChSum[0]==DriveReply[11]))// && (DriveReplyLong[1]==GetCurrent[5]) && (DriveReplyLong[2]==GetCurrent[6]) && (DriveReplyLong[5]==GetCurrent[1]) && (DriveReplyLong[6]==GetCurrent[2])) 
	{CommsOkFlag = false;}
	return value;
}

void SetDigiOut() // only for general purpose dig.out 0 & 1.
{
	char DigOutSet[] = {0x08, 0x00, 0x10, 0xEC, 0x00, 0x00, 0x01, 0x00, 0x00, 0x05}; //Axis1, DO0, set to 0
	char ChSum[1];
	int ID, DO;

	switch (DigOutNumber)
	{
	case(1):
						ID = 1;
	DO = 0;
	break;
	case(2):
						ID = 1;
	DO = 1;
	break;
	case(3):
						ID = 2;
	DO = 0;
	break;
	case(4):
						ID = 2;
	DO = 1;
	break;
	}

	DigOutSet[2] = DigOutSet[2] + (ID-1)*16; //set axis
	DigOutSet[6] = DigOutSet[6] + DO;
	if (StateDO[DigOutNumber-1]==1)
	{
		DigOutSet[8] = 0x01;
		for (int i1=0;i1<DO;i1++)
		{DigOutSet[8] = DigOutSet[8] * 2;}
	}
	else
	{DigOutSet[8] = 0x00;}
	DigOutSet[9] = 0;
	for (int i1=0;i1<9;i1++)
	{DigOutSet[9] += DigOutSet[i1];} //create checksum
	os.write(DigOutSet,sizeof(DigOutSet));
	write(port,outputBuffer);
	DigOutNumber++;
}

void SetReference(int ID, float REF) //REF from 0 to 1. 1 represents full thrust
{
	char SetRefID1[] = {0x06, 0x00, 0x10, 0x20, 0xA9, 0x00, 0x00, 0xDF}; //Set Reference 0 for Axis1
	char SetRef[8];

	SetRef[0] = SetRefID1[0];
	SetRef[1] = SetRefID1[1];
	SetRef[2] = SetRefID1[2] + (ID-1)*16;
	SetRef[3] = SetRefID1[3];
	SetRef[4] = SetRefID1[4];

	double max = 11138;
	// Motors are 3,4A max. Value of ext. reference (3272) corresponds to 1A. Full thrust (3.4A) is then ext. reference of 11138
	int16_t ref_current = int16_t(REF*max);
	const char* p=reinterpret_cast<const char*>(&ref_current);
	SetRef[5] = p[1];
	SetRef[6] = p[0];;
	SetRef[7] = SetRef[0] + SetRef[1] + SetRef[2] + SetRef[3] + SetRef[4] + SetRef[5] + SetRef[6];

	Mode = "OK";
	Mode1 = "";
	os.write(SetRef,sizeof(SetRef));
	write(port,outputBuffer);
}

void ReadHandler(const boost::system::error_code& e, std::size_t size)
//void ReadHandler(std::size_t size)
{
	//std::cout<<"Mode:"<<Mode<<std::endl;
	char ChSum[1];
	int MessageLength=1;
	float ByteHigh, ByteLow, value;
	if (e)
	{
		std::cout<<"Async Read Error"<<std::endl;
		//
	}	
	CommsOkFlag= false;
	for (int i1=0;i1<1;i1++)
	{
		if (Mode=="Sync")
		{
			if (strncmp(DriveReply,"\r",1)==0)//is sync
			{CommsOkFlag= true;}
			MessageLength=1;
			break;
		}

		if (Mode=="Data_V")// && (strncmp(DriveReply,"\r",1)==0)) //is sync
		{
			MessageLength = 1;
			CommsOkFlag= true;

			Supply_Voltage = parsData() * 53 / 65472;		

			Mode = "OK";
			Mode1 = "Data_T";
			os.write(GetTemperature,sizeof(GetTemperature));
			write(port,outputBuffer);
			break;
		}

		if (Mode=="Data_T")// && (strncmp(DriveReply,"\r",1)==0)) //is sync
		{
			MessageLength = 1;
			CommsOkFlag= true;

			Temperature = 2 + parsData() * 500 / 65472;		

			Mode = "OK";
			Mode1 = "Data_C";
			os.write(GetVehicleCurrent,sizeof(GetVehicleCurrent));
			write(port,outputBuffer);
			break;
		}

		if (Mode=="Data_C")// && (strncmp(DriveReply,"\r",1)==0)) //is sync
		{
			MessageLength = 1;
			CommsOkFlag= true;

			Vehicle_Current = parsData() * 18 / 65472 * 1.25;		

			Mode = "OK";
			Mode1 = "Data_C1";
			//os.write(GetCurrent1,sizeof(GetCurrent1));
			os.write(GetPosition1,sizeof(GetPosition1));
			write(port,outputBuffer);
			break;
		}

		if (Mode=="Data_C1")// && (strncmp(DriveReply,"\r",1)==0)) //is sync
		{
			MessageLength = 1;
			CommsOkFlag= true;

			//Motor_Current[0] = (parsData()-32736) * 2.5 * 4 / 65472;
			Load_Position[0] = parsData();

			Mode = "OK";
			Mode1 = "Data_C2";
			//os.write(GetCurrent2,sizeof(GetCurrent2));
			os.write(GetPosition2,sizeof(GetPosition2));
			write(port,outputBuffer);
			break;
		}

		if (Mode=="Data_C2")// && (strncmp(DriveReply,"\r",1)==0)) //is sync
		{
			MessageLength = 1;
			CommsOkFlag= true;
			// IF current
			//Motor_Current[1] = (parsData()-32736) * 2.5 * 4 / 65472;
			Load_Position[1] = parsData();

			Mode = "OK";
			Mode1 = "DigOut";
			SetDigiOut();
			break;
		}

		if (Mode=="OK")
		{
			MessageLength = 1;
			if (strncmp(DriveReply,"O",1)==0) //is sync
			{
				CommsOkFlag= true;
				if ((Mode1 == "Data_V") || (Mode1 == "Data_T") || (Mode1 == "Data_C") || (Mode1 == "Data_C1") || (Mode1 == "Data_C2"))
				{
					Mode = Mode1;
					MessageLength = 12;
				}
				if (Mode1 == "DigOut")
				{			
					if (DigOutNumber<5)
					{
						MessageLength = 1;
						SetDigiOut();
					}
					else
					{DigOutNumber = 1;}
				}
			}
			break;
		}

	}

	DriveReply[0] = 0x00;
	async_read(port, buffer(&DriveReply,MessageLength), //boost::asio::transfer_at_least(1),
			boost::bind(&ReadHandler, placeholders::error, placeholders::bytes_transferred));
	//ReadFlag = true;
}

bool Initialization(int ID)
{
	bool b1=true;
	int counter, delay = 5;
	char DisableEncoderWireID1[] = {0x08, 0x00, 0x10, 0x59, 0x03, 0xFF, 0xBF, 0x00, 0x00, 0x32};
	char DisableEncoderWireID2[] = {0x08, 0x00, 0x20, 0x59, 0x03, 0xFF, 0xBF, 0x00, 0x00, 0x42};
	char ModeTorquesTESID1[] = {0x08, 0x00, 0x10, 0x59, 0x09, 0xB1, 0xC0, 0x81, 0x00, 0x6C};
	char ModeTorquesTESID2[] = {0x08, 0x00, 0x20, 0x59, 0x09, 0xB1, 0xC0, 0x81, 0x00, 0x7C}; 
	char ExtReferenceModeID1[] = {0x08, 0x00, 0x10, 0x59, 0x09, 0xFF, 0x3F, 0x00, 0x00, 0xB8};
	char ExtReferenceModeID2[] = {0x08, 0x00, 0x20, 0x59, 0x09, 0xFF, 0x3F, 0x00, 0x00, 0xC8}; 
	char UpdID1[] = {0x04, 0x00, 0x10, 0x01, 0x08, 0x1D}; //Update Axis1
	char UpdID2[] = {0x04, 0x00, 0x20, 0x01, 0x08, 0x2D}; //Update Axis2
	char DisableEncoderWire[10];
	char ModeTorquesTES[10];
	char ExtReferenceMode[10];
	char Upd[6];

	switch (ID)
	{
	case 1:
		for (int i1=0;i1<10;i1++)
		{
			DisableEncoderWire[i1] = DisableEncoderWireID1[i1];
			ModeTorquesTES[i1] = ModeTorquesTESID1[i1];
			ExtReferenceMode[i1] = ExtReferenceModeID1[i1];
			if (i1<6)
			{Upd[i1] = UpdID1[i1];}
		}
		break;
	case 2:
		for (int i1=0;i1<10;i1++)
		{
			DisableEncoderWire[i1] = DisableEncoderWireID2[i1];
			ModeTorquesTES[i1] = ModeTorquesTESID2[i1];
			ExtReferenceMode[i1] = ExtReferenceModeID2[i1];
			if (i1<6)
			{Upd[i1] = UpdID2[i1];}
		}
		break;

	}

	// Disable encoder wire break, (encoder does not exist)
	Mode = "OK";
	Mode1 = "";
	counter=0;
	do
	{
		os.write(DisableEncoderWire,sizeof(DisableEncoderWire));
		write(port,outputBuffer);
		//read(port,boost::asio::buffer(&a,1));
		counter++;
		usleep(1000*delay);
	}
	while (!CommsOkFlag && (counter<2));
	if (counter>1)
	{b1 = false;}

	// Set torque Mode of operation
	counter=0;
	do
	{
		os.write(ModeTorquesTES,sizeof(ModeTorquesTES));
		write(port,outputBuffer);
		counter++;
		usleep(1000*delay);
	}
	while (!CommsOkFlag && (counter<2));
	if (counter>1)
	{b1 = false;}

	// Set external reference serial - RS 232
	counter=0;
	do
	{
		os.write(ExtReferenceMode,sizeof(ExtReferenceMode));
		write(port,outputBuffer);
		counter++;
		usleep(1000*delay);
	}
	while (!CommsOkFlag && (counter<2));
	if (counter>1)
	{b1 = false;}

	// Update Axis
	counter=0;
	do
	{
		os.write(Upd,sizeof(Upd));
		write(port,outputBuffer);
		counter++;
		usleep(1000*delay);
	}
	while (!CommsOkFlag && (counter<2));
	if (counter>1)
	{b1 = false;}
	if (!b1)
	{CommsOkFlag = false;
	}

	return b1;
}

void ReadData()
{
	Mode = "OK";
	Mode1 = "Data_V";
	//std::cout<<"Poziv"<<std::endl;
	os.write(GetVoltage,sizeof(GetVoltage));
	write(port,outputBuffer);
}

void EstablishSerialComm()
{
	char SyncByte[] = {0x0D};
	char SCIBR[] = {0x06, 0x00, 0x10, 0x08, 0x20, 0x00, 0x04, 0x42};
	int i1;

	//TechnoSoft port configuration
	if (port.is_open())
	{
		port.set_option(serial_port::baud_rate(9600)); //
		port.set_option(serial_port::flow_control(serial_port::flow_control::none));
		port.set_option(serial_port::parity(serial_port::parity::none));
		port.set_option(serial_port::stop_bits(serial_port::stop_bits::two));
		port.set_option(serial_port::character_size(8));
		std::cout<<"IPOS Multiaxis Port open 9600"<<std::endl;
	}

	for (i1=0;i1<3;i1++)
	{
		CommsOkFlag = false;
		Mode = "Sync";
		os.write(SyncByte,sizeof(SyncByte));
		write(port,outputBuffer);
		//read(port,boost::asio::buffer(&a,1));
		usleep(1000*15);

		if (CommsOkFlag)
		{
			std::cout<<"Sync na 9600, Reset Axis"<<std::endl;

			Mode = "OK";
			os.write(AxisOFFID1,sizeof(AxisOFFID1));
			write(port,outputBuffer);

			os.write(AxisOFFID2,sizeof(AxisOFFID2));
			write(port,outputBuffer);

			os.write(ResetID2,sizeof(ResetID2));
			write(port,outputBuffer);

			os.write(ResetID1,sizeof(ResetID1));
			write(port,outputBuffer);

			usleep(1000*1000);
		}
		Mode = "Sync";
		os.write(SyncByte,sizeof(SyncByte));
		write(port,outputBuffer);
		usleep(1000*15);
		if (CommsOkFlag)
		{std::cout<<"Sync na 9600"<<std::endl;
		break;}

	}

	if(i1<2)
	{	
		Mode = "OK";
		os.write(SCIBR,sizeof(SCIBR));
		write(port,outputBuffer);
		usleep(1000*15);
		//read(port,boost::asio::buffer(&a,1));

		if (CommsOkFlag)
		{
			//std::cout<<"promjenio na 115200 - OK"<<std::endl;
			if (port.is_open())
			{
				port.set_option(serial_port::baud_rate(115200)); //
				port.set_option(serial_port::flow_control(serial_port::flow_control::none));
				port.set_option(serial_port::parity(serial_port::parity::none));
				port.set_option(serial_port::stop_bits(serial_port::stop_bits::two));
				port.set_option(serial_port::character_size(8));
				//std::cout<<"IPOS Port 115200"<<std::endl;
				//return true;
			}
			Mode = "Sync";
			os.write(SyncByte,sizeof(SyncByte));
			write(port,outputBuffer);
			usleep(1000*15);
			if (CommsOkFlag)
			{std::cout<<"Sync na 115200"<<std::endl;}
		}
	}
}

void watch()
{
	///\todo Check this timeout watch. What is it supposed to be doing ?
	std::cout<<"Isteklo vrijeme"<<std::endl;
	boost::thread t(boost::bind(&boost::asio::io_service::run,&io));
	t.join();
	return;
}

/////////ROS HANDLERS ADDED HERE///////////////////
void handleTau(TauC* tauOut, const auv_msgs::BodyForceReq::ConstPtr tauIn)
{
	tauOut->Frw = tauIn->wrench.force.x;
	tauOut->Yaw = tauIn->wrench.torque.z;
}

void sendDiagnostics(ros::Publisher& diag, bool CommsOkFlag)
{
	diagnostic_msgs::DiagnosticStatus status;

	status.level = status.OK;
	status.name = "CART2 Driver";
	status.message = "Voltage and current report";
	status.hardware_id = "None";

	diagnostic_msgs::KeyValue data;
	std::ostringstream out;
	out<<Supply_Voltage;
	data.key = "Supply_Voltage";
	data.value = out.str();
	status.values.push_back(data);

	out.str("");
	out<<Vehicle_Current;
	data.key = "Vehicle_Current";
	data.value = out.str();
	status.values.push_back(data);

	out.str("");
	out<<Temperature;
	data.key = "Temperature";
	data.value = out.str();
	status.values.push_back(data);

	out.str("");
	out<<RPM[0];
	data.key = "RPM1";
	data.value = out.str();
	status.values.push_back(data);

	out.str("");
	out<<RPM[1];
	data.key = "RPM2";
	data.value = out.str();
	status.values.push_back(data);

	if (!CommsOkFlag)
	{
		status.level = status.ERROR;
		status.message = "Communication error!";
	}

	diag.publish(status);
}

double wrapRPM(double value)
{
  if (value > 30000)
  { 
     return value - 65536;
  }
  else if (value < -30000)
  {
     return value + 65536;
  }
  return value;  
}  

struct CalibrationData
{
  CalibrationData():
  	state(stop),
  	cycleWait(0),
  	trigger(false){}

  enum {stop=0,start,xy,xz,mag,gyrosOnly};
  enum {cyclesMax = 500};
  enum {cyclesMin = 20};
  enum {cyclesGyro = 120};
  enum {calibrationPin = 1};
  int state;
  int cycleWait;
  bool trigger;
};

void onCalibration(CalibrationData& data, const std_msgs::Bool::ConstPtr& calibration)
{
  if (calibration->data)
  {
     std::cout<<"Change state."<<std::endl;
     data.cycleWait = 0;
     data.trigger = true;
     switch (data.state)
     {
       case CalibrationData::stop:
          data.state = data.start;
          break;
       case CalibrationData::start:
      	 data.state = CalibrationData::xz;
      	 break;
       case CalibrationData::xz:
      	 data.state = CalibrationData::mag;
      	 break;
       case CalibrationData::mag:
         break;
     }
  }
}

void onGyroCalibration(CalibrationData& data, const std_msgs::Bool::ConstPtr& calibration)
{
  if (calibration->data)
  {
  	data.state = CalibrationData::gyrosOnly;
    data.cycleWait = 0;
    data.trigger = true;
  }
  else
  {
  	data.state = CalibrationData::stop;
    data.cycleWait = 0;
    data.trigger = false;
  }
}

void onResetPin(const std_msgs::Bool::ConstPtr& resetPin)
{
  StateDO[0] = resetPin->data;
}

void onLights(const std_msgs::Bool::ConstPtr& lights)
{
}  

///////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	std::vector<int> medianPort, medianStbd;
	float AverCurrent[10], AC;
	int CommsCount = 0, AverCount, Count = 0;
	size_t tSum = 10000;
	size_t tLoop = 100;
	//labust::tools::watchdog WD(watch, tSum, tLoop);
	char SyncByte[] = {0x0D};
	TauC TauControl, TauControlOld;
	TauControl.Frw = 0;
	TauControl.Yaw = 0;
	TauControlOld.Frw = 0;
	TauControlOld.Yaw = 0;
	std::string path, configToUse;
	CalibrationData calibration;
	//std::cout<<"Please enter path to config file"<<std::endl;
	//std::cin>>path;ž


	/*path="c:/config.xml";
	LABUST::XML::Reader reader(path, true);
	std::string configQuery;
	configQuery = "/configurations/config[@type='program']";
	_xmlNode* configNode = NULL;

	if (reader.try_value(configQuery, &configNode))
   	{
      	reader.useNode(configNode);
	}

	std::string joystickConfig("LogitechWireless");
	reader.useNode(configNode);
	joystick = new LABUST::JoystickReader(reader,joystickConfig);*/

	/////ADD ROS STUFF PRELIMINARY HERE/////
	//Init node
	ros::init(argc,argv,"cart2_node");
	//Get handles
	ros::NodeHandle nh,ph("~");
	//Setup subscribers
	ros::Subscriber tauIn = nh.subscribe<auv_msgs::BodyForceReq>("tauIn",1,boost::bind(&handleTau,&TauControl,_1));
	ros::Subscriber cflag = nh.subscribe<std_msgs::Bool>("calibration_on",1,boost::bind(&onCalibration, boost::ref(calibration),_1));
	ros::Subscriber cgflag = nh.subscribe<std_msgs::Bool>("calibration_gyro_on",1,boost::bind(&onGyroCalibration, boost::ref(calibration),_1));
	ros::Subscriber rflag = nh.subscribe<std_msgs::Bool>("reset_pin",1,boost::bind(&onResetPin, _1));
	//Setup publishers
	ros::Publisher tauAch = nh.advertise<auv_msgs::BodyForceReq>("tauAch",1);
	ros::Publisher diagnostic = nh.advertise<diagnostic_msgs::DiagnosticStatus>("diagnostics",1);
	ros::Publisher rpm_info = nh.advertise<cart2::ImuInfo>("cart2_info",1);

	//Get port name from configuration and open.
	std::string portName("/dev/ttyUSB0");
	ph.param("PortName", portName,portName);
	port.open(portName);
	bool useRPMControl(false);
	ph.param("UseRPMControl", useRPMControl, useRPMControl);
	cart2::ImuInfo cart2_info;
	cart2_info.data.resize(9);;
	float rpm_port(0), rpm_stbd(0), curr_port(0), curr_stbd(0);
	double Kp(0.0001),Ki(0.001);
	int median_size(5);
	ph.param("Kp_rpm", Kp, Kp);
	ph.param("Ki_rpm", Ki, Ki);
	ph.param("MedianSize", median_size, median_size);
	medianPort.assign(median_size,0);
	medianStbd.assign(median_size,0);
	labust::control::PIDController<labust::control::details::PID,labust::control::UseLimits> cport(Kp,Ki,0,0), cstbd(Kp,Ki,0,0);
	labust::math::Limit<double> limits(-1,1);
	cport.setLimits(limits);
	cstbd.setLimits(limits);
	ros::Rate rate(10);
	////////////////////////////////////////

	std::cout<<"first "<<std::endl;

	//WD.start();
	if (port.is_open())
	{
		boost::asio::write(port, boost::asio::buffer(ResetID1,sizeof(ResetID1)));
		boost::asio::write(port, boost::asio::buffer(ResetID2,sizeof(ResetID2)));
		port.set_option(serial_port::baud_rate(115200)); //
		port.set_option(serial_port::flow_control(serial_port::flow_control::none));
		port.set_option(serial_port::parity(serial_port::parity::none));
		port.set_option(serial_port::stop_bits(serial_port::stop_bits::two));
		port.set_option(serial_port::character_size(8));
		std::cout<<"IPOS Multiaxis Port open 115200"<<std::endl;
		//return true;
	}

	// read port for the first time
	async_read(port, buffer(&DriveReply,1),
			boost::bind(&ReadHandler, placeholders::error, placeholders::bytes_transferred));
	boost::thread t(boost::bind(&boost::asio::io_service::run,&io));
	//This we change with ros::ok
	//while (1)
	int n=0;
	while (ros::ok())
	{
		if (port.is_open())
		{
			port.set_option(serial_port::baud_rate(115200)); //
			port.set_option(serial_port::flow_control(serial_port::flow_control::none));
			port.set_option(serial_port::parity(serial_port::parity::none));
			port.set_option(serial_port::stop_bits(serial_port::stop_bits::two));
			port.set_option(serial_port::character_size(8));
			std::cout<<"IPOS Multiaxis Port open 115200"<<std::endl;
			//return true;
		}
		usleep(1000*200);
		CommsOkFlag = false;
		CommsCount = 0;
		Mode = "Sync";
		Mode1 = "";
		os.write(SyncByte,sizeof(SyncByte));
		write(port,outputBuffer);
		usleep(1000*200);

		if ((CommsOkFlag) && (Count>0) && (Count<20))
		{
			std::cout<<"Sync OK, main"<<std::endl;

			Mode = "OK";
			os.write(AxisOFFID1,sizeof(AxisOFFID1));
			write(port,outputBuffer);

			os.write(AxisOFFID2,sizeof(AxisOFFID2));
			write(port,outputBuffer);

			os.write(ResetID2,sizeof(ResetID2));
		write(port,outputBuffer);

		os.write(ResetID1,sizeof(ResetID1));
		write(port,outputBuffer);

			os.write(AxisONID1,sizeof(AxisONID1));
			write(port,outputBuffer);

			os.write(AxisONID2,sizeof(AxisONID2));
			write(port,outputBuffer);

			usleep(1000*100);

			Count++;
		}
		else
		{
			Count = 1;
			EstablishSerialComm();
			if (CommsOkFlag)
			{
				if ((Initialization(1)) && (Initialization(2)))
				{
					std::cout<<"Axis initialization is OK "<<std::endl;

					//Set reference torque zero
					SetReference(1,0);

					if (CommsOkFlag)
					{SetReference(2,0);}

					if (CommsOkFlag)
					{os.write(AxisONID1,sizeof(AxisONID1));
					write(port,outputBuffer);}

					if (CommsOkFlag)
					{os.write(AxisONID2,sizeof(AxisONID2));
					write(port,outputBuffer);}
					usleep(1000*50);
				}
				else
				{
					std::cout<<"Initialization Failed "<<std::endl;

					os.write(AxisOFFID1,sizeof(AxisOFFID1));
					write(port,outputBuffer);

					os.write(AxisOFFID2,sizeof(AxisOFFID2));
					write(port,outputBuffer);

					CommsOkFlag = false;
					usleep(1000*50);
				}
			}
		}

		AverCount = 0;
		while (CommsOkFlag && ros::ok())
		{
			double time = ros::Time::now().toSec();
			//LABUST::JoystickData joystickData;
			//joystickData = joystick->ReadJoystickData();

			/*if (abs(joystickData.axes[1])>2000)
				{TauControl.Frw = float(-joystickData.axes[1])/intMax;}
			else
				{TauControl.Frw = 0;}
			if (abs(joystickData.axes[5])>2000)
				{TauControl.Yaw = float(joystickData.axes[5])/intMax;}
			else
				{TauControl.Yaw = 0;}

			//if ((TauControl.Frw!=TauControlOld.Frw) || (TauControl.Yaw!=TauControlOld.Yaw))
			//{
				TauControlOld = TauControl;*/
			TauT thrust(TauControl);
			TauC achTau = thrust.getTauC();
			///////////////ADDED ROS STUFF//////////////////////
			auv_msgs::BodyForceReq tau;
			tau.wrench.force.x = achTau.Frw;
			tau.wrench.torque.z = achTau.Yaw;
			bool windup = (TauControl.Yaw != achTau.Yaw);
			tau.disable_axis.x = windup;
			tau.disable_axis.yaw = windup;
			tauAch.publish(tau);

			StateDO[CalibrationData::calibrationPin] = calibration.trigger;
			printf("DIO state: %d\n", StateDO[CalibrationData::calibrationPin]);
			if ((calibration.state != CalibrationData::stop) && (calibration.state != CalibrationData::gyrosOnly))
			{
				++calibration.cycleWait;

				if (calibration.state == CalibrationData::start) 
				{
					if (calibration.cycleWait > calibration.cyclesMax)
					{
					  calibration.trigger = false;
					}
				}
				else if (calibration.cycleWait > calibration.cyclesMin)
				{
					calibration.trigger = false;
					if (calibration.state == CalibrationData::mag) calibration.state = CalibrationData::stop;
				}
			}

			if (calibration.state == CalibrationData::gyrosOnly)
			{
				++calibration.cycleWait;
				if (calibration.cycleWait > calibration.cyclesGyro)
				{
					calibration.trigger = false;
					calibration.state = CalibrationData::stop;
				}
			}

			////////////////////////////////////////////////////
			// (TauC TauControl, float Limit)*/
			//thrust.Port = 0;
			//thrust.Stb = 0;
			AverCount++;
			//if (AverCount < 20)
			//{thrust.Port = 0;
			//thrust.Stb = 0.15;}
			double an=0.045315881, wn= 0.0002496019;
			double ann=0.0419426925, wnn= 0.0002506513;

			//double rpm_port = (thrust.Port>=0)?log(thrust.Port/an)/wn:-log(-thrust.Port/ann)/wnn;
			//double rpm_stbd = (thrust.Stb>=0)?log(thrust.Stb/an)/wn:-log(-thrust.Stb/ann)/wnn;
			double rpm_port(0), rpm_stbd(0);
			int rpm_port_meas(0), rpm_stbd_meas(0);

				medianPort.push_back(RPM[1]);
				medianStbd.push_back(RPM[0]);
				if (medianPort.size()>median_size) medianPort.erase(medianPort.begin());
				if (medianStbd.size()>median_size) medianStbd.erase(medianStbd.begin());

				std::vector<int> med(medianPort);
				std::sort(med.begin(),med.end());
				rpm_port_meas = med[median_size/2];
				med.assign(medianStbd.begin(), medianStbd.end());
				std::sort(med.begin(),med.end());
				rpm_stbd_meas = med[median_size/2];
			if (useRPMControl)
			{
				//thrust.Port = -thrust.Port;
				if ((thrust.Port>=0) && (thrust.Port/an <= 1)) thrust.Port = an;
				if ((thrust.Port<0) && (-thrust.Port/ann <= 1)) thrust.Port = -ann;
				if ((thrust.Stb >=0) && (thrust.Stb/an <= 1)) thrust.Stb = an;
				if ((thrust.Stb <0) && (-thrust.Stb/ann <= 1)) thrust.Stb = -ann;
				rpm_port = (thrust.Port>=0)?log(thrust.Port/an)/wn:-log(-thrust.Port/ann)/wnn;
				rpm_stbd = (thrust.Stb>=0)?log(thrust.Stb/an)/wn:-log(-thrust.Stb/ann)/wnn;
				double max_current = 1;
				//rpm_port = int(rpm_port);
				//rpm_stbd = int(rpm_stbd);
				rpm_port = int(rpm_port/75)*75;
				rpm_stbd = int(rpm_stbd/75)*75;
				double error[]={rpm_port-rpm_port_meas,rpm_stbd-rpm_stbd_meas};
				integralRPM[0]+=-Ki*error[0]*0.1;
				integralRPM[1]+=Ki*error[1]*0.1;

				//curr_port=labust::math::coerce(-Kp*error[0] + integralRPM[0],-max_current,max_current);
				curr_port=cport.step(-rpm_port, rpm_port_meas);
				//curr_stbd=labust::math::coerce(Kp*error[1] + integralRPM[1],-max_current,max_current);
				curr_stbd=cstbd.step(rpm_stbd, -rpm_stbd_meas);
				SetReference(2,curr_port);
				usleep(1000*5);
				if (!CommsOkFlag)
				{break;}
				SetReference(1,curr_stbd);
				usleep(1000*5);
				if (!CommsOkFlag)
				{break;}
			}
			else
			{
				curr_port = thrust.Port;
				SetReference(2,-thrust.Port);
				//SetReference(1,0.5);
				usleep(1000*5);
				if (!CommsOkFlag)
				{break;}
				curr_stbd = thrust.Stb;
				SetReference(1,thrust.Stb);
				//SetReference(2,0.5);
				usleep(1000*5);
				if (!CommsOkFlag)
				{break;}
			}
			std::cout<<thrust.Port<<" "<<thrust.Stb<<" "<<TauControl.Frw<<" "<<TauControl.Yaw<<" "<<std::endl;
			//}
			enum {port_rpm_desired=0,
				stbd_rpm_desired,
				port_rpm_meas,
				stbd_rpm_meas,
				port_curr_desired,
				stbd_curr_desired,
				current,
				temp,
				voltage
			};

			cart2_info.data[port_rpm_desired] = rpm_port;
			cart2_info.data[stbd_rpm_desired] = rpm_stbd;
			cart2_info.data[port_curr_desired] = curr_port;
			cart2_info.data[stbd_curr_desired] = curr_stbd;
			cart2_info.data[port_rpm_meas] = rpm_port_meas;
			cart2_info.data[stbd_rpm_meas] = rpm_stbd_meas;
			cart2_info.data[voltage] = Supply_Voltage;
			cart2_info.data[current] = Vehicle_Current;
			cart2_info.data[temp] = Temperature;
			rpm_info.publish(cart2_info);

			if (AverCount == 40)
			{AverCount = 0;}
			//for(int i=0; i<2; ++i) currentAvg[i] = (currentAvg[i] + 0.01*Motor_Current[i])/1.01;
			Supply_Voltage_Previous = Supply_Voltage;
			//Motor_Current_Previous[0] = Motor_Current[0];
			//Motor_Current_Previous[1] = Motor_Current[1];
			Load_Position_Previous[0] = Load_Position[0];
			Load_Position_Previous[1] = Load_Position[1];
			ReadData();
			//GetMotorCurrent(2);
			/*if (!CommsOkFlag)
				{CommsOkFlag = true;
				CommsAll++;
			}*/
			/*GetSupplyVoltage();
			if (!CommsOkFlag)
				{CommsOkFlag = true;
				CommsAll++;}*/	

			if ((AverCount == 5) || (AverCount == 15))
				//{printf("%f\n",Motor_Current[0]);
				//printf("%f\n",abs(Motor_Current[1])*100);
			{printf("%f\n",Supply_Voltage);
			printf("%f\n",Vehicle_Current);
			printf("%f\n",Load_Position[0]);
			printf("%f\n",Load_Position[1]);
			printf("%f\n",Temperature);
			printf("%f\n",RPM[0]);
			printf("%f\n",RPM[1]);
			}

			//WD.reset();}
			///////////ADDED ROS STUFF/////////////
			sendDiagnostics(diagnostic, CommsOkFlag);
			//This we exchange with ros rate
			//rate.sleep();
			usleep(1000*90);
			ros::spinOnce();
			

			RPM[0] = wrapRPM(Load_Position[0] - Load_Position_Previous[0])*75;
			RPM[1] = wrapRPM(Load_Position[1] - Load_Position_Previous[1])*75;

			CommsOkFlag = true;

			bool VoltageTest = fabs(Supply_Voltage_Previous - Supply_Voltage)<0.0000001;
			bool portTest = (fabs(thrust.Stb) > 0.2) && (fabs(RPM[0])<10);
			bool stbdTest = (fabs(thrust.Port) > 0.2) && (fabs(RPM[1])<10);
			//stbdTest = portTest = false;
			if (VoltageTest || portTest || stbdTest)
			{
				std::cout<<"Voltage:"<<Supply_Voltage<<", "<<Supply_Voltage_Previous<<std::endl; 
				std::cout<<"Thust:"<<thrust.Port<<", "<<thrust.Stb<<std::endl; 
				std::cout<<"RPM:"<<RPM[0]<<", "<<RPM[1]<<std::endl; 
				CommsCount++;
				if (CommsCount == 20)
				{CommsOkFlag = false;}
				if (CommsCount == 19)
				{
					std::cout<<"Going to reset."<<std::endl;
				}
			}
			else
			{
				CommsCount = 0;
			}
			/////////////////////////////////////////

			std::cout<<"Loop time:"<<ros::Time::now().toSec() - time<<std::endl;
		}
		sendDiagnostics(diagnostic, CommsOkFlag);
		//t.join();
		//WD.reset();
	}

	//io.run();

	//imu.closePort();
	io.stop();
	t.join();
	std::cout<<"Exited."<<std::endl;
	return 0;
}





