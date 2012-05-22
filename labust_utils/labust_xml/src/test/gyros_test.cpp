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
#include <labust/xml/XMLException.hpp>
#include <labust/xml/GyrosReader.hpp>
#include <labust/xml/GyrosWriter.hpp>
#include <labust/xml/GyrosMatrix.hpp>
#include <labust/tools/TimingTools.hpp>
#include <boost/foreach.hpp>

#include <iostream>

#define TEST_STRING_LENGTH 3000
#define TEST_DICTIONARY_SIZE 20
#define TEST_DICTIONARY_KEY_SIZE 20
#define TEST_DICTIONARY_VALUE_SIZE 40
#define TEST_METADATA_SIZE 10
#define TEST_METADATA_KEY_SIZE 5
#define TEST_METADATA_VALUE_SIZE 10
#define TEST_BINARY_BYTES 1024

#define TEST_MATRIX_ROWS 3
#define TEST_MATRIX_COLS 3
#define TEST_MATRIX_TEXT_LENGTH 10

bool test_gyros_scalar(int count)
try
{
	int numberToEncode, decodedNumber;
	double decimalNumberToEncode, decodedDecimalNumber;
	std::string textToEncode, decodedText;

	numberToEncode = rand();
	decimalNumberToEncode = (double)rand()/(double)rand()*(double)rand();

	std::string chars(
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "1234567890"
        "!@#$%^&*()"
        "`~-_=+[{]{\\|;:'\",<.>/? ");
	std::stringstream tmpString;
	for(int i = 0; i < TEST_STRING_LENGTH; i++) 
	{
		tmpString << chars[rand()%chars.size()];
	}
	textToEncode = tmpString.str();

	std::cout<<"Testing reader/writer with "<<count<<" iterations, integer number"<<std::endl;
	double startTime = labust::tools::unix_time();	

	for(int i=0; i<count; i++)
	{
		std::string gyrosMessage;
		labust::xml::GyrosWriter writer(numberToEncode);
		writer.SetTimeStamp(true);
		double timestampOriginal = writer.GetTimeStamp();
		writer.SetLabel("Test");
		gyrosMessage = writer.GyrosXML();
  		labust::xml::GyrosWriter writer2(numberToEncode);
		labust::xml::GyrosReader reader(gyrosMessage);
		std::string label = reader.GetLabel();
		double timestamp = reader.GetTimeStamp();
		decodedNumber = 0;
		reader.scalar(decodedNumber);

		if(!boost::iequals(label,"Test"))
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding label in iteration "<< i << " (original = Test, decoded = "<<label<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}

		if(abs(timestamp-timestampOriginal)>0.1)
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding timestamp in iteration "<< i << " (original = "<<timestampOriginal<<", decoded = "<<timestamp<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}	
		
		if(decodedNumber!= numberToEncode)
		{
			std::stringstream errorMsg;
			errorMsg<< "Error decoding data in iteration "<< i<<" (original = "<<numberToEncode<<", decoded = "<<decodedNumber<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}
		
	}
	double duration = labust::tools::unix_time() - startTime;
	std::cout<<"Test successful, duration = "<<duration<<" seconds"<<std::endl;

	std::cout<<"Testing reader/writer with "<<count<<" iterations, floating point number"<<std::endl;
	startTime = labust::tools::unix_time();	

	for(int i=0; i<count; i++)
	{
		std::string gyrosMessage;
		labust::xml::GyrosWriter writer(decimalNumberToEncode);
		writer.SetTimeStamp(true);
		double timestampOriginal = writer.GetTimeStamp();
		writer.SetLabel("Test");
		gyrosMessage = writer.GyrosXML();
		labust::xml::GyrosReader reader(gyrosMessage);
		std::string label = reader.GetLabel();
		double timestamp = reader.GetTimeStamp();
		decodedDecimalNumber = 0;
		reader.scalar<double>(decodedDecimalNumber);

		if(!boost::equals(label,"Test"))
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding label in iteration "<< i << " (original = Test, decoded = "<<label<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}

		if(abs(timestamp-timestampOriginal)>0.1)
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding timestamp in iteration "<< i << " (original = "<<timestampOriginal<<", decoded = "<<timestamp<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}	
		
		if(abs(decodedDecimalNumber - decimalNumberToEncode)>0.1)
		{
			std::stringstream errorMsg;
			errorMsg<<std::setprecision(20)<<std::setiosflags(std::ios::fixed)<<std::setiosflags(std::ios::showpoint)<<"Error decoding data in iteration "<< i<<" (original = "<<decimalNumberToEncode<<", decoded = "<<decodedDecimalNumber<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}
	}
	duration = labust::tools::unix_time() - startTime;
	std::cout<<"Test successful, duration = "<<duration<<" seconds"<<std::endl;

	std::cout<<"Testing reader/writer with "<<count<<" iterations, random text ("<<TEST_STRING_LENGTH<<" chars)"<<std::endl;
	startTime = labust::tools::unix_time();	

	for(int i=0; i<count; i++)
	{
		std::string gyrosMessage;
		labust::xml::GyrosWriter writer(textToEncode);
		writer.SetTimeStamp(true);
		double timestampOriginal = writer.GetTimeStamp();
		writer.SetLabel("Test");
		gyrosMessage = writer.GyrosXML();
		labust::xml::GyrosReader reader(gyrosMessage);
		std::string label = reader.GetLabel();
		double timestamp = reader.GetTimeStamp();
		decodedText = "";
		reader.scalar(decodedText);

		if(!boost::equals(label,"Test"))
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding label in iteration "<< i << " (original = Test, decoded = "<<label<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}

		if(abs(timestamp-timestampOriginal)>0.1)
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding timestamp in iteration "<< i << " (original = "<<timestampOriginal<<", decoded = "<<timestamp<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}	
		
		if(boost::equals(textToEncode,decodedText))
		{
			std::stringstream errorMsg;
			errorMsg<< "Error decoding data in iteration "<< i<<" (original = "<<textToEncode<<", decoded = "<<decodedText<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}
		
	}
	duration = labust::tools::unix_time() - startTime;
	std::cout<<"Test successful, duration = "<<duration<<" seconds"<<std::endl;

	return true;
}
catch (labust::xml::XMLException& e)
{
	std::cout<<"Gyros Scalar test: "<<e.what()<<std::endl;
	return false;
};

bool test_gyros_error(int count)
try
{
	std::string textToEncode, decodedText;

	std::string chars(
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "1234567890"
        "!@#$%^&*()"
        "`~-_=+[{]{\\|;:'\",<.>/? ");
	std::stringstream tmpString;
	for(int i = 0; i < TEST_STRING_LENGTH; i++) 
	{
		tmpString << chars[rand()%chars.size()];
	}
	textToEncode = tmpString.str();

	std::cout<<"Testing reader/writer with "<<count<<" iterations, random text ("<<TEST_STRING_LENGTH<<" chars)"<<std::endl;
	double startTime = labust::tools::unix_time();	

	for(int i=0; i<count; i++)
	{
		std::string gyrosMessage;
		labust::xml::GyrosWriter writer(textToEncode);
		writer.SetTimeStamp(true);
		double timestampOriginal = writer.GetTimeStamp();
		writer.SetLabel("Test");
		gyrosMessage = writer.GyrosXML();
		labust::xml::GyrosReader reader(gyrosMessage);
		std::string label = reader.GetLabel();
		double timestamp = reader.GetTimeStamp();
		decodedText = "";
		reader.scalar(decodedText);

		if(!boost::equals(label,"Test"))
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding label in iteration "<< i << " (original = Test, decoded = "<<label<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}

		if(abs(timestamp-timestampOriginal)>0.1)
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding timestamp in iteration "<< i << " (original = "<<timestampOriginal<<", decoded = "<<timestamp<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}	
		
		if(boost::equals(textToEncode,decodedText))
		{
			std::stringstream errorMsg;
			errorMsg<< "Error decoding data in iteration "<< i<<" (original = "<<textToEncode<<", decoded = "<<decodedText<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}
		
	}
	double duration = labust::tools::unix_time() - startTime;
	std::cout<<"Test successful, duration = "<<duration<<" seconds"<<std::endl;

	return true;
}
catch (labust::xml::XMLException& e)
{
	std::cout<<"Gyros Error test: "<<e.what()<<std::endl;
	return false;
};

bool test_gyros_matrix(int count)
try
{
	std::vector<std::vector<std::string> > textMatrixToEncode, decodedTextMatrix;
	std::vector<std::vector<double> > numericMatrixToEncode, decodedNumericMatrix;
	for(int i=0; i<TEST_MATRIX_ROWS; i++)
	{
		std::vector<std::string> newRowText;
		std::vector<double> newRowDouble;
		for(int j=0; j<TEST_MATRIX_COLS; j++)
		{
			std::string chars(
			"abcdefghijklmnopqrstuvwxyz"
			"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
			"1234567890"
			);
			std::stringstream tmpString;
			for(int i = 0; i < TEST_MATRIX_TEXT_LENGTH; i++) 
			{
				tmpString << chars[rand()%chars.size()];
			}
			newRowText.push_back(tmpString.str());

			newRowDouble.push_back((double)rand()/(double)rand()*(double)rand());
		}
		textMatrixToEncode.push_back(newRowText);
		numericMatrixToEncode.push_back(newRowDouble);
	}
	
	std::cout<<"Testing reader/writer with "<<count<<" iterations, randomized textual matrix with "<<TEST_MATRIX_COLS<<" columns and "<<TEST_MATRIX_ROWS<<" rows."<<std::endl;
	double startTime = labust::tools::unix_time();	
	for(int i=0; i<count; i++)
	{
		labust::xml::GyrosMatrix gyrosMatrix(textMatrixToEncode,true);
		std::string gyrosMessage;
		labust::xml::GyrosWriter writer(gyrosMatrix);
		writer.SetTimeStamp(true);
		double timestampOriginal = writer.GetTimeStamp();
		writer.SetLabel("Test");
		gyrosMessage = writer.GyrosXML();
		labust::xml::GyrosReader reader(gyrosMessage);
		std::string label = reader.GetLabel();
		double timestamp = reader.GetTimeStamp();
		labust::xml::GyrosMatrix decodedGyrosMatrix = *reader.matrix<boost::shared_ptr<labust::xml::GyrosMatrix> >();

		if(!boost::equals(label,"Test"))
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding label in iteration "<< i << " (original = Test, decoded = "<<label<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}

		if(abs(timestamp-timestampOriginal)>0.1)
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding timestamp in iteration "<< i << " (original = "<<timestampOriginal<<", decoded = "<<timestamp<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}	
		
		bool matrixOK = true;

		decodedGyrosMatrix.GetFullData(decodedTextMatrix);
		
		for(int j=0; j<TEST_MATRIX_ROWS; j++)
		{
			std::vector<std::string> newRowText;
			std::vector<double> newRowDouble;
			for(int k=0; k<TEST_MATRIX_COLS; k++)
			{
				if(!boost::iequals(decodedTextMatrix[j][k],textMatrixToEncode[j][k]))
				{
					matrixOK = false;
				}	
			}
		}
		
		if(!matrixOK)
		{
			std::stringstream errorMsg;
			errorMsg<< "Error decoding text matrix data in iteration "<< i<<"."<<std::endl;;
			throw(labust::xml::XMLException(errorMsg.str()));
		}
		
	}
	double duration = labust::tools::unix_time() - startTime;
	std::cout<<"Test successful, duration = "<<duration<<" seconds"<<std::endl;

	std::cout<<"Testing reader/writer with "<<count<<" iterations, randomized numeric matrix with "<<TEST_MATRIX_COLS<<" columns and "<<TEST_MATRIX_ROWS<<" rows."<<std::endl;
	startTime = labust::tools::unix_time();	
	for(int i=0; i<count; i++)
	{
		labust::xml::GyrosMatrix gyrosMatrix(numericMatrixToEncode,false);
		std::string gyrosMessage;
		labust::xml::GyrosWriter writer(gyrosMatrix);
		writer.SetTimeStamp(true);
		double timestampOriginal = writer.GetTimeStamp();
		writer.SetLabel("Test");
		gyrosMessage = writer.GyrosXML();
		labust::xml::GyrosReader reader(gyrosMessage);
		std::string label = reader.GetLabel();
		double timestamp = reader.GetTimeStamp();
		labust::xml::GyrosMatrix decodedGyrosMatrix = *reader.matrix<boost::shared_ptr<labust::xml::GyrosMatrix> >();

		if(!boost::equals(label,"Test"))
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding label in iteration "<< i << " (original = Test, decoded = "<<label<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}

		if(abs(timestamp-timestampOriginal)>0.1)
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding timestamp in iteration "<< i << " (original = "<<timestampOriginal<<", decoded = "<<timestamp<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}	
		
		bool matrixOK = true;

		decodedGyrosMatrix.GetFullData(decodedNumericMatrix);
		
		for(int j=0; j<TEST_MATRIX_ROWS; j++)
		{
			for(int k=0; k<TEST_MATRIX_COLS; k++)
			{
				if(decodedNumericMatrix[j][k]-numericMatrixToEncode[j][k]>0.1)
				{
					matrixOK = false;
				}	
			}
		}
		
		if(!matrixOK)
		{
			std::stringstream errorMsg;
			errorMsg<< "Error decoding numeric matrix data in iteration "<< i<<"."<<std::endl;;
			throw(labust::xml::XMLException(errorMsg.str()));
		}
		
	}
	duration = labust::tools::unix_time() - startTime;
	std::cout<<"Test successful, duration = "<<duration<<" seconds"<<std::endl;
	
	return true;
}
catch (labust::xml::XMLException& e)
{
	std::cout<<e.what()<<std::endl;
	return false;
};

bool test_gyros_dictionary(int count)
try
{
	std::string textToEncode, decodedText;

	std::string chars(
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "1234567890"
        "!@#$%^&*()"
        "`~-_=+[{]{\\|;:'\",<.>/? ");

	std::map<std::string,std::string> textDictionaryToEncode, decodedTextDictionary;
	std::map<int, double> numericDictionaryToEncode, decodedNumericDictionary;

	for(int i=0; i<TEST_DICTIONARY_SIZE; i++)
	{
		std::string texKey, texValue;
		std::stringstream tmpKey, tmpValue;
		for(int i = 0; i < TEST_DICTIONARY_KEY_SIZE; i++) 
		{
			tmpKey << chars[rand()%chars.size()];
		}
		for(int i = 0; i < TEST_DICTIONARY_VALUE_SIZE; i++) 
		{
			tmpValue << chars[rand()%chars.size()];
		}
		textDictionaryToEncode[tmpKey.str()]=tmpValue.str();
		int key = rand();
		double value = (double)rand()/(double)rand()*(double)rand();
		numericDictionaryToEncode[key]=value;
	}

	std::cout<<"Testing reader/writer with "<<count<<" iterations, randomized textual dictionary of size "<<TEST_DICTIONARY_SIZE<<"."<<std::endl;
	double startTime = labust::tools::unix_time();	
	for(int i=0; i<count; i++)
	{
		std::string gyrosMessage;
		labust::xml::GyrosWriter writer(textDictionaryToEncode.begin(),textDictionaryToEncode.end());
		writer.SetTimeStamp(true);
		double timestampOriginal = writer.GetTimeStamp();
		writer.SetLabel("Test");
		gyrosMessage = writer.GyrosXML();
		labust::xml::GyrosReader reader(gyrosMessage);
		std::string label = reader.GetLabel();
		double timestamp = reader.GetTimeStamp();
		reader.dictionary(decodedTextDictionary);

		if(!boost::equals(label,"Test"))
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding label in iteration "<< i << " (original = Test, decoded = "<<label<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}

		if(abs(timestamp-timestampOriginal)>0.1)
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding timestamp in iteration "<< i << " (original = "<<timestampOriginal<<", decoded = "<<timestamp<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}	
		
		bool dictOK = true;

		for(std::map<std::string,std::string>::iterator it = decodedTextDictionary.begin(); it!=decodedTextDictionary.end();it++)
		{
			if(!boost::equals(textDictionaryToEncode[it->first],it->second))
			{
				dictOK = false;
			}
		}
		
		if(!dictOK)
		{
			std::stringstream errorMsg;
			errorMsg<< "Error decoding data in iteration "<< i<<"."<<std::endl;;
			throw(labust::xml::XMLException(errorMsg.str()));
		}
		
	}
	double duration = labust::tools::unix_time() - startTime;
	std::cout<<"Test successful, duration = "<<duration<<" seconds"<<std::endl;

	std::cout<<"Testing reader/writer with "<<count<<" iterations, randomized numeric dictionary of size "<<TEST_DICTIONARY_SIZE<<"."<<std::endl;
	startTime = labust::tools::unix_time();	
	for(int i=0; i<count; i++)
	{
		std::string gyrosMessage;
		labust::xml::GyrosWriter writer(numericDictionaryToEncode.begin(),numericDictionaryToEncode.end());
		writer.SetTimeStamp(true);
		double timestampOriginal = writer.GetTimeStamp();
		writer.SetLabel("Test");
		gyrosMessage = writer.GyrosXML();
		labust::xml::GyrosReader reader(gyrosMessage);
		std::string label = reader.GetLabel();
		double timestamp = reader.GetTimeStamp();
		reader.dictionary(decodedNumericDictionary);

		if(!boost::equals(label,"Test"))
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding label in iteration "<< i << " (original = Test, decoded = "<<label<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}

		if(abs(timestamp-timestampOriginal)>0.1)
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding timestamp in iteration "<< i << " (original = "<<timestampOriginal<<", decoded = "<<timestamp<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}	
		
		bool dictOK = true;

		for(std::map<int,double>::iterator it = decodedNumericDictionary.begin(); it!=decodedNumericDictionary.end();it++)
		{
			if(abs(numericDictionaryToEncode[it->first]-it->second)>0.1)
			{
				dictOK = false;
			}
		}
		
		if(!dictOK)
		{
			std::stringstream errorMsg;
			errorMsg<< "Error decoding data in iteration "<< i<<".";
			throw(labust::xml::XMLException(errorMsg.str()));
		}
		
	}
	duration = labust::tools::unix_time() - startTime;
	std::cout<<"Test successful, duration = "<<duration<<" seconds"<<std::endl;
	return true;
}
catch (labust::xml::XMLException& e)
{
	std::cout<<e.what()<<std::endl;
	return false;
};

bool test_gyros_binary(int count)
try
{
	std::string textToEncode, decodedText;

	std::string chars(
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "1234567890"
        );

	std::map<std::string,std::string> metadataToEncode;

	for(int i=0; i<TEST_METADATA_SIZE; i++)
	{
		std::string texKey, texValue;
		std::stringstream tmpKey, tmpValue;
		for(int i = 0; i < TEST_METADATA_KEY_SIZE; i++) 
		{
			tmpKey << chars[rand()%chars.size()];
		}
		for(int i = 0; i < TEST_METADATA_VALUE_SIZE; i++) 
		{
			tmpValue << chars[rand()%chars.size()];
		}
		metadataToEncode[tmpKey.str()]=tmpValue.str();
	}

	std::vector<char> dataToEncode;
	for(int i=0; i<TEST_BINARY_BYTES; i++)
	{
		dataToEncode.push_back(chars[rand()%chars.size()]);
	}

	std::cout<<"Testing reader/writer with "<<count<<" iterations, randomized metadata of size "<<TEST_METADATA_SIZE<<", randomized raw data, "<<TEST_BINARY_BYTES<<" bytes."<<std::endl;
	double startTime = labust::tools::unix_time();	
	for(int i=0; i<count; i++)
	{
		std::string gyrosMessage;
		labust::xml::GyrosWriter writer(dataToEncode.begin(),dataToEncode.end(),labust::xml::GYROS::raw,metadataToEncode);
		writer.SetTimeStamp(true);
		double timestampOriginal = writer.GetTimeStamp();
		writer.SetLabel("Test");
		gyrosMessage = writer.GyrosXML();
		labust::xml::GyrosReader reader(gyrosMessage);
		std::string label = reader.GetLabel();
		double timestamp = reader.GetTimeStamp();
		
		labust::xml::GyrosBinaryReturn decodedDataStruct;		
		reader.binary(decodedDataStruct);

		if(!boost::equals(label,"Test"))
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding label in iteration "<< i << " (original = Test, decoded = "<<label<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}

		if(abs(timestamp-timestampOriginal)>0.1)
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding timestamp in iteration "<< i << " (original = "<<timestampOriginal<<", decoded = "<<timestamp<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}	
		
		if(decodedDataStruct.encoding !=labust::xml::GYROS::raw)
		{
			std::stringstream errorMsg;
			errorMsg <<"Error decoding encoding type in iteration "<< i << " (original = "<<labust::xml::GYROS::raw<<", decoded = "<<decodedDataStruct.encoding<<")";
			throw(labust::xml::XMLException(errorMsg.str()));
		}

		bool metadataOK = true;

		for(std::map<std::string,std::string>::iterator it = decodedDataStruct.metadata.begin(); it!=decodedDataStruct.metadata.end();it++)
		{
			if(!boost::equals(metadataToEncode[it->first],it->second))
			{
				metadataOK = false;
			}
		}
		
		if(!metadataOK)
		{
			std::stringstream errorMsg;
			errorMsg<< "Error decoding metadata in iteration "<< i<<"."<<std::endl;;
			throw(labust::xml::XMLException(errorMsg.str()));
		}

		bool dataOK = true;


		if(decodedDataStruct.data.size()==TEST_BINARY_BYTES)
		{
			for(unsigned int j=0; j<decodedDataStruct.data.size();j++)
			{
				if(dataToEncode[j]!=decodedDataStruct.data[j])
				{
					dataOK = false;
				}
			}
		}
		else
		{
			dataOK = false;
		}
		
		if(!dataOK)
		{
			std::stringstream errorMsg;
			errorMsg<< "Error decoding data in iteration "<< i<<"."<<std::endl;;
			throw(labust::xml::XMLException(errorMsg.str()));
		}
		
	}
	double duration = labust::tools::unix_time() - startTime;
	std::cout<<"Test successful, duration = "<<duration<<" seconds"<<std::endl;

	return true;	
}
catch (labust::xml::XMLException& e)
{
	std::cout<<e.what()<<std::endl;
	return false;
};

int main(int argc, char* argv[])
try
{
	srand (time(NULL));
	bool scalarTestOK, errorTestOK, matrixTestOK, dictionaryTestOK, binaryTestOK; 
	int iterationCount = 10000;
	if(argc>1)
		iterationCount = atoi(argv[1]);
	std::cout<<"Beginning test of Gyros Scalar."<<std::endl;
	scalarTestOK = test_gyros_scalar(iterationCount);
	std::cout<<"Gyros Scalar test "<<(scalarTestOK ? "successful.":"failed.")<<std::endl<<std::endl;

	std::cout<<"Beginning test of Gyros Error."<<std::endl;
	errorTestOK = test_gyros_error(iterationCount);
	std::cout<<"Gyros Error test "<<(errorTestOK ? "successful.":"failed.")<<std::endl<<std::endl;
	
	std::cout<<"Beginning test of Gyros Matrix."<<std::endl;
	matrixTestOK = test_gyros_matrix(iterationCount);
	std::cout<<"Gyros Matrix test "<<(matrixTestOK ? "successful.":"failed.")<<std::endl<<std::endl;

	std::cout<<"Beginning test of Gyros Dictionary."<<std::endl;
	dictionaryTestOK = test_gyros_dictionary(iterationCount);
	std::cout<<"Gyros Dictionary test "<<(matrixTestOK  ? "successful.":"failed.")<<std::endl<<std::endl;

	std::cout<<"Beginning test of Gyros Binary."<<std::endl;
	binaryTestOK = test_gyros_binary(iterationCount);
	std::cout<<"Gyros Binary test "<<(binaryTestOK ? "successful.":"failed.")<<std::endl<<std::endl;

	std::cout<<"Gyros library "<<(scalarTestOK && errorTestOK && matrixTestOK && dictionaryTestOK && binaryTestOK ? "passed" : "failed") <<" test."<<std::endl;

	return 0;
}
catch (std::exception& e)
{
	std::cout<<e.what()<<std::endl;
	return -1;
}
catch (...)
{
	std::cout<<"Unknown error."<<std::endl;
	return -1;
}


