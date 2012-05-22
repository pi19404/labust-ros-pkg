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
#ifndef CSVLOGGER_H_
#define CSVLOGGER_H_

#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <stdexcept>

namespace labust
{
  namespace tools
  {
    /*
     * The class helps data logging into a comma separated file.
     */
    class CSVLogger
    {
    	typedef unsigned char uint8;
    public:
      /*
       * Main constructor that creates the CSV file on the disk.
       *
       * \param fileName Name of the CSV file.
       * \param delimiter What to use for field separation. Defaults to a comma ','.
       * \param precision Decimal places to write for floating point numbers.
       */
    	CSVLogger(const std::string& fileName, char delimiter = ',', uint8 precision = 6):
    		csvFile(fileName.c_str()),
        delimiter(delimiter)
    	{
    		if (!csvFile.is_open())
        {
          throw std::runtime_error("Error: Unable to open CSV file.");
        }
        //Set precision for double values
        csvFile.precision(precision);
        csvFile<<std::fixed;
      }

      /*
       * The method adds a comment to the file. Comments start with '%' by default.
       *
       * \param comment Comment to written into the file
       * \param skip The flag for skipping the initial '%' sign for this comment.
       */
      inline void addComment(const std::string& comment, bool skip = false)
      {
        if (!skip) csvFile<<'%';
        csvFile<<comment;
      }
      /*
       * The method adds comments to the CSV file. Comments start with '%' by default.
       * Useful when mapping named values. The names can be written directly from the
       * map or name vector into the comment header.
       *
       * \param begin Pointer to the start of the comment list.
       * \param end Pointer to the end of the comment list.
       */
      template <class InputIterator>
      void addComment(const InputIterator begin, const InputIterator end, bool skip = false)
      {
        InputIterator it = begin;
        if (!skip) csvFile<<'%';
        while (it!=end){log_comment(*it++);}
      }
      /*
       * The method logs a data range specified by the iterator range.
       *
       * \param begin Pointer to the start of the sequence.
       * \param end Pointer to the end of the sequence.
       */
      template<class InputIterator>
      inline void log(const InputIterator begin, const InputIterator end)
      {
        InputIterator it = begin;
        while (it!=end){log(*it++);}
      }
      /**
       * The method logs a single value.
       *
       * \param value Value to write into the file.
       */
      template<class Type>
      inline void log(const Type& value){csvFile<<value<<delimiter;}
      /**
       * The method specializes the single value logger for unsigned char values.
       * We would like the unsigned char values to be written as numbers.
       *
       * \param value Value to write into the file.
       */
      inline void log(uint8 value){csvFile<<int(value)<<delimiter;}

      /**
       * The method skips to the next line in the file.
       */
      inline void newLine()
      {
        csvFile.seekp((int)csvFile.tellp()-1);
        csvFile<<"\n";
      };

    protected:
      /**
       * The method for logging key-value pairs. Only the second argument is logged.
       * Useful for vectors with named values.
       */
      template <class Key, class Value>
      inline void log(const std::pair<Key,Value>& pair){log(pair.second);}
      /**
       * The method for logging key-value pairs. Only the first argument is logged.
       * Useful for vectors with named values.
       */
      template <class Key, class Value> inline void log_comment(const std::pair<Key,Value>& pair){log(pair.first);};
      /**
       * The method logs normal comments.
       */
      template <class Value> inline void log_comment(const Value& value){log(value);};
      /*
       * The file stream object.
       */
      std::ofstream csvFile;
      /*
       * The value delimiter.
       */
      char delimiter;
    };
  }
}
/* CSVLOGGER_H_ */
#endif
