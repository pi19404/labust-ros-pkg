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
#ifndef _TABLE_H
#define	_TABLE_H

#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <labust/xml/XMLReader.hpp>

namespace labust
{
    namespace xml
    {
        /**
         * this class assists parsing of the GYROS-Matrix xml string
         * It handles conversion to and from twodimensional data fields to gyros and allows
         * insertion of data into table
         */
        class GyrosMatrix
        {
        public:
            /*
             * Constructor for empty table
             */
            GyrosMatrix();
            
            /*
             * Constructor from xml reader that is parsing xml)
             * \throws std::string exception of errors occur while parsing
             *
             * \param reader reader for parsing data
             *
             */
            GyrosMatrix(labust::xml::Reader &reader);

            /*
             * Constructor from twodimensional string array
             *
             * \param matrix reference to 2D vector of strings which is the data of the table. GyrosMatrix dimensions are determined from this 2d array
             * \param text if true, all cells remain as given, otherwise, non number cells are converted to "null" (except label cells), default true
             * \param labelRows indexes of rowss which contain labels, default empty
             * \param labelCols indexes of columns which contain labels, default empty
             *
             */
            GyrosMatrix(std::vector<std::vector<std::string> > &matrix, bool text = true, std::vector<int> labelRows = std::vector<int>(), std::vector<int> labelCols = std::vector<int>());

            /*
             * Constructor from twodimensional double array
             *
             * \param matrix reference to 2D vector of double precision floating point numbers which is the data of the table. GyrosMatrix dimensions are determined from this 2d array
             * \param text if true, all cells remain as given, otherwise, non number cells are converted to "null" (except label cells), default false
             * \param labelRows indexes of rowss which contain labels, default empty
             * \param labelCols indexes of columns which contain labels, default empty
             *
             */
            GyrosMatrix(std::vector<std::vector<double> > &matrix, bool text  = false, std::vector<int> labelRows = std::vector<int>(), std::vector<int> labelCols = std::vector<int>());
            
            /*
             * Copy constructor
             */ 
            GyrosMatrix(const GyrosMatrix& orig);

            /*
             * Desctuctor - empty
             */
            virtual ~GyrosMatrix();

            /*
             * Templated getter for all data in the table (including label cols/rows)
             * Tries to parse all cells into the given datatype
             *
             * \param dataHold 2D vector to accept data from table
             * \returns 0 if all OK, -1 table is empty, -2 if no data was written to vector
             */
            template<class Type>
            int GetFullData(std::vector<std::vector<Type> >& dataHold)
            {
                int pushedRows = 0, retVal = 0;
                if (data.size() > 0)
                {
                    dataHold.clear();
                    for (int row = 0; row < rows; row++)
                    {
                        dataHold.push_back(std::vector<Type > ());
                        for (int column = 0; column < cols; column++)
                        {
                            std::stringstream buffer;
                            Type cell;
                            buffer << data[row][column];
                            buffer >> cell;
                            dataHold[row].push_back(cell);
                        }
                    }
                }
                else
                {
                    retVal = -1;
                }
                if (pushedRows == 0)
                {
                    retVal = -2;
                }
                return retVal;
            }

            /*
             * Templated getter for actual data in the table (excluding label cols/rows)
             * Tries to parse all cells into the given datatype
             *
             * \param dataHold 2D vector to accept data from table
             * \returns 0 if all OK, -1 table is empty, -2 if no data was written to vector
             */
            template<class Type>
            int GetData(std::vector<std::vector<Type> >& pureData)
            {
                int pushedRows = 0, retVal = 0;
                if (data.size() > 0)
                {
                    pureData.clear();
                    for (int row = 0; row < rows; row++)
                    {
                        if (std::find(labelRows.begin(), labelRows.end(), row) == labelRows.end())
                        {
                            pureData.push_back(std::vector<Type > ());
                            for (int column = 0; column < cols; column++)
                            {
                                if (std::find(labelCols.begin(), labelCols.end(), column) == labelCols.end())
                                {
                                    std::stringstream buffer;
                                    Type cell;
                                    buffer << data[row][column];
                                    buffer >> cell;
                                    pureData[pushedRows].push_back(cell);
                                }
                            }
                            pushedRows++;
                        }
                    }
                }
                else
                {
                    retVal = -1;
                }
                if(pushedRows == 0)
                {
                    retVal = -2;
                }
                return retVal;
            }

            /*
             * Getter for columns marked as labels
             *
             * \returns all columns marked as labels
             */
            std::vector<std::vector<std::string> > GetLabelCols();

            /*
             * Getter for rows marked as labels
             *
             * \returns all rows marked as labels
             */
            std::vector<std::vector<std::string> > GetLabelRows();

            /*
             * Inserts data from another table into this one, while maintaining labels only from first table
             * If the position of the new table is outside the original one, a new table is created which holds them both
             *
             * \param tabletoInsert new table to insert data from
             * \param row row at which to place the first cell of new table
             * \param column column at which to place the first cell of new table
             * \param horizontalExpand if true and if tables overlap, old one is made wider to accept the new one, if this and next one are false, data is overwritten
             * \param verticalExpand if true and if tables overlap, old one is made taller to accept the new one, if this and previous one are false, data is overwritten
             * \param pad data to write to cells with indefined values upon expansion
             *
             */
            void InsertData(GyrosMatrix tabletoInsert, int row = 0, int column = 0, bool horizontalExpand = false, bool verticalExpand=false, std::string pad="null");

            /*
             * Converts the table to gyros xml encoding (<matrix...>...</matrix>
             * \returns Gyros XML string
             */
            std::string ToGyrosXML();

            /*
             * Accessor to enable data editing as in a 2D array
             */
            std::vector<std::string> operator[](int i)
            {
                return data[i];
            }

            /*
             * Marks table as diagonal if param is true, unmarks if false
             * If table is not square, throws error
             * \throws XMLEXCEPTION with error
             * \param diagonal value to set diagonality to
             */
            inline void Diagonal(bool diagonal)
            {
                if(cols!=rows && diagonal)
                {
                    throw labust::xml::XMLException("Error: Non square matrix cannot be marked as diagonal");
                }
                else
                {
                    this->diagonal = diagonal;
                }
            }

            /*
             * Checks if table is diagonal
             * \returns diagonality as a boolean var
             */
            inline bool IsDiagonal()
            {
                return diagonal;
            }

            /*
             * Title of the matrix
             */
            std::string title;
        private:
            bool diagonal;
            int rows, cols;
            std::vector<int> labelCols, labelRows;
            std::vector<std::vector<std::string> > data;
            
            bool text;
        };
        
        template<>
        int GyrosMatrix::GetFullData<std::string>(std::vector<std::vector<std::string> >& pureData);

        template<>
        int GyrosMatrix::GetData<std::string>(std::vector<std::vector<std::string> >& pureData);
    }
}



#endif	/* _GYROSTABLE_H */

