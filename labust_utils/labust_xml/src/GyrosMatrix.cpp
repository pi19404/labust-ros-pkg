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
#include <labust/xml/GyrosMatrix.hpp>
#include <labust/xml/XMLReader.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <math.h>


namespace labust
{
    namespace xml
    {
        GyrosMatrix::GyrosMatrix() :
        title(""),
        diagonal(false),
        rows(0),
        cols(0),
        text(false)
        {
        }

        GyrosMatrix::GyrosMatrix(labust::xml::Reader &reader) :
        title(""),
        diagonal(false),
        rows(0),
        cols(0),
        text(false)
        {
            using namespace labust::xml;
            //Reader reader(gyrosMatrixXML, std::string("//matrix"));
            _xmlNode *matrixNode(0), *labelsNode(0), *dataNode(0);

            if (reader.try_value("//matrix", &matrixNode) && reader.try_value("//data", &dataNode))
            {
                reader.useNode(dataNode);
                std::string matrixData, boolString;
                reader.value<std::string > ("text()", &matrixData);
                reader.try_value("@rows", &rows);
                reader.try_value("@columns", &cols);
                reader.try_value("@text", &boolString);
                text = boost::iequals(boolString, "true");
                boolString="";
                reader.try_value("@diagonal", &boolString);
                diagonal = boost::iequals(boolString, "true");

                reader.useNode(matrixNode);
                reader.try_value("@title", &title);
                if (reader.try_value("//labels", &labelsNode))
                {
                    reader.useNode(labelsNode);
                    try
                    {
                        NodeCollectionPtr rows = reader.value<NodeCollectionPtr > ("//row");

                        for (unsigned int i = 0; i < rows->size(); ++i)
                        {
                        		reader.useNode((*rows)[i]);
                            //labelRows.push_back(reader.value<int>("text()", (*rows)[i]));
                        		labelRows.push_back(reader.value<int>("text()"));
                        }
                    }
                    catch (XMLException&){};

                    reader.useNode(labelsNode);
                    try
                    {
                        NodeCollectionPtr cols = reader.value<NodeCollectionPtr > ("//column");

                        for (unsigned int i = 0; i < cols->size(); ++i)
                        {
                        		reader.useNode((*cols)[i]);
                            labelCols.push_back(reader.value<int>("text()"));
                        }
                    }
                    catch (XMLException&){};
                }

                if (cols == 0 || rows == 0)
                {
                    data.clear();
                }
                else
                {
                    if (diagonal && rows != cols)
                    {
                        throw XMLException("Non square matrix declared as diagonal");
                    }

                    //if there are not enough cells, fill the matrix up with null cells at the end
                    int cellCount = std::count(matrixData.begin(), matrixData.end(), '|') + 1;
                    for (int i = 0; i < (rows * cols - cellCount); i++)
                    {
                        matrixData = matrixData + "|null";
                    }

                    int cell = 0;

                    boost::char_separator<char> separator("|");

                    boost::tokenizer<boost::char_separator<char> > splitter(matrixData, separator);
                    for (boost::tokenizer<boost::char_separator<char> >::iterator it = splitter.begin(); it != splitter.end(); it++)
                    {
                        if (!diagonal)
                        {
                            int row = cell / cols;
                            int col = cell % cols;
                            if (col == 0)
                            {
                                data.push_back(std::vector<std::string > ());
                            }
                            if (text || std::find(labelCols.begin(), labelCols.end(), col) != labelCols.end() || std::find(labelRows.begin(), labelRows.end(), row) != labelRows.end())
                            {
                                data[row].push_back(*it);
                            }
                            else
                            {
                                std::stringstream buffer;
                                bool isNumber = true;
                                for (unsigned int i = 0; i < (*it).length(); i++)
                                {
                                    isNumber = isNumber & (((*it)[i] >= '0' && (*it)[i] <= '9') || (*it)[i] == ',' || (*it)[i] == '.');
                                }
                                if (isNumber)
                                {
                                    data[row].push_back(*it);
                                }
                                else
                                {
                                    data[row].push_back("null");
                                }
                            }
                            cell++;
                        }
                        else
                        {
                            std::vector<std::string> newRow;
                            for (int rowCol = 0; rowCol < cols; rowCol++)
                            {
                                if (rowCol == cell)
                                {
                                    if (text || std::find(labelCols.begin(), labelCols.end(), rowCol) != labelCols.end() || std::find(labelRows.begin(), labelRows.end(), rowCol) != labelRows.end())
                                    {
                                        newRow.push_back(*it);
                                    }
                                    else
                                    {
                                        std::stringstream buffer;
                                        bool isNumber = true;
                                        for (unsigned int i = 0; i < (*it).length(); i++)
                                        {
                                            isNumber = isNumber & (((*it)[i] >= '0' && (*it)[i] <= '9') || (*it)[i] == ',' || (*it)[i] == '.');
                                        }
                                        if (isNumber)
                                        {
                                            newRow.push_back(*it);
                                        }
                                        else
                                        {
                                            newRow.push_back("null");
                                        }
                                    }

                                }
                                else
                                {
                                    newRow.push_back("null");
                                }
                            }
                            data.push_back(newRow);
                            cell++;
                        }
                    }
                }
            }
        }

        GyrosMatrix::GyrosMatrix(std::vector<std::vector<std::string> >& matrix, bool text, std::vector<int> labelRows, std::vector<int> labelCols) :
        title(""),
        diagonal(false),
        rows(matrix.size()),
        cols((matrix.size() > 0 ? matrix[0].size() : 0)),
        labelCols(labelCols),
        labelRows(labelRows),
        data(matrix),
        text(text)
        {

            for (std::vector<std::vector <std::string> >::iterator row = data.begin(); row != data.end(); row++)
            {
                int sizeDifference = cols - row->size();
                if (sizeDifference > 0)
                {
                    row->insert(row->end(), sizeDifference, std::string("null"));
                }
                else if (sizeDifference < 0)
                {
                    row->resize(cols);
                }
            }
        }

        GyrosMatrix::GyrosMatrix(std::vector<std::vector<double> >& matrix, bool text, std::vector<int> labelRows, std::vector<int> labelCols) :
        title(""),
        diagonal(false),
        rows(matrix.size()),
        cols((matrix.size() > 0 ? matrix[0].size() : 0)),
        labelCols(labelCols),
        labelRows(labelRows),
        text(text)
        {
            int rowCount = 0;
            for (std::vector<std::vector <double> >::iterator row = matrix.begin(); row != matrix.end(); row++)
            {
                data.push_back(std::vector<std::string > ());
                int colCount = 0;
                for (std::vector<double>::iterator cell = row->begin(); cell != row->end(); cell++)
                {
                    //stop adding cols if too many are passed. (possible since matrix can be jagged and first row defines column count)
                    if (colCount >= cols)
                        break;
                    std::stringstream buffer;
                    buffer << *cell;
                    data[rowCount].push_back(buffer.str());
                    colCount++;
                }
                if (colCount < cols)
                { //add additional cols if too few are passed (possible since matrix can be jagged and first row defines column count)
                    data[rowCount].insert(data[rowCount].end(), (cols - colCount), std::string("null"));
                }
                rowCount++;
            }
        }

        GyrosMatrix::GyrosMatrix(const GyrosMatrix& orig) :
        title(orig.title),
        diagonal(orig.diagonal),
        rows(orig.rows),
        cols(orig.cols),
        labelCols(orig.labelCols),
        labelRows(orig.labelRows),
        data(orig.data),
        text(orig.text)
        {
        }

        GyrosMatrix::~GyrosMatrix()
        {
        }

        template<>
        int GyrosMatrix::GetFullData<std::string>(std::vector<std::vector<std::string> >& dataHold)
        {
            int pushedRows = 0, retVal = 0;
            if (data.size() > 0)
            {
                dataHold.clear();
                for (int row = 0; row < rows; row++)
                {
                    dataHold.push_back(std::vector<std::string> ());
                    for (int column = 0; column < cols; column++)
                    {
                        dataHold[row].push_back(data[row][column]);
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

        template<>
        int GyrosMatrix::GetData<std::string>(std::vector<std::vector<std::string> >& pureData)
        {
            int pushedRows = 0, retVal = 0;
            if (data.size() > 0)
            {
                pureData.clear();
                for (int row = 0; row < rows; row++)
                {
                    if (std::find(labelRows.begin(), labelRows.end(), row) == labelRows.end())
                    {
                        pureData.push_back(std::vector<std::string> ());
                        for (int column = 0; column < cols; column++)
                        {
                            if (std::find(labelCols.begin(), labelCols.end(), column) == labelCols.end())
                            {
                                pureData[pushedRows].push_back(data[row][column]);
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
            if (pushedRows == 0)
            {
                retVal = -2;
            }
            return retVal;
        }

        std::vector<std::vector<std::string> > GyrosMatrix::GetLabelCols()
        {
            std::vector<std::vector<std::string> > labelColsReturn;
            for(int column=0; column<cols; column++)
            {
                if (std::find(labelCols.begin(), labelCols.end(), column) != labelCols.end())
                {
                    std::vector<std::string> labelCol;
                    for(int row=0; row<rows; row++)
                    {
                        labelCol.push_back(data[row][column]);
                    }
                    labelColsReturn.push_back(labelCol);
                }
            }
            return labelColsReturn;
        }

        std::vector<std::vector<std::string> > GyrosMatrix::GetLabelRows()
        {
            std::vector<std::vector<std::string> > labelRowsReturn;
            for(int row=0; row<rows; row++)
            {
                if (std::find(labelRows.begin(), labelRows.end(), row) != labelRows.end())
                {
                    std::vector<std::string> labelRow;
                    for(int column=0; column<cols; column++)
                    {
                        labelRow.push_back(data[row][column]);
                    }
                    labelRowsReturn.push_back(labelRow);
                }
            }
            return labelRowsReturn;
        }

        void GyrosMatrix::InsertData(GyrosMatrix tabletoInsert, int insertionRow, int insertionColumn, bool horizontalExpand, bool verticalExpand, std::string pad)
        {
            std::vector<std::vector <std::string> > dataToInsert, originalData;
            tabletoInsert.GetFullData(dataToInsert);
            int rowsToInsert, colsToInsert;
            rowsToInsert=dataToInsert.size();
            colsToInsert = (dataToInsert.size() > 0 ? dataToInsert[0].size() : 0);
            if (rowsToInsert > 0 && colsToInsert > 0)
            {
                int beginRow = (insertionRow > 0 ? 0 : insertionRow);
                int endRow = 0;
                if (verticalExpand)
                {
                    if (insertionRow >= 0)
                    {
                        endRow = (((rows + rowsToInsert - 1) > (insertionRow + rowsToInsert - 1)) ? (rows + rowsToInsert - 1) : (insertionRow + rowsToInsert - 1));
                    }
                    else
                    {
                        endRow = (((rows + rowsToInsert + insertionRow - 1) > (rows - 1)) ? (rows + rowsToInsert + insertionRow - 1) : (rows - 1));
                    }
                }
                else
                {
                    endRow = (((rows - 1) > (insertionRow + rowsToInsert - 1)) ? (rows - 1) : (insertionRow + rowsToInsert - 1));
                }

                int beginCol = (insertionColumn > 0 ? 0 : insertionColumn);
                int endCol = 0;

                if (horizontalExpand)
                {
                    if (insertionColumn >= 0)
                    {
                        endCol = (((cols + colsToInsert - 1) > (insertionColumn + colsToInsert - 1)) ? (cols + colsToInsert - 1) : (insertionColumn + colsToInsert - 1));
                    }
                    else
                    {
                        endCol = (((cols + colsToInsert + insertionColumn - 1) > (cols - 1)) ? (cols + colsToInsert + insertionColumn - 1) : (cols - 1));
                    }
                }
                else
                {
                    endCol = (((cols - 1) > (insertionColumn + colsToInsert - 1)) ? (cols - 1) : (insertionColumn + colsToInsert - 1));
                }

                originalData = data;

                data.clear();

                enum writingModeEnum
                {
                    empty, newOnly, origOnly, both
                };

                writingModeEnum writingMode = empty;

                int origGyrosMatrixRow = 0, newGyrosMatrixRow = 0;
                for (int row = beginRow; row <= endRow; row++)
                {
                    writingMode = empty;
                    int origGyrosMatrixCol = 0, newGyrosMatrixCol = 0;
                    std::vector<std::string> newRow;
                    if(row>=0 && origGyrosMatrixRow<rows && (!verticalExpand || (verticalExpand && (row<insertionRow || row>=insertionRow+rowsToInsert))))
                    {//this row includes data from old table
                        writingMode = origOnly;
                    }
                    if(row>=insertionRow && row<insertionRow+rowsToInsert)
                    {//this row includes data from new table
                        if(writingMode==origOnly)
                        {
                            writingMode=both;
                        }
                        else
                        {
                            writingMode = newOnly;
                        }
                    }
                    for (int column = beginCol; column <= endCol; column++)
                    {
                        switch (writingMode)
                        {
                            case empty:
                            {
                                newRow.push_back(pad);
                            }
                                break;
                            case origOnly:
                            {
                                if((!horizontalExpand || column<insertionColumn || column>=insertionColumn+colsToInsert) && column>=0 && origGyrosMatrixCol<cols)
                                {//inside orig table
                                    
                                    newRow.push_back(originalData[origGyrosMatrixRow][origGyrosMatrixCol++]);
                                }
                                else
                                {
                                    newRow.push_back(pad);
                                }
                            }
                                break;
                            case newOnly:
                            {
                                if(column>=insertionColumn && column<insertionColumn+colsToInsert)
                                {//inside new
                                    newRow.push_back(dataToInsert[newGyrosMatrixRow][newGyrosMatrixCol++]);
                                }
                                else
                                {
                                    newRow.push_back(pad);
                                }
                            }
                                break;
                            case both:
                            {
                                if((column<insertionColumn || column>=insertionColumn+colsToInsert) && column>=0 && origGyrosMatrixCol<cols)
                                {//outside new table, inside old
                                    newRow.push_back(originalData[origGyrosMatrixRow][origGyrosMatrixCol++]);
                                }
                                else if(column>=insertionColumn && column<insertionColumn+colsToInsert)
                                {//inside new one
                                    newRow.push_back(dataToInsert[newGyrosMatrixRow][newGyrosMatrixCol++]);
                                    if(!horizontalExpand && column>=0 && origGyrosMatrixCol<cols)
                                    {//overlap and no expansion - skip a cell from orig
                                        origGyrosMatrixCol++;
                                    }
                                }
                                else
                                {//outside both
                                    newRow.push_back(pad);
                                }
                            }
                                break;
                            default:
                                break;
                        }

                    }
                    data.push_back(newRow);
                    if(origGyrosMatrixCol!=0)
                    {
                        origGyrosMatrixRow++;
                    }
                    if(newGyrosMatrixCol!=0)
                    {
                        newGyrosMatrixRow++;
                    }
                }

                int horWidening, verWidening;

                verWidening = 1 + endRow - beginRow - rows;
                horWidening = 1 + endCol - beginCol - cols;

                rows = 1 + endRow - beginRow;
                cols = 1 + endCol - beginCol;

                if (horWidening>0)
                {
                    for (std::vector<int>::iterator labelCol = labelCols.begin(); labelCol != labelCols.end(); labelCol++)
                    {
                        if ((*labelCol) >= insertionColumn)
                        {
                            (*labelCol) += horWidening;
                        }
                    }
                }

                if (verWidening>0)
                {
                    for (std::vector<int>::iterator labelRow = labelRows.begin(); labelRow != labelRows.end(); labelRow++)
                    {
                        if ((*labelRow) >= insertionRow)
                        {
                            (*labelRow) += verWidening;
                        }
                    }
                }
            }
        }

        std::string GyrosMatrix::ToGyrosXML()
        {
            std::stringstream gyrosXML;
            gyrosXML << "<matrix";
            if (!title.empty())
            {
                gyrosXML << " title=\"" << title << "\"";
            }
            gyrosXML << ">";
            if (labelCols.size() > 0 || labelRows.size() > 0)
            {
                gyrosXML << "<labels>";
                for (std::vector<int>::iterator it = labelCols.begin(); it != labelCols.end(); it++)
                {
                    gyrosXML << "<column>" << *it << "</column>";
                }
                for (std::vector<int>::iterator it = labelRows.begin(); it != labelRows.end(); it++)
                {
                    gyrosXML << "<row>" << *it << "</row>";
                }
                gyrosXML << "</labels>";
            }
            gyrosXML << "<data rows=\"" << rows << "\" columns=\"" << cols << "\"";
            if (text)
            {
                gyrosXML << " text=\"true\"";
            }

            if (diagonal)
            {
                gyrosXML << " diagonal=\"true\">";
                for (int rowCol = 0; rowCol < rows; rowCol++)
                {
                    gyrosXML << data[rowCol][rowCol];
                    if (rowCol != rows - 1)
                    {
                        gyrosXML << "|";
                    }
                }
            }
            else
            {
                gyrosXML << ">";
                for (int i = 0; i < rows; i++)
                {
                    for (int j = 0; j < cols; j++)
                    {
                        gyrosXML << data[i][j];
                        if (i != rows - 1 || j != cols - 1)
                        {
                            gyrosXML << "|";
                        }
                    }
                }
            }
            gyrosXML << "</data></matrix>";
            return gyrosXML.str();
        }       
    }
}


