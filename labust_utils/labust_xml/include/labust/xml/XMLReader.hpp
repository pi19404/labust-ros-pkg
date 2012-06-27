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
#ifndef XMLREADER_HPP_
#define XMLREADER_HPP_
#include <labust/xml/xmlfwd.hpp>
#include <labust/xml/XMLException.hpp>

#include <boost/type_traits/is_float.hpp>
#include <boost/type_traits/is_integral.hpp>
#include <boost/type_traits/is_enum.hpp>
#include <boost/mpl/or.hpp>

#include <string>
#include <sstream>

namespace labust
{
  /**
   * Namespace for XML related classes. Use <labust/xml/xmlfwd.hpp> for the forward
   * declaration of this library.
   */
  namespace xml
  {
    /**
     * The labust::xml::Reader reads a xml formated string or file into the memory.
     * The XML document can be searched using XPath expressions. Template methods enable
     * type-safe decoding of the XML element values.
     *
     * For built-in types, strings and similar smaller objects you can use return-by-value
     * methods. For larger types pass the memory address where to decode the message.
     * The class uses string streams for reading data into objects. If you wish to use your
     * own objects with the labust::xml::Reader be sure to specify the usual operator>> for
     * your class. Alternatively, specify the labust::xml::Reader& operator>>(labust::xml::Reader&, ...).
     *
     * \todo Check for const-correctness.
     * \todo Switch try_* functions to boolean.
     * \todo Add method hasNode() to test if a node exists (it can be empty).
     * \todo Extract the throw and nothrow into policies.
     * \todo Allow for policies optimizing during conversions.
     * \todo Handle xmlNode* and xmlNodeSets to not throw exceptions.
     */
    class Reader
    {
    public:
      /**
       * Main constructor that instantiates the labust::xml::Reader object.
       * The methods throws a labust::xml::XMLException.
       *
       * \param xmlData The string or path that contains the encoded XML.
       * \param isFile Indicate whether to open a file or use the string directly.
       */
      Reader(const std::string& xmlData, bool isFile = false);
      /**
       * Constructor that instantiates the labust::xml::Reader object and selects a subnode.
       * The methods throws a labust::xml::XMLException.
       *
       * \param xmlData The string or path that contains the encoded XML.
       * \param subNode The node that will be set as the root node.
       * \param isFile Indicate whether to open a file or use the string directly.
       */
      Reader(const std::string& xmlData, const std::string& subNode, bool isFile = false);
      /**
       * Generic destructor.
       */
      ~Reader();

      /**
       * The method sets the current XPath context node to the desired value. It
       * return the unset root node. Useful when you want to return the object in
       * the original state.
       *
       * \param node The desired XPath node to be used as the root node.
       *
       * \return Returns the unset root node pointer.
       */
      _xmlNode* useNode(_xmlNode* node);
      /**
       * The method sets the XPath context node to the original root node pointer.
       */
      void useRootNode();
      /**
       * The method return the current XPath context root node.
       *
       * \return The current XPath context root node.
       */
      _xmlNode* currentNode() const;

      /**
       * The method evaluates the given XPath expression and returns the content. If the evaluation
       * fails or it evaluates to a node it will throws a labust::xml::XMLException.
       *
       * \param xpath_expression The desired expression to evaluate.
       *
       * \return Return the evaluated node content.
       */
      const char* evaluate(const char* xpath_expression) const;
      /**
       * The method evaluates the given XPath expression and returns the content. If the evaluation
       * fails or it evaluates to a node it will return false.
       *
       * \param xpath_expression The desired expression to evaluate.
       * \param content The content of the node in a constant string expression.
       * \return Return true if node found, false otherwise.
       */
      bool evaluate(const char* xpath_expression, const char** content) const;

      /**
       * The method tests if the specified XPath expression can be evaluated. Useful if you want to
       * test if a node exists.
       *
       * \param xpath_expression The XPath expression.
       *
       * \return Returns true if the XPath expression was evaluated successfully and false otherwise.
       */
      bool try_expression(const std::string& xpath_expression) const
      {
        try
        {
          this->evaluate(xpath_expression.c_str());
          return true;
        }
        catch (XMLException&){};
        return false;
      }
      /**
       * Evaluates the XPath expression and returns the desired value. If the expression evaluation
       * fails it return false.
       * Useful to avoid try-catch phrases when multiple optional parameters exist.
       *
       * \param xpath_expression The XPath expression.
       * \param data Address of the return object.
       *
       * \tparam ReturnType Adjusts to the desired return type.
       *
       * \return False if evaluation failed, true otherwise.
       */
      template <typename ReturnType>
      inline bool try_value(const std::string& xpath_expression, ReturnType* const data) const
      {
      	typedef typename boost::mpl::or_<boost::is_float<ReturnType>, boost::is_integral<ReturnType>, boost::is_enum<ReturnType> > decision_type;
      	return this->cvalue(xpath_expression, data, decision_type());
      }

      /**
       * Evaluates the XPath expression and returns the desired value. If the expression evaluation
       * fails it throws a XMLException.
       *
       * \param xpath_expression The XPath expression.
       * \param data Address of the return object.
       * \param noThrow Does not throw a exception when evaluation fails.
       *
       * \tparam ReturnType Adjusts to the desired return type.
       *
       * \return True if expression evaluated successfully.
       */
      template <typename ReturnType>
      inline void value(const std::string& xpath_expression, ReturnType* const data) const
      {
      	if (!this->try_value(xpath_expression,data)) throw XMLException(lastErrorMessage);
      }
      /**
       * Evaluates the XPath expression and returns the desired value. If the expression evaluation
       * fails it throws a labust::xml::XMLException. Sometimes it is convinient to directly read the
       * value and return by-value. Use this method with smaller and built-in types.
       *
       * \param xpath_expression The XPath expression.
       *
       * \tparam ReturnType xpath_expression The xpath expression to the node which value you want
       *
       * \return Returns the desired value directly By-value. Inefficient with large classes.
       */
      template <typename ReturnType>
      inline ReturnType value(const std::string& xpath_expression) const
      {
      	typedef typename boost::mpl::or_<boost::is_float<ReturnType>, boost::is_integral<ReturnType>, boost::is_enum<ReturnType> > decision_type;
      	return this->cvalue<ReturnType>(xpath_expression, decision_type());
      }

			/**
			 * The operator can be used to read parameters in conjunction with setParamName and
			 * setTypeName functions. The data can be called as:
			 * 	writer.setParamName("name").setParamType("")>>object;
			 * The reader assumes the parameters are written as: <param name="name" value="objectValue" [type="type"] />
			 * The type specifier is optional and will not be used for empty types.
			 * If no paramName is specified the only the plain value will be written. This can be also
			 * achieved with: writer.plainValue()<<object
			 *
			 * \param value The value to be written.
			 *
			 * \tparam Value The template parameter to enable polymorphism.
			 * \return The writer object for chaining.
			 */
			template <class Value>
			const Reader& operator>>(Value& value) const
			{
				if (nextParamType.empty())
				{
					this->value("param[@name='"+this->nextParamName+"']/@value",&value);
				}
				else
				{
					this->value("param[@name='"+this->nextParamName+"' and @type='"+this->nextParamType+"']/@value",&value);
				}
				return *this;
			}
			/**
			 * Specifies the next parameter name to be used with the left shift operator.
			 *
			 * \param name The next parameter name.
			 * \return The writer object for chaining.
			 */
			inline Reader& setParamName(const std::string& name){this->nextParamName = name; return *this;}
			/**
			 * Specifies the next parameter type to be used with the left shift operator.
			 *
			 * \param type The next parameter type.
			 * \return The writer object for chaining.
			 */
			inline Reader& setParamType(const std::string& name){this->nextParamType = name; return *this;}

    private:
      /**
       * The method perform XML document instantiation. If the instantiation fails it throws a
       * labust::xml::XMLException.
       *
       * \param xmlData The string or path that contains the encoded XML.
       * \param isFile Indicate whether to open a file or use the string directly.
       */
      void initDoc(const std::string& xmlData, bool isFile);

      /**
       * The method evaluates the XPath expression and returns the pointer vector to all
       * all the nodes that match the expression.
       *
       * \param xpath_expression The XPath expression.
       */
      bool evaluate2nodeset(const char* xpath_expression, NodeCollectionPtr& nodeCollection) const;
      /**
       * The method returns the content of the given node. Throws a XMLException if unsuccessful.
       *
       * \param xml_node The pointer to the XML node.
       *
       * \return Pointer to the extracted string value.
       */
      const char* content(const _xmlNode*  const xml_node) const;
      /**
       * The method returns the content of the given node.
       *
       * \param xml_node The pointer to the XML node.
       * \param content The extracted content in string value.
       * \param doThrow Throw XML exception.
       *
       * \return Pointer to the extracted string value.
       */
      bool content(const _xmlNode*  const xml_node, const char** content) const;

      /**
       * The generic converter method. Uses a string stream to convert the evaluated value.
       *
       * \param xpath_expression The XPath expression.
       * \param data Address of the return object.
       *
       * \tparam ReturnType Adjusts to the desired return type.
       */
      template<typename ReturnType>
      bool cvalue(const std::string& xpath_expression, ReturnType* const data, boost::mpl::false_) const
      {
      	const char* content(0);
      	if (this->evaluate(xpath_expression.c_str(), &content))
      	{
      		std::stringstream out(content);
      		out>>(*data);
      		return true;
      	}
      	return false;
      }
      /**
       * The converter specialization for numerics. Uses a ASCII to number converter.
       *
       * \param xpath_expression The XPath expression.
       * \param data Address of the return object.
       *
       * \tparam ReturnType Adjusts to the desired return type.
       */
      template<typename ReturnType>
      bool cvalue(const std::string& xpath_expression, ReturnType* const data, boost::mpl::true_) const
      {
      	const char* content(0);

      	if (this->evaluate(xpath_expression.c_str(), &content))
      	{
      		if (boost::is_float<ReturnType>::value)
      		{
      			(*data) = static_cast<ReturnType>(atof(content)); //added explicit casts to handle enums correctly
      			return true;
      		}

      		if (boost::is_integral<ReturnType>::value || boost::is_enum<ReturnType>::value)
      		{
      			(*data) = static_cast<ReturnType>(atoi(content));
      			return true;
      		}
      	}
      	return false;
      }
      /**
       * The generic converter method. Uses a string stream to convert the evaluated value.
       *
       * \param xpath_expression The XPath expression.
       *
       * \tparam ReturnType Adjusts to the desired return type.
       */
      template <typename ReturnType>
      inline ReturnType cvalue(const std::string& xpath_expression, boost::mpl::false_) const
      {
      	ReturnType temp;
        std::stringstream out(this->evaluate(xpath_expression.c_str()));
        out>>temp;
        return temp;
      }
      /**
       * The converter specialization for numerics. Uses a ASCII to number converter.
       *
       * \param xpath_expression The XPath expression.
       *
       * \tparam ReturnType Adjusts to the desired return type.
       */
      template <typename ReturnType>
      inline ReturnType cvalue(const std::string& xpath_expression, boost::mpl::true_) const
      {
      	//Internal specialization for floating point types.
      	if (boost::is_float<ReturnType>::value && !boost::is_enum<ReturnType>::value)
      	{
      		return static_cast<ReturnType>(atof(this->evaluate(xpath_expression.c_str()))); //added explicit cast to handle enums correctly
      	}
      	else
      	{
      		return static_cast<ReturnType>(atoi(this->evaluate(xpath_expression.c_str())));
      	}
      }

      /**
       * Pointer to the loaded XML document.
       */
      xmlDocPtr document;
      /**
       * Pointer to the XPath context.
       */
      xmlXPathContextPtr xpath_context;
      /**
       * The pointer to the XPath expression evaluation.
       */
      mutable xmlXPathObjectPtr xpath_object;
      /**
       * Pointer to the root node of the XPath context.
       */
      mutable xmlNodePtr root_node;
			/**
			 * The next parameter and type name.
			 */
			std::string nextParamName, nextParamType;
			/**
			 * The last error message.
			 */
			mutable std::string lastErrorMessage;
    };

    /**
     * Specialization for returning XML node pointers. Useful for retrieving a desired node group
     * under which we can do relative searches. If the expression evaluation fails it throws a
     * labust::xml::XMLException.
     *
     * \param xpath_expression The XPath expression.
     * \param data Address of the return object.
     */
    template <>
    inline bool Reader::try_value<_xmlNode*>(const std::string& xpath_expression, _xmlNode** const data) const
    {
    	NodeCollectionPtr collection;
    	this->evaluate2nodeset(xpath_expression.c_str(), collection);
    	if (collection != 0)
    	{
    		*data = collection->at(0);
    		return true;
    	}
    	return false;
    }
    /**
     * Specialization for returning XML node pointers. Useful for retrieving a desired node group
     * under which we can do relative searches. If the expression evaluation fails it throws a
     * labust::xml::XMLException.
     *
     * \param xpath_expression The XPath expression.
     *
     * \return Returns a xmlNode pointer. Do not delete this pointer.
     */
    template <>
    inline _xmlNode* Reader::value<_xmlNode*>(const std::string& xpath_expression) const
		{
    	NodeCollectionPtr collection;
    	this->evaluate2nodeset(xpath_expression.c_str(), collection);
    	if (collection != 0)
    	{
    		return collection->at(0);
    	}
    	else
    	{
    		throw XMLException(lastErrorMessage);
    	}
		}

    /**
     * Specialization for strings. Useful when a string contains whitespace characters. If the
     * expression evaluation fails it throws a labust::xml::XMLException.
     *
     * \param xpath_expression The XPath expression.
     * \param data Address of the return object.
     */
    template <>
    inline bool Reader::try_value<std::string>(const std::string& xpath_expression, std::string* const data) const
    {
    	const char* content(0);
      if (this->evaluate(xpath_expression.c_str(),&content))
      {
      	*data = content;
      	return true;
      }
      return false;
    }
    /**
     * Specialization for strings. Useful when a string contains whitespace characters. If the
     * expression evaluation fails it throws a labust::xml::XMLException.
     *
     * \param xpath_expression The XPath expression.
     *
     * \return Returns a xmlNode pointer. Do not delete this pointer.
     */
    template <>
    inline std::string Reader::value<std::string>(const std::string& xpath_expression) const
		{
    	return this->evaluate(xpath_expression.c_str());
		}
    /**
     * The method returns all the node pointers that match the evaluated XPath expression.
     *
     * \param xpath_expression The XPath expression.
     *
     * \return Pointer to the vector of XML nodes.
     */
    template <>
    inline NodeCollectionPtr Reader::value<NodeCollectionPtr>(const std::string& xpath_expression) const
    {
    	NodeCollectionPtr collection;
    	this->evaluate2nodeset(xpath_expression.c_str(),collection);
    	if (collection != 0)
    	{
    		return collection;
    	}
    	else
    	{
    		throw XMLException(lastErrorMessage);
    	}
    }
    /**
     * The method returns all the node pointers that match the evaluated XPath expression.
     *
     * \param xpath_expression The XPath expression.
     *
     * \return Pointer to the vector of XML nodes.
     */
    template <>
    inline bool Reader::try_value<NodeCollectionPtr>(const std::string& xpath_expression, NodeCollectionPtr* nodes) const
    {
    	this->evaluate2nodeset(xpath_expression.c_str(),*nodes);
    	if ((*nodes) != 0)
    	{
    		return true;
    	}
    	return false;
    }
  }
}
/* XMLREADER_HPP_ */
#endif
