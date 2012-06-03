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
#ifndef PLUGINLOADER_HPP_
#define PLUGINLOADER_HPP_
#include <labust/xml/xmlfwd.hpp>

namespace labust
{
  namespace plugins
  {
		/**
		 * This class loads and configures the vehicle plugins.
		 *
		 * \todo Consider moving this into the drivers themself so that they hold a
		 * plugin reference.
		 * \todo Consider moving this into the TmplFactory for easiers adaptation.
		 */
  	template <class PluginPtr, class DriverPtr>
		class Loader
		{
		public:
			/**
			 * The method loads a vehicle driver and configures it with the given XML data.
			 *
			 * \param pluginName Name of the plugin to be loaded.
			 * \param reader The XML data for vehicle configuration.
			 */
			inline typename DriverPtr::element_type& loadPlugin(const std::string& pluginName, const std::string& factory, const labust::xml::ReaderPtr reader)
			{
				static const std::string FactoryName("create" + factory + "Factory");
				driver.reset();
				plugin.reset(new typename PluginPtr::element_type(pluginName,FactoryName));
				driver.reset((*plugin)(reader));

				return *driver;
			}

			/**
			 * The method loads a vehicle driver and configures it with the given XML data.
			 *
			 * \param pluginName Name of the plugin to be loaded.
			 * \param reader The XML data for vehicle configuration.
			 */
			inline typename DriverPtr::element_type& loadPlugin(const std::string& pluginName, const labust::xml::ReaderPtr reader)
			{
				return this->loadPlugin(pluginName,"",reader);
			}

			/**
			 * The vehicle reference.
			 */
			inline typename DriverPtr::element_type& operator()()
			{
				return *driver;
			}

		private:
			/**
			 * The plugin implementation pointer.
			 */
			PluginPtr plugin;
			/**
			 * The vehicle implementation pointer.
			 */
			DriverPtr driver;
		};

    /**
     * This function performs plugin loading based on a supplied XML configuration file.
     *
     * The plugin configuration that is expected is:
     *  <plugin name="plugin-name-plug" config_file="model.xml" name="falcon" />
     *
     *  name - is the physical name of the plugin without OS specific suffixes and prefixes
     *  config_file - (optional) defines the location of the vehicle configuration file
     *                if no file was supplied the XML document containing the plugin definition
     *                will be used.
     *  id - (optional) The configuration identification in the document. If none is supplied
     *       and empty string will be passed to the plugin configuration.
     *
     */
    /*template <class DLLoader>
    inline typename DLLoader::TypePtr createConfiguredInstance(const labust::xml::ReaderPtr reader, const DLLoader& dll)
    {
      std::string config_file(reader->try_expression("plugin/@config_file") ? (reader->value<std::string>("plugin/@config_file")) : "");
      std::string id(reader->try_expression("plugin/@id") ? (reader->value<std::string>("plugin/@id")) : "");

      if (config_file.empty())
      {
        return typename DLLoader::TypePtr(dll(reader,id));
      }
      else
      {
        return typename DLLoader::TypePtr(dll(labust::xml::ReaderPtr(new labust::xml::Reader(config_file,true)),id));
      }
    };*/
  }
}

/* PLUGINLOADER_HPP_ */
#endif
