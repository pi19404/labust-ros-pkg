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
#ifndef SO_LOADER_HPP_
#define SO_LOADER_HPP_
#include <string>
#include <stdexcept>

#include <dlfcn.h>

namespace labust
{
  namespace plugins
  {
    /**
     * This is class load the DLL library and becomes a factory for instance creation.
     */
    template <class PluginFactory>
    class DLLoad : public PluginFactory::FactoryLoader
    {
      typedef typename PluginFactory::FactoryLoader::FactoryCreator FactoryCreator;
    public:
      /**
       * Main constructor. Loads the specified DLL and extracts the creator function.
       */
      DLLoad(const std::string& pluginName, std::string creatorName = "createInstance")
      {
        libraryHandle = dlopen(("lib" + pluginName + ".so").c_str(),RTLD_NOW);
        const char* error = dlerror();
        if (libraryHandle)
        {
          this->factory = reinterpret_cast<FactoryCreator>(dlsym(libraryHandle,creatorName.c_str()));
          if ((error = dlerror()) != NULL)
          {
            throw std::runtime_error(error);
          }
        }
        else
        {
          throw std::runtime_error(error);
        }
      };

      ~DLLoad()
      {
        if (libraryHandle)
        {
          dlclose(libraryHandle);
        }
      };

    private:
      /**
       * Loaded library handle.
       */
      void* libraryHandle;
    };
  }
}
/* SO_LOADER_HPP_ */
#endif