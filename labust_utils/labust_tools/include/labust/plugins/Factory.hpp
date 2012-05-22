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
#ifndef FACTORY_HPP_
#define FACTORY_HPP_

#include <boost/preprocessor/repetition.hpp>

namespace labust
{
  namespace plugins
  {
		#define PARAM_NUM 2
    /**
     * Plugin factory template defines the AbstractFactory class that can be used
     * polymorphically and also creates a template class, Impl, that can be used for
     * implementation of the AbstractFactory class. The FactoryLoader class is used
     * to adjust the DLLoad interface accordingly to the plugin type that is expected.
     * If needed this can be expanded to implement factories that create multiple objects.
     */
    template <class Type, BOOST_PP_ENUM_PARAMS_WITH_A_DEFAULT(PARAM_NUM, class T, void)>
    struct TmplPluginFactory
    {
      typedef Type* TypePtr;
      /**
       * This is the AbstractFactory which implementations is expected to be in the plugin.
       */
      struct AbstractFactory
      {
        virtual ~AbstractFactory(){};
        virtual TypePtr operator()(BOOST_PP_ENUM_PARAMS(PARAM_NUM, T)) const = 0;
      };

      /**
       * This is the implementation helper class. It is added for convenience.
       */
      template <class Object>
      struct Impl : virtual AbstractFactory
      {
        virtual ~Impl(){};
        virtual TypePtr operator()(BOOST_PP_ENUM_BINARY_PARAMS(PARAM_NUM, T, U)) const
        {
          return new Object(BOOST_PP_ENUM_PARAMS(PARAM_NUM, U));
        };
      };

      /**
       * This is the interface adjustor for the DLLoad class. It is added for convenience.
       */
      struct FactoryLoader
      {
        typedef AbstractFactory* AbstractFactoryPtr;
        typedef AbstractFactoryPtr(*FactoryCreator)();
        typedef Type* TypePtr;

        inline TypePtr operator()(BOOST_PP_ENUM_BINARY_PARAMS(PARAM_NUM, T, U)) const
        {
          return (*factory())(BOOST_PP_ENUM_PARAMS(PARAM_NUM, U));
        };

        inline AbstractFactoryPtr getFactory()
        {
          return factory();
        };

      protected:
        FactoryCreator factory;
      };
    };

    //This macro actually creates template<class ReturnType, classT0, ..., class Tn> template based on the
    //number n. These are all specializations of the above class.

#define VOID_print(z,n,data) data

#define FACTORY_specialization(z,n,unused)                                              \
    template <class Type BOOST_PP_COMMA_IF(n) BOOST_PP_ENUM_PARAMS(n, class T)>         \
    struct TmplPluginFactory<Type,                                                      \
    BOOST_PP_ENUM_PARAMS(n, T)                                                          \
    BOOST_PP_COMMA_IF(n)                                                                \
    BOOST_PP_ENUM(BOOST_PP_SUB(PARAM_NUM,n), VOID_print, void)>                         \
    {                                                                                   \
      typedef Type* TypePtr;                                                            \
                                                                                        \
      struct AbstractFactory                                                            \
      {                                                                                 \
        virtual ~AbstractFactory(){};                                                   \
        virtual TypePtr operator()(BOOST_PP_ENUM_PARAMS(n, T)) const = 0;               \
      };                                                                                \
                                                                                        \
      template <class Object>                                                           \
      struct Impl : virtual AbstractFactory                                             \
      {                                                                                 \
        virtual ~Impl(){};                                                              \
        virtual TypePtr operator()(BOOST_PP_ENUM_BINARY_PARAMS(n, T, U)) const          \
        {                                                                               \
          return new Object(BOOST_PP_ENUM_PARAMS(n, U));                                \
        };                                                                              \
      };                                                                                \
                                                                                        \
      struct FactoryLoader                                                              \
      {                                                                                 \
        typedef AbstractFactory*(*FactoryCreator)();                                      \
        typedef AbstractFactory* AbstractFactoryPtr;                                    \
                                                                                        \
        inline TypePtr operator()(BOOST_PP_ENUM_BINARY_PARAMS(n, T, U)) const           \
        {                                                                               \
          return (*factory())(BOOST_PP_ENUM_PARAMS(n, U));                              \
        };                                                                              \
                                                                                        \
        inline AbstractFactoryPtr getFactory()                                          \
        {                                                                               \
          return factory();                                                             \
        };                                                                              \
      protected:                                                                        \
        FactoryCreator factory;                                                         \
      };                                                                                \
    };

    BOOST_PP_REPEAT(PARAM_NUM, FACTORY_specialization,~)

#undef FACTORY_specialization
#undef PARAM_NUM
#undef VOID_print
  }
}
/* FACTORY_HPP_ */
#endif
