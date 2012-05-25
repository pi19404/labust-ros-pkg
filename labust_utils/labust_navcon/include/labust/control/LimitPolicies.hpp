#ifndef LIMITPOLICIES_HPP_
#define LIMITPOLICIES_HPP_
/*
 * LimitPolicies.hpp
 *
 *  Created on: Sep 1, 2011
 *      Author: dnad
 */
#include <labust/math/Limits.hpp>

namespace labust
{
  namespace control
  {
    /**
     * This class specifies the behaviour when we use I/O limits on an object.
     */
    class UseLimits
    {
    public:
      /**
       * This method sets the input output limits. Output limits are used for anti-windup.
       * Input limits act on the desired (reference) value.
       *
       * \param limits The desired input-output limits
       */
      inline void setLimits(const labust::math::Limit<double>& limits){this->limits = limits;};

    protected:
      /**
       * I/O limits.
       */
      labust::math::Limit<double> limits;
      /**
       * Private destructor
       */
      ~UseLimits(){};
    };

    /**
     * This class specifies behaviour when we do not use I/O limits.
     */
    struct NoLimits{protected:~NoLimits(){};};
  }
};
/* LIMITPOLICIES_HPP_ */
#endif
