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

#ifndef CONCURRENTQUEUE_HPP_
#define CONCURRENTQUEUE_HPP_
/*
 * ConcurrentQueue.hpp
 *
 *  Created on: Jan 11, 2011
 *      Author: dnad
 */
#include <boost/thread/mutex.hpp>
#include <queue>

namespace LABUST
{
  /**
   * This concurrent queue implementation based on BOOST is taken from:
   *  "http://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html"
   * We have expanded it with some additional methods.
   * This queue is intended to be used in a multi-threaded environment for passing data
   * between threads.
   */
  template<typename Data>
  class ConcurrentQueue
  {
  public:
    /**
     * Main constructor. Set the default maximum size of elements to 1000.
     *
     * \param max_elements Maximum number of elements the queue should hold.
     */
    ConcurrentQueue(size_t max_elements = 1000):max_elements(max_elements){};

    /**
     * Pushes the element on to a queue. If the number of elements in the queue
     * is at the maximum it will pop the last element from the queue before the
     * push.
     *
     * \param data Element to be pushed onto the queue.
     */
    void push(Data const& data)
    {
      boost::mutex::scoped_lock lock(the_mutex);
      if (the_queue.size() > max_elements)
      {
        the_queue.pop();
      }
      the_queue.push(data);
      lock.unlock();
      the_condition_variable.notify_one();
    }
    /**
     * Pushes the element on to a queue. If the number of elements in the queue
     * is at the maximum it will block until there is room to push a new element
     * onto a queue.
     *
     * \param data Element to be pushed onto the queue.
     */
    void block_push(Data const& data)
    {
      boost::mutex::scoped_lock lock(the_mutex);
      while(the_queue.size() > max_elements)
      {
        push_condition.wait(lock);
      }

      the_queue.push(data);
      lock.unlock();
      the_condition_variable.notify_one();
    }
    /**
     * Tries to pushes the element on to a queue.
     *
     * If the number of elements in the queue
     * is at the maximum it will block until there is room to push a new element
     * onto a queue.
     *
     * \param data Element to be pushed onto the queue.
     * \return False if there is no room to perform the push, true if the push
     * was successful.
     */
    bool try_push(Data const& data)
    {
      boost::mutex::scoped_lock lock(the_mutex);
      if (the_queue.size()>max_elements)
      {
        return false;
      }

      the_queue.push(data);
      lock.unlock();
      the_condition_variable.notify_one();
    }
    /**
     * Empties the queue.
     */
    bool empty() const
    {
      boost::mutex::scoped_lock lock(the_mutex);
      return the_queue.empty();
    }
    /**
     * Tries to pop the object from the queue.
     *
     * \param popped_value Reference to the return object.
     * \return False if there are no object to pop, true if the pop was successful.
     */
    bool try_pop(Data& popped_value)
    {
      boost::mutex::scoped_lock lock(the_mutex);
      if(the_queue.empty())
      {
        return false;
      }

      popped_value=the_queue.front();
      the_queue.pop();
      lock.unlock();
      push_condition.notify_one();
      return true;
    }
    /**
     * Blocks until there is an element to pop and returns.
     *
     * \param popped_value Reference to the return object.
     */
    void wait_and_pop(Data& popped_value)
    {
      boost::mutex::scoped_lock lock(the_mutex);
      while(the_queue.empty())
      {
        the_condition_variable.wait(lock);
      }

      popped_value=the_queue.front();
      the_queue.pop();
      lock.unlock();
      push_condition.notify_one();
    }
    /**
     * \return The size of the queue.
     */
    size_t size()
    {
      boost::mutex::scoped_lock lock(the_mutex);
      return the_queue.size();
    }

  private:
    std::queue<Data> the_queue;
    mutable boost::mutex the_mutex;
    boost::condition_variable the_condition_variable;
    boost::condition_variable push_condition;
    size_t max_elements;
  };
}
/* CONCURRENTQUEUE_HPP_ */
#endif
