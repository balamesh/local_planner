/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Nicolas Limpert
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Nicolas Limpert
*********************************************************************/
#ifndef LOCAL_PLANNER_
#define LOCAL_PLANNER_

//#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <twist.h>
#include <posestamped.h>
#include <string>
#include <thread>
//#include <costmap_2d/costmap_2d_ros.h>
//#include <tf/transform_listener.h>

namespace local_planner {
  /**
   * @class BaseLocalPlanner
   * @brief Provides an interface for local planners used in navigation. All local planners written as plugins for the navigation stack must adhere to this interface.
   */
  class LocalPlanner{
    public:
      bool set_plan(const std::vector<PoseStamped>& plan);
      bool set_planner(BasePlanner* planner);
      bool set_executor(BaseExecutor* planner);
      bool get_planners(std::vector<std::String>& planners);
      bool get_executors(std::vector<std::String>& executors);
      bool execute_plan();
      bool pause();
      bool cancel();

      ~LocalPlanner(){}

    protected:

      LocalPlanner(){}

    private:
      std::map<std::String, BasePlanner> planners;
      std::map<std::String, BaseExecutors> executors;
      std::thread * act_thread;

      bool is_paused;
      bool is_cancelled;
  };
};

#endif
