/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 * Author: TKruse
 *********************************************************************/

#include <additional_dwa_plugins/jerk_cost_function.h>
#include <angles/angles.h>
using base_local_planner::Trajectory;

namespace dwa_plugins {

void JerkCostFunction::initialize(std::string name, base_local_planner::LocalPlannerUtil *planner_util) {
    TrajectoryCostFunction::initialize(name, planner_util);

    ros::NodeHandle nh("~/" + name_);
    nh.param("weight_x", xw_, 1.0);
    nh.param("weight_y", yw_, 1.0);
    nh.param("weight_theta", tw_, 1.0);
    init_ = false;
}

bool JerkCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
  return true;
}


double JerkCostFunction::scoreTrajectory(Trajectory &traj) {
  if(!init_)
    return 0.0;
    
  return xw_ * fabs(last_x_ - traj.xv_) 
       + yw_ * fabs(last_y_ - traj.yv_) 
       + tw_ * fabs(last_theta_ - traj.thetav_);
}

void JerkCostFunction::debrief(base_local_planner::Trajectory &result)
{
    last_x_ = result.xv_;
    last_y_ = result.yv_;
    last_theta_ = result.thetav_;
    init_ = true;
}

} /* namespace dwa_local_planner */

PLUGINLIB_EXPORT_CLASS(dwa_plugins::JerkCostFunction, dwa_local_planner::TrajectoryCostFunction)
