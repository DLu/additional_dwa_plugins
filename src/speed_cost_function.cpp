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

#include <additional_dwa_plugins/speed_cost_function.h>
#include <angles/angles.h>
using base_local_planner::Trajectory;

namespace dwa_plugins {

void SpeedCostFunction::initialize(std::string name, base_local_planner::LocalPlannerUtil *planner_util) {
    TrajectoryCostFunction::initialize(name, planner_util);

    ros::NodeHandle nh("~/" + name_);
    nh.param("target_speed", target_speed_, 0.5);
}

bool SpeedCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
  return true;
}


double SpeedCostFunction::scoreTrajectory(Trajectory &traj) {
  if(traj.getPointsSize()==0)
    return 0.0;
    
  double px, py, pth;
  traj.getPoint(traj.getPointsSize()-1, px, py, pth);  
  
  double dist_to_goal = sqrt( pow(px-goal_x_,2) + pow(py-goal_y_,2) );
  
  if(dist_to_goal < target_speed_ * traj.getPointsSize() * traj.time_delta_){
    return 0.0;
  }
  
  double abs_speed = sqrt( pow(traj.xv_, 2) + pow(traj.yv_, 2) );

  return fabs(abs_speed - target_speed_);
}

void SpeedCostFunction::setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan, double goal_x, double goal_y)
{
    goal_x_ = goal_x;
    goal_y_ = goal_y;
}

} /* namespace dwa_local_planner */

PLUGINLIB_EXPORT_CLASS(dwa_plugins::SpeedCostFunction, dwa_local_planner::TrajectoryCostFunction)
