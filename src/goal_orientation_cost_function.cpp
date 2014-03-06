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

#include <additional_dwa_plugins/goal_orientation_cost_function.h>
#include <angles/angles.h>
using base_local_planner::Trajectory;

namespace dwa_plugins {

void GoalOrientationCostFunction::initialize(std::string name, base_local_planner::LocalPlannerUtil *planner_util) {
    TrajectoryCostFunction::initialize(name, planner_util);

    ros::NodeHandle nh("~/" + name_);
    nh.param("approach_dist", approach_dist_, 0.5);
}

bool GoalOrientationCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
  return true;
}


double GoalOrientationCostFunction::scoreTrajectory(Trajectory &traj) {
  if(traj.getPointsSize()==0)
    return 1.0;
    
  double sx, sy, sth, px, py, pth;
  traj.getPoint(0, sx, sy, sth);
  traj.getPoint(traj.getPointsSize()-1, px, py, pth);  
  
  double dist_to_goal = sqrt( pow(sx-goal_x_,2) + pow(sy-goal_y_,2) );
  double angle_to_goal = atan2(goal_y_ - py, goal_x_ - px);
  
  /*
  if(dist_to_goal < approach_dist_){
    double weight = (approach_dist_ - dist_to_goal) / approach_dist_;
    double target = angle_to_goal * (1.0 - weight) + goal_yaw_ * weight;
    return fabs(angles::shortest_angular_distance(target, pth));  
  }
  double aim_diff = fabs(angles::shortest_angular_distance(pth, angle_to_goal));
  return aim_diff;*/
  if(dist_to_goal>=approach_dist_){
    return 1.0;
  }
  double aim_diff = fabs(angles::shortest_angular_distance(pth, angle_to_goal));
  double scalar = aim_diff / M_PI;
  return scalar;
  
}

void GoalOrientationCostFunction::setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan, double goal_x, double goal_y)
{
  if(orig_global_plan.size()==0){
    return;
  }
  geometry_msgs::PoseStamped goal = orig_global_plan[ orig_global_plan.size() - 1];
  goal_x_ = goal.pose.position.x;
  goal_y_ = goal.pose.position.y;
  goal_yaw_ = tf::getYaw(goal.pose.orientation);
}

} /* namespace dwa_local_planner */

PLUGINLIB_EXPORT_CLASS(dwa_plugins::GoalOrientationCostFunction, dwa_local_planner::TrajectoryCostFunction)
