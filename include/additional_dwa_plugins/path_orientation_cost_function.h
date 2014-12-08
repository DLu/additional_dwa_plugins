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

#ifndef PATH_ORIENTATION_COST_FUNCTION_H_
#define PATH_ORIENTATION_COST_FUNCTION_H_

#include <dwa_local_planner/trajectory_cost_function.h>

#include <costmap_2d/costmap_2d.h>
#include <additional_dwa_plugins/closest_target_point_map.h>

namespace dwa_plugins {

class PathOrientationCostFunction: public dwa_local_planner::TrajectoryCostFunction {
public:

  PathOrientationCostFunction() {}

  void initialize(std::string name, base_local_planner::LocalPlannerUtil *planner_util);
  virtual bool prepare(tf::Stamped<tf::Pose> global_pose,
		       tf::Stamped<tf::Pose> global_vel,
		       std::vector<geometry_msgs::Point> footprint_spec) { return true; }

  double scoreTrajectory(base_local_planner::Trajectory &traj);

  virtual void setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan, double goal_x, double goal_y);

protected:
  std::vector<geometry_msgs::PoseStamped> target_poses_;
  std::vector<double> yaws_;
  double max_trans_angle_, front_offset_angle_;

  ClosestTargetPointMap map_;

};

}
#endif /* PATH_ORIENTATION_COST_FUNCTION_H_ */
