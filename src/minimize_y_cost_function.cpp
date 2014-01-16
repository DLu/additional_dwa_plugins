#include <dwa_local_planner/trajectory_cost_function.h>

using base_local_planner::Trajectory;


namespace dwa_plugins {

class MinimizeYCostFunction : public dwa_local_planner::TrajectoryCostFunction {
public:

  /**
   * General updating of context values if required.
   * Subclasses may overwrite. Return false in case there is any error.
   */
  virtual bool prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec)
  {
    return true;
  }

  /**
   * return a score for trajectory traj
   */
  virtual double scoreTrajectory(Trajectory &traj)
  {
     return fabs(traj.yv_);
  }

};


}


PLUGINLIB_EXPORT_CLASS(dwa_plugins::MinimizeYCostFunction, dwa_local_planner::TrajectoryCostFunction)
