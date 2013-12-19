#include <dwa_local_planner/trajectory_cost_function.h>

using base_local_planner::Trajectory;


namespace dwa_plugins {

class BaseCostFunction : public dwa_local_planner::TrajectoryCostFunction {
public:

  void initialize(std::string name, base_local_planner::LocalPlannerUtil *planner_util) {
    TrajectoryCostFunction::initialize(name, planner_util);
  }

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

  virtual void reset() {}

  /**
   * Set Global Plan with plan transformed and cropped into local costmap frame
   */
  virtual void setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan, double goal_x, double goal_y)
  {
  
  }

  /**
   * return a score for trajectory traj
   */
  virtual double scoreTrajectory(Trajectory &traj)
  {
     return 0.0;  
  }

  virtual void debrief(base_local_planner::Trajectory &result) {}

  /* Only returns a value for grid based functions */
  virtual float getCost(unsigned int cx, unsigned int cy){ return 0.0; }

};


}


PLUGINLIB_EXPORT_CLASS(dwa_plugins::BaseCostFunction, dwa_local_planner::TrajectoryCostFunction)
