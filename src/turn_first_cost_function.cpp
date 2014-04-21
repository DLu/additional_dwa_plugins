#include <dwa_local_planner/trajectory_cost_function.h>

#include <additional_dwa_plugins/closest_target_point_map.h>
#include <angles/angles.h>
using base_local_planner::Trajectory;

int sign(double f){
    if(f>=0) return 1.0;
    return -1.0;
}


namespace dwa_plugins {

class TurnFirstCostFunction : public dwa_local_planner::TrajectoryCostFunction {
public:

  void initialize(std::string name, base_local_planner::LocalPlannerUtil *planner_util) {
    TrajectoryCostFunction::initialize(name, planner_util);
    
    ros::NodeHandle nh("~/" + name_);
    nh.param("max_trans_angle", max_trans_angle_, M_PI);
    // TODO: Give the option to turn if the CURRENT angle is less than max_trans_angle
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
      target_poses_ = orig_global_plan;
      if(target_poses_.size()==0)
          return;
      yaws_.clear();
      double x0, y0, x1, y1;
      x0 = target_poses_[0].pose.position.x;
      y0 = target_poses_[0].pose.position.y;
      
      for(unsigned int i=0; i<target_poses_.size()-1;i++){
        x1 = target_poses_[i+1].pose.position.x;
        y1 = target_poses_[i+1].pose.position.y;
        
        double angle = atan2(y1-y0,x1-x0);
        yaws_.push_back(angle);
        
        x0 = x1;
        y0 = y1;
      }
     
      map_.reset();  
      map_.setTargetCells(*costmap_, target_poses_);
  }

  /**
   * return a score for trajectory traj
   */
  virtual double scoreTrajectory(Trajectory &traj)
  { 
      if(traj.getPointsSize()==0)
        return 0.0;

      double px, py, pth;
      traj.getPoint(traj.getPointsSize()-1, px, py, pth);

      unsigned int cell_x, cell_y;
      //we won't allow trajectories that go off the map... shouldn't happen that often anyways
      if ( ! costmap_->worldToMap(px, py, cell_x, cell_y)) {
          //we're off the map
          ROS_WARN("Off Map %f, %f", px, py);
          return -4.0;
      }

      unsigned int path_index = map_(cell_x, cell_y).index;
      if(path_index>=yaws_.size())
       return 0.0;
      double diff = angles::shortest_angular_distance(pth, yaws_[path_index]);
      if(fabs(diff) > max_trans_angle_ && (fabs(traj.xv_) > 0.0 || fabs(traj.yv_) > 0.0))
        return -1.0;
        
      if( sign(diff) != sign(traj.thetav_) )
        return -1.0;
      else
        return fabs(diff);
  }

  virtual void debrief(base_local_planner::Trajectory &result) {}

  /* Only returns a value for grid based functions */
  virtual float getCost(unsigned int cx, unsigned int cy){ return 0.0; }

  std::vector<geometry_msgs::PoseStamped> target_poses_;
  std::vector<double> yaws_;
  double max_trans_angle_;

  ClosestTargetPointMap map_;

};


}


PLUGINLIB_EXPORT_CLASS(dwa_plugins::TurnFirstCostFunction, dwa_local_planner::TrajectoryCostFunction)
