
#include <additional_dwa_plugins/closest_target_point_map.h>
#include <costmap_2d/cost_values.h>
using namespace std;

namespace dwa_plugins {

  ClosestTargetPointMap::ClosestTargetPointMap()
    : size_x_(0), size_y_(0)
  {
  }

  ClosestTargetPointMap::ClosestTargetPointMap(unsigned int size_x, unsigned int size_y) 
    : size_x_(size_x), size_y_(size_y)
  {
    commonInit();
  }

  void ClosestTargetPointMap::commonInit(){
    //don't allow construction of zero size grid
    ROS_ASSERT(size_y_ != 0 && size_x_ != 0);

    map_.resize(size_y_ * size_x_);

    //make each cell aware of its location in the grid
    for(unsigned int i = 0; i < size_y_; ++i){
      for(unsigned int j = 0; j < size_x_; ++j){
        unsigned int id = size_x_ * i + j;
        map_[id].cx = j;
        map_[id].cy = i;
      }
    }
  }

  size_t ClosestTargetPointMap::getIndex(int x, int y){
    return size_x_ * y + x;
  }

  void ClosestTargetPointMap::sizeCheck(unsigned int size_x, unsigned int size_y){
    if(map_.size() != size_x * size_y)
      map_.resize(size_x * size_y);

    if(size_x_ != size_x || size_y_ != size_y){
      size_x_ = size_x;
      size_y_ = size_y;

      for(unsigned int i = 0; i < size_y_; ++i){
        for(unsigned int j = 0; j < size_x_; ++j){
          int index = size_x_ * i + j;
          map_[index].cx = j;
          map_[index].cy = i;
        }
      }
    }
  }

  //reset the path_dist and goal_dist fields for all cells
  void ClosestTargetPointMap::reset(){
    for(unsigned int i = 0; i < map_.size(); ++i) {
      map_[i].index = 0;
      map_[i].seen = false;
    }
  }

  //update what map cells are considered path based on the global_plan
  void ClosestTargetPointMap::setTargetCells(const costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::PoseStamped>& global_plan) {
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    bool started_path = false;

    queue<MapCell*> path_dist_queue;

    unsigned int i;
    // put global path points into local map until we reach the border of the local map
    for (i = 0; i < global_plan.size(); ++i) {
      double g_x = global_plan[i].pose.position.x;
      double g_y = global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      if (costmap.worldToMap(g_x, g_y, map_x, map_y)) {
        MapCell& current = getCell(map_x, map_y);
        current.index = i;
        current.seen = true;
        path_dist_queue.push(&current);
        started_path = true;
      } else if (started_path) {
          break;
      }
    }
    if (!started_path) {
      ROS_ERROR("None of the %d first of %zu (%zu) points of the global plan were in the local costmap and free",
          i, global_plan.size(), global_plan.size());
      return;
    }

    computeTargetDistance(path_dist_queue);
  }

  void ClosestTargetPointMap::computeTargetDistance(queue<MapCell*>& dist_queue){
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = size_x_ - 1;
    unsigned int last_row = size_y_ - 1;
    while(!dist_queue.empty()){
      current_cell = dist_queue.front();
      dist_queue.pop();
      unsigned int index = current_cell->index;

      if(current_cell->cx > 0){
        check_cell = current_cell - 1;
        if(!check_cell->seen){
          //mark the cell as visited
          check_cell->seen = true;
          check_cell->index = index;
          dist_queue.push(check_cell);
        }
      }

      if(current_cell->cx < last_col){
        check_cell = current_cell + 1;
        if(!check_cell->seen){
          check_cell->seen = true;
          check_cell->index = index;
          dist_queue.push(check_cell);
        }
      }

      if(current_cell->cy > 0){
        check_cell = current_cell - size_x_;
        if(!check_cell->seen){
          check_cell->seen = true;
          check_cell->index = index;
          dist_queue.push(check_cell);
        }
      }

      if(current_cell->cy < last_row){
        check_cell = current_cell + size_x_;
        if(!check_cell->seen){
          check_cell->seen = true;
          check_cell->index = index;
          dist_queue.push(check_cell);
        }
      }
    }
  }

};
