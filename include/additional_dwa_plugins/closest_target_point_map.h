#ifndef CLOSEST_TARGET_POINT_MAP_H_
#define CLOSEST_TARGET_POINT_MAP_H_

#include <vector>
#include <iostream>
#include <base_local_planner/trajectory_inc.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <base_local_planner/map_cell.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

namespace dwa_plugins {
  class MapCell {
   public:
    unsigned int cx, cy, index;
    bool seen;
  };


  /**
   * @class MapGrid
   * @brief A grid of MapCell cells that is used to propagate path and goal distances for the trajectory controller.
   */
  class ClosestTargetPointMap {
    public:
      /**
       * @brief  Creates a 0x0 map by default
       */
      ClosestTargetPointMap();

      /**
       * @brief  Creates a map of size_x by size_y
       * @param size_x The width of the map 
       * @param size_y The height of the map 
       */
      ClosestTargetPointMap(unsigned int size_x, unsigned int size_y);


      /**
       * @brief  Returns a map cell accessed by (col, row)
       * @param x The x coordinate of the cell 
       * @param y The y coordinate of the cell 
       * @return A reference to the desired cell
       */
      inline MapCell& operator() (unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }

      /**
       * @brief  Returns a map cell accessed by (col, row)
       * @param x The x coordinate of the cell 
       * @param y The y coordinate of the cell 
       * @return A copy of the desired cell
       */
      inline MapCell operator() (unsigned int x, unsigned int y) const {
        return map_[size_x_ * y + x];
      }

      inline MapCell& getCell(unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }

      /**
       * @brief  Destructor for a MapGrid
       */
      ~ClosestTargetPointMap(){}

      /**
       * @brief reset path distance fields for all cells
       */
      void reset();

      /**
       * @brief  check if we need to resize
       * @param size_x The desired width
       * @param size_y The desired height
       */
      void sizeCheck(unsigned int size_x, unsigned int size_y);

      /**
       * @brief Utility to share initialization code across constructors
       */
      void commonInit();

      /**
       * @brief  Returns a 1D index into the MapCell array for a 2D index
       * @param x The desired x coordinate
       * @param y The desired y coordinate
       * @return The associated 1D index 
       */
      size_t getIndex(int x, int y);

      /**
       * @brief  Compute the distance from each cell in the local map grid to the planned path
       * @param dist_queue A queue of the initial cells on the path 
       */
      void computeTargetDistance(std::queue<MapCell*>& dist_queue);

      /**
       * @brief  Compute the distance from each cell in the local map grid to the local goal point
       * @param goal_queue A queue containing the local goal cell 
       */
      void computeGoalDistance(std::queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap);

      /**
       * @brief Update what cells are considered path based on the global plan 
       */
      void setTargetCells(const costmap_2d::Costmap2D& costmap, const std::vector<geometry_msgs::PoseStamped>& global_plan);

      double goal_x_, goal_y_; /**< @brief The goal distance was last computed from */

      unsigned int size_x_, size_y_; ///< @brief The dimensions of the grid

    private:

      std::vector<MapCell> map_; ///< @brief Storage for the MapCells

  };
};

#endif
