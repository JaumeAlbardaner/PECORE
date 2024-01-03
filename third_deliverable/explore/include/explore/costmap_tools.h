#ifndef COSTMAP_TOOLS_H_
#define COSTMAP_TOOLS_H_

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace frontier_exploration
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

/**
 * @brief Determine 4-connected neighbourhood of an input cell, checking for map
 * edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
std::vector<unsigned int> nhood4(unsigned int idx,
                                 const nav2_costmap_2d::Costmap2D& costmap)
{
  // get 4-connected neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out;

  unsigned int size_x_ = costmap.getSizeInCellsX(),
               size_y_ = costmap.getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    RCLCPP_WARN(rclcpp::get_logger("FrontierExploration"), "Evaluating nhood "
                                                           "for offmap point");
    return out;
  }

  if (idx % size_x_ > 0) {
    out.push_back(idx - 1);
  }
  if (idx % size_x_ < size_x_ - 1) {
    out.push_back(idx + 1);
  }
  if (idx >= size_x_) {
    out.push_back(idx - size_x_);
  }
  if (idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + size_x_);
  }
  return out;
}

/**
 * @brief Determine 8-connected neighbourhood of an input cell, checking for map
 * edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
std::vector<unsigned int> nhood8(unsigned int idx,
                                 const nav2_costmap_2d::Costmap2D& costmap)
{
  // get 8-connected neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out = nhood4(idx, costmap);

  unsigned int size_x_ = costmap.getSizeInCellsX(),
               size_y_ = costmap.getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    return out;
  }

  if (idx % size_x_ > 0 && idx >= size_x_) {
    out.push_back(idx - 1 - size_x_);
  }
  if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx - 1 + size_x_);
  }
  if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
    out.push_back(idx + 1 - size_x_);
  }
  if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + 1 + size_x_);
  }

  return out;
}

/**
 * @brief Find nearest cell of a specified value
 * @param result Index of located cell
 * @param start Index initial cell to search from
 * @param val Specified value to search for
 * @param costmap Reference to map data
 * @return True if a cell with the requested value was found
 */
bool nearestCell(unsigned int& result, unsigned int start, unsigned char val,
                 const nav2_costmap_2d::Costmap2D& costmap)
{
  const unsigned char* map = costmap.getCharMap();
  const unsigned int size_x = costmap.getSizeInCellsX(),
                     size_y = costmap.getSizeInCellsY();

  if (start >= size_x * size_y) {
    return false;
  }

  // initialize breadth first search
  std::queue<unsigned int> bfs;
  std::vector<bool> visited_flag(size_x * size_y, false);

  // push initial cell
  bfs.push(start);
  visited_flag[start] = true;

  // search for neighbouring cell matching value
  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // return if cell of correct value is found
    if (map[idx] == val) {
      result = idx;
      return true;
    }

    // iterate over all adjacent unvisited cells
    for (unsigned nbr : nhood8(idx, costmap)) {
      if (!visited_flag[nbr]) {
        bfs.push(nbr);
        visited_flag[nbr] = true;
      }
    }
  }

  return false;
}


/**
 * @brief Find actual distance from start to finish
 * @param start Index initial cell to search from
 * @param finish Specified cell to move to
 * @param costmap Reference to map data
 * @return Number of cells to be traversed to reach the finish from the start
 */
unsigned int trueCost(unsigned int start, unsigned char finish,
                 const nav2_costmap_2d::Costmap2D& costmap)
{
  const unsigned char* map = costmap.getCharMap();
  const unsigned int size_x = costmap.getSizeInCellsX(),
                     size_y = costmap.getSizeInCellsY();

  if (start >= size_x * size_y) {
    return -1;
  }

  // initialize breadth first search
  std::queue<unsigned int> bfs;
  std::vector<bool> visited_flag(size_x * size_y, false);
  std::vector<unsigned int> dist_cell(size_x * size_y, 0);

  // push initial cell
  bfs.push(start);
  visited_flag[start] = true;

  // search for neighbouring cell matching value
  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // return if cell of correct value is found
    if (idx == finish) {
      return dist_cell[idx]+ 1;
    }

    // iterate over all adjacent unvisited cells
    for (unsigned nbr : nhood8(idx, costmap)) {
      if (!visited_flag[nbr] && map[idx] == FREE_SPACE) {
        bfs.push(nbr);
        visited_flag[nbr] = true;
        dist_cell[nbr] = dist_cell[idx] + 1;
      }
    }
  }

  return -1;
}


/**
 * @brief Find furthest known cells
 * @param idx Index initial cell to search from
 * @param costmap Reference to map data
 * @return Closest unknown cells to position
 */
std::vector<unsigned int> nearestCells(unsigned int idx,
                 const nav2_costmap_2d::Costmap2D& costmap)
{
  const unsigned char* map = costmap.getCharMap();
  const unsigned int size_x = costmap.getSizeInCellsX(),
                     size_y = costmap.getSizeInCellsY();

  // get neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out;

  if (idx > size_x * size_y - 1) {
    return out;
  }

  // 1 2 3
  // 4 x 5
  // 6 7 8

  // 6
  unsigned int possible_lim = idx;
  while (map[possible_lim] == FREE_SPACE)
  {
    if (possible_lim % size_x > 0 && possible_lim >= size_x) {
      possible_lim = possible_lim - 1 - size_x;
    }
    else break;
  }
  // Found a border
  if(map[possible_lim] == NO_INFORMATION){
    out.push_back(possible_lim + 1 + size_x);
  }

  // 1
  possible_lim = idx;
  while (map[possible_lim] == FREE_SPACE)
  {
    if (idx % size_x > 0 && idx < size_x * (size_y - 1)) {
      possible_lim = possible_lim - 1 + size_x;
    }
    else break;
  }
  // Found a border
  if(map[possible_lim] == NO_INFORMATION){
    out.push_back(possible_lim + 1 - size_x);
  }

  // 8
  possible_lim = idx;
  while (map[possible_lim] == FREE_SPACE)
  {
    if (idx % size_x < size_x - 1 && idx >= size_x) {
      possible_lim = possible_lim + 1 - size_x;
    }
    else break;
  }
  // Found a border
  if(map[possible_lim] == NO_INFORMATION){
    out.push_back(possible_lim - 1 + size_x);
  }

  // 3
  possible_lim = idx;
  while (map[possible_lim] == FREE_SPACE)
  {
    if (idx % size_x < size_x - 1 && idx < size_x * (size_y - 1)) {
      possible_lim = possible_lim + 1 + size_x;
    }
    else break;
  }
  // Found a border
  if(map[possible_lim] == NO_INFORMATION){
    out.push_back(possible_lim - 1 - size_x);
  }

  // // 4
  // possible_lim = idx;
  // while (map[possible_lim] == FREE_SPACE)
  // {
  //   if (idx % size_x > 0) {
  //     possible_lim = possible_lim - 1 ;
  //   }
  //   else break;
  // }
  // // Found a border
  // if(map[possible_lim] == NO_INFORMATION){
  //   out.push_back(possible_lim + 1);
  // }

  // // 5
  // possible_lim = idx;
  // while (map[possible_lim] == FREE_SPACE)
  // {
  //   if (idx % size_x < size_x - 1) {
  //     possible_lim = possible_lim + 1 ;
  //   }
  //   else break;
  // }
  //   // Found a border
  //   if(map[possible_lim] == NO_INFORMATION){
  //     out.push_back(possible_lim - 1);
  //   }

  // // 7
  // possible_lim = idx;
  // while (map[possible_lim] == FREE_SPACE)
  // {
  //   if (idx >= size_x) {
  //     possible_lim = possible_lim - size_x;
  //   }
  //   else break;
  // }
  // // Found a border
  // if(map[possible_lim] == NO_INFORMATION){
  //   out.push_back(possible_lim + size_x);
  // }

  // // 2
  //   possible_lim = idx;
  // while (map[possible_lim] == FREE_SPACE)
  // {
  //   if (idx < size_x * (size_y - 1)) {
  //     possible_lim = possible_lim  + size_x;
  //   }
  //   else break;
  // }
  //   // Found a border
  //   if(map[possible_lim] == NO_INFORMATION){
  //     out.push_back(possible_lim - size_x);
  //   }
  

  return out;
}
}  // namespace frontier_exploration


#endif
