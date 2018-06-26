#ifndef COLLISION_H
#define COLLISION_H

#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

// classes and structures
class Collision
{
private:
public:
  Collision(void);
  void filterByGridMap(std::vector<std::vector<std::vector<double> > >& trajectoryArrayCartesianListIn,
                       std::vector<std::vector<std::vector<double> > >& trajectoryArrayCartesianListOut,
                       grid_map::GridMap map, double LengthIn, double WidthIn, double gammaMin, double gammaMax,
                       double vMin, double vMax);
};

#endif
