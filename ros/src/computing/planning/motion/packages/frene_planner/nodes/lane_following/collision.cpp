#include "collision.h"
#include <iterator>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>

using namespace std;

Collision::Collision(void)
{
}

void Collision::filterByGridMap(std::vector<std::vector<std::vector<double> > >& trajectoryArrayCartesianListIn,
                                std::vector<std::vector<std::vector<double> > >& trajectoryArrayCartesianListOut,
                                grid_map::GridMap map, double LengthIn, double WidthIn, double gammaMin,
                                double gammaMax, double vMin, double vMax)
{
  for (std::vector<std::vector<std::vector<double> > >::iterator itr1 = trajectoryArrayCartesianListIn.begin();
       itr1 != trajectoryArrayCartesianListIn.end(); itr1++)
  {
    // do for each trajectories
    std::vector<std::vector<double> > tmp;
    bool isSafe = true;

    // Infer path from 2 consecute points in the path
    for (std::vector<std::vector<double> >::iterator itr2 = itr1->begin(); itr2 != itr1->end() - 1; itr2++)
    {
      // do for each points in the trajectory
      double x = (*itr2)[0];
      double y = (*itr2)[1];
      double theta = (*itr2)[3];
      double vx = (*itr2)[5];

      // scaling factor
      double gamma = (gammaMax - gammaMin) / (vMax - vMin) * vx + gammaMin;
      gamma = max(min(gamma, gammaMin), gammaMax);
      double Length = gamma * LengthIn;
      double Width = gamma * WidthIn;

      tf::Vector3 point(x, y, 0);
      tf::Quaternion orientation = tf::createQuaternionFromYaw(theta);
      tf::Vector3 footprint[4];
      footprint[0] = tf::Vector3(Length / 2, Width / 2, 0);
      footprint[1] = tf::Vector3(-Length / 2, Width / 2, 0);
      footprint[2] = tf::Vector3(Length / 2, -Width / 2, 0);
      footprint[3] = tf::Vector3(-Length / 2, -Width / 2, 0);
      for (int i = 0; i <= 3; i++)
      {
        tf::Vector3 preCorner = tf::quatRotate(orientation, footprint[i]) + point;
        tf::Vector3 corner = preCorner;
        Eigen::Vector2d pos(corner.getX(), corner.getY());

        if (map.isInside(pos) &&
            map.atPosition("traversability", pos, grid_map::InterpolationMethods::INTER_NEAREST) > 0.0)
        {
          isSafe = false;
        }
      }
      if (!isSafe)
      {
        break;
      }

      tmp.push_back(*itr2);
    }

    if (isSafe)
    {
      trajectoryArrayCartesianListOut.push_back(tmp);
    }
  }
}
