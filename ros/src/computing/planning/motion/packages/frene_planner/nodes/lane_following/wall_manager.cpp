#include "wall_manager.h"
#include <ros/ros.h>
#include <iterator>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

using namespace std;

WallManager::WallManager(void)
{
}

// txt data should be folowing format
// id numCorner(could be different to 4 in the future) x y z seperated by tab
// 01 4 x y z
// 01 4 x y z
// 01 4 x y z
// 01 4 x y z
// 02 4 x y z
// ...
// each wall mush contain 4 points and closed
int WallManager::createWallNodeListFromTxt(std::string& filename, std::vector<WallNode>& result)
{
  result.clear();

  ifstream ifs(filename.c_str());
  string str;

  if (ifs.fail())
  {
    ROS_ERROR("File do not exist.");
    return -1;
  }

  int id;
  int numCorner;
  double x, y, z;
  int count = 0;

  WallNode wallNode;
  while (getline(ifs, str))
  {
    id = 0;
    x = 0;
    y = 0;
    z = 0;

    sscanf(str.data(), "%02d %d %lf %lf %lf", &id, &numCorner, &x, &y, &z);
    if (count == 0)
    {
      wallNode.resizeCorner(numCorner);
    }
    if (count < numCorner)
    {
      wallNode.id = id;
      wallNode.numCorner = numCorner;
      wallNode.corner[count].x = x;
      wallNode.corner[count].y = y;
      wallNode.corner[count].z = z;

      count++;
    }

    if (count >= numCorner)
    {
      // txt data must not be 'tortioned'
      // otherwise reorder of points must be needed
      result.push_back(wallNode);
      count = 0;
    }
  }

  return 0;
}

void WallManager::findForwardBlindCorner(std::vector<WallNode>& wallNodeList, double x0, double y0, double yaw,
                                         std::vector<BlindCorner>& result)
{
  result.clear();

  Eigen::VectorXd p0(2);
  Eigen::VectorXd p1(2);
  int numCorner = 0;
  double smallDistance = 1e-01;

  Eigen::VectorXd vehicleDirection(2);
  vehicleDirection(0) = cos(yaw);
  vehicleDirection(1) = sin(yaw);

  for (std::vector<WallNode>::iterator itrw = wallNodeList.begin(); itrw != wallNodeList.end(); itrw++)
  {
    // do for each wall
    numCorner = itrw->numCorner;

    p0(0) = x0;
    p0(1) = y0;

    for (int i = 0; i < numCorner; i++)
    {
      // do for each corner
      p1(0) = itrw->corner[i].x;
      p1(1) = itrw->corner[i].y;

      Eigen::VectorXd line = p1 - p0;
      Eigen::VectorXd delta = smallDistance * line.normalized();
      Eigen::VectorXd linePlus = line + delta;
      Eigen::VectorXd lineMinus = line - delta;

      // ring buffer
      int prevItr = 0;
      if (i == 0)
      {
        prevItr = numCorner - 1;
      }
      else
      {
        prevItr = i - 1;
      }
      int nextItr = 0;
      if (i == numCorner - 1)
      {
        nextItr = 0;
      }
      else
      {
        nextItr = i + 1;
      }
      CornerNode& prevCorner = itrw->corner[prevItr];
      CornerNode& nextCorner = itrw->corner[nextItr];

      Eigen::VectorXd p1prev(2);
      p1prev(0) = prevCorner.x;
      p1prev(1) = prevCorner.y;
      Eigen::VectorXd p1next(2);
      p1next(0) = nextCorner.x;
      p1next(1) = nextCorner.y;

      Eigen::VectorXd edge1 = p1next - p1;
      Eigen::VectorXd edge2 = p1prev - p1;

      Eigen::MatrixXd h(2, 2);
      h << edge1(0), edge2(0), edge1(1), edge2(1);
      Eigen::VectorXd alpha1(2);
      Eigen::VectorXd alpha2(2);
      alpha1 = h.colPivHouseholderQr().solve(delta);
      alpha2 = h.colPivHouseholderQr().solve(-delta);

      double dot1 = edge1.dot(delta);
      double dot2 = edge2.dot(delta);

      bool isLineTangentialToCorner = true;
      if ((alpha1(0) > 0 && alpha1(1) > 0) || (alpha2(0) > 0 && alpha2(1) > 0))
      {
        isLineTangentialToCorner = false;
      }

      if (isLineTangentialToCorner)
      {
        // candidate of blind corner

        double forward = line.dot(vehicleDirection);

        if (forward > 0)
        {
          // blind corner in front
          BlindCorner blindCorner;
          Eigen::VectorXd rushoutDirection(2);

          // compute rush out direction
          if (dot1 > 0)
          {
            // edge1 is the rush out direction
            rushoutDirection = -edge1.normalized();
          }
          else
          {
            // dot2 > 0
            // edge2 is the rush out direction
            rushoutDirection = -edge2.normalized();
          }

          // compute left/right w.r.t vehicle position
          Eigen::VectorXd rot90(2, 2);
          rot90 << 0, -1, 1, 0;
          Eigen::VectorXd e = rot90 * vehicleDirection;
          double tmpDotProduct = e.dot(p1 - p0);
          bool isLeft = true;
          if (tmpDotProduct < 0)
          {
            isLeft = false;
          }

          // return result
          blindCorner.x = p1(0);
          blindCorner.y = p1(1);
          blindCorner.z = itrw->corner[i].z;  // z is just for visualization
          blindCorner.vx = rushoutDirection(0);
          blindCorner.vy = rushoutDirection(1);
          blindCorner.vz = 0.0;
          blindCorner.isLeft = isLeft;
          result.push_back(blindCorner);
        }
      }
      else
      {
      }
    }
  }
}
