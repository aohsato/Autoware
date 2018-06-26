#include "lane_list.h"
#include "spline.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <autoware_msgs/lane.h>

void LaneList::copyLaneListToLaneList(std::vector<std::vector<double> >& laneList,
                                      std::vector<std::vector<double> >& returnLaneList)
{
  // input data
  returnLaneList.clear();
  for (std::vector<std::vector<double> >::iterator itr1 = laneList.begin(); itr1 != laneList.end(); itr1++)
  {
    returnLaneList.push_back(*itr1);
  }
}

void LaneList::copyLaneMessageToLaneList(std_msgs::Float64MultiArray& laneMessage,
                                         std::vector<std::vector<double> >& returnLaneList)
{
  returnLaneList.clear();

  int dimSize = laneMessage.layout.dim.size();

  if (dimSize > 1)
  {
    int nRow = laneMessage.layout.dim[0].size;
    int nCol = laneMessage.layout.dim[1].size;
    returnLaneList.reserve(nRow);
    std::vector<double> tmpVector;
    tmpVector.reserve(nCol);

    for (int i = 0; i < nRow; i++)
    {
      // row
      tmpVector.clear();
      for (int j = 0; j < nCol; j++)
      {
        // column
        tmpVector.push_back(laneMessage.data[nCol * i + j]);
      }
      returnLaneList.push_back(tmpVector);
    }
  }
}

void LaneList::copyPathMessageToLaneMessage(nav_msgs::Path& pathMessage, std_msgs::Float64MultiArray& returnLaneMessage)
{
  // check input message size
  if (pathMessage.poses.size() <= 0)
  {
    ROS_ERROR("pathMessage.poses.size() must be larger than 0");
    return;
  }

  // input data
  returnLaneMessage.data.clear();
  for (int itr = 0; itr < pathMessage.poses.size(); itr++)
  {
    returnLaneMessage.data.push_back(pathMessage.poses[itr].pose.position.x);
    returnLaneMessage.data.push_back(pathMessage.poses[itr].pose.position.y);
    returnLaneMessage.data.push_back(pathMessage.poses[itr].pose.position.z);
    returnLaneMessage.data.push_back(pathMessage.poses[itr].pose.orientation.x);
    returnLaneMessage.data.push_back(pathMessage.poses[itr].pose.orientation.y);
    returnLaneMessage.data.push_back(pathMessage.poses[itr].pose.orientation.z);
    returnLaneMessage.data.push_back(pathMessage.poses[itr].pose.orientation.w);
  }

  // copy to message
  std_msgs::MultiArrayDimension dimTmp;
  returnLaneMessage.layout.dim.clear();
  returnLaneMessage.layout.dim.push_back(dimTmp);
  returnLaneMessage.layout.dim.push_back(dimTmp);

  int nCol = 7;  // xyz,xyzw
  int nRow = pathMessage.poses.size();

  returnLaneMessage.layout.dim[0].label = "row";
  returnLaneMessage.layout.dim[0].size = nRow;
  returnLaneMessage.layout.dim[0].stride = nCol * nRow;
  returnLaneMessage.layout.dim[1].label = "column";
  returnLaneMessage.layout.dim[1].size = nCol;
  returnLaneMessage.layout.dim[1].stride = nCol;
}

void LaneList::copyTrajectoryPathMessageToLaneMessage(autoware_msgs::lane& pathMessage,
                                                      std_msgs::Float64MultiArray& returnLaneMessage)
{
  // check input message size
  if( pathMessage.waypoints.size() <= 0 )
  {
    ROS_ERROR( "pathMessage.waypoints.size() must be larger than 0");
    return;
  }

  // input data
  returnLaneMessage.data.clear();
  for (int itr = 0; itr < pathMessage.waypoints.size(); itr++)
  {
    returnLaneMessage.data.push_back(pathMessage.waypoints[itr].pose.pose.position.x);
    returnLaneMessage.data.push_back(pathMessage.waypoints[itr].pose.pose.position.y);
    returnLaneMessage.data.push_back(pathMessage.waypoints[itr].pose.pose.position.z);
    returnLaneMessage.data.push_back(pathMessage.waypoints[itr].pose.pose.orientation.x);
    returnLaneMessage.data.push_back(pathMessage.waypoints[itr].pose.pose.orientation.y);
    returnLaneMessage.data.push_back(pathMessage.waypoints[itr].pose.pose.orientation.z);
    returnLaneMessage.data.push_back(pathMessage.waypoints[itr].pose.pose.orientation.w);
  }

  // copy to message
  std_msgs::MultiArrayDimension dimTmp;
  returnLaneMessage.layout.dim.clear();
  returnLaneMessage.layout.dim.push_back(dimTmp);
  returnLaneMessage.layout.dim.push_back(dimTmp);

  int nCol = 7;  // xyz,xyzw
  int nRow = pathMessage.waypoints.size();

  returnLaneMessage.layout.dim[0].label = "row";
  returnLaneMessage.layout.dim[0].size = nRow;
  returnLaneMessage.layout.dim[0].stride = nCol * nRow;
  returnLaneMessage.layout.dim[1].label = "column";
  returnLaneMessage.layout.dim[1].size = nCol;
  returnLaneMessage.layout.dim[1].stride = nCol;
}

void LaneList::copyLaneListToLaneMessage(std::vector<std::vector<double> >& laneList,
                                         std_msgs::Float64MultiArray& returnLaneMessage)
{
  // check input list size
  if (laneList.size() <= 0)
  {
    ROS_ERROR("laneList.size() must be larger than 0");
    return;
  }

  // input data
  returnLaneMessage.data.clear();
  for (std::vector<std::vector<double> >::iterator itr1 = laneList.begin(); itr1 != laneList.end(); itr1++)
  {
    for (std::vector<double>::iterator itr2 = itr1->begin(); itr2 != itr1->end(); itr2++)
    {
      returnLaneMessage.data.push_back(*itr2);
    }
  }

  // copy to message
  std_msgs::MultiArrayDimension dimTmp;
  returnLaneMessage.layout.dim.clear();
  returnLaneMessage.layout.dim.push_back(dimTmp);
  returnLaneMessage.layout.dim.push_back(dimTmp);

  int nCol = laneList[0].size();
  int nRow = laneList.size();

  returnLaneMessage.layout.dim[0].label = "row";
  returnLaneMessage.layout.dim[0].size = nRow;
  returnLaneMessage.layout.dim[0].stride = nCol * nRow;
  returnLaneMessage.layout.dim[1].label = "column";
  returnLaneMessage.layout.dim[1].size = nCol;
  returnLaneMessage.layout.dim[1].stride = nCol;
}

void LaneList::drawLane(std::vector<std::vector<double> >& laneList, std::string& baseTfName,
                        visualization_msgs::Marker& lineStrip)  // x0,y0; x1,y1;...
{
  // start compute lane
  for (std::vector<std::vector<double> >::iterator itr = laneList.begin(); itr != laneList.end(); itr++)
  {
    // draw lane
    lineStrip.header.frame_id = baseTfName;
    lineStrip.header.stamp = ros::Time::now();
    lineStrip.type = visualization_msgs::Marker::SPHERE_LIST;
    lineStrip.scale.x = 0.5;
    lineStrip.color.r = 0.0;
    lineStrip.color.g = 1.0;
    lineStrip.color.b = 0.0;
    lineStrip.color.a = 1.0;
    geometry_msgs::Point point;

    if (itr->size() < 3)
    {
      // expecting x,y,s data
      return;
    }
    point.x = (*itr)[0];
    point.y = (*itr)[1];

    lineStrip.points.push_back(point);
  }
}

// xyList x0 y0; x1 y1; ...
// xyInterpList x0 y0; x1' y1'; ...
void LaneList::interpXYListWithResamplingDenseData(std::vector<std::vector<double> >& xyList,
                                                   std::vector<std::vector<double> >& xyInterpList, double interpTick,
                                                   double resamplingThreshold)
{
  // resampling
  double xp0 = 0.0;
  double yp0 = 0.0;

  std::vector<std::vector<double> > xyResamplingList;
  xyResamplingList.clear();
  bool isFirstRun = true;
  for (std::vector<std::vector<double> >::iterator xy = xyList.begin(); xy != xyList.end(); xy++)
  {
    double xp1 = (*xy)[0];
    double yp1 = (*xy)[1];

    double distance = sqrt(pow(xp1 - xp0, 2.0) + pow(yp1 - yp0, 2.0));

    if (isFirstRun || distance > resamplingThreshold)
    {
      std::vector<double> point;
      point.push_back(xp1);
      point.push_back(yp1);
      xyResamplingList.push_back(point);

      xp0 = xp1;
      yp0 = yp1;
      isFirstRun = false;
    }
    else
    {
      // skip
    }
  }

  interpXYList(xyResamplingList, xyInterpList, interpTick);
}

void LaneList::interpXYList(std::vector<std::vector<double> >& xyList, std::vector<std::vector<double> >& xyInterpList,
                            double interpTick)
{
  xyInterpList.clear();

  //============================================================================
  // check xy list size
  if (xyList.size() <= 0)
  {
    ROS_ERROR("Erorr: xyList.size() must be larger than0");
    return;
  }
  if (xyList[0].size() < 2)
  {
    ROS_ERROR("Erorr: xyList[0].size() must be larger than 2");
    return;
  }

  // first compute distance between each points and total distance
  double xp0 = 0.0;
  double yp0 = 0.0;
  double sumd = 0.0;
  bool isFirstRun = true;
  std::vector<double> dvec;
  for (std::vector<std::vector<double> >::iterator xy = xyList.begin(); xy != xyList.end(); xy++)
  {
    double xp1 = (*xy)[0];
    double yp1 = (*xy)[1];
    double d = sqrt(pow(xp1 - xp0, 2.0) + pow(yp1 - yp0, 2.0));

    if (isFirstRun)
    {
      d = 0.0;
      isFirstRun = false;
    }

    dvec.push_back(d);

    xp0 = xp1;
    yp0 = yp1;
    sumd = sumd + d;
  }

  // compute arc length s and make s vector
  std::vector<double> svec;
  double s0 = 0.0;
  for (std::vector<double>::iterator d = dvec.begin(); d != dvec.end(); d++)
  {
    double s1 = s0 + (*d) / sumd;
    svec.push_back(s1);
    s0 = s1;
  }

  // make x, y position vector
  std::vector<double> yvec;
  std::vector<double> xvec;
  for (std::vector<std::vector<double> >::iterator xy = xyList.begin(); xy != xyList.end(); xy++)
  {
    double xpoint = (*xy)[0];
    double ypoint = (*xy)[1];

    xvec.push_back(xpoint);
    yvec.push_back(ypoint);
  }

  // now spline interpolation
  tk::spline splineX;
  splineX.set_points(svec, xvec);
  tk::spline splineY;
  splineY.set_points(svec, yvec);

  for (double ss = 0; ss < 1.0; ss += interpTick)
  {
    double valueX = splineX(ss);
    double valueY = splineY(ss);

    double valuedX = splineX.deriv(1, ss);
    double valuedY = splineY.deriv(1, ss);

    double valueddX = splineX.deriv(2, ss);
    double valueddY = splineY.deriv(2, ss);

    double valuedddX = splineX.deriv(3, ss);
    double valuedddY = splineY.deriv(3, ss);

    std::vector<double> xx;
    xx.push_back(valueX);
    xx.push_back(valueY);
    xx.push_back(ss);  // LaneList optional invariant values
    xx.push_back(valuedX);
    xx.push_back(valuedY);
    xx.push_back(valueddX);
    xx.push_back(valueddY);
    xx.push_back(valuedddX);
    xx.push_back(valuedddY);

    xyInterpList.push_back(xx);
  }
  //=====================================================================================
}
