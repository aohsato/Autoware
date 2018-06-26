#include "trajectory_array.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

void TrajectoryArray::copyTrajectoryArrayListToTrajectoryArrayList(
    std::vector<std::vector<std::vector<double> > >& laneList,
    std::vector<std::vector<std::vector<double> > >& returnLaneList)
{
  // input data
  returnLaneList.clear();
  for (std::vector<std::vector<std::vector<double> > >::iterator itr1 = laneList.begin(); itr1 != laneList.end();
       itr1++)
  {
    std::vector<std::vector<double> > tmp;
    for (std::vector<std::vector<double> >::iterator itr2 = itr1->begin(); itr2 != itr1->end(); itr2++)
    {
      tmp.push_back(*itr2);
    }
    returnLaneList.push_back(tmp);
  }
}

void TrajectoryArray::copyTrajectoryArrayMessageToLaneList(
    std_msgs::Float64MultiArray& laneMessage, std::vector<std::vector<std::vector<double> > >& returnLaneList)
{
  returnLaneList.clear();

  int dimSize = laneMessage.layout.dim.size();

  if (dimSize > 2)
  {
    int nDim = laneMessage.layout.dim[0].size;
    int nRow = laneMessage.layout.dim[1].size;
    int nCol = laneMessage.layout.dim[2].size;

    for (int k = 0; k < nDim; k++)
    {
      // page
      std::vector<std::vector<double> > tmpMat;
      for (int i = 0; i < nRow; i++)
      {
        // row
        std::vector<double> tmpVector;
        for (int j = 0; j < nCol; j++)
        {
          // column
          tmpVector.push_back(laneMessage.data[nCol * nRow * k + nCol * i + j]);
        }
        tmpMat.push_back(tmpVector);
      }
      returnLaneList.push_back(tmpMat);
    }
  }
}

void TrajectoryArray::copyTrajectoryArrayListToTrajectoryArrayMessage(
    std::vector<std::vector<std::vector<double> > >& trajectoryArrayList,
    std_msgs::Float64MultiArray& trajectoryArrayMessage)
{
  // check input list size
  if (trajectoryArrayList.size() <= 0)
  {
    ROS_ERROR_ONCE("trajectoryArrayList.size() must be larger than 0");
    return;
  }

  // input data
  trajectoryArrayMessage.data.clear();
  for (std::vector<std::vector<std::vector<double> > >::iterator itr1 = trajectoryArrayList.begin();
       itr1 != trajectoryArrayList.end(); itr1++)
  {
    for (std::vector<std::vector<double> >::iterator itr2 = itr1->begin(); itr2 != itr1->end(); itr2++)
    {
      for (std::vector<double>::iterator itr3 = itr2->begin(); itr3 != itr2->end(); itr3++)
      {
        trajectoryArrayMessage.data.push_back(*itr3);
      }
    }
  }

  // copy to message
  std_msgs::MultiArrayDimension dimTmp;
  trajectoryArrayMessage.layout.dim.clear();
  trajectoryArrayMessage.layout.dim.push_back(dimTmp);
  trajectoryArrayMessage.layout.dim.push_back(dimTmp);
  trajectoryArrayMessage.layout.dim.push_back(dimTmp);

  int nCol = trajectoryArrayList[0][0].size();
  int nRow = trajectoryArrayList[0].size();
  int nDim = trajectoryArrayList.size();

  trajectoryArrayMessage.layout.dim[0].label = "dim";
  trajectoryArrayMessage.layout.dim[0].size = nDim;
  trajectoryArrayMessage.layout.dim[0].stride = nDim * nCol * nRow;
  trajectoryArrayMessage.layout.dim[1].label = "row";
  trajectoryArrayMessage.layout.dim[1].size = nRow;
  trajectoryArrayMessage.layout.dim[1].stride = nCol * nRow;
  trajectoryArrayMessage.layout.dim[2].label = "column";
  trajectoryArrayMessage.layout.dim[2].size = nCol;
  trajectoryArrayMessage.layout.dim[2].stride = nCol;
}

void TrajectoryArray::mergeTrajectoryList(std::vector<std::vector<double> >& t1, std::vector<std::vector<double> >& t2,
                                          std::vector<std::vector<double> >& tMerge)
{
  int nRow1 = t1.size();
  int nRow2 = t2.size();

  if (nRow1 != nRow2 || nRow1 < 0)
  {
    ROS_ERROR("Error. TrajectoryArray::mergeTrajectoryArrayList");
    return;
  }

  int nCol1 = t1[0].size();
  int nCol2 = t2[0].size();

  tMerge.clear();
  tMerge.resize(nRow1);
  for (int i = 0; i < nRow1; i++)
  {
    tMerge[i].clear();
    tMerge[i].reserve(nCol1 + nCol2);
    for (int j1 = 0; j1 < nCol1; j1++)
    {
      tMerge[i].push_back(t1[i][j1]);
    }
    for (int j2 = 0; j2 < nCol2; j2++)
    {
      tMerge[i].push_back(t2[i][j2]);
    }
  }
}
