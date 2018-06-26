#ifndef LANE_LIST_H
#define LANE_LIST_H
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <string>
#include <autoware_msgs/lane.h>

// LaneList class
// LaneList is a 2d list composed by a row of [x,y,optional invariant values].
// Optioinal invariant values contain s:arc length, k: curvature, dsdk: derivative of curvature.
// classes and structures
class LaneList
{
private:
public:
  LaneList(void){};

  // general list mananaging methods
  static void copyLaneListToLaneList(std::vector<std::vector<double> >& laneList,
                                     std::vector<std::vector<double> >& returnLaneList);
  static void interpXYList(std::vector<std::vector<double> >& xyList, std::vector<std::vector<double> >& xyInterpList,
                           double interpTick);
  static void interpXYListWithResamplingDenseData(std::vector<std::vector<double> >& xyList,
                                                  std::vector<std::vector<double> >& xyInterpList, double interpTick,
                                                  double resamplingThreshold);

  // ros related
  static void copyLaneMessageToLaneList(std_msgs::Float64MultiArray& laneMessage,
                                        std::vector<std::vector<double> >& returnLaneList);
  static void copyPathMessageToLaneMessage(nav_msgs::Path& pathMessage, std_msgs::Float64MultiArray& returnLaneMessage);
  static void copyTrajectoryPathMessageToLaneMessage(autoware_msgs::lane& pathMessage,
                                                     std_msgs::Float64MultiArray& returnLaneMessage);
  static void copyLaneListToTrajectoryPathMessage(std::vector<std::vector<double> >& laneList,
                                                  autoware_msgs::lane& returnLaneMessage);

  static void copyLaneListToLaneMessage(std::vector<std::vector<double> >& laneList,
                                        std_msgs::Float64MultiArray& returnLaneMessage);

  static void drawLane(std::vector<std::vector<double> >& laneList, std::string& baseTfName,
                       visualization_msgs::Marker& lineStrip);
};

#endif
