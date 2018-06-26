#ifndef TRAJECTORY_ARRAY_H
#define TRAJETORY_ARRAY_H
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <string>

// TrajectoryArray class
// TrajectoryArray is a 3d list composed by a list of matrix [x0,y0,o0; x1,y1,o1,...]
// classes and structures
class TrajectoryArray
{
private:
public:
  TrajectoryArray(void){};

  // ros related
  static void
  copyTrajectoryArrayListToTrajectoryArrayMessage(std::vector<std::vector<std::vector<double> > >& trajectoryArrayList,
                                                  std_msgs::Float64MultiArray& trajectoryArrayMessage);

  static void copyTrajectoryArrayMessageToLaneList(std_msgs::Float64MultiArray& laneMessage,
                                                   std::vector<std::vector<std::vector<double> > >& returnLaneList);

  static void mergeTrajectoryList(std::vector<std::vector<double> >& t1, std::vector<std::vector<double> >& t2,
                                  std::vector<std::vector<double> >& tMerge);
  static void
  copyTrajectoryArrayListToTrajectoryArrayList(std::vector<std::vector<std::vector<double> > >& laneList,
                                               std::vector<std::vector<std::vector<double> > >& returnLaneList);
};

#endif
