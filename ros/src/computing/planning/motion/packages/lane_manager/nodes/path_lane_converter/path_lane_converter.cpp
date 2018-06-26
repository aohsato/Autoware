#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <vector>
#include <autoware_msgs/lane.h>
#include "lane_list.h"

using namespace std;

#define NODENAME_DEFAULT ("path_lane_converter")
#define VEHICLETFNAME_DEFAULT ("/ndt_frame")
#define PATHTOPICNAME_DEFAULT ("/path")
#define LANETOPICNAME_DEFAULT ("/lane")
#define SAMPLINGTIMEHZ_DEFAULT (10)
#define QUESIZE_DEFAULT (10)

// define global values
std::string _vehicleTfName;
static ros::Publisher _lanePublisher;
static ros::Publisher _laneMarkPublisher;
std::vector<std::vector<double> > _targetLaneListInterpVehicle;
std_msgs::Float64MultiArray _laneMessage;

static void pathCallback(const autoware_msgs::lane::ConstPtr &input)
{
  autoware_msgs::lane path = *input;

  LaneList::copyTrajectoryPathMessageToLaneMessage(path, _laneMessage);
}

void publishLane(void)
{
  // Publish
  _lanePublisher.publish(_laneMessage);

  // Just for visualization
  std::vector<std::vector<double> > laneList;
  visualization_msgs::Marker lineStrip;
  LaneList::copyLaneMessageToLaneList(_laneMessage, laneList);
  std::string tfName = "map";
  LaneList::drawLane(laneList, tfName, lineStrip);
  _laneMarkPublisher.publish(lineStrip);
}

int main(int argc, char *argv[])
{
  // init ros loop
  ros::init(argc, argv, NODENAME_DEFAULT);
  ros::NodeHandle nh("~");

  // input parameters
  int samplingTimeHz = SAMPLINGTIMEHZ_DEFAULT;
  int queSize = QUESIZE_DEFAULT;
  std::string pathTopicName = PATHTOPICNAME_DEFAULT;
  std::string laneTopicName = LANETOPICNAME_DEFAULT;

  nh.getParam("samplingTimeHz", samplingTimeHz);
  nh.getParam("queSize", queSize);
  nh.getParam("pathTopicName", pathTopicName);
  nh.getParam("laneTopicName", laneTopicName);

  std::cout << "nodeName:   " << ros::this_node::getName() << std::endl;
  std::cout << "samplingTimeHz: " << samplingTimeHz << std::endl;
  std::cout << "queSize: " << queSize << std::endl;
  std::cout << "pathTopicName: " << pathTopicName << std::endl;
  std::cout << "laneTopicName: " << laneTopicName << std::endl;

  // copy to global memory

  // define subsciber
  ros::Subscriber pathSubscriber = nh.subscribe(pathTopicName, queSize, pathCallback);

  // define publisher
  _lanePublisher = nh.advertise<std_msgs::Float64MultiArray>(laneTopicName, 0);
  _laneMarkPublisher = nh.advertise<visualization_msgs::Marker>("/lane_converted_from_path", 0);

  // starting ros loop
  ros::Rate loopRate(samplingTimeHz);  // by Hz
  while (ros::ok())
  {
    ros::spinOnce();

    publishLane();

    loopRate.sleep();
  }
}
