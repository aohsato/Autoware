#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <vector>
#include "trajectory_array.h"
#include "lane_list.h"
#include "frene.h"
#include "unit.h"
#include <autoware_msgs/lane.h>
#include <frene_planner/optimalPath.h>

using namespace std;

// define global values
static ros::Publisher _optimalTrajectoryToTrajectoryTrackerPublisher;
static ros::Publisher _optimalTrajectoryToFreneNodesPublisher;
static ros::Publisher _optimalTrajectoryToMPCPublisher;
static ros::Publisher _optimalTrajectoryStatusPublisher;
std::vector<std::vector<std::vector<double> > > _optimalTrajectoryVelocityKeeping;
std::vector<std::vector<std::vector<double> > > _optimalTrajectoryFollowing;
std::vector<std::vector<std::vector<double> > > _previousOptimalTrajectory;
std::vector<std::vector<std::vector<double> > > _trajectoryZero;
std::vector<std::vector<std::vector<double> > > _trajectorySmoothStopping;
bool _isDebug = false;

#define NODENAME_DEFAULT ("optimal_path_selector")

#define STATUS_ERROR (-1)
#define STATUS_VELOCITY_KEEPING (0)
#define STATUS_FOLLOWING (1)
#define STATUS_STOPPING (2)
#define STATUS_SMOOTH_STOPPING (4)
#define STATUS_PREVIOUS_OPTIMAL (5)
#define STATUS_ZERO (10)
#define STATUS_DEFAULT (-2)

static void previousOptimalTrajectoryCallback(const std_msgs::Float64MultiArray::ConstPtr& input)
{
  std_msgs::Float64MultiArray trajectoryArrayMessage = *input;
  TrajectoryArray::copyTrajectoryArrayMessageToLaneList(trajectoryArrayMessage, _previousOptimalTrajectory);
}

static void optimalTrajectoryVelocityKeepingCallback(const std_msgs::Float64MultiArray::ConstPtr& input)
{
  std_msgs::Float64MultiArray trajectoryArrayMessage = *input;
  TrajectoryArray::copyTrajectoryArrayMessageToLaneList(trajectoryArrayMessage, _optimalTrajectoryVelocityKeeping);
}

static void optimalTrajectoryFollowingCallback(const std_msgs::Float64MultiArray::ConstPtr& input)
{
  std_msgs::Float64MultiArray trajectoryArrayMessage = *input;
  TrajectoryArray::copyTrajectoryArrayMessageToLaneList(trajectoryArrayMessage, _optimalTrajectoryFollowing);
}

// publish one optimal trajectory to velocity_keeping and stopping nodes
void publishOptimalTrajectoryToFreneNodes(std::vector<std::vector<std::vector<double> > >& trajectoryArrayList)
{
  std_msgs::Float64MultiArray trajectoryArrayMessage;
  TrajectoryArray::copyTrajectoryArrayListToTrajectoryArrayMessage(trajectoryArrayList, trajectoryArrayMessage);
  _optimalTrajectoryToFreneNodesPublisher.publish(trajectoryArrayMessage);
}

void copyTrajectoryArrayListToTrajectoryPathMessage(std::vector<std::vector<std::vector<double> > >& laneList,
                                                    autoware_msgs::lane& returnPathMessage,
                                                    string& mapFrameName, int status)
{
  // input data
  ros::Time currentTime = ros::Time::now();

  returnPathMessage.waypoints.clear();

  returnPathMessage.header.stamp = currentTime;
  returnPathMessage.header.frame_id = mapFrameName;

  int seqCount = 0;

  static double prevTheta = 0.0;
  static double thetaWrapped = 0.0;

  for (std::vector<std::vector<std::vector<double> > >::iterator itr0 = laneList.begin(); itr0 != laneList.end();
       itr0++)
  {
    // for mpc tracker
    double xprev = 0;
    double yprev = 0;
    bool isFirstFlag = true;
    double uprev = 0;

    for (std::vector<std::vector<double> >::iterator itr1 = itr0->begin(); itr1 != itr0->end(); itr1++)
    {
      autoware_msgs::waypoint message;

      double x = (*itr1)[0];
      double y = (*itr1)[1];
      double t = (*itr1)[2];
      double theta = (*itr1)[3];
      double dtTheta = (*itr1)[4];
      double vx = (*itr1)[5];
      double ax = (*itr1)[6];
      double kx = (*itr1)[7];

      // for mpc tracker
      if (isFirstFlag)
      {
        xprev = x;
        yprev = y;
        isFirstFlag = false;
      }
      double diff = sqrt(pow(x - xprev, 2.0) + pow(y - yprev, 2.0));
      double u = diff + uprev;
      xprev = x;
      yprev = y;
      uprev = u;
      double dt = theta - prevTheta;
      while (dt < -M_PI)
      {
        dt += 2.0 * M_PI;
      }
      while (dt > M_PI)
      {
        dt -= 2.0 * M_PI;
      }
      thetaWrapped += dt;
      prevTheta = theta;

      message.pose.header.stamp = message.twist.header.stamp = currentTime + ros::Duration(t);
      message.pose.header.seq = message.twist.header.seq = seqCount;
      message.pose.header.frame_id = message.twist.header.frame_id = mapFrameName;

      // some values are for mpc tracker
      message.pose.pose.position.x = x;
      message.pose.pose.position.y = y;
      message.pose.pose.position.z = Frene::getGroundMapZ();
      message.pose.pose.orientation.x = 0;//theta;
      message.pose.pose.orientation.y = 0;//thetaWrapped;
      message.pose.pose.orientation.z = 0;//u;
      message.pose.pose.orientation.w = 1;//sqrt(pow(theta,2.0)+pow(thetaWrapped,2.0)+pow(u,2.0));

      std::vector<double> lastElement = (*itr0).back();
      message.twist.twist.linear.x = vx;
      message.twist.twist.linear.y = ax;
      message.twist.twist.linear.z = kx;
      message.twist.twist.angular.x = thetaWrapped;
      message.twist.twist.angular.y = u;
      message.twist.twist.angular.z = dtTheta*kx;

      returnPathMessage.waypoints.push_back(message);

      seqCount++;
    }
  }
}

// publish to trajectory tracker
void publishOptimalTrajectoryToTrajectoryTracker(
    std::vector<std::vector<std::vector<double> > >& optimalTrajectoryArrayList, string& mapFrameName, int status)
{
  autoware_msgs::lane trajectoryPathMessage;
  copyTrajectoryArrayListToTrajectoryPathMessage(optimalTrajectoryArrayList, trajectoryPathMessage, mapFrameName, status);

  _optimalTrajectoryToTrajectoryTrackerPublisher.publish(trajectoryPathMessage);
}

int selectOptimalTrajectory(std::vector<std::vector<std::vector<double> > >& optimalTrajectory,
                            double thresholdDesiredLongPosition, double thresholdLowSpeed)
{
  double dddts0VelocityKeeping = 1e+09;
  for (std::vector<std::vector<std::vector<double> > >::iterator trajectory = _optimalTrajectoryVelocityKeeping.begin();
       trajectory != _optimalTrajectoryVelocityKeeping.end(); trajectory++)
  {
    for (std::vector<std::vector<double> >::iterator it = trajectory->begin(); it != trajectory->end(); ++it)
    {
      // x,y,t,theta,dtTheta,vx,ax,kx
      // ts,s,dts,ddts,dddts
      dddts0VelocityKeeping = (*it)[12];
      break;
    }
  }
  double dddts0Following = 1e+09;
  for (std::vector<std::vector<std::vector<double> > >::iterator trajectory = _optimalTrajectoryFollowing.begin();
       trajectory != _optimalTrajectoryFollowing.end(); trajectory++)
  {
    for (std::vector<std::vector<double> >::iterator it = trajectory->begin(); it != trajectory->end(); ++it)
    {
      // x,y,t,theta,dtTheta,vx,ax,kx
      // ts,s,dts,ddts,dddts
      dddts0Following = (*it)[12];
      break;
    }
  }
  bool isValidVelocityKeeping = !Frene::isInvalidTrajectory(_optimalTrajectoryVelocityKeeping);
  bool isValidFollowing = !Frene::isInvalidTrajectory(_optimalTrajectoryFollowing);

  int status = STATUS_DEFAULT;
  if (_optimalTrajectoryVelocityKeeping.size() <= 0)
  {
    // velocityKeeping not exist
    if (_optimalTrajectoryFollowing.size() <= 0)
    {
      // following not exist
      // no solution, needs to beep to warn the driver
      ROS_ERROR_ONCE("There is no solution");
      status = STATUS_ERROR;
    }
    else
    {
      // following exists

      int n = _optimalTrajectoryFollowing[0].size();
      double s = _optimalTrajectoryFollowing[0][0][9];
      double sf = _optimalTrajectoryFollowing[0][n - 1][9];
      double dts = _optimalTrajectoryFollowing[0][0][10];
      double dtsf = _optimalTrajectoryFollowing[0][n - 1][10];

      if (s < sf - thresholdDesiredLongPosition)
      {
        // if vehicle is before sf point
        optimalTrajectory = _optimalTrajectoryFollowing;
        status = STATUS_FOLLOWING;
      }
      else
      {
        // if vehicle is after sf-thresholdDesiredLongPosition point
        if (_trajectorySmoothStopping.size() <= 0)
        {
          // if smoothStopping not exist
          if (_previousOptimalTrajectory.size() <= 0)
          {
            optimalTrajectory = _trajectoryZero;
            status = STATUS_ZERO;
          }
          else
          {
            optimalTrajectory = _previousOptimalTrajectory;
            status = STATUS_PREVIOUS_OPTIMAL;
          }
        }
        else
        {
          // if smoothStopping exist
          optimalTrajectory = _trajectorySmoothStopping;
          status = STATUS_SMOOTH_STOPPING;
        }
      }
    }
  }
  else
  {
    // velocityKeeping exists
    if (_optimalTrajectoryFollowing.size() <= 0)
    {
      // following not exists
      optimalTrajectory = _optimalTrajectoryVelocityKeeping;
      status = STATUS_VELOCITY_KEEPING;
    }
    else
    {
      // following exists
      int n = _optimalTrajectoryFollowing[0].size();
      double s = _optimalTrajectoryFollowing[0][0][9];
      double sf = _optimalTrajectoryFollowing[0][n - 1][9];
      double dts = _optimalTrajectoryFollowing[0][0][10];
      double dtsf = _optimalTrajectoryFollowing[0][n - 1][10];

      if (s < sf - thresholdDesiredLongPosition)
      {
        // if vehicle is before sf point

        if (isValidVelocityKeeping && isValidFollowing)
        {
          // path with lower dddts0 will be selected
          if (dddts0VelocityKeeping < dddts0Following)
          {
            optimalTrajectory = _optimalTrajectoryVelocityKeeping;
            status = STATUS_VELOCITY_KEEPING;
          }
          else
          {
            optimalTrajectory = _optimalTrajectoryFollowing;
            status = STATUS_FOLLOWING;
          }
        }
        else if (isValidVelocityKeeping && !isValidFollowing)
        {
          optimalTrajectory = _optimalTrajectoryVelocityKeeping;
          status = STATUS_VELOCITY_KEEPING;
        }
        else if (!isValidVelocityKeeping && isValidFollowing)
        {
          optimalTrajectory = _optimalTrajectoryFollowing;
          status = STATUS_FOLLOWING;
        }
        else
        {
          ROS_ERROR_ONCE("There is no valid trajectory");
          if (_previousOptimalTrajectory.size() <= 0)
          {
            optimalTrajectory = _trajectoryZero;
            status = STATUS_ZERO;
          }
          else
          {
            optimalTrajectory = _previousOptimalTrajectory;
            status = STATUS_PREVIOUS_OPTIMAL;
          }
        }
      }
      else
      {
        // if vehicle is after sf point

        if (_trajectorySmoothStopping.size() <= 0)
        {
          // if smoothStopping not exist
          if (_previousOptimalTrajectory.size() <= 0)
          {
            optimalTrajectory = _trajectoryZero;
            status = STATUS_ZERO;
          }
          else
          {
            optimalTrajectory = _previousOptimalTrajectory;
            status = STATUS_PREVIOUS_OPTIMAL;
          }
        }
        else
        {
          // if smoothStopping exist
          optimalTrajectory = _trajectorySmoothStopping;
          status = STATUS_SMOOTH_STOPPING;
        }
      }
    }
  }

  // check illegal flag, sometimes safety constraint prohibits the each trajectory generation nodes
  // from compting the solution, in this case, trajectory will be sent with a certain flag showing that the trajectory
  // is invalid
  if (optimalTrajectory.size() > 0)
  {
    if (Frene::isInvalidTrajectory(optimalTrajectory))
    {
      ROS_ERROR_ONCE("There is no trajectory found");
    }
    else
    {
    }
  }

  std_msgs::Int16 statusMessage;
  statusMessage.data = status;
  _optimalTrajectoryStatusPublisher.publish(statusMessage);

  return status;
}

int main(int argc, char* argv[])
{
  // init ros loop
  ros::init(argc, argv, NODENAME_DEFAULT);
  ros::NodeHandle nh("~");

  //=== input parameters =======================
  std::cout << "nodeName:   " << ros::this_node::getName() << std::endl;

  int samplingTimeHz;
  nh.param<int>("samplingTimeHz", samplingTimeHz, 100);
  std::cout << "samplingTimeHz: " << samplingTimeHz << std::endl;

  int queSize;
  nh.param<int>("queSize", queSize, 10);
  std::cout << "queSize: " << queSize << std::endl;

  std::string optimalTrajectoryVelocityKeepingTopicName;
  nh.param<std::string>("optimalTrajectoryVelocityKeepingTopicName", optimalTrajectoryVelocityKeepingTopicName,
                        "/optimal_trajectory_velocity_keeping");
  std::cout << "optimalTrajectoryVelocityKeepingTopicName: " << optimalTrajectoryVelocityKeepingTopicName.c_str()
            << std::endl;

  std::string optimalTrajectoryFollowingTopicName;
  nh.param<std::string>("optimalTrajectoryFollowingTopicName", optimalTrajectoryFollowingTopicName,
                        "/optimal_trajectory_following");
  std::cout << "optimalTrajectoryFollowingTopicName: " << optimalTrajectoryFollowingTopicName.c_str() << std::endl;

  std::string optimalTrajectoryToFreneNodesTopicName;
  nh.param<std::string>("optimalTrajectoryToFreneNodesTopicName", optimalTrajectoryToFreneNodesTopicName,
                        "/optimal_trajectory_to_frene_nodes");
  std::cout << "optimalTrajectoryToFreneNodesTopicName: " << optimalTrajectoryToFreneNodesTopicName.c_str()
            << std::endl;

  std::string optimalTrajectoryToMPCTopicName;
  nh.param<std::string>("optimalTrajectoryToMPCTopicName", optimalTrajectoryToMPCTopicName, "/optimal_trajectory_to_"
                                                                                            "mpc");
  std::cout << "optimalTrajectoryToMPCTopicName: " << optimalTrajectoryToMPCTopicName.c_str() << std::endl;

  std::string optimalTrajectoryToTrajectoryTrackerTopicName;
  nh.param<std::string>("optimalTrajectoryToTrajectoryTrackerTopicName", optimalTrajectoryToTrajectoryTrackerTopicName,
                        "/optimal_trajectory_to_trajectory_tracker");
  std::cout << "optimalTrajectoryToTrajectoryTrackerTopicName: "
            << optimalTrajectoryToTrajectoryTrackerTopicName.c_str() << std::endl;

  std::string previousOptimalTrajectoryTopicName;
  nh.param<std::string>("previousOptimalTrajectoryTopicName", previousOptimalTrajectoryTopicName,
                        "/local_planner/trajectory_array_cartesian");
  std::cout << "previousOptimalTrajectoryTopicName: " << previousOptimalTrajectoryTopicName.c_str() << std::endl;

  std::string optimalTrajectoryStatusTopicName;
  nh.param<std::string>("optimalTrajectoryStatusTopicName", optimalTrajectoryStatusTopicName, "/optimal_trajectory_"
                                                                                              "status");
  std::cout << "optimalTrajectoryStatusTopicName: " << optimalTrajectoryStatusTopicName.c_str() << std::endl;

  std::string mapFrameName;
  nh.param<std::string>("mapFrameName", mapFrameName, "/map");
  std::cout << "mapFrameName: " << mapFrameName.c_str() << std::endl;

  bool isDebug;
  nh.param<bool>("isDebug", isDebug, true);
  std::cout << "isDebug: " << isDebug << std::endl;

  double thresholdDesiredLongPosition;
  nh.param<double>("thresholdDesiredLongPosition", thresholdDesiredLongPosition, 5.0);
  std::cout << "thresholdDesiredLongPosition: " << thresholdDesiredLongPosition << std::endl;

  double thresholdLowSpeed;
  nh.param<double>("thresholdLowSpeed", thresholdLowSpeed, 1e-02);
  std::cout << "thresholdLowSpeed: " << thresholdLowSpeed << std::endl;

  double trajectoryZeroLength;
  nh.param<double>("trajectoryZeroLength", trajectoryZeroLength, 10.0);
  std::cout << "trajectoryZeroLength: " << trajectoryZeroLength << std::endl;

  int trajectoryZeroNumOfPoints;
  nh.param<int>("trajectoryZeroNumOfPoints", trajectoryZeroNumOfPoints, 10);
  std::cout << "trajectoryZeroNumOfPoints: " << trajectoryZeroNumOfPoints << std::endl;

  double trajectorySmoothStoppingLength;
  nh.param<double>("trajectorySmoothStoppingLength", trajectorySmoothStoppingLength, 5.0);
  std::cout << "trajectorySmoothStoppingLength: " << trajectorySmoothStoppingLength << std::endl;

  int trajectorySmoothStoppingNumOfPoints;
  nh.param<int>("trajectorySmoothStoppingNumOfPoints", trajectorySmoothStoppingNumOfPoints, 1000);
  std::cout << "trajectorySmoothStoppingNumOfPoints: " << trajectorySmoothStoppingNumOfPoints << std::endl;

  double trajectorySmoothStoppingSmoothnessFactor;
  nh.param<double>("trajectorySmoothStoppingSmoothnessFactor", trajectorySmoothStoppingSmoothnessFactor, 3.0);
  std::cout << "trajectorySmoothStoppingSmoothnessFactor: " << trajectorySmoothStoppingSmoothnessFactor << std::endl;

  // copy to global memory
  //...

  // change unit

  // define subsciber
  ros::Subscriber optimalTrajectoryVelocityKeepingSubscriber =
      nh.subscribe(optimalTrajectoryVelocityKeepingTopicName, queSize, optimalTrajectoryVelocityKeepingCallback);
  ros::Subscriber optimalTrajectoryFollowingSubscriber =
      nh.subscribe(optimalTrajectoryFollowingTopicName, queSize, optimalTrajectoryFollowingCallback);
  ros::Subscriber previousOptimalTrajectorySubscriber =
      nh.subscribe(previousOptimalTrajectoryTopicName, queSize, previousOptimalTrajectoryCallback);

  // define publisher
  _optimalTrajectoryToFreneNodesPublisher =
      nh.advertise<std_msgs::Float64MultiArray>(optimalTrajectoryToFreneNodesTopicName, queSize);
  _optimalTrajectoryToTrajectoryTrackerPublisher =
      nh.advertise<autoware_msgs::lane>(optimalTrajectoryToTrajectoryTrackerTopicName, queSize);
  _optimalTrajectoryToMPCPublisher = nh.advertise<frene_planner::optimalPath>(optimalTrajectoryToMPCTopicName, queSize);
  _optimalTrajectoryStatusPublisher = nh.advertise<std_msgs::Int16>(optimalTrajectoryStatusTopicName, queSize);

  // copy to global memory
  _isDebug = isDebug;

  // starting ros loop
  ros::Rate loopRate(samplingTimeHz);  // by Hz

  // for testing following
  ros::Time beginTime = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();

    std::vector<std::vector<std::vector<double> > > optimalTrajectory = _optimalTrajectoryVelocityKeeping;
    int status = selectOptimalTrajectory(optimalTrajectory, thresholdDesiredLongPosition, thresholdLowSpeed);

    Frene::computeTrajectoryZero(_previousOptimalTrajectory, _trajectoryZero, trajectoryZeroLength,
                                 trajectoryZeroNumOfPoints);
    Frene::computeTrajectorySmoothStopping(_previousOptimalTrajectory, _trajectorySmoothStopping,
                                           trajectorySmoothStoppingLength, trajectorySmoothStoppingSmoothnessFactor,
                                           trajectorySmoothStoppingNumOfPoints);
    publishOptimalTrajectoryToFreneNodes(optimalTrajectory);
    publishOptimalTrajectoryToTrajectoryTracker(optimalTrajectory, mapFrameName, status);
    loopRate.sleep();
  }
}
