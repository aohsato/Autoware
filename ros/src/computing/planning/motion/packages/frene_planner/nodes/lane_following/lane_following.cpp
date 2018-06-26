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
#include <geometry_msgs/TwistWithCovarianceStamped.h>
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
#include <frene_planner/optimalPath.h>
#include "wall_manager.h"
#include "speed_model.h"
#include <eigen3/Eigen/Dense>

#include "collision.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>

using namespace std;

// define global values
static ros::Publisher _trajectoryArrayPublisher;
static ros::Publisher _trajectoryMarkPublisher;
static ros::Publisher _trajectoryFilter1MarkPublisher;
static ros::Publisher _trajectoryFilter2MarkPublisher;
static ros::Publisher _trajectoryFilter3MarkPublisher;
static ros::Publisher _optimalTrajectoryMarkPublisher;
static ros::Publisher _targetLaneInterpVehicleMarkPublisher;

static ros::Publisher _testLeadingVehicleMarkPublisher;
static ros::Publisher _previousOptimalTrajectoryMarkPublisher;
static ros::Publisher _blindCornerMarkPublisher;
#define maxBlindCornerListSizeForVisualization (15)
static ros::Publisher _speedModelTrajectoryMarkPublisher[maxBlindCornerListSizeForVisualization];
static ros::Publisher _desiredStatePublisher;

geometry_msgs::Vector3Stamped _vehicleSpeed;
geometry_msgs::Vector3Stamped _vehicleAcceleration;
geometry_msgs::Vector3Stamped _vehicleYawRate;
std::vector<std::vector<double> > _targetLaneListInterpVehicle;
double _resamplingThreshold;
geometry_msgs::PoseStamped _currentPose;
bool _isUpdateTargetPath = true;
geometry_msgs::Vector3Stamped _desiredStateFollowing;
std::vector<std::vector<std::vector<double> > > _previousOptimalTrajectory;
bool _useOccupancyGridMap = true;
bool _doTransform = true;
visualization_msgs::MarkerArray Cars;
grid_map::GridMap _occMap({ "Traversability" });
grid_map::GridMap _transformed;
bool _useCollision;
geometry_msgs::PoseStamped _navGoal;

#define NODENAME_DEFAULT ("velocity_keeping")

static void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  _navGoal = *input;
}

grid_map::Position applyTransform(grid_map::Position in, double x, double y, double yaw)
{
  Eigen::Rotation2Dd rot(yaw);
  Eigen::Vector2d trans(x, y);
  return rot * in + trans;
}

static void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& input)
{
  if (!_useOccupancyGridMap)
  {
    grid_map::GridMap tmp;
    grid_map::GridMapRosConverter::fromMessage(*input, tmp);
    _occMap = tmp;
  }
}

static void occupancyGridMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& input)
{
  if (_useOccupancyGridMap)
    grid_map::GridMapRosConverter::fromOccupancyGrid(*input, "traversability", _occMap);
}

static void targetLaneGlobalCallback(const std_msgs::Float64MultiArray::ConstPtr& input)
{
  std_msgs::Float64MultiArray targetLaneMessageInterpVehicle;
  targetLaneMessageInterpVehicle = *input;
  LaneList::copyLaneMessageToLaneList(targetLaneMessageInterpVehicle, _targetLaneListInterpVehicle);
}

static void desiredStateFollowingCallback(const geometry_msgs::Vector3Stamped::ConstPtr& input)
{
  _desiredStateFollowing = *input;
}

static double calc_ax(geometry_msgs::Vector3Stamped pv, geometry_msgs::Vector3Stamped v)
{
  if (pv.header.stamp.sec == 0 && pv.header.stamp.nsec == 0)
    return 0;

  double dv = v.vector.x - pv.vector.x;
  double dsec = (double)v.header.stamp.sec - pv.header.stamp.sec;
  double dnsec = (double)v.header.stamp.nsec - pv.header.stamp.nsec;
  return dv / (dsec + dnsec * 0.000000001);
}

static void vehicleSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& input)
{
  geometry_msgs::Vector3Stamped pv = _vehicleSpeed;
  _vehicleSpeed.header = (*input).header;
  _vehicleSpeed.vector = (*input).twist.linear;

  _vehicleYawRate.header = (*input).header;
  _vehicleYawRate.vector = (*input).twist.angular;
  _vehicleYawRate.vector.x = _vehicleYawRate.vector.z;
  _vehicleYawRate.vector.z = 0;

  _vehicleAcceleration.vector.x = calc_ax(pv, _vehicleSpeed);
}
static void vehicleAccelerationCallback(const geometry_msgs::Vector3Stamped::ConstPtr& input)
{
  _vehicleAcceleration = *input;
}
static void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  _currentPose = *input;
}

static void previousOptimalTrajectoryCallback(const std_msgs::Float64MultiArray::ConstPtr& input)
{
  std_msgs::Float64MultiArray trajectoryArrayMessage = *input;
  TrajectoryArray::copyTrajectoryArrayMessageToLaneList(trajectoryArrayMessage, _previousOptimalTrajectory);
}

void publishTrajectoryArray(std::vector<std::vector<std::vector<double> > >& trajectoryArrayList)
{
  std_msgs::Float64MultiArray trajectoryArrayMessage;
  TrajectoryArray::copyTrajectoryArrayListToTrajectoryArrayMessage(trajectoryArrayList, trajectoryArrayMessage);
  _trajectoryArrayPublisher.publish(trajectoryArrayMessage);
}

std_msgs::ColorRGBA parulaColorMap(double value)
{
  static const double parulaMap[][3] = { // 64 rows
                                         { 0.2081, 0.1663, 0.5292 },
                                         { 0.2116, 0.1898, 0.5777 },
                                         { 0.2123, 0.2138, 0.6270 },
                                         { 0.2081, 0.2386, 0.6771 },
                                         { 0.1959, 0.2645, 0.7279 },
                                         { 0.1707, 0.2919, 0.7792 },
                                         { 0.1253, 0.3242, 0.8303 },
                                         { 0.0591, 0.3598, 0.8683 },
                                         { 0.0117, 0.3875, 0.8820 },
                                         { 0.0060, 0.4086, 0.8828 },
                                         { 0.0165, 0.4266, 0.8786 },
                                         { 0.0329, 0.4430, 0.8720 },
                                         { 0.0498, 0.4586, 0.8641 },
                                         { 0.0629, 0.4737, 0.8554 },
                                         { 0.0723, 0.4887, 0.8467 },
                                         { 0.0779, 0.5040, 0.8384 },
                                         { 0.0793, 0.5200, 0.8312 },
                                         { 0.0749, 0.5375, 0.8263 },
                                         { 0.0641, 0.5570, 0.8240 },
                                         { 0.0488, 0.5772, 0.8228 },
                                         { 0.0343, 0.5966, 0.8199 },
                                         { 0.0265, 0.6137, 0.8135 },
                                         { 0.0239, 0.6287, 0.8038 },
                                         { 0.0231, 0.6418, 0.7913 },
                                         { 0.0228, 0.6535, 0.7768 },
                                         { 0.0267, 0.6642, 0.7607 },
                                         { 0.0384, 0.6743, 0.7436 },
                                         { 0.0590, 0.6838, 0.7254 },
                                         { 0.0843, 0.6928, 0.7062 },
                                         { 0.1133, 0.7015, 0.6859 },
                                         { 0.1453, 0.7098, 0.6646 },
                                         { 0.1801, 0.7177, 0.6424 },
                                         { 0.2178, 0.7250, 0.6193 },
                                         { 0.2586, 0.7317, 0.5954 },
                                         { 0.3022, 0.7376, 0.5712 },
                                         { 0.3482, 0.7424, 0.5473 },
                                         { 0.3953, 0.7459, 0.5244 },
                                         { 0.4420, 0.7481, 0.5033 },
                                         { 0.4871, 0.7491, 0.4840 },
                                         { 0.5300, 0.7491, 0.4661 },
                                         { 0.5709, 0.7485, 0.4494 },
                                         { 0.6099, 0.7473, 0.4337 },
                                         { 0.6473, 0.7456, 0.4188 },
                                         { 0.6834, 0.7435, 0.4044 },
                                         { 0.7184, 0.7411, 0.3905 },
                                         { 0.7525, 0.7384, 0.3768 },
                                         { 0.7858, 0.7356, 0.3633 },
                                         { 0.8185, 0.7327, 0.3498 },
                                         { 0.8507, 0.7299, 0.3360 },
                                         { 0.8824, 0.7274, 0.3217 },
                                         { 0.9139, 0.7258, 0.3063 },
                                         { 0.9450, 0.7261, 0.2886 },
                                         { 0.9739, 0.7314, 0.2666 },
                                         { 0.9938, 0.7455, 0.2403 },
                                         { 0.9990, 0.7653, 0.2164 },
                                         { 0.9955, 0.7861, 0.1967 },
                                         { 0.9880, 0.8066, 0.1794 },
                                         { 0.9789, 0.8271, 0.1633 },
                                         { 0.9697, 0.8481, 0.1475 },
                                         { 0.9626, 0.8705, 0.1309 },
                                         { 0.9589, 0.8949, 0.1132 },
                                         { 0.9598, 0.9218, 0.0948 },
                                         { 0.9661, 0.9514, 0.0755 },
                                         { 0.9763, 0.9831, 0.0538 }
  };

  std_msgs::ColorRGBA ret;

  int num = 0;
  num = (int)(floor((64 - 1) * value));  // value: 0-1

  if (num < 0)
  {
    num = 0;
  }
  if (num >= 64)
  {
    num = 63;
  }

  ret.r = parulaMap[num][0];
  ret.g = parulaMap[num][1];
  ret.b = parulaMap[num][2];
  ret.a = 1;

  return ret;
}

void drawTrajectory(std::vector<std::vector<std::vector<double> > >& trajectoryList,
                    ros::Publisher& trajectoryMarkPublisher, std::string frameId, double rColor, double gColor,
                    double bColor, int skipCount, int type)
{
  visualization_msgs::Marker lineStrip;
  lineStrip.points.clear();

  // start compute trajectory
  static int count = 1;
  for (std::vector<std::vector<std::vector<double> > >::iterator trajectory = trajectoryList.begin();
       trajectory != trajectoryList.end(); trajectory++)
  {
    if (count > skipCount)
    {
      // draw trajectory
      lineStrip.header.frame_id = frameId;
      lineStrip.type = type;
      lineStrip.scale.x = 0.02;
      lineStrip.color.r = rColor;
      lineStrip.color.g = gColor;
      lineStrip.color.b = bColor;
      lineStrip.color.a = 1.0;
      geometry_msgs::Point point;
      std_msgs::ColorRGBA each_color;

      int numOfPoints = 0;
      for (std::vector<std::vector<double> >::iterator it = trajectory->begin(); it != trajectory->end(); ++it)
      {
        point.x = (*it)[0];
        point.y = (*it)[1];
        double vx = (*it)[5];  // vx
        double kx = (*it)[7];  // kx
        point.z = vx + Frene::getGroundMapZ();

        double vxmax = kmph2mps(20);
        double gamma = min(vx / vxmax, 1.0);
        std_msgs::ColorRGBA parula = parulaColorMap(gamma);
        each_color = parula;

        lineStrip.points.push_back(point);
        lineStrip.colors.push_back(each_color);

        if (numOfPoints == 0)
        {
        }
        else
        {
          lineStrip.points.push_back(point);
          lineStrip.colors.push_back(each_color);
          numOfPoints++;
        }
        numOfPoints++;
      }
      if (fabs(numOfPoints / 2.0) > 1e-04)
      {
        lineStrip.points.push_back(point);
        lineStrip.colors.push_back(each_color);
      }
      numOfPoints = 0;

      count = 1;
    }
    count++;
  }
  count = 1;
  trajectoryMarkPublisher.publish(lineStrip);
}

void drawBlindCorner(std::vector<BlindCorner>& blindCornerList, ros::Publisher& blindCornerMarkPublisher,
                     std::string frameId, double rColor, double gColor, double bColor, int type)
{
  visualization_msgs::Marker marker;
  marker.points.clear();

  // start compute trajectory
  for (std::vector<BlindCorner>::iterator itr = blindCornerList.begin(); itr != blindCornerList.end(); itr++)
  {
    // draw trajectory
    marker.header.frame_id = frameId;
    marker.type = type;
    marker.scale.x = 0.1;
    marker.color.r = rColor;
    marker.color.g = gColor;
    marker.color.b = bColor;
    marker.color.a = 1.0;
    geometry_msgs::Point point;

    point.x = itr->x;
    point.y = itr->y;
    point.z = itr->z;
    marker.points.push_back(point);

    point.x = itr->x + itr->vx;
    point.y = itr->y + itr->vy;
    point.z = itr->z + itr->vz;
    marker.points.push_back(point);
  }
  blindCornerMarkPublisher.publish(marker);
}

void computeTrajectoryArray(
    std::vector<std::vector<std::vector<double> > >& trajectoryArrayList, string& mapFrameName,

    // for following, enable when freneMode=0,3
    double desiredSpeed, double desiredSpeedMinus, double desiredSpeedPlus, double desiredSpeedDelta,
    double desiredLatPosition, double desiredLatPositionMinus, double desiredLatPositionPlus,
    double desiredLatPositionDelta,

    // for following, enable when freneMode=1,2
    double desiredLongSpeed, double desiredLongAcceleration, double desiredLongPosition,
    double desiredLongPositionMinus, double desiredLongPositionPlus, double desiredLongPositionDelta,

    double timeMinSec, double timeMaxSec, double timeDeltaSec, double kjlong, double ktlong, double kplong,
    double kjlat, double ktlat, double kplat, double klong, double klat, double resamplingThreshold,
    double arcLengthTickMeter,

    double roadMarginRight, double roadMarginLeft, double maxCentrifugalForce, double maxAcceleration,
    double maxCurvature,

    double resamplingTickTime, double polyTickTime, bool usePreviousTrajectoryForInit, bool isDrawTrajectory,
    int freneMode,

    double leadingVehiclePosition,

    double stopSpeed, double stopPosition1, double stopPosition2, double stopCountMax1, double stopCountMax2,

    bool useSpeedModel, std::string& wallTxtFileName, double paramSpeedModelVo, double paramSpeedModelDy,
    double paramSpeedModelAd, double paramSpeedModelAmax, double paramSpeedModelDv,

    double carLength, double carWidth, double collisionScalingFactorGammaMin, double collisionScalingFactorGammaMax,
    double collisionScalingFactorVMin, double collisionScalingFactorVMax

    )
{
  if (_targetLaneListInterpVehicle.size() <= 0)
  {
    ROS_ERROR_ONCE("frene planner is waiting target path !!!...");
    return;
  }

  // pool sensors to memory
  double x0 = _currentPose.pose.position.x;
  double y0 = _currentPose.pose.position.y;
  double roll, pitch, yaw;

  tf::Quaternion q(_currentPose.pose.orientation.x, _currentPose.pose.orientation.y, _currentPose.pose.orientation.z,
                   _currentPose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  double theta0 = yaw;
  double vx0 = _vehicleSpeed.vector.x;
  double vx0MinLimit = kmph2mps(1);  // TODO: move to param
  if (vx0 < vx0MinLimit)
  {
    vx0 = vx0MinLimit;
  }
  double ax0 = _vehicleAcceleration.vector.x;
  double kx0 = _vehicleYawRate.vector.x / vx0;
  double dtTheta0 = _vehicleYawRate.vector.x;

  // init frene frame
  std::vector<std::vector<double> > xyResamplingList;  // xyResamplingList return is just for visualization
  Frene frene(_targetLaneListInterpVehicle, _resamplingThreshold, xyResamplingList);

  // define initial freneState
  FreneState initialStateFromSensor;
  FreneState initialStateFromPreviousTrajectory;
  FreneState initialState;

  static std::vector<std::vector<std::vector<double> > > previousOptimalTrajectoryArrayCartesianList;

  // compute initial freneState
  // init initial freneState from current sensor
  frene.convertFromCartesianState(0, x0, y0, theta0, vx0, ax0, kx0, dtTheta0, initialStateFromSensor,
                                  arcLengthTickMeter);
  initialState = initialStateFromSensor;
  // init initial freneState using previous computed optimal trajectory
  if (previousOptimalTrajectoryArrayCartesianList.size() > 0 && usePreviousTrajectoryForInit)
  {
    double xCurrent = x0;
    double yCurrent = y0;
    double vxCurrent = vx0;
    double t0;
    frene.computeInitialStateDirectFromPreviousTrajectoryWithSplineResampling(
        xCurrent, yCurrent, previousOptimalTrajectoryArrayCartesianList, initialStateFromPreviousTrajectory,
        resamplingTickTime);
    // HACK
    initialState = initialStateFromPreviousTrajectory;
    initialState.s = initialStateFromSensor.s;
    initialState.d = initialStateFromSensor.d;
  }

  geometry_msgs::TwistWithCovarianceStamped desiredState;
  desiredState.twist.twist.linear.x = desiredLongPosition;
  desiredState.twist.twist.linear.y = desiredLatPosition;
  desiredState.twist.twist.linear.z = desiredLongSpeed;
  desiredState.twist.twist.angular.x = initialState.s;
  _desiredStatePublisher.publish(desiredState);

  // generate lateral & longitudinal trajectory
  std::vector<std::vector<std::vector<double> > > trajectoryArrayFreneList;
  std::vector<std::vector<std::vector<double> > > paramArrayList;

  switch (freneMode)
  {
    case 0:  // velocity keeping
    default:
      frene.generateLateralAndLongitudinalTrajectoryVelocityKeeping(
          initialState, trajectoryArrayFreneList, paramArrayList, timeMinSec, timeMaxSec, timeDeltaSec, desiredSpeed,
          desiredSpeedMinus, desiredSpeedPlus, desiredSpeedDelta, desiredLatPosition, desiredLatPositionMinus,
          desiredLatPositionPlus, desiredLatPositionDelta, polyTickTime);
      break;
    case 1:  // following
      frene.generateLateralAndLongitudinalTrajectoryFollowing(
          initialState, trajectoryArrayFreneList, paramArrayList, desiredLongSpeed, desiredLongAcceleration, timeMinSec,
          timeMaxSec, timeDeltaSec, desiredLongPosition, desiredLongPositionMinus, desiredLongPositionPlus,
          desiredLongPositionDelta, desiredLatPosition, desiredLatPositionMinus, desiredLatPositionPlus,
          desiredLatPositionDelta, polyTickTime);
      break;
    case 2:  // stopping is a variation of following
      desiredLongAcceleration = 0;

      // compute desired long position from nav goal
      double navGoalX = _navGoal.pose.position.x;
      double navGoalY = _navGoal.pose.position.y;
      FreneState freneStateStopping;
      frene.convertFromCartesianState(0, navGoalX, navGoalY, 0, 0, 0, 0, 0, freneStateStopping, arcLengthTickMeter);
      desiredLongPosition = freneStateStopping.s;
      if (desiredLongPosition < initialState.s)
      {
        desiredLongPosition = 1e+04;
      }
      frene.generateLateralAndLongitudinalTrajectoryStopping(
          initialState, trajectoryArrayFreneList, paramArrayList, desiredLongSpeed, desiredLongAcceleration, timeMinSec,
          timeMaxSec, timeDeltaSec, desiredLongPosition, desiredLongPositionMinus, desiredLongPositionPlus,
          desiredLongPositionDelta, desiredLatPosition, desiredLatPositionMinus, desiredLatPositionPlus,
          desiredLatPositionDelta, polyTickTime);
      break;
  }

  // now convert merged trajectories to Cartesian coordinate
  std::vector<std::vector<std::vector<double> > > trajectoryArrayCartesianList[4];
  frene.convertMergedTrajectoryArrayToCartesian(trajectoryArrayFreneList, trajectoryArrayCartesianList[0]);

  // filter by safety
  frene.filterBySafety(trajectoryArrayCartesianList[0], trajectoryArrayCartesianList[1], roadMarginRight,
                       roadMarginLeft, maxCentrifugalForce, maxAcceleration, maxCurvature);

  // filter by safety concerning potential danger
  static WallManager wallManager;
  static bool isFirstTime = true;
  static std::vector<WallNode> wallNodeList;

  if (useSpeedModel)
  {
    if (isFirstTime)
    {
      if (wallManager.createWallNodeListFromTxt(wallTxtFileName, wallNodeList) < 0)
      {
        ROS_ERROR_ONCE("wall data is not correct.");
      }
      isFirstTime = false;
    }
    std::vector<BlindCorner> blindCornerList;
    wallManager.findForwardBlindCorner(wallNodeList, x0, y0, yaw, blindCornerList);

    SpeedModel speedModel;
    speedModel.filterByInvisibleDanger(trajectoryArrayCartesianList[1], trajectoryArrayCartesianList[2],
                                       blindCornerList, x0, y0, yaw, paramSpeedModelVo, paramSpeedModelDy,
                                       paramSpeedModelAd, paramSpeedModelAmax, paramSpeedModelDv);

    if (blindCornerList.size() > 0)
    {
      // for visualization
      for (int id = 0; id < blindCornerList.size(); id++)
      {
        if (id < maxBlindCornerListSizeForVisualization)
        {
          std::vector<std::vector<std::vector<double> > > speedModelTrajectoryArrayCartesianList;
          speedModel.computeSafeSpeedForBlindCorner(blindCornerList[id], speedModelTrajectoryArrayCartesianList,
                                                    paramSpeedModelVo, paramSpeedModelDy, paramSpeedModelAd,
                                                    paramSpeedModelAmax, paramSpeedModelDv);
          drawTrajectory(speedModelTrajectoryArrayCartesianList, _speedModelTrajectoryMarkPublisher[id], mapFrameName,
                         1.0, 0, 0, 0, visualization_msgs::Marker::LINE_STRIP);
        }
      }
    }

    // draw
    drawBlindCorner(blindCornerList, _blindCornerMarkPublisher, mapFrameName, 1.0, 0, 0,
                    visualization_msgs::Marker::LINE_LIST);
  }
  else
  {
    trajectoryArrayCartesianList[2] = trajectoryArrayCartesianList[1];
  }

  // filter by potential collision
  if (_useCollision)
  {
    Collision collision;
    if (_doTransform)
    {
      double X = _currentPose.pose.position.x;
      double Y = _currentPose.pose.position.y;
      double yaw = tf::getYaw(_currentPose.pose.orientation);
      _transformed.clear("traversability");
      grid_map::Matrix& data = _occMap["traversability"];
      for (grid_map::GridMapIterator iterator(_occMap); !iterator.isPastEnd(); ++iterator)
      {
        const int i = iterator.getLinearIndex();
        if (data(i) > 0.0)
        {
          grid_map::Position position;
          _occMap.getPosition(*iterator, position);
          grid_map::Position newPos = applyTransform(position, X, Y, yaw);
          if (_transformed.isInside(newPos))
            _transformed.atPosition("traversability", newPos) = 1.0;
        }
      }

      collision.filterByGridMap(trajectoryArrayCartesianList[2], trajectoryArrayCartesianList[3], _transformed,
                                carLength, carWidth, collisionScalingFactorGammaMin, collisionScalingFactorGammaMax,
                                collisionScalingFactorVMin, collisionScalingFactorVMax);
    }
    else
    {
      collision.filterByGridMap(trajectoryArrayCartesianList[2], trajectoryArrayCartesianList[3], _occMap, carLength,
                                carWidth, collisionScalingFactorGammaMin, collisionScalingFactorGammaMax,
                                collisionScalingFactorVMin, collisionScalingFactorVMax);
    }
  }
  else
  {
    trajectoryArrayCartesianList[3] = trajectoryArrayCartesianList[2];
  }

  // filter by optimality cost to find one optimal trajectory
  std::vector<std::vector<std::vector<double> > > optimalTrajectoryArrayCartesianListTmp;
  int arraySize;

  int size[4];
  size[0] = trajectoryArrayCartesianList[0].size();  // all
  size[1] = trajectoryArrayCartesianList[1].size();  // filter by axmax, kxmax, cxmax,...
  size[2] = trajectoryArrayCartesianList[2].size();  // filter by vo speed model
  size[3] = trajectoryArrayCartesianList[3].size();  // filter by collsion using grid map

  // find filtered list with zero size
  bool isFoundZeroElement = false;
  for (int i = 1; i < 4; i++)
  {
    if (size[i] == 0)
    {
      isFoundZeroElement = true;
      int previousSize = size[i - 1];

      if (previousSize == 1)
      {
        optimalTrajectoryArrayCartesianListTmp = trajectoryArrayCartesianList[i - 1];
        break;
      }
      else if (previousSize > 1)
      {
        frene.filterByOptimalityCostFromCartesianTrajectoryArray(
            trajectoryArrayCartesianList[i - 1], optimalTrajectoryArrayCartesianListTmp, kjlong, ktlong, kplong, kjlat,
            ktlat, kplat, klong, klat, freneMode, desiredSpeed, desiredLatPosition, desiredLongPosition);
        Frene::setAsInvalidTrajectory(optimalTrajectoryArrayCartesianListTmp);
        break;
      }
      else
      {
        // error
        ROS_ERROR("cannot find trajectory");
        break;
      }
    }
  }

  if (!isFoundZeroElement)
  {
    arraySize = trajectoryArrayCartesianList[3].size();

    if (arraySize == 1)
    {
      optimalTrajectoryArrayCartesianListTmp = trajectoryArrayCartesianList[3];
    }
    else if (arraySize > 1)
    {
      frene.filterByOptimalityCostFromCartesianTrajectoryArray(
          trajectoryArrayCartesianList[3], optimalTrajectoryArrayCartesianListTmp, kjlong, ktlong, kplong, kjlat, ktlat,
          kplat, klong, klat, freneMode, desiredSpeed, desiredLatPosition, desiredLongPosition);
    }
    else
    {
      ROS_ERROR("cannot find trajectory");
    }
  }

  arraySize = trajectoryArrayCartesianList[1].size();
  if (arraySize < 1)
  {
    // each node publish a path with a certain predefined flag when safe constraint is not satisfied.
    // optimal_path_selector checks the flag of the trajectory and detects unsafe situation
    ROS_ERROR_ONCE("Warning: HARD safety constraint");
    std::cout << "Warning: HARD safety constraint freneMode :" << freneMode << std::endl;
  }

  // now convert the optimal trajectory to Cartesian coordinate
  std::vector<std::vector<std::vector<double> > > optimalTrajectoryArrayCartesianList;
  optimalTrajectoryArrayCartesianList = optimalTrajectoryArrayCartesianListTmp;

  // save current optimal trajectory
  if (_previousOptimalTrajectory.size() <= 0)
  {
    // if there is no previous optimal trajectory outside to be subscribed, copy own previous trajectory to local memory
    TrajectoryArray::copyTrajectoryArrayListToTrajectoryArrayList(optimalTrajectoryArrayCartesianList,
                                                                  previousOptimalTrajectoryArrayCartesianList);
  }
  else
  {
    // if there is a previous optimal trajectory outside to be subscribed, copy that trajectory to local memory
    TrajectoryArray::copyTrajectoryArrayListToTrajectoryArrayList(_previousOptimalTrajectory,
                                                                  previousOptimalTrajectoryArrayCartesianList);
  }

  // return list
  TrajectoryArray::copyTrajectoryArrayListToTrajectoryArrayList(optimalTrajectoryArrayCartesianList,
                                                                trajectoryArrayList);

  // draw trajectory
  if (isDrawTrajectory)
  {
    drawTrajectory(trajectoryArrayCartesianList[0], _trajectoryMarkPublisher, mapFrameName, 0.5, 0.5, 0.5, 0,
                   visualization_msgs::Marker::LINE_LIST);  // all
    drawTrajectory(trajectoryArrayCartesianList[1], _trajectoryFilter1MarkPublisher, mapFrameName, 0.0, 1.0, 0.0, 0,
                   visualization_msgs::Marker::LINE_LIST);  // filter by safety
    drawTrajectory(trajectoryArrayCartesianList[2], _trajectoryFilter2MarkPublisher, mapFrameName, 0.0, 0.0, 1.0, 0,
                   visualization_msgs::Marker::LINE_LIST);  // filter by safety concerning potential danger
    drawTrajectory(trajectoryArrayCartesianList[3], _trajectoryFilter3MarkPublisher, mapFrameName, 0.0, 0.0, 1.0, 0,
                   visualization_msgs::Marker::LINE_LIST);  // filter by safety concerning potential danger
    drawTrajectory(optimalTrajectoryArrayCartesianList, _optimalTrajectoryMarkPublisher, mapFrameName, 1, 0, 0, 0,
                   visualization_msgs::Marker::LINE_LIST);

    std::vector<std::vector<std::vector<double> > > xyResamplingArrayList;
    xyResamplingArrayList.push_back(xyResamplingList);
    drawTrajectory(xyResamplingArrayList, _targetLaneInterpVehicleMarkPublisher, mapFrameName, 0, 1, 0, 0,
                   visualization_msgs::Marker::LINE_LIST);

    drawTrajectory(previousOptimalTrajectoryArrayCartesianList, _previousOptimalTrajectoryMarkPublisher, mapFrameName,
                   0, 1, 1, 0, visualization_msgs::Marker::LINE_LIST);
  }
}

int main(int argc, char* argv[])
{
  // init ros loop
  ros::init(argc, argv, NODENAME_DEFAULT);
  ros::NodeHandle nh("~");

  //=== input parameters =======================
  std::cout << "nodeName:   " << ros::this_node::getName() << std::endl;

  std::string vehicleSpeedTopicName;
  nh.param<std::string>("vehicleSpeedTopicName", vehicleSpeedTopicName, "/vehicle_speed");
  std::cout << "vehicleSpeedTopicName: " << vehicleSpeedTopicName.c_str() << std::endl;

  std::string vehicleAccelerationTopicName;
  nh.param<std::string>("vehicleAccelerationTopicName", vehicleAccelerationTopicName, "/vehicle_acceleration");
  std::cout << "vehicleAccelerationTopicName: " << vehicleAccelerationTopicName.c_str() << std::endl;

  int samplingTimeHz;
  nh.param<int>("samplingTimeHz", samplingTimeHz, 100);
  std::cout << "samplingTimeHz: " << samplingTimeHz << std::endl;

  int queSize;
  nh.param<int>("queSize", queSize, 10);
  std::cout << "queSize: " << queSize << std::endl;

  std::string targetLaneGlobalTopicName;
  nh.param<std::string>("targetLaneGlobalTopicName", targetLaneGlobalTopicName, "/global_plan");
  std::cout << "targetLaneGlobalTopicName: " << targetLaneGlobalTopicName.c_str() << std::endl;

  std::string trajectoryArrayTopicName;
  nh.param<std::string>("trajectoryArrayTopicName", trajectoryArrayTopicName, "/trajectory_array");
  std::cout << "trajectoryArrayTopicName: " << trajectoryArrayTopicName.c_str() << std::endl;

  double timeMinSec;
  nh.param<double>("timeMinSec", timeMinSec, 1.0);
  std::cout << "timeMinSec: " << timeMinSec << std::endl;

  double timeMaxSec;
  nh.param<double>("timeMaxSec", timeMaxSec, 1.0);
  std::cout << "timeMaxSec: " << timeMaxSec << std::endl;

  double timeDeltaSec;
  nh.param<double>("timeDeltaSec", timeDeltaSec, 1.0);
  std::cout << "timeDeltaSec: " << timeDeltaSec << std::endl;

  double desiredSpeedKmPerHour;
  nh.param<double>("desiredSpeedKmPerHour", desiredSpeedKmPerHour, 1.0);
  std::cout << "desiredSpeedKmPerHour: " << desiredSpeedKmPerHour << std::endl;

  double desiredSpeedPlusKmPerHour;
  nh.param<double>("desiredSpeedPlusKmPerHour", desiredSpeedPlusKmPerHour, 1.0);
  std::cout << "desiredSpeedPlusKmPerHour: " << desiredSpeedPlusKmPerHour << std::endl;

  double desiredSpeedMinusKmPerHour;
  nh.param<double>("desiredSpeedMinusKmPerHour", desiredSpeedMinusKmPerHour, 1.0);
  std::cout << "desiredSpeedMinusKmPerHour: " << desiredSpeedMinusKmPerHour << std::endl;

  double desiredSpeedDeltaKmPerHour;
  nh.param<double>("desiredSpeedDeltaKmPerHour", desiredSpeedDeltaKmPerHour, 1.0);
  std::cout << "desiredSpeedDeltaKmPerHour: " << desiredSpeedDeltaKmPerHour << std::endl;

  double desiredLatPositionMeter;
  nh.param<double>("desiredLatPositionMeter", desiredLatPositionMeter, 1.0);
  std::cout << "desiredLatPositionMeter: " << desiredLatPositionMeter << std::endl;
  double desiredLatPositionPlusMeter;
  nh.param<double>("desiredLatPositionPlusMeter", desiredLatPositionPlusMeter, 1.0);
  std::cout << "desiredLatPositionPlusMeter: " << desiredLatPositionPlusMeter << std::endl;
  double desiredLatPositionMinusMeter;
  nh.param<double>("desiredLatPositionMinusMeter", desiredLatPositionMinusMeter, 1.0);
  std::cout << "desiredLatPositionMinusMeter: " << desiredLatPositionMinusMeter << std::endl;
  double desiredLatPositionDeltaMeter;
  nh.param<double>("desiredLatPositionDeltaMeter", desiredLatPositionDeltaMeter, 1.0);
  std::cout << "desiredLatPositionDeltaMeter: " << desiredLatPositionDeltaMeter << std::endl;

  double desiredLongSpeedKmPerHour;
  nh.param<double>("desiredLongSpeedKmPerHour", desiredLongSpeedKmPerHour, 1.0);
  std::cout << "desiredLongSpeedKmPerHour: " << desiredLongSpeedKmPerHour << std::endl;

  double desiredLongAccelerationMeterPerSec;
  nh.param<double>("desiredLongAccelerationMeterPerSec", desiredLongAccelerationMeterPerSec, 1.0);
  std::cout << "desiredLongAccelerationMeterPerSec: " << desiredLongAccelerationMeterPerSec << std::endl;

  double desiredLongPositionMeter;
  nh.param<double>("desiredLongPositionMeter", desiredLongPositionMeter, 1.0);
  std::cout << "desiredLongPositionMeter: " << desiredLongPositionMeter << std::endl;
  double desiredLongPositionPlusMeter;
  nh.param<double>("desiredLongPositionPlusMeter", desiredLongPositionPlusMeter, 1.0);
  std::cout << "desiredLongPositionPlusMeter: " << desiredLongPositionPlusMeter << std::endl;
  double desiredLongPositionMinusMeter;
  nh.param<double>("desiredLongPositionMinusMeter", desiredLongPositionMinusMeter, 1.0);
  std::cout << "desiredLongPositionMinusMeter: " << desiredLongPositionMinusMeter << std::endl;
  double desiredLongPositionDeltaMeter;
  nh.param<double>("desiredLongPositionDeltaMeter", desiredLongPositionDeltaMeter, 1.0);
  std::cout << "desiredLongPositionDeltaMeter: " << desiredLongPositionDeltaMeter << std::endl;

  std::string navGoalTopicName;
  nh.param<std::string>("navGoalTopicName", navGoalTopicName, "/move_base_simple/goal");
  std::cout << "navGoalTopicName: " << navGoalTopicName.c_str() << std::endl;

  double kjlong;
  nh.param<double>("kjlong", kjlong, 1.0);
  std::cout << "kjlong: " << kjlong << std::endl;

  double kplong;
  nh.param<double>("kplong", kplong, 1.0);
  std::cout << "kplong: " << kplong << std::endl;

  double ktlong;
  nh.param<double>("ktlong", ktlong, 1.0);
  std::cout << "ktlong: " << ktlong << std::endl;

  double klong;
  nh.param<double>("klong", klong, 1.0);
  std::cout << "klong: " << klong << std::endl;

  double kjlat;
  nh.param<double>("kjlat", kjlat, 1.0);
  std::cout << "kjlat: " << kjlat << std::endl;

  double kplat;
  nh.param<double>("kplat", kplat, 1.0);
  std::cout << "kplat: " << kplat << std::endl;

  double ktlat;
  nh.param<double>("ktlat", ktlat, 1.0);
  std::cout << "ktlat: " << ktlat << std::endl;

  double klat;
  nh.param<double>("klat", klat, 1.0);
  std::cout << "klat: " << klat << std::endl;

  double resamplingThreshold;
  nh.param<double>("resamplingThreshold", resamplingThreshold, 1.0);
  std::cout << "resamplingThreshold: " << resamplingThreshold << std::endl;

  std::string currentPoseTopicName;
  nh.param<std::string>("currentPoseTopicName", currentPoseTopicName, "/current_pose");
  std::cout << "currentPoseTopicName: " << currentPoseTopicName.c_str() << std::endl;

  double arcLengthTickMeter;
  nh.param<double>("arcLengthTickMeter", arcLengthTickMeter, 0.01);
  std::cout << "arcLengthTickMeter: " << arcLengthTickMeter << std::endl;

  double roadMarginLeft;
  nh.param<double>("roadMarginLeft", roadMarginLeft, 1.0);
  std::cout << "roadMarginLeft: " << roadMarginLeft << std::endl;

  double roadMarginRight;
  nh.param<double>("roadMarginRight", roadMarginRight, 1.0);
  std::cout << "roadMarginRight: " << roadMarginRight << std::endl;

  double maxCentrifugalForce;
  nh.param<double>("maxCentrifugalForce", maxCentrifugalForce, 1.0);
  std::cout << "maxCentrifugalForce: " << maxCentrifugalForce << std::endl;

  double maxAcceleration;
  nh.param<double>("maxAcceleration", maxAcceleration, 1.0);
  std::cout << "maxAcceleration: " << maxAcceleration << std::endl;

  double maxCurvature;
  nh.param<double>("maxCurvature", maxCurvature, 1.0);
  std::cout << "maxCurvature: " << maxCurvature << std::endl;

  double resamplingTickTime;
  nh.param<double>("resamplingTickTime", resamplingTickTime, 1.0);
  std::cout << "resamplingTickTime: " << resamplingTickTime << std::endl;

  double polyTickTime;
  nh.param<double>("polyTickTime", polyTickTime, 1.0);
  std::cout << "polyTickTime: " << polyTickTime << std::endl;

  bool usePreviousTrajectoryForInit;
  nh.param<bool>("usePreviousTrajectoryForInit", usePreviousTrajectoryForInit, 1.0);
  std::cout << "usePreviousTrajectoryForInit: " << usePreviousTrajectoryForInit << std::endl;

  bool isDrawTrajectory;
  nh.param<bool>("isDrawTrajectory", isDrawTrajectory, false);
  std::cout << "isDrawTrajectory: " << isDrawTrajectory << std::endl;

  std::string mapFrameName;
  nh.param<std::string>("mapFrameName", mapFrameName, "/map");
  std::cout << "mapFrameName: " << mapFrameName.c_str() << std::endl;

  int freneMode;
  nh.param<int>("freneMode", freneMode, 0);
  std::cout << "freneMode: " << freneMode << std::endl;

  std::string desiredStateFollowingTopicName;
  nh.param<std::string>("desiredStateFollowingTopicName", desiredStateFollowingTopicName, "/desired_state_following");
  std::cout << "desiredStateFollowingTopicName: " << desiredStateFollowingTopicName.c_str() << std::endl;

  std::string previousOptimalTrajectoryTopicName;
  nh.param<std::string>("previousOptimalTrajectoryTopicName", previousOptimalTrajectoryTopicName,
                        "/local_planner/trajectory_array_cartesian");
  std::cout << "previousOptimalTrajectoryTopicName: " << previousOptimalTrajectoryTopicName.c_str() << std::endl;

  // to stop vehicle
  double stopSpeedKmph;
  nh.param<double>("stopSpeedKmph", stopSpeedKmph, 3.0);
  std::cout << "stopSpeedKmph: " << stopSpeedKmph << std::endl;
  double stopPosition1;
  nh.param<double>("stopPosition1", stopPosition1, 50.0);
  std::cout << "stopPosition1: " << stopPosition1 << std::endl;
  double stopPosition2;
  nh.param<double>("stopPosition2", stopPosition2, 100.0);
  std::cout << "stopPosition2: " << stopPosition2 << std::endl;
  double stopCountMax1;
  nh.param<double>("stopCountMax1", stopCountMax1, 100.0);
  std::cout << "stopCountMax1: " << stopCountMax1 << std::endl;
  double stopCountMax2;
  nh.param<double>("stopCountMax2", stopCountMax2, 100.0);
  std::cout << "stopCountMax2: " << stopCountMax2 << std::endl;

  // to use speed model
  bool useSpeedModel;
  nh.param<bool>("useSpeedModel", useSpeedModel, true);
  std::cout << "useSpeedModel: " << useSpeedModel << std::endl;
  std::string wallTxtFileName;
  nh.param<std::string>("wallTxtFileName", wallTxtFileName, "wall_shift.txt");
  std::cout << "wallTxtFileName: " << wallTxtFileName.c_str() << std::endl;
  double paramSpeedModelDy;
  nh.param<double>("paramSpeedModelDy", paramSpeedModelDy, 1.0);
  std::cout << "paramSpeedModelDy: " << paramSpeedModelDy << std::endl;
  double paramSpeedModelAd;
  nh.param<double>("paramSpeedModelAd", paramSpeedModelAd, 1.0);
  std::cout << "paramSpeedModelAd: " << paramSpeedModelAd << std::endl;
  double paramSpeedModelAmax;
  nh.param<double>("paramSpeedModelAmax", paramSpeedModelAmax, 1.0);
  std::cout << "paramSpeedModelAmax: " << paramSpeedModelAmax << std::endl;
  double paramSpeedModelVoKmph;
  nh.param<double>("paramSpeedModelVoKmph", paramSpeedModelVoKmph, 1.0);
  std::cout << "paramSpeedModelVoKmph: " << paramSpeedModelVoKmph << std::endl;
  double paramSpeedModelDv;
  nh.param<double>("paramSpeedModelDv", paramSpeedModelDv, 1.0);
  std::cout << "paramSpeedModelDv: " << paramSpeedModelDv << std::endl;

  // to use Collision filter
  bool useCollision;
  nh.param<bool>("/useCollision", useCollision, false);
  std::cout << "useCollision: " << useCollision << std::endl;
  bool useOccupancyGridMap;
  nh.param<bool>("useOccupancyGridMap", useOccupancyGridMap, false);
  std::cout << "useOccupancyGridMap: " << useOccupancyGridMap << std::endl;
  double carLength;
  nh.param<double>("carLength", carLength, 10);
  std::cout << "carLength: " << carLength << std::endl;
  double carWidth;
  nh.param<double>("carWidth", carWidth, 10);
  std::cout << "carWidth: " << carWidth << std::endl;
  bool doTransform;
  nh.param<bool>("doTransform", doTransform, true);

  double collisionScalingFactorGammaMin;
  nh.param<double>("collisionScalingFactorGammaMin", collisionScalingFactorGammaMin, 1.0);
  std::cout << "collisionScalingFactorGammaMin: " << collisionScalingFactorGammaMin << std::endl;

  double collisionScalingFactorGammaMax;
  nh.param<double>("collisionScalingFactorGammaMax", collisionScalingFactorGammaMax, 2.0);
  std::cout << "collisionScalingFactorGammaMax: " << collisionScalingFactorGammaMax << std::endl;

  double collisionScalingFactorVMax;
  nh.param<double>("collisionScalingFactorVMax", collisionScalingFactorVMax, 0.0);
  std::cout << "collisionScalingFactorVMax: " << collisionScalingFactorVMax << std::endl;

  double collisionScalingFactorVMin;
  nh.param<double>("collisionScalingFactorVMin", collisionScalingFactorVMin, kmph2mps(20.0));
  std::cout << "collisionScalingFactorVMin: " << collisionScalingFactorVMin << std::endl;

  // copy to global memory
  _useOccupancyGridMap = useOccupancyGridMap;
  _doTransform = doTransform;
  _useCollision = useCollision;
  //...

  // change unit
  // for velocity keeping
  double desiredSpeed = kmph2mps(desiredSpeedKmPerHour);
  double desiredSpeedMinus = kmph2mps(desiredSpeedMinusKmPerHour);
  double desiredSpeedPlus = kmph2mps(desiredSpeedPlusKmPerHour);
  double desiredSpeedDelta = kmph2mps(desiredSpeedDeltaKmPerHour);
  double desiredLatPosition = desiredLatPositionMeter;
  double desiredLatPositionMinus = desiredLatPositionMinusMeter;
  double desiredLatPositionPlus = desiredLatPositionPlusMeter;
  double desiredLatPositionDelta = desiredLatPositionDeltaMeter;

  // for following
  double desiredLongSpeed = kmph2mps(desiredLongSpeedKmPerHour);
  double desiredLongAcceleration = desiredLongAccelerationMeterPerSec;
  double desiredLongPosition = desiredLongPositionMeter;
  double desiredLongPositionMinus = desiredLongPositionMinusMeter;
  double desiredLongPositionPlus = desiredLongPositionPlusMeter;
  double desiredLongPositionDelta = desiredLongPositionDeltaMeter;

  // for model speed
  double paramSpeedModelVo = kmph2mps(paramSpeedModelVoKmph);

  if (_doTransform)
  {
    _transformed.add("traversability", 0.0);
    _transformed.setGeometry(grid_map::Length(100, 100), 0.1, grid_map::Position(50, 25));
  }
  // define subsciber
  ros::Subscriber gridMapSubscriber = nh.subscribe("/Grid_Map_Sim/grid_map_test", queSize, gridMapCallback);
  ros::Subscriber occupancyGridMapSubscriber = nh.subscribe("/urg_sim_map", queSize, occupancyGridMapCallback);

  ros::Subscriber vehicleSpeedSubscriber = nh.subscribe(vehicleSpeedTopicName, queSize, vehicleSpeedCallback);
  ros::Subscriber vehicleAccelerationSubscriber =
      nh.subscribe(vehicleAccelerationTopicName, queSize, vehicleAccelerationCallback);
  ros::Subscriber targetLaneGlobalSubscriber =
      nh.subscribe(targetLaneGlobalTopicName, queSize, targetLaneGlobalCallback);
  ros::Subscriber currentPoseSubscriber = nh.subscribe(currentPoseTopicName, queSize, currentPoseCallback);
  ros::Subscriber desiredStateFollowingSubscriber =
      nh.subscribe(desiredStateFollowingTopicName, queSize, desiredStateFollowingCallback);
  ros::Subscriber previousOptimalTrajectorySubscriber =
      nh.subscribe(previousOptimalTrajectoryTopicName, queSize, previousOptimalTrajectoryCallback);

  ros::Subscriber navGoalSubscriber = nh.subscribe(navGoalTopicName, queSize, navGoalCallback);

  // define publisher
  std::string trajectoryMarkTopicName;
  nh.param<std::string>("trajectoryMarkTopicName", trajectoryMarkTopicName, "/trajectory_mark");
  std::cout << "trajectoryMarkTopicName: " << trajectoryMarkTopicName.c_str() << std::endl;
  std::string trajectoryMarkFilter1TopicName;
  nh.param<std::string>("trajectoryMarkFilter1TopicName", trajectoryMarkFilter1TopicName, "/trajectory_mark_filter1");
  std::cout << "trajectoryMarkFilter1TopicName: " << trajectoryMarkFilter1TopicName.c_str() << std::endl;
  std::string trajectoryMarkFilter2TopicName;
  nh.param<std::string>("trajectoryMarkFilter2TopicName", trajectoryMarkFilter2TopicName, "/trajectory_mark_filter2");
  std::cout << "trajectoryMarkFilter2TopicName: " << trajectoryMarkFilter2TopicName.c_str() << std::endl;
  std::string trajectoryMarkFilter3TopicName;
  nh.param<std::string>("trajectoryMarkFilter3TopicName", trajectoryMarkFilter3TopicName, "/trajectory_mark_filter3");
  std::cout << "trajectoryMarkFilter3TopicName: " << trajectoryMarkFilter3TopicName.c_str() << std::endl;
  std::string optimalTrajectoryMarkTopicName;
  nh.param<std::string>("optimalTrajectoryMarkTopicName", optimalTrajectoryMarkTopicName, "/optimal_trajectory_mark");
  std::cout << "optimalTrajectoryMarkTopicName: " << optimalTrajectoryMarkTopicName.c_str() << std::endl;
  std::string targetLaneInterpVehicleMarkTopicName;
  nh.param<std::string>("targetLaneInterpVehicleMarkTopicName", targetLaneInterpVehicleMarkTopicName,
                        "/target_lane_interp_vehicle_mark");
  std::cout << "targetLaneInterpVehicleMarkTopicName: " << targetLaneInterpVehicleMarkTopicName.c_str() << std::endl;

  std::string previousOptimalTrajectoryMarkTopicName;
  nh.param<std::string>("previousOptimalTrajectoryMarkTopicName", previousOptimalTrajectoryMarkTopicName,
                        "/previousOptimalTrajectoryMarkTopicName");
  std::cout << "previousOptimalTrajectoryMarkTopicName: " << previousOptimalTrajectoryMarkTopicName.c_str()
            << std::endl;

  std::string blindCornerMarkTopicName;
  nh.param<std::string>("blindCornerMarkTopicName", blindCornerMarkTopicName, "/blind_corner_mark");
  std::cout << "blindCornerMarkTopicName: " << blindCornerMarkTopicName.c_str() << std::endl;

  std::string desiredStateTopicName;
  nh.param<std::string>("desiredStateTopicName", desiredStateTopicName, "/desired_state");
  std::cout << "desiredStateTopicName: " << desiredStateTopicName.c_str() << std::endl;

  if (useSpeedModel)
  {
    std::string speedModelTrajectoryMarkTopicName;
    nh.param<std::string>("speedModelTrajectoryMarkTopicName", speedModelTrajectoryMarkTopicName,
                          "/speed_model/speed_model_trajectory_mark");
    std::cout << "speedModelTrajectoryMarkTopicName: " << speedModelTrajectoryMarkTopicName.c_str() << std::endl;

    for (int i = 0; i < maxBlindCornerListSizeForVisualization; i++)
    {
      std::stringstream topicName;
      topicName << speedModelTrajectoryMarkTopicName << std::setw(2) << std::setfill('0') << i;
      _speedModelTrajectoryMarkPublisher[i] = nh.advertise<visualization_msgs::Marker>(topicName.str(), queSize);
    }
  }

  _trajectoryArrayPublisher = nh.advertise<std_msgs::Float64MultiArray>(trajectoryArrayTopicName, queSize);
  _trajectoryMarkPublisher = nh.advertise<visualization_msgs::Marker>(trajectoryMarkTopicName, queSize);
  _trajectoryFilter1MarkPublisher = nh.advertise<visualization_msgs::Marker>(trajectoryMarkFilter1TopicName, queSize);
  _trajectoryFilter2MarkPublisher = nh.advertise<visualization_msgs::Marker>(trajectoryMarkFilter2TopicName, queSize);
  _trajectoryFilter3MarkPublisher = nh.advertise<visualization_msgs::Marker>(trajectoryMarkFilter3TopicName, queSize);
  _optimalTrajectoryMarkPublisher = nh.advertise<visualization_msgs::Marker>(optimalTrajectoryMarkTopicName, queSize);
  _targetLaneInterpVehicleMarkPublisher =
      nh.advertise<visualization_msgs::Marker>(targetLaneInterpVehicleMarkTopicName, queSize);
  _testLeadingVehicleMarkPublisher = nh.advertise<visualization_msgs::Marker>("test_leading_vehicle_mark", queSize);
  _previousOptimalTrajectoryMarkPublisher =
      nh.advertise<visualization_msgs::Marker>(previousOptimalTrajectoryMarkTopicName, queSize);
  _blindCornerMarkPublisher = nh.advertise<visualization_msgs::Marker>(blindCornerMarkTopicName, queSize);
  _desiredStatePublisher = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(desiredStateTopicName, queSize);

  ros::Publisher occMapVisualizer = nh.advertise<nav_msgs::OccupancyGrid>("grid_map_visual", 10, true);

  // copy to global memory
  _resamplingThreshold = resamplingThreshold;

  // starting ros loop
  ros::Rate loopRate(samplingTimeHz);  // by Hz

  // for testing following
  ros::Time beginTime = ros::Time::now();
  ros::Time beforeTime = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();
    double leadingVehiclePosition = 0.0;

    std::vector<std::vector<std::vector<double> > > trajectoryArrayList;
    computeTrajectoryArray(trajectoryArrayList, mapFrameName,

                           // for velocity keeping
                           desiredSpeed, desiredSpeedMinus, desiredSpeedPlus, desiredSpeedDelta, desiredLatPosition,
                           desiredLatPositionMinus, desiredLatPositionPlus, desiredLatPositionDelta,

                           // for following, stopping
                           desiredLongSpeed, desiredLongAcceleration, desiredLongPosition, desiredLongPositionMinus,
                           desiredLongPositionPlus, desiredLongPositionDelta,

                           timeMinSec, timeMaxSec, timeDeltaSec, kjlong, ktlong, kplong, kjlat, ktlat, kplat, klong,
                           klat, resamplingThreshold, arcLengthTickMeter,

                           roadMarginRight, roadMarginLeft, maxCentrifugalForce, maxAcceleration, maxCurvature,

                           resamplingTickTime, polyTickTime, usePreviousTrajectoryForInit, isDrawTrajectory,
                           // velocity keeping:0, following:1
                           freneMode,

                           leadingVehiclePosition, stopSpeedKmph, stopPosition1, stopPosition2, stopCountMax1,
                           stopCountMax2,

                           useSpeedModel, wallTxtFileName, paramSpeedModelVo, paramSpeedModelDy, paramSpeedModelAd,
                           paramSpeedModelAmax, paramSpeedModelDv,

                           carLength, carWidth, collisionScalingFactorGammaMin, collisionScalingFactorGammaMax,
                           collisionScalingFactorVMin, collisionScalingFactorVMax);
    publishTrajectoryArray(trajectoryArrayList);
    if (_doTransform)
    {
      nav_msgs::OccupancyGrid grid;
      grid_map::GridMapRosConverter::toOccupancyGrid(_transformed, "traversability", 0, 1, grid);
      occMapVisualizer.publish(grid);
    }
    loopRate.sleep();
  }
}
