/*
 *  Copyright (c) 2018, TierIV, Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <tf/tf.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>

#include <autoware_msgs/ConfigCompareMapFilter.h>

class BooleanVoxelGrid
{
public:

  BooleanVoxelGrid(const pcl::PointCloud<pcl::PointXYZI>& points, const double& reso, const double& nn_dist);
  ~BooleanVoxelGrid();
  bool isMatched(pcl::PointXYZI point);
  void compare(const pcl::PointCloud<pcl::PointXYZI>& points, pcl::PointCloud<pcl::PointXYZI>& matched, pcl::PointCloud<pcl::PointXYZI>& unmatched);
private:
  bool*** grid_;   // boolean TDF grid
  double reso_;    // voxel resolution [m/pixel]
  double nn_dist_; // NN search distance [m]
  Eigen::Vector3i ijk_size_;  // TDF area [pixel]
  Eigen::Vector3f xyz_min_, xyz_max_; // TDF area [m]
  Eigen::Vector3i ijk_tmp_;
  Eigen::Vector3f xyz_tmp_;

  void initialize(const pcl::PointCloud<pcl::PointXYZI>& points);

  bool xyz2ijk(const Eigen::Vector3f& xyz, Eigen::Vector3i& ijk)
  {
    for (int i = 0; i < 3; ++i)
    {
      if (xyz(i) < xyz_min_(i) || xyz_max_(i) < xyz(i))
      {
        return false;
      }
      ijk(i) = (xyz(i) - xyz_min_(i)) / reso_;
    }
    return true;
  }
  bool ijk2xyz(const Eigen::Vector3i& ijk, Eigen::Vector3f& xyz)
  {
    for (int i = 0; i < 3; ++i)
    {
      if (ijk(i) < 0 || ijk_size_(i) < ijk(i))
      {
        return false;
      }
      xyz(i) = reso_ * ijk(i) - (reso_ / 2.0) + xyz_min_(i);
    }
    return true;
  }
};

BooleanVoxelGrid::BooleanVoxelGrid(const pcl::PointCloud<pcl::PointXYZI>& points, const double& reso, const double& nn_dist)
{
  reso_ = reso;
  nn_dist_ = nn_dist ;
  initialize(points);
}

BooleanVoxelGrid::~BooleanVoxelGrid()
{
  for (int i = 0; i < ijk_size_(0); ++i)
  {
    delete grid_[i];
    for (int j = 0; j < ijk_size_(1); ++j)
    {
      delete grid_[i][j];
    }
  }
}

void BooleanVoxelGrid::initialize(const pcl::PointCloud<pcl::PointXYZI>& points)
{
  assert(points.size() == 0);

  Eigen::Vector4f xyzi_min_, xyzi_max_;
  pcl::getMinMax3D(points, xyzi_min_, xyzi_max_);
  xyz_min_ << xyzi_min_(0), xyzi_min_(1), xyzi_min_(2);
  xyz_max_ << xyzi_max_(0), xyzi_max_(1), xyzi_max_(2);

  ijk_size_ << (xyz_max_(0) - xyz_min_(0)) / reso_ + 1,
               (xyz_max_(1) - xyz_min_(1)) / reso_ + 1,
               (xyz_max_(2) - xyz_min_(2)) / reso_ + 1;

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(points.makeShared());

  grid_ = (bool***)malloc(sizeof(bool**)*(ijk_size_(0)));
  for (int i = 0; i < ijk_size_(0); ++i)
  {
    grid_[i] = (bool**)malloc(sizeof(bool*)*(ijk_size_(1)));
    for (int j = 0; j < ijk_size_(1); ++j)
    {
      grid_[i][j] = (bool*)malloc(sizeof(bool)*(ijk_size_(2)));
    }
  }

#pragma omp parallel for
  for (int i = 0; i < ijk_size_(0); ++i)
  {
    for (int j = 0; j < ijk_size_(1); ++j)
    {
      for (int k = 0; k < ijk_size_(2); ++k)
      {
        // for openmp
        std::vector<int> idx;
        std::vector<float> dist;
        pcl::PointXYZI cent;
        Eigen::Vector3i ijk(0, 0, 0);
        Eigen::Vector3f xyz(0.0, 0.0, 0.0);
        // centroid of grid
        ijk << i, j, k;
        ijk2xyz(ijk, xyz);
        cent.x = xyz(0);
        cent.y = xyz(1);
        cent.z = xyz(2);
        // NN search
        grid_[i][j][k] = (kdtree.radiusSearch(cent, nn_dist_, idx, dist) > 0);
      }
    }
  }
}

bool BooleanVoxelGrid::isMatched(pcl::PointXYZI point)
{
  xyz_tmp_ << point.x, point.y, point.z;
  if (!xyz2ijk(xyz_tmp_, ijk_tmp_))
  {
    return false;
  }
  return grid_[ijk_tmp_(0)][ijk_tmp_(1)][ijk_tmp_(2)];
}

void BooleanVoxelGrid::compare(const pcl::PointCloud<pcl::PointXYZI>& points, pcl::PointCloud<pcl::PointXYZI>& matched, pcl::PointCloud<pcl::PointXYZI>& unmatched)
{
  matched.clear();
  unmatched.clear();
  for (const auto& p : points)
  {
    if (isMatched(p))
    {
      matched.push_back(p);
    }
    else
    {
      unmatched.push_back(p);
    }
  }
}

class CompareMapFilter
{
public:
  CompareMapFilter();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber config_sub_;
  ros::Subscriber sensor_points_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher match_points_pub_;
  ros::Publisher unmatch_points_pub_;

  tf::TransformListener* tf_listener_;

  pcl::KdTreeFLANN<pcl::PointXYZI> tree_;

  BooleanVoxelGrid* btdf_;

  double distance_threshold_;
  double min_clipping_height_;
  double max_clipping_height_;

  std::string map_frame_;

  void configCallback(const autoware_msgs::ConfigCompareMapFilter::ConstPtr& config_msg_ptr);
  void pointsMapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_cloud_msg_ptr);
  void sensorPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& sensorTF_cloud_msg_ptr);
  void searchMatchingCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr match_cloud_ptr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr unmatch_cloud_ptr);
};

CompareMapFilter::CompareMapFilter()
  : nh_()
  , nh_private_("~")
  , tf_listener_(new tf::TransformListener)
  , distance_threshold_(0.2)
  , min_clipping_height_(-2.0)
  , max_clipping_height_(0.5)
  , map_frame_("/map")
{
  nh_private_.param("distance_threshold", distance_threshold_, distance_threshold_);
  nh_private_.param("min_clipping_height", min_clipping_height_, min_clipping_height_);
  nh_private_.param("max_clipping_height", max_clipping_height_, max_clipping_height_);

  config_sub_ = nh_.subscribe("/config/compare_map_filter", 10, &CompareMapFilter::configCallback, this);
  sensor_points_sub_ = nh_.subscribe("/points_raw", 1, &CompareMapFilter::sensorPointsCallback, this);
  map_sub_ = nh_.subscribe("/points_map", 10, &CompareMapFilter::pointsMapCallback, this);
  match_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/match_points", 10);
  unmatch_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/unmatch_points", 10);
}

void CompareMapFilter::configCallback(const autoware_msgs::ConfigCompareMapFilter::ConstPtr& config_msg_ptr)
{
  distance_threshold_ = config_msg_ptr->distance_threshold;
  min_clipping_height_ = config_msg_ptr->min_clipping_height;
  max_clipping_height_ = config_msg_ptr->max_clipping_height;
}

void CompareMapFilter::pointsMapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_cloud_msg_ptr)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*map_cloud_msg_ptr, *map_cloud_ptr);
  tree_.setInputCloud(map_cloud_ptr);

  btdf_ = new BooleanVoxelGrid(*map_cloud_ptr, 0.2, 0.4);

  map_frame_ = map_cloud_msg_ptr->header.frame_id;
}

void CompareMapFilter::sensorPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& sensorTF_cloud_msg_ptr)
{
  const ros::Time sensor_time = sensorTF_cloud_msg_ptr->header.stamp;
  const std::string sensor_frame = sensorTF_cloud_msg_ptr->header.frame_id;

  pcl::PointCloud<pcl::PointXYZI>::Ptr sensorTF_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*sensorTF_cloud_msg_ptr, *sensorTF_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr sensorTF_clipping_height_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  sensorTF_clipping_height_cloud_ptr->header = sensorTF_cloud_ptr->header;
  for (size_t i = 0; i < sensorTF_cloud_ptr->points.size(); ++i)
  {
    if (sensorTF_cloud_ptr->points[i].z > min_clipping_height_ &&
        sensorTF_cloud_ptr->points[i].z < max_clipping_height_)
    {
      sensorTF_clipping_height_cloud_ptr->points.push_back(sensorTF_cloud_ptr->points[i]);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr mapTF_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  try
  {
    tf_listener_->waitForTransform(map_frame_, sensor_frame, sensor_time, ros::Duration(3.0));
    pcl_ros::transformPointCloud(map_frame_, sensor_time, *sensorTF_clipping_height_cloud_ptr, sensor_frame,
                                 *mapTF_cloud_ptr, *tf_listener_);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Transform error: %s", ex.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr mapTF_match_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr mapTF_unmatch_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  searchMatchingCloud(mapTF_cloud_ptr, mapTF_match_cloud_ptr, mapTF_unmatch_cloud_ptr);

  sensor_msgs::PointCloud2 mapTF_match_cloud_msg;
  pcl::toROSMsg(*mapTF_match_cloud_ptr, mapTF_match_cloud_msg);
  mapTF_match_cloud_msg.header.stamp = sensor_time;
  mapTF_match_cloud_msg.header.frame_id = map_frame_;
  mapTF_match_cloud_msg.fields = sensorTF_cloud_msg_ptr->fields;

  sensor_msgs::PointCloud2 sensorTF_match_cloud_msg;
  try
  {
    pcl_ros::transformPointCloud(sensor_frame, mapTF_match_cloud_msg, sensorTF_match_cloud_msg, *tf_listener_);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Transform error: %s", ex.what());
    return;
  }
  match_points_pub_.publish(sensorTF_match_cloud_msg);

  sensor_msgs::PointCloud2 mapTF_unmatch_cloud_msg;
  pcl::toROSMsg(*mapTF_unmatch_cloud_ptr, mapTF_unmatch_cloud_msg);
  mapTF_unmatch_cloud_msg.header.stamp = sensor_time;
  mapTF_unmatch_cloud_msg.header.frame_id = map_frame_;
  mapTF_unmatch_cloud_msg.fields = sensorTF_cloud_msg_ptr->fields;

  sensor_msgs::PointCloud2 sensorTF_unmatch_cloud_msg;
  try
  {
    pcl_ros::transformPointCloud(sensor_frame, mapTF_unmatch_cloud_msg, sensorTF_unmatch_cloud_msg, *tf_listener_);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Transform error: %s", ex.what());
    return;
  }
  unmatch_points_pub_.publish(sensorTF_unmatch_cloud_msg);
}

void CompareMapFilter::searchMatchingCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr match_cloud_ptr,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr unmatch_cloud_ptr)
{
  match_cloud_ptr->points.clear();
  unmatch_cloud_ptr->points.clear();

  match_cloud_ptr->points.reserve(in_cloud_ptr->points.size());
  unmatch_cloud_ptr->points.reserve(in_cloud_ptr->points.size());

  btdf_->compare(*in_cloud_ptr, *match_cloud_ptr, *unmatch_cloud_ptr);

  // std::vector<int> nn_indices(1);
  // std::vector<float> nn_dists(1);
  // for (size_t i = 0; i < in_cloud_ptr->points.size(); ++i)
  // {
  //   tree_.nearestKSearch(in_cloud_ptr->points[i], 1, nn_indices, nn_dists);
  //   if (nn_dists[0] <= distance_threshold_)
  //   {
  //     match_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
  //   }
  //   else
  //   {
  //     unmatch_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
  //   }
  // }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compare_map_filter");
  CompareMapFilter node;
  ros::spin();

  return 0;
}
