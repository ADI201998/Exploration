#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

pcl::visualization::PCLVisualizer::Ptr simpleVox(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("VOX"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb1(cloud, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb1, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
  viewer->initCameraParameters ();
  return (viewer);
}

void callback(const PointCloud::ConstPtr& msg)
{
  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>());
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pass1 (new pcl::PointCloud<pcl::PointXYZRGB>()); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vox (new pcl::PointCloud<pcl::PointXYZRGB>());
  /*pcl::PassThrough<pcl::PointXYZRGB> pass;
  pcl::PassThrough<pcl::PointXYZRGB> pass1;
  pass.setInputCloud (msg);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0.0, 1.0);
  pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_pass);*/
  
  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setInputCloud (msg);
  vox.setLeafSize (0.02f, 0.02f, 0.02f);
  vox.filter (*cloud_vox);

  std::vector<int> inliers;

  // created RandomSampleConsensus object and compute the appropriated model
  //pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> (cloud_vox));
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud_vox));

  pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
  ransac.setDistanceThreshold (0.05);
  ransac.computeModel();
  ransac.getInliers(inliers);

  pcl::copyPointCloud (*cloud_vox, inliers, *final);
  
  viewer = simpleVox(cloud_vox);
  viewer = simpleVox(final);
  while (!viewer->wasStopped ())
  {
      viewer->spinOnce();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ransac");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/camera/depth/points", 1, callback);
  ros::spin();
  return 0;
} 