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
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/don.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

pcl::visualization::PCLVisualizer::Ptr simpleVox(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("VOX"));
  viewer->setBackgroundColor(255, 255, 255);
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pass (new pcl::PointCloud<pcl::PointXYZRGB>());
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
  
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud_vox);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud_vox);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud_vox);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  //pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer = simpleVis(colored_cloud);
  while (!viewer->wasStopped ())
  {
      viewer->spinOnce();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "don");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/camera/depth/points", 1, callback);
  ros::spin();
  return 0;
} 