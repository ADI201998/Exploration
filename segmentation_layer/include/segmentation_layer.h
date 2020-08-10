#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <multi_level_map_messages/PoseOfCluster.h>

namespace segmentation_layer
{

class GridLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
    GridLayer();
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener* tf;
    ros::Subscriber cluster_pose;
    std::vector<float> ranges;
    std::vector<float> angles;
    std::vector<int> indices;
    bool flag;
    int seq;
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    bool isDiscretized()
    {
    return true;
    }
    void cluster_poseCB(const multi_level_map_messages::PoseOfCluster::ConstPtr& msg);
    bool getRobotPose(geometry_msgs::PoseStamped& global_pose);
    virtual void matchSize();

private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif