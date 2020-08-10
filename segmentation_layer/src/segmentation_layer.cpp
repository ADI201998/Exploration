#include <segmentation_layer.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <multi_level_map_messages/PoseOfCluster.h>

PLUGINLIB_EXPORT_CLASS(segmentation_layer::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
#define PI 3.142857142857143

namespace segmentation_layer
{
/******************************************************************************/

/******************************************************************************/
GridLayer::GridLayer() 
{
    tf = new tf2_ros::TransformListener(tf_buffer);
    tf_buffer.setUsingDedicatedThread(true);
}
/******************************************************************************/

/******************************************************************************/
void GridLayer::onInitialize()
{
    ROS_INFO_STREAM("Loading Plugin .................. ");
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    cluster_pose = nh.subscribe("/pose_of_cluster", 100, &GridLayer::cluster_poseCB, this);
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
        &GridLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}
/******************************************************************************/

/******************************************************************************/
void GridLayer::cluster_poseCB(const multi_level_map_messages::PoseOfCluster::ConstPtr& msg)
{
    ranges.clear();
    angles.clear();
    seq = msg->header.seq;
    this->flag = true;
    ROS_INFO_STREAM("Subscriber");
    ranges.insert(ranges.begin(), std::begin(msg->ranges), std::end(msg->ranges));
    angles.insert(angles.begin(), std::begin(msg->angles), std::end(msg->angles));
    /*for(int i = 0; i < ranges.size(); i++)
    {
        geometry_msgs::PoseStamped global_pose;
        if(getRobotPose(global_pose))
        {
            tf2::Quaternion q(global_pose.pose.orientation.x, global_pose.pose.orientation.y, global_pose.pose.orientation.z, global_pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            if (yaw<=0)
            {
                yaw = yaw + 2*PI;
            }
            double wx = global_pose.pose.position.x + ranges[i]*cos(yaw + angles[i]);
            double wy = global_pose.pose.position.y + ranges[i]*sin(yaw + angles[i]);
            unsigned int mx, my;
            if(worldToMap(wx, wy, mx, my))
            {
                int index = getIndex(mx, my);
                indices.push_back(index);
            }
        }
    }
    std::sort(indices.begin(), indices.end());*/

}
/******************************************************************************/

/******************************************************************************/
void GridLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}
/******************************************************************************/

/******************************************************************************/
void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
}
/******************************************************************************/

/******************************************************************************/
void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
    if (!enabled_ || !flag)
        return;
    ROS_INFO_STREAM("updateBounds ");//<<"    "<<origin_x_<<"     "<<origin_y_<<"     "<<resolution_);
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    if(seq%5==0)
    {
        unsigned int span = getSizeInCellsX();
        for (int j = 0; j < getSizeInCellsY(); j++)
        {
            for (int i = 0; i < span; i++)
            {
                unsigned int it = getIndex(i, j);
                costmap_[it] = NO_INFORMATION;
            }
        }
    }
    for(int i=0; i<ranges.size(); i++)
    {
        //if(ranges[i] > 5.0)
            //continue;
        //ROS_WARN_STREAM(ranges[i]<<"   "<<angles[i]);
        double wx = robot_x + ranges[i]*cos(robot_yaw + angles[i]);
        double wy = robot_y + ranges[i]*sin(robot_yaw + angles[i]);
        unsigned int mx, my;
        if (!worldToMap(wx, wy, mx, my))
        {
            ROS_WARN("Computing map coords failed");
            continue;
        }

        //ROS_WARN_STREAM(wx<<"   "<<wy<<"    "<<mx<<"    "<<my<<"    "<<origin_x_<<"     "<<origin_y_<<"     "<<resolution_);
        unsigned int index = getIndex(mx, my);
        costmap_[index] = LETHAL_OBSTACLE;

        *min_x = std::min(*min_x, wx);
        *min_y = std::min(*min_y, wy);
        *max_x = std::max(*max_x, wx);
        *max_y = std::max(*max_y, wy);
    }
    /*double wx_ = robot_x + cos(robot_yaw);
    double wy_ = robot_y + sin(robot_yaw);
    unsigned int mx, my;
    if (!worldToMap(wx_, wy_, mx, my))
    {
        ROS_WARN("Computing map coords failed");
    }

    //ROS_WARN_STREAM(wx<<"   "<<wy<<"    "<<mx<<"    "<<my<<"    "<<origin_x_<<"     "<<origin_y_<<"     "<<resolution_);
    unsigned int index = getIndex(mx, my);
    costmap_[index] = LETHAL_OBSTACLE;

    *min_x = std::min(*min_x, wx_);
    *min_y = std::min(*min_y, wy_);
    *max_x = std::max(*max_x, wx_);
    *max_y = std::max(*max_y, wy_);*/
}
/******************************************************************************/

/******************************************************************************/
void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
    ROS_INFO_STREAM("Inside updatecosts() ");
    if (!enabled_ || !flag)
        return;
    flag = false;
    //ROS_INFO_STREAM(master_grid.getSizeInCellsX()<<"    "<<master_grid.getSizeInCellsY()<<" "<<min_i<<"  "<<min_j<<"  "<<max_i<<"  "<<max_j);
    //auto map = master_grid.getCharMap();
    //geometry_msgs::PoseStamped global_pose;
    /*if(getRobotPose(global_pose))
    {
        for (int i = 0; i < master_grid.getSizeInCellsX(); i++)
        {
            for (int j = 0; j < master_grid.getSizeInCellsY(); j++)
            {
                tf2::Quaternion q(global_pose.pose.orientation.x, global_pose.pose.orientation.y, global_pose.pose.orientation.z, global_pose.pose.orientation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                if (yaw<=0)
                {
                    yaw = yaw + 2*PI;
                }
                int index = getIndex(i, j);
                double x, y;
                master_grid.mapToWorld(i, j, x, y);
                float rad = atan2(global_pose.pose.orientation.x - x, global_pose.pose.orientation.y - y);
                float new_ang = 2*PI - (yaw - rad);
                if(new_ang>2*PI)
                    new_ang-=2*PI;

                if(std::binary_search(indices.begin(), indices.end(), index))
                    map[index] = costmap_2d::LETHAL_OBSTACLE;
                else
                    map[index] = costmap_2d::FREE_SPACE;
                //ROS_INFO_STREAM(index<<"    "<<map[index]);

            }
        }
        for(int index = 0; index < ranges.size(); index++)
        {
            //ROS_INFO_STREAM(angles[index]<<"        "<<ranges[index]);
            double wx = global_pose.pose.position.x + ranges[index]*cos(yaw + angles[index]);
            double wy = global_pose.pose.position.y + ranges[index]*sin(yaw + angles[index]);
            unsigned int mx, my;
            if(worldToMap(wx, wy, mx, my))
            {
                int index = getIndex(mx, my);
                map[index] = costmap_2d::LETHAL_OBSTACLE;
            }
        }
    }
    else
        ROS_WARN_STREAM("No Robot Pose");*/
    unsigned char* master = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();

    for (int j = min_j; j < max_j; j++)
    {
        unsigned int it = span*j+min_i;
        for (int i = min_i; i < max_i; i++)
        {
            if (costmap_[it] == NO_INFORMATION)
            {
                it++;
                continue;
            }
            //ROS_WARN_STREAM("NOT NO INFO");
            unsigned char old_cost = master[it];
            if (old_cost == NO_INFORMATION || old_cost < costmap_[it])
                master[it] = costmap_[it];
            it++;
        }
    }
    //ranges.clear();
    //angles.clear();
    indices.clear();
    //master_grid.resetMaps();
}
/******************************************************************************/

/******************************************************************************/
bool GridLayer::getRobotPose(geometry_msgs::PoseStamped& global_pose)
{
    global_pose.pose.position.x = 0;
    global_pose.pose.position.y = 0;
    global_pose.pose.position.z = 0;
    global_pose.pose.orientation.w = 0;
    global_pose.pose.orientation.x = 1;
    global_pose.pose.orientation.y = 0;
    global_pose.pose.orientation.z = 0;
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.pose.position.x = 0;
    robot_pose.pose.position.y = 0;
    robot_pose.pose.position.z = 0;
    robot_pose.pose.orientation.w = 0;
    robot_pose.pose.orientation.x = 1;
    robot_pose.pose.orientation.y = 0;
    robot_pose.pose.orientation.z = 0;
    robot_pose.header.frame_id = std::string("base_link");
    robot_pose.header.stamp = ros::Time();
    ros::Time current_time = ros::Time::now();
    std::string robot_map_frame = std::string("map");
    std::string robot_base_frame_ = std::string("base_link");
    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(robot_map_frame, robot_base_frame_, current_time, ros::Duration(1.0));
        tf2::doTransform(robot_pose, global_pose, transform);
    }
    catch (tf2::LookupException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "No Transform available. Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    //tf2::Quaternion q(global_pose.pose.orientation.x, global_pose.pose.orientation.y, global_pose.pose.orientation.z, global_pose.pose.orientation.w);
    //tf2::Matrix3x3 m(q);
    //double roll, pitch, yaw;
    //m.getRPY(roll, pitch, yaw);
    //ROS_INFO_STREAM("Pose of Robot is "<<global_pose.pose.position.x<<"  "<<global_pose.pose.position.y<<"  "<<roll<<"      "<<pitch<<"     "<<yaw);
    return true;
}

} // end namespace
