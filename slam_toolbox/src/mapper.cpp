#include <ros/ros.h>
#include <multi_level_map_messages/ScanWithPose.h>
#include <multi_level_map_messages/MultiLevelOccupancyMapData.h>
#include <multi_level_map_messages/LevelDataWithOccupancyMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <boost/thread.hpp>
#include "slam_toolbox/slam_toolbox_sync.hpp"


boost::mutex mtx;
/****************************************************************************/
class Mapper
{
private:
    ros::Subscriber laser_scan;
    std::vector<ros::Subscriber> map_data;
    ros::Subscriber ar_tag;
    ros::Publisher scan_with_pose;
    ros::Publisher multi_level_map;
    std::string robot_hostname_;
    std::string robot_laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string scan_topic_;
    std::string map_name_;
    double resolution_;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener* tf;
    int level_id;
    int ar_val;
    bool flag0;
    bool flag1;
    ros::NodeHandle nh_;
    std::unique_ptr<boost::thread> sst;
    std::vector<bool> mapper_created;
    std::vector<ros::NodeHandle> nh_sst;
    
    public:
    Mapper(tf2_ros::Buffer& buffer);
    ~Mapper();
    bool should_process;
    void laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan);
    void map_data_cb(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void marker_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& data);
    bool getRobotSensorPose(geometry_msgs::Pose& pose);
    bool getRobotSensorPoseStamped(geometry_msgs::PoseStamped& pose);
    void slam_mapper(ros::NodeHandle &nh_sst);
};
/****************************************************************************/

/****************************************************************************/
Mapper::Mapper(tf2_ros::Buffer& buffer)
{
    tf = new tf2_ros::TransformListener(tf_buffer);
    tf_buffer.setUsingDedicatedThread(true);
    this->flag0 = false;
    this->flag1 = false;
    this->level_id = 0;
    //nh_ = nh;
    ros::NodeHandle nh("mapper");
    this->robot_hostname_ = std::string("robot_0");
    this->mapper_created.push_back(false);
    this->mapper_created.push_back(false);
    nh.param("laser_link", this->robot_laser_frame_, std::string("base_laser_link"));
    this->scan_with_pose = nh.advertise<multi_level_map_messages::ScanWithPose>("/out_localized_scan", 10, true);
    this->multi_level_map = nh.advertise<multi_level_map_messages::MultiLevelOccupancyMapData>("/multi_level_map_topic", 10, true);
    this->laser_scan = nh.subscribe("/base_scan", 1, &Mapper::laser_scan_cb, this);
    for(int i=0; i<2; i++)
    {
        //std::string map_topic = std::string("/level_"+std::to_string(i)+"/map");
        map_data.push_back(nh.subscribe("/level_"+std::to_string(i)+"/map", 1, &Mapper::map_data_cb, this));
    }
    this->ar_tag = nh.subscribe("/ar_pose_marker", 1, &Mapper::marker_cb, this);
    slam_mapper(nh);
    //this->sst = std::make_unique<boost::thread>(boost::bind(&Mapper::slam_mapper, this));
}
/****************************************************************************/

/****************************************************************************/
Mapper::~Mapper()
{
    //this->sst->join();

}
/****************************************************************************/

/****************************************************************************/
void Mapper::slam_mapper(ros::NodeHandle &nh_sst)
{
    //mtx.lock();
    double map_update_interval;
    if(!nh_sst.getParam("map_update_interval", map_update_interval))
        map_update_interval = 10.0;
    ros::Rate r(1.0 / map_update_interval);

    std::vector<slam_toolbox::SynchronousSlamToolbox*> dum;
    dum.push_back(nullptr);
    dum.push_back(nullptr);
     while(ros::ok())
    {
        ros::spinOnce();
        nh_sst.param("odom_frame", this->odom_frame_, std::string("odom"));
        nh_sst.param("map_frame", this->map_frame_, std::string("map"));
        nh_sst.param("base_frame", this->base_frame_, std::string("base_link"));
        nh_sst.param("resolution", this->resolution_, 0.05);
        if(!(this->flag0^this->flag1) && this->level_id == 0)
        {
            //std::unique_ptr<slam_toolbox::SynchronousSlamToolbox> sst_mapper;
            ROS_INFO_STREAM("Mapper for level "<<this->level_id);
            if(!this->mapper_created[0])
            {
                //nh_sst.param("mapper/map_name", this->map_name_, std::string("/map_0"));
                //nh_sst.param("mapper/should_process", this->should_process, true);
                //nh_.param("scan_topic", this->scan_topic_, std::string("/base_scan"));
                std::unique_ptr<slam_toolbox::SynchronousSlamToolbox> sst_mapper = std::make_unique<slam_toolbox::SynchronousSlamToolbox>(nh_sst, true, this->level_id);
                while(ros::ok() && !(this->flag0^this->flag1))
                {
                    //std::cout<<"spin once"<<std::endl;
                    ros::spinOnce();
                    r.sleep();
                }
                //transfer ownership to normal pointer
                ROS_INFO_STREAM("Changing level");
                sst_mapper->should_process = false;
                dum[0] = sst_mapper.release();
                //delete nh_sst[0];
                //delete a;
            }
            else 
            {
                //transfer ownership of normal pointer to unique pointer
                ROS_INFO_STREAM("transfer ownership of normal pointer to unique pointer");
                std::unique_ptr<slam_toolbox::SynchronousSlamToolbox> res1{dum[0]};
                //res1 = std::make_unique<slam_toolbox::SynchronousSlamToolbox>(nh_);;
                while(ros::ok() && !(this->flag0^this->flag1))
                {
                    ros::spinOnce();
                    r.sleep();
                }
                //transfer ownership to normal pointer
                ROS_INFO_STREAM("tChanging level");
                res1->should_process = false;
                dum[0] = res1.release();
            }
        }
        else if(!(this->flag0^this->flag1) && this->level_id == 1)
        {
            ROS_INFO_STREAM("Mapper for level "<<this->level_id);
            if(!this->mapper_created[1])
            {
                nh_sst.param("map_name", this->map_name_, std::string("/map"));
                std::unique_ptr<slam_toolbox::SynchronousSlamToolbox> sst_mapper = std::make_unique<slam_toolbox::SynchronousSlamToolbox>(nh_sst, true, this->level_id);
                while(ros::ok() && !(this->flag0^this->flag1))
                {
                    //std::cout<<"spin";
                    ros::spinOnce();
                    r.sleep();
                }
                //transfer ownership to normal pointer
                ROS_INFO_STREAM("Changing level");
                sst_mapper->should_process = false;
                dum[1] = sst_mapper.release();
                //delete nh_sst[1];
            }
            else 
            {
                //transfer ownership of normal pointer to unique pointer
                std::unique_ptr<slam_toolbox::SynchronousSlamToolbox> res1{dum[1]};
                while(ros::ok() && !(this->flag0^this->flag1))
                {
                    ros::spinOnce();
                    r.sleep();
                }
                //transfer ownership to normal pointer
                ROS_INFO_STREAM("Changing level");
                res1->should_process = false;
                dum[1] = res1.release();
            }
        }
        r.sleep();
    }
    //mtx.unlock();
}
/****************************************************************************/

/****************************************************************************/
void Mapper::marker_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& data)
{
    //ROS_INFO_STREAM_THROTTLE(2.0, "marker_cb");
    if(data->markers.size()==0)
        return;
    //mtx.lock();
    if((this->flag0 == 1 && this->flag1 == 1) && data->markers.size()==0)
    {
        this->flag0 = false;
        this->flag1 = false;
    }
    if(data->markers[0].id == 0 || data->markers[0].id == 255)
    {
        this->flag0 = true;
        this->level_id = data->markers[0].id;
    }
    else if(data->markers[0].id == 1)
    {
        this->flag1 = true;
        this->level_id = data->markers[0].id;
    } 
    //ROS_INFO_STREAM(this->flag0<<"  "<<this->flag1<<"   "<<(this->flag0^this->flag1));  
    //mtx.unlock();
}
/****************************************************************************/

/****************************************************************************/
void Mapper::map_data_cb(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    //mtx.lock();
    //ROS_INFO_STREAM("sa");
    multi_level_map_messages::MultiLevelOccupancyMapData momd;
    momd.number_of_levels = 2;
    momd.map_data_id = 0;
    multi_level_map_messages::LevelDataWithOccupancyMap level;
    level.grid.header = map->header;
    level.grid.info = map->info;
    level.grid.data = map->data;
    level.level_id = this->level_id;
    level.level_name = std::to_string(this->level_id);
    momd.level_occupancy_maps.push_back(level);
    multi_level_map.publish(momd);
    //mtx.unlock();
}
/****************************************************************************/

/****************************************************************************/
void Mapper::laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    geometry_msgs::Pose pose;
    auto s = scan->ranges.size();
    if(getRobotSensorPose(pose))
    {
        multi_level_map_messages::ScanWithPose swp;
        swp.robot_hostname = this->robot_hostname_;
        swp.pose_of_scan = pose;
        swp.scan.header.stamp = scan->header.stamp;
        swp.scan.header.seq = scan->header.seq;
        swp.scan.header.frame_id = scan->header.frame_id;
        swp.scan.angle_min =scan->angle_min;
        swp.scan.angle_max = scan->angle_max;
        swp.scan.angle_increment = scan->angle_increment;
        swp.scan.range_max = scan->range_max;
        swp.scan.range_min = scan->range_min;
        swp.scan.time_increment = scan->time_increment;
        swp.scan.scan_time = scan->scan_time;
        swp.scan.time_increment =scan->time_increment;
        scan_with_pose.publish(swp);
    }
    else
    {
       ROS_WARN_STREAM_THROTTLE(1," Unable to get the pose of the robot, not publishing the ScanWithPose msg for robot.");
    }
}
/****************************************************************************/

/****************************************************************************/
bool Mapper::getRobotSensorPose(geometry_msgs::Pose& pose)
{
    geometry_msgs::PoseStamped poseStamped;
    if(getRobotSensorPoseStamped(poseStamped))
    {
        pose = poseStamped.pose;
    }
    else
        return false;
}
/****************************************************************************/

/****************************************************************************/
bool Mapper::getRobotSensorPoseStamped(geometry_msgs::PoseStamped &pose)
{
    geometry_msgs::PoseStamped global_pose;
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
    robot_pose.header.frame_id = robot_laser_frame_;
    robot_pose.header.stamp = ros::Time();
    ros::Time current_time = ros::Time::now();
    std::string robot_map_frame = map_frame_;
    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(robot_laser_frame_, robot_map_frame, current_time, ros::Duration(1.0));
        tf2::doTransform(robot_pose, global_pose, transform);
        //std::cout<<"www"<<std::endl;
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
    pose.pose.position.x = global_pose.pose.position.x;
    pose.pose.position.y = global_pose.pose.position.x;
    pose.pose.position.z = global_pose.pose.position.x;
    pose.pose.orientation.x = global_pose.pose.orientation.x;
    pose.pose.orientation.y = global_pose.pose.orientation.y;
    pose.pose.orientation.z = global_pose.pose.orientation.z;
    pose.pose.orientation.w = global_pose.pose.orientation.w;
    pose.header.frame_id = global_pose.header.frame_id;
    pose.header.stamp = global_pose.header.stamp;
    return true;
}
/****************************************************************************/

/****************************************************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapper");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    buffer.setUsingDedicatedThread(true);
    Mapper m(buffer);
    //ros::spin();
    return 0;
}
/****************************************************************************/