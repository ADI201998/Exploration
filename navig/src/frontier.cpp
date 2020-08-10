#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
#include <multi_level_map_messages/travelAction.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <math.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <geometry_msgs/Twist.h>

boost::mutex mtx;
#define PI 3.142857142857143
/******************************************************************************/

/******************************************************************************/
typedef actionlib::ActionClient <multi_level_map_messages::travelAction> TravelActionClient;
class Exploration
{
private:
    ros::Subscriber ar_tag;
    ros::Subscriber laser_scan;
    std::vector<ros::Subscriber> map_data_sub;
    ros::Publisher markers;
    ros::Publisher markers_acc;
    ros::Publisher goal_marker;
    ros::Publisher msg;
    std::pair<float, float> fpoint;
    std::vector<std::pair<float, float>> robot_pose;
    std::pair<std::vector<std::pair<float, float>>, std::vector<std::pair<float, float>>> robot_frontier_pose;
    std::string map_frame_;
    std::string map_topic;
    std::string robot_base_frame_;
    std::string robot_base_laser_frame_;
    std::string robot_hostname;
    std::string scan_topic;
    double inflation_radius;
    geometry_msgs::PoseStamped ggpp;
    double robot_tolerance;
    double frontier_tolerance;
    float goal_duration;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener* tf;
    bool is_robot_moving;
    double inflation;
    bool flag0;
    bool flag1; 
    int level_id;
    //double cost = frontier_dist*semantic_label*dist_to_frontier;
    
public:
    Exploration(tf2_ros::Buffer& buffer);
    ~Exploration();
    int robot_id;
    float max_range;
    bool should_append;
    bool should_process;
    bool ar_tag_detected;
    bool stair_transition;
    geometry_msgs::Twist vel_cmd;
    nav_msgs::OccupancyGrid map_data;
    TravelActionClient::GoalHandle gh;
    costmap_2d::Costmap2DROS* cmr;
    costmap_2d::Costmap2D* cm;
    TravelActionClient* travel_action_client_;
    std::map<int , TravelActionClient::GoalHandle> pose_to_frontier;
    //std::vector <std::pair <float, float> > frontier_list;
    std::vector <std::pair<std::pair <float, float>, int> > frontier_list;
    std::vector <std::pair<std::pair <float, float>, int> > blacklist;
    multi_level_map_messages::travelResultConstPtr tr;
    void marker_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& data);
    void laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan);
    void map_data_cb(const nav_msgs::OccupancyGrid::ConstPtr& map);
    bool getRobotPose(geometry_msgs::PoseStamped& global_pose);
    bool sendGoal(std::vector <std::pair<std::pair <float, float>, int> >& frontier_list);
    void transitionCB(TravelActionClient::GoalHandle gh, int robot_id);
    void stairtransitionCB(TravelActionClient::GoalHandle gh, int robot_id);
    void feedbackCB(TravelActionClient::GoalHandle gh, int robot_id);
    std::vector <std::pair<std::pair <float, float>, int> > calculate_frontiers(std::vector<float> ranges, std::vector<float> angles, std::vector<float> intensities, visualization_msgs::MarkerArray& farray);
    std::vector <std::pair<std::pair <float, float>, int> > acceptable_frontiers(std::vector <std::pair<std::pair <float, float>, int> > frontier_pose, visualization_msgs::MarkerArray& farray);
    geometry_msgs::PoseStamped calculate_goal(std::vector <std::pair<std::pair <float, float>, int> >& frontier_list);
    std::string convert_response_flag_to_string(int flag);
    bool is_explored(float x, float y, const nav_msgs::OccupancyGrid map);
    void add_frontier_to_map(int frontier_id, TravelActionClient::GoalHandle& gh);
    bool isServerActive();
    bool in_inflation(float x, float y);
    bool stairTransition(bool flag0, bool flag1);
    void turn(float dir);

};
/******************************************************************************/

/******************************************************************************/
Exploration::Exploration(tf2_ros::Buffer& buffer)
{
    this->flag0 = false;
    this->flag1 = false;
    this->level_id = 0;
    this->ar_tag_detected = false;
    this->stair_transition = false;
    robot_id = 1;
    tf = new tf2_ros::TransformListener(tf_buffer);
    tf_buffer.setUsingDedicatedThread(true);
    this->is_robot_moving = false;
    this->should_append = true;
    should_process = false;
    ros::NodeHandle as_nh;
    ros::NodeHandle nh("frontiers_node");
    this->travel_action_client_ = new TravelActionClient(as_nh, "travel_action_server");
    nh.param("frontiers_node/map_frame", this->map_frame_, std::string("map"));
    nh.param("frontiers_node/robot_base_frame", this->robot_base_frame_, std::string("base_link"));
    nh.param("frontiers_node/local_costmap/inflation_layer/inflation_radius", this->inflation_radius, 1.0);
    nh.param("robot_base_laser_frame", this->robot_base_laser_frame_, std::string("base_laser_link"));
    nh.param("robot_hostname", this->robot_hostname, std::string("robot_0"));
    nh.param("robot_tolerance", this->robot_tolerance, 1.2);
    nh.param("frontier_tolerance", this->frontier_tolerance, 1.5);
    nh.param("map_topic", this->map_topic, std::string("map"));                                                                            
    nh.param("scan_topic", this->scan_topic, std::string("base_scan"));
    nh.param("inflation", this->inflation, 0.5);
    
    //this->map_frame_ = std::string("map");
    //this->robot_base_frame_ = std::string("base_link");
    //this->robot_tolerance = 1.2;
    //this->frontier_tolerance = 1.5;
    //this->robot_hostname = std::string("robot_0");
    this->goal_duration = 30.0;
    ROS_INFO_STREAM(this->robot_base_frame_<<"  "<<this->scan_topic);
    cmr = new costmap_2d::Costmap2DROS("local_costmap", tf_buffer);
    tr = nullptr;
    markers = nh.advertise<visualization_msgs::MarkerArray>("frontiers_pose", 1, true);
    markers_acc = nh.advertise<visualization_msgs::MarkerArray>("frontiers_pose_acc", 1, true);
    goal_marker = nh.advertise<visualization_msgs::Marker>("goal_pose", 1, true);
    msg = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    this->ar_tag = nh.subscribe("/ar_pose_marker", 1, &Exploration::marker_cb, this);
    laser_scan = nh.subscribe(this->scan_topic, 1, &Exploration::laser_scan_cb, this);
    for(int i=0; i<2; i++)
    {
        this->map_topic = std::string("/level_"+std::to_string(i)+"/map");
        map_data_sub.push_back(nh.subscribe(this->map_topic, 1, &Exploration::map_data_cb, this));
    }
}
/******************************************************************************/

/******************************************************************************/
Exploration::~Exploration()
{
    delete cmr;
    delete cm;
}
/****************************************************************************/

/****************************************************************************/
void Exploration::marker_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& data)
{
    //ROS_INFO_STREAM_THROTTLE(2.0, "marker_cb");
    if(data->markers.size()==0)
        return;
    //mtx.lock();
    if((this->flag0 == 1 && this->flag1 == 1) && data->markers.size()==0)
    {
        this->flag0 = false;
        this->flag1 = false;
        int i = 0;
        while(i<50)
        {
            turn(1);
            i++;
        }
    }
    if(data->markers[0].id == 0 || data->markers[0].id == 255)
    {
        this->flag0 = true;
        this->level_id = data->markers[0].id;
        //this->ar_tag_detected = true;
    }
    else if(data->markers[0].id == 1)
    {
        this->flag1 = true;
        this->level_id = data->markers[0].id;
        //this->ar_tag_detected = true;
    }
    //if(this->flag0^this->flag1)
        //this->stair_transition = true;
    if(this->flag0^this->flag1 && !this->stair_transition)
    {
        this->stair_transition = true;
        ROS_INFO_STREAM("AR tag detected");
        stairTransition(this->flag0, this->flag1);
    } 
    //ROS_INFO_STREAM(this->flag0<<"  "<<this->flag1<<"   "<<(this->flag0^this->flag1));  
    //mtx.unlock();
}
/******************************************************************************/

/******************************************************************************/
void Exploration::map_data_cb(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    //ROS_INFO_STREAM("Map cb");
    should_process = true;
    map_data.header = map->header;
    map_data.info = map->info;
    map_data.data = map->data;
    if("map_"+(std::to_string(this->level_id)) != map->header.frame_id)
        return;
    //ROS_INFO_STREAM("Size before= "<<frontier_list.size());
    for(int i=0; i<frontier_list.size(); i++)
    {
        if(frontier_list[i].second != this->level_id)
            continue;
        if(is_explored(frontier_list[i].first.first, frontier_list[i].first.second, map_data))
        {
            //ROS_INFO_STREAM("before = "<<frontier_list[i].size()<<"     "<<j<<"        "<<i);
            frontier_list.erase(frontier_list.begin() + i);
            i-=1;
            //ROS_INFO_STREAM("after = "<<frontier_list[i].size()<<"     "<<j);
        }
        //ROS_INFO_STREAM(frontier_pose[i].first<<"   "<<frontier_pose[i].second<<"     "<<frontier_index);
    }
    //ROS_INFO_STREAM("Size after = "<<frontier_list.size());
    //frontier_list.clear();
}
/******************************************************************************/

/******************************************************************************/
void Exploration::laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if(scan->header.seq%10!=0)
        return;
    if(!should_process)
        return;
    if(this->stair_transition)
        return;
    should_process = false;
    //ROS_INFO_STREAM("Laser cb "<<scan->angle_min);
    visualization_msgs::Marker cur_marker;
    visualization_msgs::MarkerArray farray;
    //markers_acc.publish(farray);
    std::vector <std::pair<std::pair <float, float>, int>> frontier_pose;
    auto intensities = scan->intensities;
    auto ranges = scan->ranges;
    max_range = scan->range_max;
    std::vector<float> angles;
    //Calculate angles
    for(int i=0; i<ranges.size(); i++)
        angles.push_back(scan->angle_min + i*scan->angle_increment);

    //Calculate all frontiers
    frontier_pose = calculate_frontiers(ranges, angles, intensities, farray);
    //ROS_INFO_STREAM("SIZE = "<<frontier_pose.size());
    //ROS_INFO_STREAM(__LINE__);
    //Calculate acceptable frontiers
    frontier_pose = acceptable_frontiers(frontier_pose, farray);
    //ROS_INFO_STREAM("SIZE = "<<frontier_pose.size()<<"          ");
    if(this->should_append)
        frontier_list.insert(frontier_list.begin(), std::begin(frontier_pose), std::end(frontier_pose));
    //ROS_INFO_STREAM(farray.markers.size());
    ROS_INFO_STREAM("SIZE = "<<frontier_list.size());
    markers_acc.publish(farray);
    //ROS_INFO_STREAM(__LINE__);
    if(!isServerActive())
    {
        ROS_WARN_STREAM("Couldn't send the goal. Server not ready");
        //should_append = false;
    }
    else if(!this->is_robot_moving)
    {
        sendGoal(frontier_list);
        //ROS_INFO_STREAM("Goal Sent to the bot.");
        //frontier_pose.clear();
        //farray.markers.clear();
        //angles.clear();
    }
    /*if(this->flag0^this->flag1)
    {
        ROS_INFO_STREAM("AR tag detected");
        stairTransition(this->flag0, this->flag1);
    }*/
}
/******************************************************************************/

/******************************************************************************/
bool Exploration::getRobotPose(geometry_msgs::PoseStamped& global_pose)
{
    //geometry_msgs::PoseStamped global_pose;
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
    robot_pose.header.frame_id = this->robot_base_laser_frame_;
    robot_pose.header.stamp = ros::Time();
    ros::Time current_time = ros::Time::now();
    std::string robot_map_frame = this->map_frame_;
    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(robot_map_frame, robot_base_laser_frame_, current_time, ros::Duration(5.0));
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
/******************************************************************************/

/******************************************************************************/
bool Exploration::isServerActive()
{
    if(!travel_action_client_->waitForActionServerToStart(ros::Duration(2.0)))
    {
        ROS_WARN_STREAM_NAMED("PMM_interaction"," Not able to connect to TravelActionServer ! Trvael Action may not get acted on. Returning false . HANDLE THIS CASE @TODO.");
        return false;
    }
    else
        return true;
}
/******************************************************************************/

/******************************************************************************/
bool Exploration::sendGoal(std::vector <std::pair<std::pair <float, float>, int> >& frontier_list)
{
    visualization_msgs::Marker cur_marker;
    ROS_INFO_STREAM(" Travel Action Server is active. Sending Goal");
    multi_level_map_messages::travelGoal explore_travel_goal;
    multi_level_map_messages::MultiLevelMapPose goal_pose;
    goal_pose.level_id = this->level_id;
    //ROS_INFO_STREAM("       "<<frontier_list.size());
    goal_pose.pose = calculate_goal(frontier_list);
    if(goal_pose.pose.pose.position.x == -10000 && goal_pose.pose.pose.position.y == -10000)
    {
        ROS_WARN_STREAM("Goal couldnt be calculated. Not sending anything");
        return false;
    }
    //ROS_INFO_STREAM(frontier_list.size());
    ggpp = goal_pose.pose;
    explore_travel_goal.robot_id = 1;
    explore_travel_goal.robot_hostname = this->robot_hostname;
    explore_travel_goal.type_of_frontier = 1;
    explore_travel_goal.goal_pose = goal_pose;
    cur_marker.ns ="frontier_marker_cube";
    cur_marker.header.frame_id = map_frame_;
    cur_marker.action = visualization_msgs::Marker::ADD;
    cur_marker.id = 10;
    cur_marker.type = visualization_msgs::Marker::SPHERE;
    cur_marker.color.r = 0.0;
    cur_marker.color.g = 0.0;
    cur_marker.color.b = 1.0;
    cur_marker.color.a = 1.0;
    cur_marker.pose.position.x = goal_pose.pose.pose.position.x;
    cur_marker.pose.position.y = goal_pose.pose.pose.position.y;
    cur_marker.pose.orientation.w = 1;
    cur_marker.scale.x = 0.25;
    cur_marker.scale.y = 0.25;
    cur_marker.scale.z = 0.25;
    cur_marker.text = std::to_string(10);
    goal_marker.publish(cur_marker);
    this->is_robot_moving = true;
    should_append = true;
    gh = travel_action_client_->sendGoal(explore_travel_goal, boost::bind(&Exploration::transitionCB, this, _1, robot_id), boost::bind(&Exploration::feedbackCB,this,_1, robot_id));
    //ROS_INFO_STREAM("SiZe = "<<frontier_list.size());
    //add_frontier_to_map(robot_id, gh);
}
/******************************************************************************/

/******************************************************************************/
void Exploration::transitionCB(TravelActionClient::GoalHandle g, int robot_id)
{
    //ROS_INFO_STREAM("TRANSITION");
    using actionlib::CommState;
    using actionlib::TerminalState;
    multi_level_map_messages::travelResultConstPtr tr = gh.getResult();
    //ROS_INFO_STREAM(__LINE__<<"     "<<(int)(tr->response_flag));      //Gives error
    CommState comm_state = gh.getCommState();
    if(comm_state == CommState::DONE)
    {
        TerminalState terminal_state=gh.getTerminalState();
        if(terminal_state == TerminalState::SUCCEEDED)
        {
            ROS_INFO_STREAM("Transition done ");
            frontier_list.push_back(blacklist[blacklist.size()-1]);
            blacklist.erase(blacklist.end());
            this->should_append = false;
            this->is_robot_moving = false;
        }
        else if (terminal_state == TerminalState::ABORTED)
        {
            if(tr->response_flag == tr->ABORT_THE_ROBOT_FROM_MISSION)
            {
                ROS_ERROR_STREAM("Transition state : ABORTED_THE_ROBOT_FROM_MISSION ");
                this->should_append = false;
                this->is_robot_moving = false;
            }
            else if(tr->response_flag == tr->FAILED_TO_PLAN)
            {
                ROS_WARN_STREAM("Transition state : FAILED_TO_PLAN ");
                this->should_append = false;
                sendGoal(frontier_list);
                //this->is_robot_moving = false;
            }
            else if (tr->response_flag == tr->FAILED_TO_REACH_GOAL)
            {
                ROS_WARN_STREAM("Transition state : FAILED_TO_REACH_GOAL ");
                frontier_list.push_back(blacklist[blacklist.size()-1]);
                blacklist.erase(blacklist.end());
                this->should_append = false;
                this->is_robot_moving = false;
            }
            else
            {
                ROS_INFO_STREAM("Transition state : ?? ");
                this->should_append = false;
                this->is_robot_moving = false;
            }
            
        }
        else if(terminal_state == TerminalState::PREEMPTED)
        {
            ROS_INFO_STREAM("Transition state : PREEMPTED ");
            this->should_append = false;
            this->is_robot_moving = false;
        }
        else if (terminal_state == TerminalState::REJECTED)
        {
            ROS_INFO_STREAM("Transition state :REJECTED ");
            this->should_append = false;
            this->is_robot_moving = false;
        }
        else if (terminal_state == TerminalState::RECALLED)
        {
            ROS_INFO_STREAM("Transition state :RECALLED ");
            this->should_append = false;
            this->is_robot_moving = false;
        }
        else
        {
            ROS_INFO_STREAM("Transition state : SOME STATE");
            this->is_robot_moving = false;
        }
        
    }
    else if(comm_state == CommState::WAITING_FOR_GOAL_ACK)
        ROS_INFO("WAITING_FOR_GOAL_ACK");
    else if(comm_state == CommState::WAITING_FOR_CANCEL_ACK)
        ROS_INFO("WAITING_FOR_CANCEL_ACK");
    else if(comm_state == CommState::ACTIVE)
        ROS_INFO("ACTIVE");
    else if(comm_state == CommState::PENDING)
        ROS_INFO("PENDING");
    else if(comm_state == CommState::PREEMPTING)
        ROS_INFO("PREEMPTING");
    else if(comm_state == CommState::WAITING_FOR_RESULT)
        ROS_INFO("WAITING_FOR_RESULT");
    else
        ROS_INFO("RECALLING");
    //ROS_INFO_STREAM("Transition "<<comm_state);
    //this->is_robot_moving = false;
}
/******************************************************************************/

/******************************************************************************/
void Exploration::feedbackCB(TravelActionClient::GoalHandle gh, int robot_id)
{
    multi_level_map_messages::travelResultConstPtr tr = gh.getResult();
    //ROS_INFO_STREAM("Feedback "<<tr);
    //this->is_robot_moving = false;
}
/******************************************************************************/

/******************************************************************************/
std::vector <std::pair<std::pair <float, float>, int> > Exploration::calculate_frontiers(std::vector<float> ranges, std::vector<float> angles, std::vector<float> intensities, visualization_msgs::MarkerArray& farray)
{
    std::vector <std::pair<std::pair <float, float>, int> > frontier_pose;
    //ROS_INFO_STREAM(__LINE__);
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose))
    {
        return frontier_pose;
    }
    for(int i=1; i<ranges.size(); i++)
    {
        //std::cout<<i<<std::endl;
        if((i-1)!=0 || i!=(ranges.size()-1))
        {
            if(ranges[i]>(max_range - 1))
            {
                intensities.erase(intensities.begin()+i);
                ranges.erase(ranges.begin()+i);
                angles.erase(angles.begin()+i);
                i-=1;
                continue;
            }
        }

        float angle1 = angles[i];//scan->angle_min + i*scan->angle_increment;
        float angle2 = angles[i-1];//scan->angle_min + (i-1)*scan->angle_increment;
        float dist = sqrt(pow((ranges[i]*sin(angle1) - ranges[i-1]*sin(angle2)), 2) + pow((ranges[i]*cos(angle1) - ranges[i-1]*cos(angle2)),2));
        
        if(dist>this->robot_tolerance)
        {
            //ROS_INFO_STREAM(angle1<<"   "<<angle2<<"    "<<dist);
            tf2::Quaternion q(global_pose.pose.orientation.x, global_pose.pose.orientation.y, global_pose.pose.orientation.z, global_pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            if (yaw<=0)
            {
                yaw = yaw + 2*PI;
            }
            float fdist = (ranges[i] + ranges[i-1])/2;
            float fang = (angle1 + angle2)/2;
            fpoint.first = global_pose.pose.position.x + fdist*cos(yaw + fang);
            fpoint.second = global_pose.pose.position.y + fdist*sin(yaw + fang);
            if(in_inflation(fpoint.first, fpoint.second))
                continue;
            std::pair<std::pair<float, float>, int> flevel;
            flevel.first = fpoint;
            flevel.second = this->level_id;
            frontier_pose.push_back(flevel);
            //ROS_INFO_STREAM(frontier_pose.size()<<" "<<flevel.first.first<<"    "<<flevel.first.second<<"   "<<flevel.second);
            //std::pair<float, float> cor;
            //cor.first = global_pose.pose.orientation.x;
            //cor.second = global_pose.pose.orientation.y;
            //robot_pose.push_back(cor);
            //robot_frontier_pose.first = frontier_pose;
            //robot_frontier_pose.second = robot_pose;
            visualization_msgs::Marker cur_marker;
            cur_marker.ns ="frontier_marker_cube";
            cur_marker.header.frame_id = map_frame_;
            cur_marker.action = visualization_msgs::Marker::ADD;
            cur_marker.id = i;
            cur_marker.type = visualization_msgs::Marker::CUBE;
            cur_marker.color.r = 0.0;
            cur_marker.color.g = 0.0;
            cur_marker.color.b = 1.0;
            cur_marker.color.a = 1.0;
            cur_marker.pose.position.x = fpoint.first;
            cur_marker.pose.position.y = fpoint.second;
            cur_marker.pose.orientation.w = 1;
            cur_marker.scale.x = 0.4;
            cur_marker.scale.y = 0.4;
            cur_marker.scale.z = 1.0;
            cur_marker.text = std::to_string(i);
            farray.markers.push_back(cur_marker);
        }
    }
    return frontier_pose;
}
/******************************************************************************/

/******************************************************************************/
std::vector <std::pair<std::pair <float, float>, int> > Exploration::acceptable_frontiers(std::vector <std::pair<std::pair <float, float>, int> > frontier_pose, visualization_msgs::MarkerArray& farray)
{
    //ROS_INFO_STREAM(__LINE__);
    //ROS_INFO_STREAM(frontier_pose.size());
    for(int i=0; i<frontier_pose.size(); i++)
    {
        std::vector<int> frontier_id;
        frontier_id.push_back(i);
        for(int j=i+1; j<frontier_pose.size(); j++)
        {
            float dist = sqrt(pow((frontier_pose[i].first.first - frontier_pose[j].first.first), 2) + pow((frontier_pose[i].first.second - frontier_pose[j].first.second), 2));
            if(dist<=this->frontier_tolerance)
                frontier_id.push_back(j);
        }
        if(frontier_id.size()>1)
        {
            for(int k = frontier_id.size()-1; k>=0; k--)
            {
                frontier_pose.erase(frontier_pose.begin()+frontier_id[k]);
                farray.markers.erase(farray.markers.begin()+frontier_id[k]);
            }
            i-=1;
        }
        farray.markers[i].color.r = 0.0;
        farray.markers[i].color.b = 0.0;
        farray.markers[i].color.g = 1.0;
        frontier_id.clear();
    }
    //should_process = true;
    //ROS_INFO_STREAM(farray.markers.size());
    return frontier_pose;
}
/******************************************************************************/

/******************************************************************************/
geometry_msgs::PoseStamped Exploration::calculate_goal(std::vector <std::pair<std::pair <float, float>, int> >& frontier_list)
{
    geometry_msgs::PoseStamped fp;
    fp.pose.position.x = -10000;
    fp.pose.position.y = -10000;
    float dist_to_goal = 10000;
    geometry_msgs::PoseStamped global_pose;
    int frontier_index = -1;
    int list_index = 0;
    //ROS_INFO_STREAM(__LINE__<<"     "<<frontier_list.size());
    if(getRobotPose(global_pose))
    {
        for(int i=0; i<frontier_list.size(); i++)
        {
            //ROS_INFO_STREAM(this->level_id<<"       "<<frontier_list[i].second);
            if(frontier_list[i].second != this->level_id)
                continue;
            if(in_inflation(frontier_list[i].first.first, frontier_list[i].first.second))
            {
                ROS_INFO_STREAM("costmap");
                frontier_list.erase(frontier_list.begin() + i);
                i-=1;
                continue;
            }
            if(is_explored(frontier_list[i].first.first, frontier_list[i].first.second, map_data))
            {
                //ROS_INFO_STREAM("before = "<<frontier_list[i].size()<<"     "<<j<<"        "<<i);
                ROS_INFO_STREAM("occ grid");
                frontier_list.erase(frontier_list.begin() + i);
                i-=1;
                continue;
                //ROS_INFO_STREAM("after = "<<frontier_list[i].size()<<"     "<<j);
            }
            
            float dist_to_frontier = sqrt(pow((frontier_list[i].first.first - global_pose.pose.position.x), 2) + pow((frontier_list[i].first.second - global_pose.pose.position.y), 2));
            if(dist_to_frontier>1.5 && dist_to_frontier<dist_to_goal)
            {
                dist_to_goal = dist_to_frontier;
                frontier_index = i;
            }
            //ROS_INFO_STREAM(frontier_pose[i].first<<"   "<<frontier_pose[i].second<<"     "<<frontier_index);
        }
    }
    else
    {
        geometry_msgs::PoseStamped fp;
        ROS_INFO("no gp");
        return fp;
    }
    ROS_INFO_STREAM(__LINE__<<" Size = "<<frontier_list.size());
    if(frontier_index == -1) 
        return fp;
    fp.pose.position.x = frontier_list[frontier_index].first.first;
    fp.pose.position.y = frontier_list[frontier_index].first.second;
    fp.pose.position.z = 0;
    fp.pose.orientation.x = 0;
    fp.pose.orientation.y = 0;
    fp.pose.orientation.z = 0;
    fp.pose.orientation.w = 1;
    blacklist.push_back(frontier_list[frontier_index]);
    ROS_INFO_STREAM("Sending Goal ["<<fp.pose.position.x<<", "<<fp.pose.position.y<<", "<<this->level_id<<"] ");
    frontier_list.erase(frontier_list.begin() + frontier_index);
    return fp;
}
/******************************************************************************/

/******************************************************************************/
bool Exploration::is_explored(float x, float y, const nav_msgs::OccupancyGrid map_data)
{
    //ROS_INFO_STREAM("is explored?"<<map_data.info.origin.position.y);
    //ROS_INFO_STREAM(__LINE__<<"     "<<map_data.data.size());
    int y_pos = (y - (float)map_data.info.origin.position.y)/(float)map_data.info.resolution;
    int x_pos = (x - (float)map_data.info.origin.position.x)/(float)map_data.info.resolution;
    //ROS_INFO_STREAM("index is = "<<y_pos<<"   "<<x_pos<<" with position = "<<y<<"   "<<x);
    if(y_pos+2>map_data.info.height || x_pos+2>map_data.info.width || y_pos-2<0 || x_pos-2<0)
    {
        //ROS_WARN_STREAM("Index out of bounds");
        if(in_inflation(x, y))
            return true;
        return false;
    }
    auto m_vals = map_data.data;
    auto m_width = (int)map_data.info.width;
    double res = (float)map_data.info.resolution;
    auto infl = this->inflation/res;
    //ROS_INFO_STREAM(infl<<"     "<<this->inflation<<":  "<<res<<"   "<<this->inflation/res);
    //int index = m_width*(y_pos) + x_pos;
    //int cost = (int)m_vals[index];
    /*int sum = 0;
    for(int i = 0; i<infl; i++)
    {
        for(int j = 0; j<infl; j++)
        {
            sum+=(int)m_vals[m_width*(y_pos + infl/2 - i) + (x_pos + infl/2 - j)];
            //ROS_INFO_STREAM((int)(this->inflation/(2*res)) - i<<"         "<<(int)(this->inflation/(2*res)) - j);
        }
    }*/
    int sum = (int)m_vals[m_width*(y_pos) + (x_pos - 2)] + (int)m_vals[m_width*(y_pos) + (x_pos - 1)] + (int)m_vals[m_width*(y_pos) + (x_pos)] + (int)m_vals[m_width*(y_pos) + (x_pos + 1)] + (int)m_vals[m_width*(y_pos) + (x_pos + 2)] +
            (int)m_vals[m_width*(y_pos - 2) + (x_pos - 2)] + (int)m_vals[m_width*(y_pos - 2) + (x_pos - 1)] + (int)m_vals[m_width*(y_pos - 2) + (x_pos)] + (int)m_vals[m_width*(y_pos - 2) + (x_pos + 1)] + (int)m_vals[m_width*(y_pos - 2) + (x_pos + 2)] +
            (int)m_vals[m_width*(y_pos - 1) + (x_pos - 2)] + (int)m_vals[m_width*(y_pos - 1) + (x_pos - 1)] + (int)m_vals[m_width*(y_pos - 1) + (x_pos)] + (int)m_vals[m_width*(y_pos - 1) + (x_pos + 1)] + (int)m_vals[m_width*(y_pos - 1) + (x_pos + 2)] +
            (int)m_vals[m_width*(y_pos + 2) + (x_pos - 2)] + (int)m_vals[m_width*(y_pos + 2) + (x_pos - 1)] + (int)m_vals[m_width*(y_pos + 2) + (x_pos)] + (int)m_vals[m_width*(y_pos + 2) + (x_pos + 1)] + (int)m_vals[m_width*(y_pos + 2) + (x_pos + 2)] +
            (int)m_vals[m_width*(y_pos + 1) + (x_pos - 2)] + (int)m_vals[m_width*(y_pos + 1) + (x_pos - 1)] + (int)m_vals[m_width*(y_pos + 1) + (x_pos)] + (int)m_vals[m_width*(y_pos + 1) + (x_pos + 1)] + (int)m_vals[m_width*(y_pos + 1) + (x_pos + 2)];
    //ROS_INFO_STREAM("COST is "<<sum);

    if(sum<=-14)
        return false;
    else
        return true;    
}
/******************************************************************************/

/******************************************************************************/
bool Exploration::in_inflation(float x, float y)
{
    //return false;
    cm = cmr->getCostmap();
    unsigned int mx, my;
    if(!cm->worldToMap(x, y, mx, my))
        return true;
    unsigned char cost = cm->getCost(mx, my);
    if(cost<=costmap_2d::LETHAL_OBSTACLE && cost>costmap_2d::FREE_SPACE)
        return true;
    return false;
}
/******************************************************************************/

/******************************************************************************/
std::string Exploration::convert_response_flag_to_string(int flag)
{/*
  uint8 GOAL_REACHED=1
  uint8 GOAL_IN_PROGRESS=0
  uint8 PATH_HAS_BEEN_PLANNED=2
  uint8 PATH_SEGMENT_ACHIVED=3
  uint8 FAILED_TO_REACH_GOAL=254
  uint8 FAILED_TO_PLAN=253
  uint8 PREMPTING_CURRENT_ON_NEW_GOAL=252
  uint8 FAILED_NOT_ABLE_TO_SET_LOCAL_PLANNER=251
  uint8 ABORT_THE_ROBOT_FROM_MISSION=250*/
  //ROS_INFO_STREAM("CONVERT");
  std::string string_flag="UNKNOWN";
  switch(flag)
  {
    case 1: string_flag="GOAL_REACHED"; break;
    case 0: string_flag="GOAL_IN_PROGRESS"; break;
    case 2: string_flag="PATH_HAS_BEEN_PLANNED"; break;
    case 3: string_flag="PATH_SEGMENT_ACHIVED"; break;
    case 252: string_flag="PREMPTING_CURRENT_ON_NEW_GOAL"; break;
    case 253: string_flag="FAILED_TO_PLAN"; break;
    case 254: string_flag="FAILED_TO_REACH_GOAL"; break;
    case 251: string_flag="FAILED_NOT_ABLE_TO_SET_LOCAL_PLANNER"; break;
    case 250: string_flag="ABORT_THE_ROBOT_FROM_MISSION"; break;
  }
  return string_flag;
}
/******************************************************************************/

/******************************************************************************/
void Exploration::add_frontier_to_map(int robot_id, TravelActionClient::GoalHandle& gh)
{
    try
    {
        auto& travel_goal_handle = pose_to_frontier.at(robot_id);
        travel_goal_handle = gh;
    }
    catch(std::out_of_range& oor)
    {
        pose_to_frontier.emplace(robot_id , gh);
    }
}
/******************************************************************************/

/******************************************************************************/
bool Exploration::stairTransition(bool flag0, bool flag1)
{
    ROS_INFO_STREAM("STAIR TRANSITION");
    if(flag0==true && flag1==false)
    {
        visualization_msgs::Marker cur_marker;
        multi_level_map_messages::travelGoal explore_travel_goal;
        multi_level_map_messages::MultiLevelMapPose goal_pose;
        goal_pose.level_id = 1;
        goal_pose.pose.pose.position.x = 18.57;
        goal_pose.pose.pose.position.y = 1.07;
        explore_travel_goal.robot_id = 1;
        explore_travel_goal.robot_hostname = this->robot_hostname;
        explore_travel_goal.type_of_frontier = 2;
        explore_travel_goal.goal_pose = goal_pose;
        cur_marker.ns ="frontier_marker_cube";
        cur_marker.header.frame_id = map_frame_;
        cur_marker.action = visualization_msgs::Marker::ADD;
        cur_marker.id = 10;
        cur_marker.type = visualization_msgs::Marker::SPHERE;
        cur_marker.color.r = 0.0;
        cur_marker.color.g = 0.0;
        cur_marker.color.b = 1.0;
        cur_marker.color.a = 1.0;
        cur_marker.pose.position.x = goal_pose.pose.pose.position.x;
        cur_marker.pose.position.y = goal_pose.pose.pose.position.y;
        cur_marker.pose.position.z = 4.0;
        cur_marker.pose.orientation.w = 1;
        cur_marker.scale.x = 0.25;
        cur_marker.scale.y = 0.25;
        cur_marker.scale.z = 0.25;
        cur_marker.text = std::to_string(10);
        goal_marker.publish(cur_marker);
        this->is_robot_moving = true;
        this->should_append = false;
        this->level_id = 1;
        ROS_INFO_STREAM("Going to level 1");
        gh = travel_action_client_->sendGoal(explore_travel_goal, boost::bind(&Exploration::stairtransitionCB, this, _1, robot_id), boost::bind(&Exploration::feedbackCB,this,_1, robot_id));
    }
    else if(flag1==true && flag0==false)
    {
        visualization_msgs::Marker cur_marker;
        multi_level_map_messages::travelGoal explore_travel_goal;
        multi_level_map_messages::MultiLevelMapPose goal_pose;
        goal_pose.level_id = 0;
        goal_pose.pose.pose.position.x = 6.24;
        goal_pose.pose.pose.position.y = -17.125;
        explore_travel_goal.robot_id = 1;
        explore_travel_goal.robot_hostname = this->robot_hostname;
        explore_travel_goal.type_of_frontier = 2;
        explore_travel_goal.goal_pose = goal_pose;
        cur_marker.ns ="frontier_marker_cube";
        cur_marker.header.frame_id = map_frame_;
        cur_marker.action = visualization_msgs::Marker::ADD;
        cur_marker.id = 10;
        cur_marker.type = visualization_msgs::Marker::SPHERE;
        cur_marker.color.r = 0.0;
        cur_marker.color.g = 0.0;
        cur_marker.color.b = 1.0;
        cur_marker.color.a = 1.0;
        cur_marker.pose.position.x = goal_pose.pose.pose.position.x;
        cur_marker.pose.position.y = goal_pose.pose.pose.position.y;
        cur_marker.pose.position.z = 0.0;
        cur_marker.pose.orientation.w = 1;
        cur_marker.scale.x = 0.25;
        cur_marker.scale.y = 0.25;
        cur_marker.scale.z = 0.25;
        cur_marker.text = std::to_string(10);
        goal_marker.publish(cur_marker);
        this->is_robot_moving = true;
        this->should_append = false;
        this->level_id = 0;
        ROS_INFO_STREAM("Going to level 0");
        gh = travel_action_client_->sendGoal(explore_travel_goal, boost::bind(&Exploration::stairtransitionCB, this, _1, robot_id), boost::bind(&Exploration::feedbackCB,this,_1, robot_id));
    }
    
}
/******************************************************************************/

/******************************************************************************/
void Exploration::stairtransitionCB(TravelActionClient::GoalHandle g, int robot_id)
{
    ROS_INFO_STREAM("STAIR TRANSITIONCB");
    using actionlib::CommState;
    using actionlib::TerminalState;
    multi_level_map_messages::travelResultConstPtr tr = gh.getResult();
    //ROS_INFO_STREAM(__LINE__<<"     "<<(int)(tr->response_flag));      //Gives error
    CommState comm_state = gh.getCommState();
    if(comm_state == CommState::DONE)
    {
        this->stair_transition = false;
        this->should_append = true;
        while (this->flag1^this->flag0)
        {
            turn(-1);
        }
        TerminalState terminal_state=gh.getTerminalState();
        if(terminal_state == TerminalState::SUCCEEDED)
        {
            ROS_INFO_STREAM("Transition done ");
            frontier_list.push_back(blacklist[blacklist.size()-1]);
            blacklist.erase(blacklist.end());
            this->should_append = false;
            this->is_robot_moving = false;
        }
        else if (terminal_state == TerminalState::ABORTED)
        {
            if(tr->response_flag == tr->ABORT_THE_ROBOT_FROM_MISSION)
            {
                ROS_ERROR_STREAM("Transition state : ABORTED_THE_ROBOT_FROM_MISSION ");
                this->should_append = false;
                this->is_robot_moving = false;
            }
            else if(tr->response_flag == tr->FAILED_TO_PLAN)
            {
                ROS_WARN_STREAM("Transition state : FAILED_TO_PLAN ");
                this->should_append = false;
                sendGoal(frontier_list);
                //this->is_robot_moving = false;
            }
            else if (tr->response_flag == tr->FAILED_TO_REACH_GOAL)
            {
                ROS_WARN_STREAM("Transition state : FAILED_TO_REACH_GOAL ");
                //frontier_list.push_back(blacklist[blacklist.size()-1]);
                //blacklist.erase(blacklist.end());
                this->should_append = false;
                this->is_robot_moving = false;
            }
            else
            {
                ROS_INFO_STREAM("Transition state : ?? ");
                this->should_append = false;
                this->is_robot_moving = false;
            }
            
        }
        else if(terminal_state == TerminalState::PREEMPTED)
        {
            ROS_INFO_STREAM("Transition state : PREEMPTED ");
            this->should_append = false;
            this->is_robot_moving = false;
        }
        else if (terminal_state == TerminalState::REJECTED)
        {
            ROS_INFO_STREAM("Transition state :REJECTED ");
            this->should_append = false;
            this->is_robot_moving = false;
        }
        else if (terminal_state == TerminalState::RECALLED)
        {
            ROS_INFO_STREAM("Transition state :RECALLED ");
            this->should_append = false;
            this->is_robot_moving = false;
        }
        else
        {
            ROS_INFO_STREAM("Transition state : SOME STATE");
            this->is_robot_moving = false;
        }
        
    }
    else if(comm_state == CommState::WAITING_FOR_GOAL_ACK)
        ROS_INFO("WAITING_FOR_GOAL_ACK");
    else if(comm_state == CommState::WAITING_FOR_CANCEL_ACK)
        ROS_INFO("WAITING_FOR_CANCEL_ACK");
    else if(comm_state == CommState::ACTIVE)
        ROS_INFO("ACTIVE");
    else if(comm_state == CommState::PENDING)
        ROS_INFO("PENDING");
    else if(comm_state == CommState::PREEMPTING)
        ROS_INFO("PREEMPTING");
    else if(comm_state == CommState::WAITING_FOR_RESULT)
        ROS_INFO("WAITING_FOR_RESULT");
    else
        ROS_INFO("RECALLING");
    //ROS_INFO_STREAM("Transition "<<comm_state);
    //this->is_robot_moving = false;
}
/******************************************************************************/

/******************************************************************************/
void Exploration::turn(float dir)
{
    std::cout<<"Turning...."<<std::endl;
    vel_cmd.linear.x = 0.0;
    vel_cmd.linear.y = 0.0;
    vel_cmd.linear.z = 0.0;
    vel_cmd.angular.x = 0.0;
    vel_cmd.angular.y = 0.0;
    vel_cmd.angular.z = dir*5.0;
    msg.publish(vel_cmd);
}
/******************************************************************************/

/******************************************************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontiers");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    buffer.setUsingDedicatedThread(true);
    Exploration f(buffer);
    ros::spin();
    return 0;
}