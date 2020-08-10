#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <signal.h>
#include <boost/thread.hpp>

boost::mutex mtx;

class StairTraversal
{
    private:
    bool stair_detection_enabled = false;
    ros::Subscriber ar_pose_msg;
    ros::Subscriber odom;
    ros::Publisher msg;
    geometry_msgs::Twist vel_cmd;
    double x_cor, y_cor, z_cor, x, y, z, w;
    std::string vel_cmd_topic = "/cmd_vel";
    bool not_exit = true;
    int id;
    ros::NodeHandle nh;
    std::unique_ptr<boost::thread> tr;

    public:
    StairTraversal(ros::NodeHandle& nh)
    {
	    //std::cout<<"Inside constructor"<<std::endl;
        //if(!ros::ok())
        //    exit(15);
        this->nh = nh;
        std::cout<<"inside constructor"<<std::endl;
        //boost::thread sub{&StairTraversal::subs, this};
        //boost::thread tr{&StairTraversal::traversal, this};
        //sub.join();
        //tr.join();
        msg = nh.advertise<geometry_msgs::Twist>(vel_cmd_topic, 10);
        odom = nh.subscribe("/odom", 100, &StairTraversal::odomcb, this);
	    //std::coutpublishTransformLoop<<"Inside constructor  2"<<std::endl;
        ar_pose_msg = nh.subscribe("/ar_pose_marker", 100, &StairTraversal::markercb, this); 
        //boost::thread tr{&StairTraversal::traversal, this};
        tr = std::make_unique<boost::thread>(&StairTraversal::traversal, this);
        //sub.join();
        //tr.join(); 
        //ros::spin();    
    }

    ~StairTraversal()
    {
        tr->join();
    }

    void subs()
    {
        mtx.lock();
        //ros::NodeHandle nh("~");
        std::cout<<"subs"<<std::endl;
        //msg = this->nh.advertise<geometry_msgs::Twist>(vel_cmd_topic, 100);
        odom = this->nh.subscribe("/odom", 100, &StairTraversal::odomcb, this);
        ar_pose_msg = this->nh.subscribe("/ar_pose_marker", 100, &StairTraversal::markercb, this);
        
        mtx.unlock();
    }

    void odomcb(const nav_msgs::Odometry::ConstPtr& data)
    {
	    std::cout<<"Inside odomcb"<<std::endl;
        this->x_cor = data->pose.pose.position.x;
        this->y_cor = data->pose.pose.position.y;
        this->z_cor = data->pose.pose.position.z;
        this->x = data->pose.pose.orientation.x;
        this->y = data->pose.pose.orientation.y;
        this->z = data->pose.pose.orientation.z;
        this->w = data->pose.pose.orientation.w;
    }

    void markercb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& data)
    {
        //signal(SIGABRT, signal_handler);
	    std::cout<<"Inside markercb"<<std::endl;
        ar_track_alvar_msgs::AlvarMarker ar;
        if(data->markers.size()>0)
        {
            this->id = data->markers[0].id;
        }
        else
            this->id = 20000; 
    }

    void traversal()
    {
        mtx.lock();
        ros::Rate r(20);
        while(ros::ok())
        {
            r.sleep();
            std::cout<<"Traversal"<<std::endl;
            //if(!ros::ok())
                //exit(15);
            if(this->id == 0 || this->id == 255)
            {
                //vel_cmd.linear.x = 0.0;
                //vel_cmd.linear.y = 0.0;
                //vel_cmd.linear.z = 0.0;
                //vel_cmd.angular.x = 0.0;
                //vel_cmd.angular.y = 0.0;
                //vel_cmd.angular.z = 0.0;
                //msg.publish(vel_cmd);
                std::cout<<"Marker with ID 0"<<std::endl;
                std::cout<<"At level 0"<<std::endl;
                
                double degrees = 0;
                std::cout<<" "<<degrees<<" "<<std::endl;
                
                while(ros::ok() && degrees>-89.00)
                {
                    tf2::Quaternion q(this->x, this->y, this->z, this->w);
                    tf2::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);
                    degrees = yaw * (180.0/3.141592653589793238463);
                    std::cout<<" "<<degrees<<" "<<std::endl;
                    turn(1);
                    /*std::cout<<this->id<<std::endl;
                    if(!ros::ok())
                        exit(15);
                    tf2::Quaternion q1(this->x, this->y, this->z, this->w);
                    tf2::Matrix3x3 m1(q1);
                    double r, p, y;
                    m1.getRPY(r, p, y);
                    double deg = y * (180.0/3.141592653589793238463);
                    double diff = deg - degrees;
                    std::cout<<deg<<" "<<degrees<<" "<<diff<<std::endl;
                    if(diff>180)
                        diff = 360 - diff;
                    if(diff>=0 && diff<=90)
                    {
                        //vel_cmd.linear.x = 0.0;
                        //vel_cmd.linear.y = 0.0;
                        //vel_cmd.linear.z = 0.0;
                        //vel_cmd.angular.x = 0.0;
                        //vel_cmd.angular.y = 0.0;
                        //vel_cmd.angular.z = 5.0;
                    }
                    if(diff>90)
                        break;*/
                    //msg.publish(vel_cmd);
                }
                //vel_cmd.linear.x = 0.0;
                //vel_cmd.linear.y = 0.0;
                //vel_cmd.linear.z = 0.0;
                //vel_cmd.angular.x = 0.0;
                //vel_cmd.angular.y = 0.0;
                //vel_cmd.angular.z = 0.0;
                //msg.publish(vel_cmd);
                double cur_x = this->x_cor, cur_y = this->y_cor;
                double dist = 0;
                std::cout<<"Moving straight..."<<std::endl;
                while(ros::ok() && dist<11)
                {
                    move(1);
                    //std::cout<<this->id<<std::endl;
                    //vel_cmd.linear.x = 2.0;
                    //vel_cmd.linear.y = 0.0;
                    //vel_cmd.linear.z = 0.0;
                    //vel_cmd.angular.x = 0.0;
                    //vel_cmd.angular.y = 0.0;
                    //vel_cmd.angular.z = 0.0;
                    //msg.publish(vel_cmd);
                    dist = sqrt(pow((cur_x-this->x_cor), 2) + pow((cur_y-this->y_cor), 2));
                }
                //vel_cmd.linear.x = 0.0;
                //vel_cmd.linear.y = 0.0;
                //vel_cmd.linear.z = 0.0;
                //vel_cmd.angular.x = 0.0;
                //vel_cmd.angular.y = 0.0;
                //vel_cmd.angular.z = 0.0;
                //msg.publish(vel_cmd);
                std::cout<<"Turning..."<<std::endl;
                while(ros::ok() && this->id != 1)
                {
                    turn(-1);
                    //std::cout<<this->id<<std::endl;
                    //vel_cmd.linear.x = 0.0;
                    //vel_cmd.linear.y = 0.0;
                    //vel_cmd.linear.z = 0.0;
                    //vel_cmd.angular.x = 0.0;
                    //vel_cmd.angular.y = 0.0;
                    //vel_cmd.angular.z = -5.0;
                    //msg.publish(vel_cmd);
                }
                std::cout<<"Level 1 marker found..."<<std::endl;
                move(0);
                ros::shutdown();
            }
            else
            {
                //move(1);
            }
        }
        mtx.unlock();
    }

    void turn(float dir)
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

    void move(float dir)
    {
        std::cout<<"Moving straight..."<<std::endl;
        vel_cmd.linear.x = dir*2.0;
        vel_cmd.linear.y = 0.0;
        vel_cmd.linear.z = 0.0;
        vel_cmd.angular.x = 0.0;
        vel_cmd.angular.y = 0.0;
        vel_cmd.angular.z = 0.0;
        msg.publish(vel_cmd);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "stair_traversal");
    ros::NodeHandle nh("~");
    StairTraversal st(nh);
    ros::spin();
    return 0; 
}
