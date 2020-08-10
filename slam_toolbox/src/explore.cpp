#include "slam_toolbox/slam_toolbox_sync.hpp"

class Explore
{
    std::string base_frame;
    std::string map_frame;
    std::string scan_topic;
    std::string odom_frame;
    ros::NodeHandle nh;

    public:

    class Random : public slam_toolbox::SynchronousSlamToolbox
    {
        std::string hostname;
        int level;
        //ros::NodeHandle nh;
        public:
        Random(std::string hostname_, int level_, ros::NodeHandle& nh) : slam_toolbox::SynchronousSlamToolbox(nh, true)
        {
            //this->nh = nh;
            this->hostname = hostname_;
            this->level = level_;
        }
    };

    Explore(ros::NodeHandle& nh)
    {
        this->nh = nh;
        int i=0;
        std::cout<<"EXPLORE\n";
        //for(int i=0; i<2; i++)
        //{
            //this->nh.param("base_frame", base_frame, std::string("robot_"+std::to_string(i)+"/base_link"));
            //this->nh.param("odom_frame", odom_frame, std::string("robot_"+std::to_string(i)+"/odom"));
            //this->nh.param("scan_topic", scan_topic, std::string("/robot_"+std::to_string(i)+"/base_scan"));
            //this->nh.setParam("base_frame", std::string("robot_"+std::to_string(i)+"/base_link"));
            //this->nh.setParam("odom_frame", std::string("robot_"+std::to_string(i)+"/odom"));
            //this->nh.setParam("scan_topic", std::string("/robot_"+std::to_string(i)+"/base_scan"));
            //std::cout<<"Base Frame = "<<base_frame<<" Odom Frame = "<<odom_frame<<" Scan Topic = "<<scan_topic<<std::endl;
            //Random("robot_"+std::to_string(i), 0, this->nh);
            //slam_toolbox::SynchronousSlamToolbox sst(this->nh);
            //ros::spin();    
        //}
        //ros::spin();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore");
    ros::NodeHandle nh("~");
    int i = 0;
    ros::Rate r(10);
    while(nh.ok())
    {
        //for(i=0; i<2; i++){
        //Explore ex(nh);
        //nh.param("base_frame", base_frame, std::string("robot_"+std::to_string(i)+"/base_link"));
        //nh.param("odom_frame", odom_frame, std::string("robot_"+std::to_string(i)+"/odom"));
        //nh.param("scan_topic", scan_topic, std::string("/robot_"+std::to_string(i)+"/base_scan"));
        nh.setParam("base_frame", std::string("robot_"+std::to_string(i)+"/base_link"));
        nh.setParam("odom_frame", std::string("robot_"+std::to_string(i)+"/odom"));
        nh.setParam("scan_topic", std::string("/robot_"+std::to_string(i)+"/base_scan"));
        std::unique_ptr<Explore::Random> rn(new Explore::Random("assa", 0, nh));
        Explore a(nh);
        std::cout<<"Before spinonce\n";
        ros::spinOnce();
        std::cout<<"after spinonce\n";
        r.sleep();
    
    }
    //ros::spin();
    return 0;
}