#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <fstream>
#include <sstream>
#include <ios>
#include <boost/algorithm/string.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cluster.h>

class SegClustering : public  Cluster
{
public:
    ros::NodeHandle nh_;
    bool present;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher cluster_pose_pub;
    std::vector<std::vector<std::string>> class_data;
    std::vector<Cluster> cluster;
    bool flag;
    float resolution;
    SegClustering();
    ~SegClustering();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void depthCB(const sensor_msgs::ImageConstPtr& msg);
    bool clustering(cv_bridge::CvImagePtr cv_ptr, int width, int height, int b, int g, int r);
};