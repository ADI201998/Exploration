#include "slam_toolbox/slam_toolbox_common.hpp"
#include <nav_msgs/OccupancyGrid.h>

//********************************************************************//
class Localizer
{
private:
    ros::Subscriber mapsub;
public:
    Localizer();
    //Localizer(karto::OccupancyGrid* pOccupancyGrid);
    ~Localizer();
    void map_callback(const nav_msgs::OccupancyGridConstPtr& data);
};

//********************************************************************//

Localizer::Localizer(/* args */)
{
    ros::NodeHandle nh;
    mapsub = nh.subscribe("/map", 100, &Localizer::map_callback, this);
    //ros::spin();
}

//********************************************************************//

Localizer::~Localizer()
{
}

//********************************************************************//

void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& data)
{
    int h=0, z = 0, m = 0;
    for(int i = 0; i<data->data.size(); i++)
    {
        if(data->data[i]==-1)
            m+=1;
        else if(data->data[i]==0)
            z+=1;
        else
            h+=1;
    }
    std::cout<<"hundred = "<<h<<" zero = "<<z<<" minus one = "<<m<<"\n";
}

//********************************************************************//

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_sub");
    Localizer ms;
    return 0;
}