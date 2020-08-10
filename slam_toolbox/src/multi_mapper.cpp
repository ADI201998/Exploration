#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include<karto_sdk/Karto.h>
#include<karto_sdk/Mapper.h>

class MultiMapper
{
private:
    /* data */
public:
    MultiMapper(/* args */);
    ~MultiMapper();
    karto::LaserRangeFinder* create_range_finder();
    karto::LocalizedRangeScan* add_scan(sensor_msgs::LaserScan* scan);
    void publish_map(karto::Mapper* mapper);

};

MultiMapper::MultiMapper(/* args */)
{
    
}

MultiMapper::~MultiMapper()
{
}
