#include <ros/ros.h>

class Cluster
{
public:
  std::vector<std::pair<int, int>> coordinates;
  std::vector<std::vector<std::pair<int, int>>> coordinates_each;
  std::vector<float> distance;
  std::vector<std::vector<float>> distance_each;
  float average_dist;
  int num_obj;
  std::vector<float> average_dist_each;
  std::pair<int, int> xy_for_avg_dist;
  std::vector<std::pair<int, int>> xy_for_avg_dist_each;
  std::vector<int> bgr;
  std::vector<std::vector<int>> rgb_each;
  std::string obj_class;
  Cluster()
  {}
};