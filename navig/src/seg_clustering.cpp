#include <seg_cluster.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <multi_level_map_messages/PoseOfCluster.h>

#define PI 3.141592654
/******************************************************************************/

/******************************************************************************/

SegClustering::SegClustering() : it_(nh_)
{
  ROS_INFO_STREAM("Loading CSV File .....");
  std::ifstream file("/home/cair/sim_ws/src/navig/config/seg_class.csv");
  std::string line = "";
  while(getline(file, line))
  {
    std::vector<std::string> row;
    boost::algorithm::split(row, line, boost::is_any_of(","));
    class_data.push_back(row);
  }
  file.close();
  class_data.erase(class_data.begin() + 0);
  ROS_INFO_STREAM("CSV File loaded .....");
  cluster_pose_pub = nh_.advertise<multi_level_map_messages::PoseOfCluster>("/pose_of_cluster", 100, true);
  image_sub_ = it_.subscribe("/segmented", 1, &SegClustering::imageCb, this);
  depth_sub_ = it_.subscribe("/camera/depth/image_raw", 1, &SegClustering::depthCB, this);
}
/******************************************************************************/

/******************************************************************************/
SegClustering::~SegClustering()
{
  
}
/******************************************************************************/

/******************************************************************************/
void SegClustering::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  this->flag = true;
  cluster.clear();
  resolution = (float)86/(float)msg->width;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    ROS_INFO_STREAM("Accepted Segmented Image");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  std::vector<std::vector<int>> col;
  for(std::vector<std::vector<std::string>>::iterator it = class_data.begin(), end = class_data.end(); it!=end; it++)
  {
    //ROS_INFO_STREAM(class_data[k][0]);
    if(*(it->begin()) == "floor" || *(it->begin()) == "wall" || *(it->begin()) == "ceiling" || *(it->begin()) == "sky" )
      continue;
    //if(class_data[k][0] != "person")
      //continue;
    if(clustering(cv_ptr, (int)msg->width, (int)msg->height, stoi(*(it->begin()+3)), stoi(*(it->begin()+2)), stoi(*(it->begin()+1))))
    {
      cluster.back().obj_class = *(it->begin());
      cluster.back().bgr.push_back(stoi(*(it->begin()+3)));
      cluster.back().bgr.push_back(stoi(*(it->begin()+2)));
      cluster.back().bgr.push_back(stoi(*(it->begin()+1)));
      //ROS_WARN_STREAM(cluster.back().coordinates_each.size());
      ROS_INFO_STREAM(*(it->begin())<<" present with "<<cluster.back().num_obj<<" objects");
    }
  }
  /*Cluster ob1;
  ob1.bgr.push_back(cv_ptr->image.at<cv::Vec3b>(0, 0)[0]);
  ob1.bgr.push_back(cv_ptr->image.at<cv::Vec3b>(0, 0)[1]);
  ob1.bgr.push_back(cv_ptr->image.at<cv::Vec3b>(0, 0)[2]);
  std::pair<int, int> coor;
  coor.first = 0;
  coor.second = 0;
  ob1.coordinates.push_back(coor);
  cluster.push_back(ob1);*/
  //cluster_size.push_back(1);
  //rgb.clear();
  //ROS_INFO_STREAM(__LINE__);
  auto v_char = cv_ptr->image;
  //auto r = v_char[0].val[0];
  auto abc = v_char.col(0);
  //ROS_INFO_STREAM(v_char.size());
  /*for(int j = 0; j<(int)msg->width; j+=10)
  {
    for(int i = 0; i<(int)msg->height; i+=10)
    {
      //ROS_INFO_STREAM(__LINE__<<"   "<<j<<"   "<<i<<"  "<<cluster.size());
      bool is_present = false;
      std::vector<int> bgr;
      int b = cv_ptr->image.at<cv::Vec3b>(i, j)[0];
      int g = cv_ptr->image.at<cv::Vec3b>(i, j)[1];
      int r = cv_ptr->image.at<cv::Vec3b>(i, j)[2];
      //bgr.push_back(b);
      //bgr.push_back(g);
      //bgr.push_back(r);
      //col.push_back(bgr);
      //Point3_<uchar>* p = image.ptr<Point3_<uchar>> (j, i);
      //ROS_INFO_STREAM("val = "<<b<<"  "<<g<<"   "<<r);
      Cluster c;
      for(int k = 0; k<cluster.size(); k++)
      {
        //ROS_INFO_STREAM(k<<" val = "<<cluster[k][0]<<"  "<<cluster[k][1]<<"   "<<cluster[k][2]<<" val2 = "<<b<<"  "<<g<<"   "<<r);
        if((b == cluster[k].bgr[0]) && (g == cluster[k].bgr[1]) && (r == cluster[k].bgr[2]))
        {
          std::pair<int, int> coordinates;
          coordinates.first = i;
          coordinates.second = j;
          cluster[k].coordinates.push_back(coordinates);
          is_present = true;
          break;
        }
      }
      if(!is_present)
      {
        for(int k = 0; k<class_data.size(); k++)
          if(b==stoi(class_data[k][3]) && g==stoi(class_data[k][2]) && r==stoi(class_data[k][1]))
            c.obj_class = class_data[k][0];
        c.bgr.push_back(b);
        c.bgr.push_back(g);
        c.bgr.push_back(r);
        std::pair<int, int> coordinates;
        coordinates.first = i;
        coordinates.second = j;
        c.coordinates.push_back(coordinates);
        c.num_obj = 1;
        //ROS_INFO_STREAM("APPENDING");
        cluster.push_back(c);
      }
    }
    //std::set<std::vector<std::vector<int>>> sc(col.begin(), col.begin() + col.size()-1);
  }*/
  ROS_INFO_STREAM("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Size is "<<cluster.size());
  /*for(int i=0; i<cluster.size(); i++)
  {
    ROS_INFO_STREAM(cluster[i].obj_class<<"   "<<cluster[i].coordinates.size());
  }*/
}
/******************************************************************************/

/******************************************************************************/
void SegClustering::depthCB(const sensor_msgs::ImageConstPtr& msg)
{

  multi_level_map_messages::PoseOfCluster cluster_pose;
  if(!flag)
    return;
  cv_bridge::CvImagePtr cv_ptr;
  ROS_INFO_STREAM(cluster.size());
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    ROS_INFO_STREAM("Accepted Depth Image");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  std::vector<float> angle;
  std::vector<float> range;
  for(int i = 0, e = cluster.size(); i<e; i++)
  {
    //if(cluster[i].obj_class != "swivel")
      //continue;
    std::vector<std::pair<int, int>> coordinates;
    for(int j = 0; j<cluster[i].num_obj; j++)
    {
      if(cluster[i].coordinates_each[j].size()<10)
        continue;
      std::vector<float> dist;
      for (int k = 0, end = cluster[i].coordinates_each[j].size(); k < end; k++)
      {
        if(cv_ptr->image.at<short int>(cluster[i].coordinates_each[j][k].first, cluster[i].coordinates_each[j][k].second)==0)
          continue;
        dist.push_back((float)cv_ptr->image.at<short int>(cluster[i].coordinates_each[j][k].first, cluster[i].coordinates_each[j][k].second));///(float)1000);
        range.push_back((float)cv_ptr->image.at<short int>(cluster[i].coordinates_each[j][k].first, cluster[i].coordinates_each[j][k].second));///(float)1000);
        cluster[i].distance.push_back((float)cv_ptr->image.at<short int>(cluster[i].coordinates_each[j][k].first, cluster[i].coordinates_each[j][k].second)/(float)1000);
        double cor_ang = -1*(cluster[i].coordinates_each[j][k].second*resolution-43)*(float)(PI/180);
        //ROS_WARN_STREAM((float)cv_ptr->image.at<short int>(cluster[i].coordinates_each[j][k].first, cluster[i].coordinates_each[j][k].second)<<"  "<<cor_ang);
        //ROS_WARN_STREAM(cluster[i].coordinates_each[j][k].second<<"   "<<-1*(cluster[i].coordinates_each[j][k].second*resolution-43)*(float)(PI/180));
        angle.push_back(cor_ang);
      }
      cluster[i].distance_each.push_back(dist);
    }
  }
  /*for(int i=0; i<cluster.size(); i++)
  {
    //if(cluster[i].obj_class != "swivel")
      //continue;
    //ROS_INFO_STREAM(cluster[i].obj_class<<"   "<<cluster[i].num_obj);
  }*/
  cluster_pose.angles.insert(cluster_pose.angles.begin(), std::begin(angle), std::end(angle));
  cluster_pose.ranges.insert(cluster_pose.ranges.begin(), std::begin(range), std::end(range));
  cluster_pose.header = msg->header;
  cluster_pose_pub.publish(cluster_pose);
  ROS_INFO_STREAM("SIZE IS "<<cluster_pose.ranges.size());
  range.clear();
  angle.clear();
  this->flag = false;
}
/******************************************************************************/

/******************************************************************************/
bool SegClustering::clustering(cv_bridge::CvImagePtr cv_ptr, int width, int height, int b, int g, int r)
{
  Cluster c;
  c.num_obj = 0;
  bool cont = true;
  present = false;
  bool present_in_prev_col = false;;
  std::pair<int, int> point;
  std::vector<std::pair<int, int>> point_arr;
  for(int j=5; j<width; j+=5)
  {
    for(int i=height/3; i<(height-height/4); i+=5)
    {
      //ROS_INFO_STREAM("         "<<i<<"  "<<j);
      int bc = cv_ptr->image.at<cv::Vec3b>(i, j)[0];
      int gc = cv_ptr->image.at<cv::Vec3b>(i, j)[1];
      int rc = cv_ptr->image.at<cv::Vec3b>(i, j)[2];
      //int bb = cv_ptr->image.at<cv::Vec3b>(i, j-10)[0];
      //int gb = cv_ptr->image.at<cv::Vec3b>(i, j-10)[1];
      //int rb = cv_ptr->image.at<cv::Vec3b>(i, j-10)[2];
      if((b == bc) && (g == gc) && (r == rc))
      {
        
        present = true;
        /*if((b != bb) && (g != gb) && (r != rb))
        {
          c.num_obj+=1;
          c.coordinates_each.push_back(point_arr);
          point_arr.clear();
        }*/
        if(!present_in_prev_col)
        {
          //ROS_WARN_STREAM(i<<"  "<<j);
          c.num_obj+=1;
          c.coordinates_each.push_back(point_arr);
          point_arr.clear(); 
        }
        else if(j-5 == 0)
        {
          c.num_obj+=1;
          c.coordinates_each.push_back(point_arr);
          point_arr.clear();
        }
        point.first = i;
        point.second = j;
        point_arr.push_back(point);
        c.coordinates.push_back(point);
        present_in_prev_col = true;
        cont = true;
        break;
      }
      else
        cont = false;
    }
    if(cont == false)
      present_in_prev_col = false;
  }
  if(present)
  {
    c.coordinates_each.push_back(point_arr);
    c.coordinates_each.erase(c.coordinates_each.begin() + 0);
    cluster.push_back(c);
    //ROS_WARN_STREAM("Size of coordinates "<<c.coordinates.size());
  }
  return present;
}
/******************************************************************************/

/******************************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "seg_clustering");
  SegClustering sc;
  ros::spin();
  return 0;
}
