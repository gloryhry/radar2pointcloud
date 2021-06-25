/*
 *
 *    ┏┓　　　┏┓
 *  ┏┛┻━━━┛┻┓
 *  ┃　　　　　　　┃
 *  ┃　　　━　　　┃
 *  ┃　＞　　　＜　┃
 *  ┃　　　　　　　┃
 *  ┃...　⌒　...　┃
 *  ┃　　　　　　　┃
 *  ┗━┓　　　┏━┛
 *      ┃　　　┃　
 *      ┃　　　┃
 *      ┃　　　┃
 *      ┃　　　┃  神兽保佑
 *      ┃　　　┃  代码无bug　　
 *      ┃　　　┃
 *      ┃　　　┗━━━┓
 *      ┃　　　　　　　┣┓
 *      ┃　　　　　　　┏┛
 *      ┗┓┓┏━┳┓┏┛
 *        ┃┫┫　┃┫┫
 *        ┗┻┛　┗┻┛
 *
 * @Author: Glory Huang
 * @Date: 2021-01-27 10:32:13
 * @LastEditors: Glory Huang
 * @LastEditTime: 2021-01-30 15:01:20
 * @Page: https://xjtuglory.ml
 * @Github: https://github.com/gloryhry
 * @Description: file content
 */

#include <cstdlib>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include <ecal_to_ros/RadarDetectionImage.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <point_type.h>

using namespace std;

static std::unique_ptr<tf::TransformListener> g_tf_listener;

#define THR_FAP 0.01 // thresthold of false alarm probability
#define THR_VEL 5    // thresthold of target max speed
#define THR_RANGE 10 // thresthold of close target

class Transformer
{
private:
  ros::Publisher _pub_pcl;
  ros::Subscriber _sub;
  ros::NodeHandle nh_;
  string _sub_topic;
  string _pub_topic;
  string _frame_id;
  float _offset_x = 0.0, _offset_y = 0.0;
  float _rotation = 0.0;

  void callback(const ecal_to_ros::RadarDetectionImage::ConstPtr &msg)
  {
    ROS_INFO("Receive a message!");
    pcl::PointCloud<pointXYZVvar>::Ptr cloud(
        new pcl::PointCloud<pointXYZVvar>);
    uint32_t N = msg->u_NofDetections;

    for (auto i = 0; i < N; i++)
    {
      if (msg->a_RadarDetectionList[i].f_Pdh0 < THR_FAP)
      {
        pointXYZVvar p;
        float r = msg->a_RadarDetectionList[i].f_Range;
        float v = msg->a_RadarDetectionList[i].f_VrelRad;
        float phi = -msg->a_RadarDetectionList[i].a_AzAng_hyp[0];
        p.x = r * sin(phi + _rotation) + _offset_x;
        p.y = r * cos(phi + _rotation) + _offset_y;
        p.z = 0;
        p.vel_rad = v;
        p.oritation = -phi;
        p.range_var = msg->a_RadarDetectionList[i].f_RangeVar;
        p.vel_rad_var = msg->a_RadarDetectionList[i].f_VrelRadVar;
        p.ang_var = msg->a_RadarDetectionList[i].f_AzAngVar;
        cloud->push_back(p);
      }
    }

    tf::StampedTransform transformer;
    try
    {
      g_tf_listener->waitForTransform("base_footprint", _frame_id, msg->header.stamp, ros::Duration(1));
      g_tf_listener->lookupTransform("base_footprint", _frame_id, msg->header.stamp, transformer);
    }
    catch (tf::TransformException &e)
    {
      ROS_ERROR("%s", e.what());
    }
    float yaw_delta = tf::getYaw(transformer.getRotation());
    float x_delta = transformer.getOrigin().x();
    float y_delta = transformer.getOrigin().y();

    for (int i = 0; i < cloud->size(); i++)
    {
      cloud->points[i].oritation = cloud->points[i].oritation + yaw_delta;
      cloud->points[i].vel_rad = cloud->points[i].vel_rad / cos(cloud->points[i].oritation);
    }
    sensor_msgs::PointCloud2 pcl2;
    pcl::toROSMsg(*cloud, pcl2);
    cout << "size of cloud: " << cloud->size() << endl;
    pcl2.header = msg->header;
    pcl2.header.frame_id = _frame_id;

    sensor_msgs::PointCloud2::Ptr radar_msg_trans(new sensor_msgs::PointCloud2);
    pcl_ros::transformPointCloud("base_footprint", pcl2, *radar_msg_trans,
                                 *g_tf_listener);
    _pub_pcl.publish(radar_msg_trans);
  }

public:
  Transformer(ros::NodeHandle nh, string sub_topic, string pub_topic)
  {
    // _nh = nh;
    _sub_topic = sub_topic;
    _pub_topic = pub_topic;
    _pub_pcl = nh.advertise<sensor_msgs::PointCloud2>(_pub_topic, 1);
    _sub = nh.subscribe<ecal_to_ros::RadarDetectionImage>(
        _sub_topic, 1, &Transformer::callback, this);
  }
  ~Transformer() {}

  void set_radar_install_parameters(float offset_x, float offset_y,
                                    float rotation, string frame_id)
  {
    _offset_x = offset_x;
    _offset_y = offset_y;
    _rotation = rotation;
    _frame_id = frame_id;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "radar_trans");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  string pkg_loc = ros::package::getPath("radar2pointcloud");

  g_tf_listener.reset(new tf::TransformListener());

  string radar_param_conf;
  private_nh.param<string>("radar_param", radar_param_conf,
                           "radar_param.yaml");
  YAML::Node yamlConfig = YAML::LoadFile(pkg_loc + "/cfg/" + radar_param_conf);

  Transformer radar_front(nh, yamlConfig["front"]["sub_topic"].as<string>(),
                          yamlConfig["front"]["pub_topic"].as<string>());
  radar_front.set_radar_install_parameters(
      yamlConfig["front"]["offset_x"].as<float>(),
      yamlConfig["front"]["offset_y"].as<float>(),
      yamlConfig["front"]["rotation"].as<float>(),
      yamlConfig["front"]["frame_id"].as<string>());

  Transformer radar_leftfront(
      nh, yamlConfig["leftfront"]["sub_topic"].as<string>(),
      yamlConfig["leftfront"]["pub_topic"].as<string>());
  radar_leftfront.set_radar_install_parameters(
      yamlConfig["leftfront"]["offset_x"].as<float>(),
      yamlConfig["leftfront"]["offset_y"].as<float>(),
      yamlConfig["leftfront"]["rotation"].as<float>(),
      yamlConfig["leftfront"]["frame_id"].as<string>());

  Transformer radar_leftback(nh,
                             yamlConfig["leftback"]["sub_topic"].as<string>(),
                             yamlConfig["leftback"]["pub_topic"].as<string>());
  radar_leftback.set_radar_install_parameters(
      yamlConfig["leftback"]["offset_x"].as<float>(),
      yamlConfig["leftback"]["offset_y"].as<float>(),
      yamlConfig["leftback"]["rotation"].as<float>(),
      yamlConfig["leftback"]["frame_id"].as<string>());

  Transformer radar_rightfront(
      nh, yamlConfig["rightfront"]["sub_topic"].as<string>(),
      yamlConfig["rightfront"]["pub_topic"].as<string>());
  radar_rightfront.set_radar_install_parameters(
      yamlConfig["rightfront"]["offset_x"].as<float>(),
      yamlConfig["rightfront"]["offset_y"].as<float>(),
      yamlConfig["rightfront"]["rotation"].as<float>(),
      yamlConfig["rightfront"]["frame_id"].as<string>());

  Transformer radar_rightback(
      nh, yamlConfig["rightback"]["sub_topic"].as<string>(),
      yamlConfig["rightback"]["pub_topic"].as<string>());
  radar_rightback.set_radar_install_parameters(
      yamlConfig["rightback"]["offset_x"].as<float>(),
      yamlConfig["rightback"]["offset_y"].as<float>(),
      yamlConfig["rightback"]["rotation"].as<float>(),
      yamlConfig["rightback"]["frame_id"].as<string>());
  ros::spin();
  return (0);
}
