/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 * 
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *            佛祖保佑       永不宕机     永无BUG
 * 
 *        佛曰:  
 *                写字楼里写字间，写字间里程序员；  
 *                程序人员写程序，又拿程序换酒钱。  
 *                酒醒只在网上坐，酒醉还来网下眠；  
 *                酒醉酒醒日复日，网上网下年复年。  
 *                但愿老死电脑间，不愿鞠躬老板前；  
 *                奔驰宝马贵者趣，公交自行程序员。  
 *                别人笑我忒疯癫，我笑自己命太贱；  
 *                不见满街漂亮妹，哪个归得程序员？
 * 
 * @Author: Glory Huang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Glory Huang
 * @LastEditTime: 2021-01-30 15:02:12
 * @Page: https://xjtuglory.ml
 * @Github: https://github.com/gloryhry
 * @Description: file content
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <point_type.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace message_filters;
using namespace std;

string pkg_loc;

ros::Publisher radar_combined_pub;

void combined_callback(const sensor_msgs::PointCloud2ConstPtr front_msg,
                       const sensor_msgs::PointCloud2ConstPtr leftfront_msg,
                       const sensor_msgs::PointCloud2ConstPtr rightfront_msg,
                       const sensor_msgs::PointCloud2ConstPtr leftback_msg,
                       const sensor_msgs::PointCloud2ConstPtr rightback_msg)
{
    pcl::PointCloud<pointXYZVvar>::Ptr raw_front(new pcl::PointCloud<pointXYZVvar>());
    pcl::PointCloud<pointXYZVvar>::Ptr raw_leftfront(new pcl::PointCloud<pointXYZVvar>());
    pcl::PointCloud<pointXYZVvar>::Ptr raw_rightfront(new pcl::PointCloud<pointXYZVvar>());
    pcl::PointCloud<pointXYZVvar>::Ptr raw_leftback(new pcl::PointCloud<pointXYZVvar>());
    pcl::PointCloud<pointXYZVvar>::Ptr raw_rightback(new pcl::PointCloud<pointXYZVvar>());

    pcl::fromROSMsg(*front_msg, *raw_front);
    pcl::fromROSMsg(*leftfront_msg, *raw_leftfront);
    pcl::fromROSMsg(*rightfront_msg, *raw_rightfront);
    pcl::fromROSMsg(*leftback_msg, *raw_leftback);
    pcl::fromROSMsg(*rightback_msg, *raw_rightback);

    pcl::PointCloud<pointXYZVvar>::Ptr radar_combined(new pcl::PointCloud<pointXYZVvar>());
    *radar_combined = *raw_front;
    *radar_combined = *radar_combined + *raw_leftfront;
    *radar_combined = *radar_combined + *raw_rightfront;
    *radar_combined = *radar_combined + *raw_leftback;
    *radar_combined = *radar_combined + *raw_rightback;
    for(auto &temp:radar_combined->points)
    {
        temp.z = 0.0;
    }
    sensor_msgs::PointCloud2Ptr radar_combined_pointcloud(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*radar_combined, *radar_combined_pointcloud);
    radar_combined_pointcloud->header = front_msg->header;
    radar_combined_pub.publish(radar_combined_pointcloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_combined");
    pkg_loc = ros::package::getPath("radar2pointcloud");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    string radar_front, radar_leftfront, radar_rightfront, radar_leftback, radar_rightback;
    private_nh.param<string>("front", radar_front, "/front");
    private_nh.param<string>("leftfront", radar_leftfront, "/leftfront");
    private_nh.param<string>("rightfront", radar_rightfront, "/rightfront");
    private_nh.param<string>("leftback", radar_leftback, "/leftback");
    private_nh.param<string>("rightback", radar_rightback, "/rightback");

    radar_combined_pub = nh.advertise<sensor_msgs::PointCloud2>("/RadarCombined", 10);

    message_filters::Subscriber<sensor_msgs::PointCloud2> front_sub(nh, radar_front, 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> leftfront_sub(nh, radar_leftfront, 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> rightfront_sub(nh, radar_rightfront, 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> leftback_sub(nh, radar_leftback, 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> rightback_sub(nh, radar_rightback, 10);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), front_sub, leftfront_sub, rightfront_sub, leftback_sub, rightback_sub);
    sync.registerCallback(boost::bind(&combined_callback, _1, _2, _3, _4, _5));

    ros::spin();
    return 0;
}