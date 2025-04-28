//类似stm32程序头文件的编写规则
#ifndef LINUX_STM_SERIAL_H
#define LINUX_STM_SERIAL_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

extern void serialInit();
extern void writeSpeed(double Left_v, double Right_v,unsigned char ctrlFlag);
extern bool readSpeed(double &Left_v,double &Right_v,double &Angle,unsigned char &ctrlFlag);
extern void writePose(double x, double y, double yaw, unsigned char ctrlFlag);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

// 添加回调函数声明
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

#endif
