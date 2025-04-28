//包含ros库下的ros.h头文件
#include "ros/ros.h"
//包含std_msgs库下的String.h头文件
#include "std_msgs/String.h" 
//包含mbot_linux_serial.h头文件
#include "linux_stm_serial.h"
//包含geometry_msgs库下的PoseWithCovarianceStamped.h头文件
#include "geometry_msgs/PoseWithCovarianceStamped.h"
//包含tf库下的tf.h头文件
#include "tf/tf.h"

//测试发送数据两
double testSend1=5555.0;
double testSend2=2222.0;
unsigned char testSend3=0x07;
//测试接受数据变量
double testRece1=0.0;
double testRece2=0.0;
double testRece3=0.0;
unsigned char testRece4=0x00;

// 添加全局变量用于存储位姿信息
double current_x = 0.0;
double current_y = 0.0;
double current_yaw = 0.0;

// 实现回调函数
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // 获取位置信息
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
    
    // 从四元数计算偏航角
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw = yaw;
    
    // 打印位姿信息
    ROS_INFO("Position: x=%.2f, y=%.2f, yaw=%.2f", current_x, current_y, current_yaw);
    
    // 发送位姿数据给STM32
    // 使用0x08作为控制标志，表示这是位姿数据
    writePose(current_x, current_y, current_yaw, 0x08);
}

int main(int agrc,char **argv)
{
    //创建一个ros节点,节点名称为public_node
    ros::init(agrc,argv,"public_node");
    //创建句柄，用于管理节点信息
    ros::NodeHandle nh;
    //设置频率，10HZ
    ros::Rate loop_rate(10);
    //串口初始化，相关定义在mbot_linux_serial.cpp有描述
    serialInit();
    /*
    ros::ok()在不进行任何操作时，就相当于返回True，只有在以下几种情况下会变成返回False
    （1）运行终端时，按下Ctrl-C时
    （2）我们被一个同名同姓的节点从网络中踢出。
    （3）ros::shutdown()被应用程序的另一部分调用。
    （4）所有的ros::NodeHandles都被销毁了。
    */
    
    // 添加订阅者
    ros::Subscriber pose_sub = nh.subscribe("/poseupdate", 10, poseCallback);
    
    while(ros::ok())
    {
        /*
        ros::spinOnce()和ros::spin()是ros消息回调处理函数
        ros消息回调处理函数原理：如果你的程序写了相关的消息订阅函数，那么程序在执行过程中，除了主程序以外，ROS还会自动在后台按照你规定的格式，接受订阅的消息，但是所接到的消息并不是立刻就被处理，而是必须要等到ros::spin()或ros::spinOnce()执行的时候才被调用
        他们的区别在于ros::spinOnce调用后会继续执行其后面的语句，而ros::spin（）则在调用后不会继续执行其后面的语句
        */
        ros::spinOnce();
        //向STM32端发送数据，前两个为double类型，最后一个为unsigned char类型，其函数相关定义在mbot_linux_serial.cpp有描述
	    writeSpeed(testSend1,testSend2,testSend3);
        //从STM32接收数据，输入参数依次转化为小车的线速度、角速度、航向角（角度）、预留控制位，其函数相关定义在mbot_linux_serial.cpp有描述
	    readSpeed(testRece1,testRece2,testRece3,testRece4);
        //打印数据
	    ROS_INFO("%f,%f,%f,%d\n",testRece1,testRece2,testRece3,testRece4);
	    //等待100ms的时间
        loop_rate.sleep();
    }
    return 0;
}
