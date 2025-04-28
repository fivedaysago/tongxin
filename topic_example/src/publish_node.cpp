
#include "ros/ros.h"
#include "std_msgs/String.h" //use data struct of std_msgs/String  
#include "mbot_linux_serial.h"
/*===================================================================
程序功能：串口通信测试程序
程序编写：公众号：小白学移动机器人
其他    ：如果对代码有任何疑问，可以私信小编，一定会回复的。
=====================================================================
------------------关注公众号，获得更多有趣的分享---------------------
===================================================================*/
//test send value
double testSend1=5555.0;
double testSend2=2222.0;
unsigned char testSend3=0x07;

//test receive value
double testRece1=0.0;
double testRece2=0.0;
double testRece3=0.0;
unsigned char testRece4=0x00;

int main(int agrc,char **argv)
{
    ros::init(agrc,argv,"public_node");
    ros::NodeHandle nh;

    //串口初始化
    serialInit();

    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        //向STM32端发送数据，前两个为double类型，最后一个为unsigned char类型
	    writeSpeed(testSend1,testSend2,testSend3);
        //从STM32接收数据，输入参数依次转化为小车的线速度、角速度、航向角（角度）、预留控制位
	    readSpeed(testRece1,testRece2,testRece3,testRece4);
        //打印数据
	    ROS_INFO("%f,%f,%f,%d\n",testRece1,testRece2,testRece3,testRece4);

        loop_rate.sleep();
    }
    return 0;
}
 



