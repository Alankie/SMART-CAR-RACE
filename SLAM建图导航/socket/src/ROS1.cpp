#include "ros/ros.h"	//������ʹ��ROS�ڵ�ı�Ҫ�ļ�
#include "std_msgs/String.h"	//������ʹ�õ���������
#include <sstream>     
int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_a");	//��ʼ��ROS���ڵ�������Ϊnode_a���ڵ������뱣��Ψһ
    ros::NodeHandle n;	//ʵ�����ڵ�, �ڵ���̾��
    ros::Publisher pub = n.advertise<std_msgs::String>("str_message", 1000);	//����ϵͳҪ���������ˣ�������Ϊ��str_message��������Ϊstd_msgs::String���������Ϊ1000��
    ros::Rate loop_rate(10);	//���÷������ݵ�Ƶ��Ϊ10Hz

    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello World";
        msg.data = ss.str();
        ROS_INFO("node_a is publishing %s", msg.data.c_str());
        pub.publish(msg);	//���⡰str_message��������Ϣ
        ros::spinOnce();	//���Ǳ��룬�������ж��Ļ�������룬����ص������������á�
        loop_rate.sleep();	//��ǰ�����õ�10HzƵ�ʽ��������
    }
    return 0;
}
