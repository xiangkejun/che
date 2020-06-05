#include "sineBot_base.h"
#include <signal.h>
SineBotVel get_vel();
bool init_modbus(std::string serial_port);
void set_vel(SineBotVel bot_vel);


// 速度控制消息的回调函数
void cmdCallback(const geometry_msgs::Twist& msg)
{
	SineBotVel bot_vel;

    bot_vel.linear_vel_x.vel = msg.linear.x;
    bot_vel.angular_vel_z.vel = msg.angular.z;    

    set_vel(bot_vel);
}

void exit_handler(int sig)
{
	//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    //速度重置为0
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;    
    cmdCallback(msg);
    ROS_INFO("sin_bot end.\n");
	ros::shutdown();
}

int main(int argc, char** argv )
{
    //初始化节点
    ros::init(argc, argv, "SineBot_base_node");
    ros::NodeHandle nh;
    std::string serial_port;
    nh.param<std::string>("/sinebot_base_node/serial_port", serial_port, "/dev/ttyUSB0"); 

    SineBotBase sineBot;
    SineBotVel vel;
    
    if(init_modbus(serial_port) == false)
    {
        ROS_ERROR("sin_bot initialized failed.\n");
        return -1;
    }
    //覆盖原来的Ctrl+C中断函数，原来的只会调用ros::shutdown()
	signal(SIGINT, exit_handler);
	
	ROS_INFO("sin_bot initialized successful.\n");
 
    ros::Subscriber sub = nh.subscribe("cmd_vel", 50, cmdCallback);
 
    //循环运行
    ros::Rate loop_rate(50);
 
	while (ros::ok()) 
    {
		ros::spinOnce();
        vel = get_vel();
        // 机器人控制
        sineBot.spinOnce(vel);
        
		loop_rate.sleep();
	}
    //速度重置为0
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;    
    cmdCallback(msg);
    ROS_INFO("sin_bot end.\n");
    return 0;
}
