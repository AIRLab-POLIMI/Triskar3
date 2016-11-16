#include <ros/ros.h>


class PixyTeleopJoy
{
public:
	PixyTeleopJoy(ros::NodeHandle& nh) 
		: nh(nh)
	{
		
	}
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{
		
	}
	
private:
	ros::NodeHandle& nh;
	ros::Subscriber joy_sub;
	ros::Publisher cmd_vel_pub;
	
	double scale;
	ros::Duration dt;
	int resetButton;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "teleop_pixy_joy_node");

	ros::NodeHandle nh("~");
	PixyTeleopJoy joy_teleop(&nh);

	ros::spin();
} 
