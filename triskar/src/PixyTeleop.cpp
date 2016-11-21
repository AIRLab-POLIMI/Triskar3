#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <triskar_msgs/PixyServo.h>


class PixyTeleopJoy
{
public:
	PixyTeleopJoy(ros::NodeHandle& nh) 
		: nh(nh), rate(0)
	{
		nh.param("reset_button", resetButton, -1);
		nh.param("axis_pan", panAxis, 1);
		nh.param("axis_tilt", tiltAxis, 2);
		nh.param("scale", scale, 1.0);

		double freq;
		nh.param("rate", freq, 1.0);
		rate = ros::Rate(freq);

		nh.param("minPan", minPan, 0);
		nh.param("minTilt", minTilt, 0);
		nh.param("maxPan", maxPan, 2000);
		nh.param("maxTilt", maxTilt, 2000);

		pan = (maxPan - minPan) / 2.0;
		tilt = (maxTilt - minTilt) / 2.0;

		joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 5, &PixyTeleopJoy::joyCallback, this);
		cmd_pub = nh.advertise<triskar_msgs::PixyServo>("/pixy_servo", 5);
	}
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
	{
		if(resetButton >= 0 && joy_msg->buttons[resetButton])
		{
			pan = (maxPan - minPan) / 2.0;
			tilt = (maxTilt - minTilt) / 2.0;
		}
		else
		{
			double panCommand = scale*joy_msg->axes[panAxis];
			double tiltCommand = scale*joy_msg->axes[tiltAxis];

			pan = bound(pan + panCommand, minPan, maxPan);
			tilt =  bound(tilt + tiltCommand, minTilt, maxTilt);
		}
	}
	
	void spin()
	{
		triskar_msgs::PixyServo cmd_msg;
		cmd_msg.pan = std::ceil(pan);
		cmd_msg.tilt = std::ceil(tilt);
		cmd_pub.publish(cmd_msg);

		ros::spinOnce();
		rate.sleep();
	}

private:
	static double bound(double value, int min, int max)
	{
		if(value < min)
			return min;
		else if(value > max)
			return max;
		else
			return value;
	}

private:
	ros::NodeHandle& nh;
	ros::Subscriber joy_sub;
	ros::Publisher cmd_pub;
	
	double scale;
	int resetButton;
	int panAxis, tiltAxis;
	int minPan, maxPan,	minTilt, maxTilt;

	ros::Rate rate;

	double pan;
	double tilt;

};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "teleop_pixy_joy_node");

	ros::NodeHandle nh("~");
	PixyTeleopJoy joy_teleop(nh);

	while(ros::ok())
	{
		joy_teleop.spin();
	}
} 
