#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "triskar/RungeKutta4Odometry.h"
#include "triskar/Odometry.h"
#include "triskar/EuleroOdometry.h"

using namespace std;

class OdometryNode {
public:
	OdometryNode(ros::NodeHandle& nh, double dt, string subscriber, string odometryType)
		: nh(nh), 
			rate(1/dt), 
			velocity(vector<double>(3,0)), 
			dt(dt) {
	    velSub = nh.subscribe("vel", 1/dt, &OdometryNode::velocityCallback, this);
	    odomBroadcaster = new tf::TransformBroadcaster();
	    //instance of odometry depending on what is read from the launchfile
	    if(odometryType.compare("eulero") == 0) {
	    	EuleroOdometry euleroOdometry;
	    	odometry = &euleroOdometry;
		} else {
			RungeKutta4Odometry rungeKutta4Odometry;
			odometry = &rungeKutta4Odometry;
		}
	}
	
	void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {	//called when vel is received from topic /vel
	    velocity.at(0) = msg->linear.x;
	    velocity.at(1) = msg->linear.y;
	    velocity.at(2) = msg->angular.z;
	    odometry->integrate(velocity, dt);
	}
	
	//publishes the odometry over tf
	void publishOdometry() {
		geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(odometry->getPosition().at(2));

	    geometry_msgs::TransformStamped odomTrans;
	    odomTrans.header.stamp = ros::Time::now();
	    odomTrans.header.frame_id = "odom";
	    odomTrans.child_frame_id = "base_link";
	
	    odomTrans.transform.translation.x = odometry->getPosition().at(0);
	    odomTrans.transform.translation.y = odometry->getPosition().at(1);
	    odomTrans.transform.translation.z = 0.0;
	    odomTrans.transform.rotation = odomQuat;
	
	    //send the transform over tf
	    odomBroadcaster->sendTransform(odomTrans);
	}
	
	void spin() {
		ros::spinOnce();
        
        publishOdometry();
        
     	rate.sleep();
	}
	
private:
	ros::NodeHandle nh;
	ros::Subscriber velSub;			//listener to velocity
    tf::TransformBroadcaster* odomBroadcaster;	//publishes the odometry over tf
    vector<double> velocity;
    Odometry* odometry;
    double dt;
    ros::Rate rate;
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "odometry");

	ros::NodeHandle nh("~");
	double dt;
	string subscriber, odometryType;
	
	nh.param("period", dt, 0.01);								//default period: 0.01
	nh.param("subscriber", subscriber, string("vel"));					//default subscriber: vel
	nh.param("odometryType", odometryType, string("eulero"));			//default integration: eulero
	
	if(dt <= 0) {
		dt = 0.01;
	}
	
	OdometryNode odometryNode(nh, dt, subscriber, odometryType);
	while(ros::ok()) {
        odometryNode.spin();
    }
}
