#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "triskar/RungeKutta4Odometry.h"
#include "triskar/Odometry.h"
#include "triskar/EuleroOdometry.h"

#include <ros/console.h>

using namespace std;

class OdometryNode {
public:
	OdometryNode(ros::NodeHandle& nh, double dt, string subscriber, string odometryType, string childFrameID, string headerFrameID)
		: nh(nh), 
			rate(0),
			velocity(vector<double>(3,0)), 
			dt(dt),
			childFrameID(childFrameID),
			headerFrameID(headerFrameID) {
	    velSub = nh.subscribe(subscriber, 100, &OdometryNode::velocityCallback, this);
	    rate = ros::Rate(1/dt);
	    odomBroadcaster = new tf::TransformBroadcaster();
		
	    //instance of odometry depending on what is read from the launchfile
	    if(odometryType == "eulero") {
	    	EuleroOdometry* euleroOdometry = new EuleroOdometry();
	    	odometry = euleroOdometry;
		} else {
			RungeKutta4Odometry* rungeKutta4Odometry = new RungeKutta4Odometry();
			odometry = rungeKutta4Odometry;
		}
	}
	
	void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {	//called when vel is received from topic /vel
	    velocity[0] = msg->linear.x;
	    velocity[1] = msg->linear.y;
	    velocity[2] = msg->angular.z;
	    odometry->integrate(velocity, dt);
	}
	
	//publishes the odometry over tf
	void publishOdometry() {
		geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(odometry->getPosition()[2]);

	    geometry_msgs::TransformStamped odomTrans;
	    odomTrans.header.stamp = ros::Time::now();
	    odomTrans.header.frame_id = headerFrameID;
	    odomTrans.child_frame_id = childFrameID;
	
	    odomTrans.transform.translation.x = odometry->getPosition()[0];
	    odomTrans.transform.translation.y = odometry->getPosition()[1];
	    odomTrans.transform.translation.z = 0.0;
	    odomTrans.transform.rotation = odomQuat;
	
	    //send the transform over tf
	    odomBroadcaster->sendTransform(odomTrans);
	}
	
	
	void spin() {
        publishOdometry();
		ros::spinOnce();
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
    string childFrameID, headerFrameID;
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "odometry");

	ros::NodeHandle nh("~");
	double dt;
	string subscriber, odometryType, childFrameID, headerFrameId;
	
	nh.param("period", dt, 0.01);											//default period: 0.01
	nh.param("subscriber", subscriber, string("vel"));						//default subscriber: vel
	nh.param("odometry_type", odometryType, string("eulero"));				//default integration: eulero
	nh.param("header_frame_id", headerFrameId, string("odom"));				//default header: odom
	nh.param("child_frame_id", childFrameID, string("base_link"));			//default child: base_link
	
	if(dt <= 0) {
		dt = 0.01;
	}
	
	OdometryNode odometryNode(nh, dt, subscriber, odometryType, childFrameID, headerFrameId);
	while(ros::ok()) {
        odometryNode.spin();
    }
}
