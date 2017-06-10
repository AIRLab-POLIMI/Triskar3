#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>

#include "triskar/PathPublisher.h"

using namespace std;

class PathNode {
public:
	PathNode(ros::NodeHandle& nh, double dt)
		: nh(nh), 
			rate(0) {
		pathPublishers = vector<PathPublisher*>();
		PathPublisher* pub = new PathPublisher(nh, string("/world"), string("/base_link"), string("world"), string("/trajectory"));
		pathPublishers.push_back(pub);
	    
	    rate = ros::Rate(1/dt);
	}
	
	void spin() {
		for(int i=0; i<pathPublishers.size(); i++) {
			pathPublishers[i]->getPose();
			pathPublishers[i]->publishPath();
		}
		ros::spinOnce();
     	rate.sleep();
	}
	
private:
	ros::NodeHandle nh;
	vector<PathPublisher*> pathPublishers;
    ros::Rate rate;
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "path");

	ros::NodeHandle nh;
	double dt;
	
	nh.param("period", dt, 0.01);
	
	if(dt <= 0) {
		dt = 0.01;
	}
	
	PathNode pathNode(nh, dt);
	while(ros::ok()) {
        pathNode.spin();
    }
}
