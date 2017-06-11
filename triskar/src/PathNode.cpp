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
		PathPublisher* pubOdomPath = new PathPublisher(nh, string("/world"), string("/base_link"), string("world"), string("/trajectory_odom"));
		pathPublishers.push_back(pubOdomPath);
                
        PathPublisher* pubOptiPath = new PathPublisher(nh, string("/world"), string("/Robot_2/base_link"), string("world"), string("/trajectory_opti"));
        pathPublishers.push_back(pubOptiPath);
	    
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

	ros::NodeHandle nh("~");
	double dt;
	nh.param("period", dt, 0.01);
	if(dt <= 0) {
		dt = 0.01;
	}
	
	/*XmlRpc::XmlRpcValue pathPublishers;
    nh.getParam("pathPublishers", pathPublishers);
	ROS_ASSERT(pathPublishers.getType() == XmlRpc::XmlRpcValue::TypeArray);
   
	for (int32_t i = 0; i < pathPublishers.size(); ++i) {
		ROS_ASSERT(pathPublishers[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        if(pathPublishers[i].hasMember("tfPosTargetFrame")) {
            ROS_INFO("tfPosTargetFrame");
        }
	}*/

    PathNode pathNode(nh, dt);
	while(ros::ok()) {
        pathNode.spin();
    }
}
