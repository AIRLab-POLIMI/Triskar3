#include <vector>
#include <math.h>
#include <string>
#include <ros/ros.h>
#include "triskar/Odometry.h"
#include "triskar/EuleroOdometry.h"

void integrate(const vector<double>& velocity, double dt) {
    ROS_INFO("INTEGRATING WITH EULERO");
	vector<double> variation;
    calculateVariation(&variation, position, velocity);
    for(int i=0;i<3;i++)
        position.at(i) += variation.at(i) * dt;    		//sums variation
}
