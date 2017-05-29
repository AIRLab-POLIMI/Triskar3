#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#include <iostream>
#include <string>
#include <unistd.h>

#include <sstream>
#include <cstdio>


#include <boost/array.hpp>

#include <boost/numeric/odeint.hpp>
using namespace boost::numeric::odeint;


ros::Publisher odomPub;

/*
 *  Differential Equations:
 *
 *  dx/dt  = cos(th) * vx - sin(th) * vy  = f(t, x)
 *  dy/dt  = sin(th) * vx + cos(th) * vy  = f(t, y)
 *  dth/dt = w                            = f(t, th)
 *
 */

typedef boost::array< double , 3 > vector;

vector odomPos = {0,0,0}, inputVel;

void publishOdom() {

    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = odomPos[0];
    odom.pose.pose.position.y = odomPos[1];
    odom.pose.pose.position.z = 0.0;
    
    //set the orientation
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = odomPos[2];
    odom.pose.pose.orientation.w = 0;
    
    //set the velocity
    odom.twist.twist.linear.x = inputVel[0];
    odom.twist.twist.linear.y = inputVel[1];
    odom.twist.twist.angular.z = inputVel[2];

    odomPub.publish(odom);
}

void f(vector &ds, double dt, const vector &s) {
    ds[0] = (cos(s[2]) * inputVel[0] - sin(s[2]) * inputVel[1]);
    ds[1] = (sin(s[2]) * inputVel[0] + cos(s[2]) * inputVel[1]);
    ds[2] = inputVel[2];
}

void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    inputVel[0] = msg->linear.x;
    inputVel[1] = msg->linear.y;
    inputVel[2] = msg->angular.z;
    
    double dt = 0.01; //0.01 s
    vector k1, k2, k3, k4;
    
    //calcolo k1
    f(k1, 0, odomPos);
    
    //calcolo k2
    vector sk2;
    for(int i=0; i<3; i++)
        sk2[i] = odomPos[i] + 1/2 * k1[i] * dt;
    f(k2, dt/2, sk2);
    
    //calcolo k3
    vector sk3;
    for(int i=0; i<3; i++)
        sk3[i] = odomPos[i] + 1/2 * k2[i] * dt;
    f(k3, dt/2, sk3);
    
    //calcolo k4
    vector sk4;
    for(int i=0; i<3; i++)
        sk4[i] = odomPos[i] + k3[i] * dt;
    f(k4, dt, sk4);
    
    for(int i=0; i<3; i++)
        odomPos[i] += dt/6 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    
    publishOdom();
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;
    ros::Subscriber velSub = n.subscribe("vel", 100, velCallback);
    odomPub = n.advertise<nav_msgs::Odometry>("odom", 100);

    ros::spin();
}
