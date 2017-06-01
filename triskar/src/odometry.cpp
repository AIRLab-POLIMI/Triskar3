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

//calcolo della variazione di posizione con il metodo di Eulero
void f(vector &ds, double dt, const vector &s) {
    ds[0] = (cos(s[2]) * inputVel[0] - sin(s[2]) * inputVel[1]) * dt;
    ds[1] = (sin(s[2]) * inputVel[0] + cos(s[2]) * inputVel[1]) * dt;
    ds[2] = inputVel[2] * dt;
}

void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    inputVel[0] = msg->linear.x;
    inputVel[1] = msg->linear.y;
    inputVel[2] = msg->angular.z;

    double dt = 0.01; //0.01 s
    vector ds;
    f(ds, dt, odomPos);
    for(int i=0;i<3;i++)
        odomPos[i] += ds[i];    //sommo la variazione di posizione calcolata con Eulero alla vecchia posizione

    publishOdom();
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;
    ros::Subscriber velSub = n.subscribe("vel", 100, velCallback);
    odomPub = n.advertise<nav_msgs::Odometry>("odom", 100);

    ros::spin();
}
