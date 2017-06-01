#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;
using namespace tf;

tf::TransformBroadcaster* odomBroadcaster;

typedef boost::array< double , 3 > vector;
vector odomPos = {0,0,0}, inputVel;

void publishOdom() {

    //create quaternion from theta
    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(odomPos[2]);

    geometry_msgs::TransformStamped odomTrans;
    odomTrans.header.stamp = ros::Time::now();
    odomTrans.header.frame_id = "odom";
    odomTrans.child_frame_id = "base_link_eulero";

    odomTrans.transform.translation.x = odomPos[0];
    odomTrans.transform.translation.y = odomPos[1];
    odomTrans.transform.translation.z = 0.0;
    odomTrans.transform.rotation = odomQuat;

    //send the transform over tf
    odomBroadcaster->sendTransform(odomTrans);
}

/*
 *  Differential Equations:
 *
 *  dx/dt  = cos(th) * vx - sin(th) * vy
 *  dy/dt  = sin(th) * vx + cos(th) * vy
 *  dth/dt = w
 *
 */

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
        odomPos[i] += ds[i];

    publishOdom();
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "odometry_eulero");
    ros::NodeHandle n;
    ros::Subscriber velSub = n.subscribe("vel", 100, velCallback);
    odomBroadcaster = new tf::TransformBroadcaster();

    ros::spin();
}
