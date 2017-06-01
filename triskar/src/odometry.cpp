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
    odomTrans.child_frame_id = "base_link";

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

void f(vector &dsdt, double dt, const vector &s) {
    dsdt[0] = (cos(s[2]) * inputVel[0] - sin(s[2]) * inputVel[1]);
    dsdt[1] = (sin(s[2]) * inputVel[0] + cos(s[2]) * inputVel[1]);
    dsdt[2] = inputVel[2];
}

void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    inputVel[0] = msg->linear.x;
    inputVel[1] = msg->linear.y;
    inputVel[2] = msg->angular.z;
    
    double dt = 0.01; //0.01 s
    vector k1, k2, k3, k4;
    
    //calculate k1
    f(k1, 0, odomPos);
    
    //calculate k2
    vector sk2;
    for(int i=0; i<3; i++)
        sk2[i] = odomPos[i] + 1/2 * k1[i] * dt;
    f(k2, dt/2, sk2);
    
    //calculate k3
    vector sk3;
    for(int i=0; i<3; i++)
        sk3[i] = odomPos[i] + 1/2 * k2[i] * dt;
    f(k3, dt/2, sk3);
    
    //calculate k4
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
    odomBroadcaster = new tf::TransformBroadcaster();

    ros::spin();
}
