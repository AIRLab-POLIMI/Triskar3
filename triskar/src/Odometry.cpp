#include <vector>
#include <math.h>
#include <string>
#include <ros/ros.h>
#include "triskar/RungeKutta4Odometry.h"
#include "triskar/Odometry.h"
#include "triskar/EuleroOdometry.h"

using namespace std;
Odometry:: Odometry() : position(vector<double>(3,0)){
		
}
//integration: depends on odometry type (eulero / runge_kutta4
virtual void Odometry::integrate(vector<double> velocity, double dt);
//calculates the infinitesimal variation of the function, given the actual position and the velocity
void Odometry::calculateVariation(vector<double> *variation, const vector<double>& position, const vector<double>& velocity) {
    variation->at(0) = (cos(position.at(2)) * velocity.at(0) - sin(position.at(2)) * velocity.at(0));
    variation->at(1) = (sin(position.at(2)) * velocity.at(0) + cos(position.at(2)) * velocity.at(1));
    variation->at(2) = velocity.at(2);
};
vector<double> Odometry::getPosition() {
	return position;
}
