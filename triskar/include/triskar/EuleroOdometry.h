#ifndef INCLUDE_TRISKAR_EULERO_ODOMETRY_H_
#define INCLUDE_TRISKAR_EULERO_ODOMETRY_H_

#include <vector>
#include <math.h>
#include <string>
#include "triskar/Odometry.h"

using namespace std;

class EuleroOdometry : public Odometry{
public:
	using Odometry::Odometry;
	void integrate(const vector<double>& velocity, double dt) override;
	vector<double> getPosition();
};

#endif /* INCLUDE_TRISKAR_EULERO_ODOMETRY_H_ */
