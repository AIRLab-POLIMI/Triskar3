#ifndef INCLUDE_TRISKAR_ODOMETRY_H_
#define INCLUDE_TRISKAR_ODOMETRY_H_

#include <vector>
#include <string>

using namespace std;

class Odometry {
public:
	Odometry();
	~Odometry();
	virtual void integrate(const vector<double>& velocity, double dt) = 0;
	void calculateVariation(vector<double> *variation, const vector<double>& position, const vector<double>& velocity);
	vector<double> getPosition();
protected:
	vector <double>*position;
};

#endif /* INCLUDE_TRISKAR_ODOMETRY_H_ */
