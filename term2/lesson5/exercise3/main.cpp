#include <iostream>
#include "Dense"
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {

	/*
	 * Compute the Jacobian Matrix
	 */

	//predicted state  example
	//px = 1, py = 2, vx = 0.2, vy = 0.4
	VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	MatrixXd Hj = CalculateJacobian(x_predicted);

	cout << "Hj:" << endl << Hj << endl;

	return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE

	//pre-compute a set of terms to avoid repeated calculation
	float rho_sq = px*px+py*py;
	float rho = sqrt(rho_sq);
	float rho_cube = (rho_sq*rho);

	//check division by zero
	if(fabs(rho_sq) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/rho), (py/rho), 0, 0,
		  -(py/rho_sq), (px/rho_sq), 0, 0,
		  py*(vx*py - vy*px)/rho_cube, px*(px*vy - py*vx)/rho_cube, px/rho, py/rho;

	return Hj;
}
