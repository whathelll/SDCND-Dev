#include <iostream>
#include "ukf.h"

UKF::UKF() {
  //TODO Auto-generated constructor stub
  Init();
}

UKF::~UKF() {
  //TODO Auto-generated destructor stub
}

void UKF::Init() {

}


/*******************************************************************************
* Programming assignment functions:
*******************************************************************************/

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
     Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; //time diff in sec
/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //predict sigma points
  for (int i = 0; i< 2*n_aug+1; i++) {
	  VectorXd x = Xsig_aug.col(i);

	  double px = x(0);
	  double py = x(1);
	  double v = x(2);
	  double psi = x(3);
	  double psi_d = x(4);
	  double nu_a = x(5);
	  double nu_a_dd = x(6);

	  double newX, newY;
	  //avoid division by zero
	  if(fabs(psi_d) > 0.001) {
		  newX = px + v/psi_d * (sin(psi + psi_d*delta_t) - sin(psi));
		  newY = py + v/psi_d * (-cos(psi + psi_d*delta_t) + cos(psi));
	  } else {
		  newX = px + v * cos(psi) * delta_t;
		  newY = py + v * sin(psi) * delta_t;
	  }
	  double newV = v;
	  double newPsi = psi + psi_d * delta_t;
	  double newPsi_d = psi_d;

	  //add noise
	  newX += 0.5 * delta_t * delta_t * cos(psi) * nu_a;
	  newY += 0.5 * delta_t * delta_t * sin(psi) * nu_a;
	  newV += delta_t * nu_a;
	  newPsi += 0.5 * delta_t * delta_t * nu_a_dd;
	  newPsi_d += delta_t * nu_a_dd;

	  //write predicted sigma points into right column
	  Xsig_aug(0, i) = newX;
	  Xsig_aug(1, i) = newY;
	  Xsig_aug(2, i) = newV;
	  Xsig_aug(3, i) = newPsi;
	  Xsig_aug(4, i) = newPsi_d;
  }

//  std::cout << "debug = " << std::endl << x << std::endl;
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_aug << std::endl;

  //write result
  *Xsig_out = Xsig_pred;

}
