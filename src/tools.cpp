#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if((estimations.size() != ground_truth.size()) || estimations.size() == 0){
  	cout << "INVALID INPUT(estimations AND ground_truth)" << endl;
    return rmse;
  }
  
  for(int i = 0; i < estimations.size(); i++){
  	VectorXd residual = estimations[i] - ground_truth[i];
  	residual = residual.array() * residual.array();
  	rmse += residual;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float temp1 = px*px +py*py;

  if(temp1 == 0){
  	cout << "ERROR - Division by zero" <<endl;
  	return Hj;
  }

  float temp2 = sqrt(temp1);

  Hj << px/temp2, py/temp2, 0, 0,
  		-py/temp1, px/temp1, 0, 0,
  		(py*(vx*py-vy*px)/pow(temp2,3)), (px*(vy*px-vx*py))/pow(temp2,3), px/temp2, py/temp2;

  return Hj;
}
