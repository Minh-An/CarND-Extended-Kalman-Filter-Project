#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check estimation vector size is equal to ground truth vector size
  if (estimations.size() != ground_truth.size())
  {
      std::cout << "Invalid input." << std::endl;
      return rmse;
  }

  // accumulate squared residuals
  for (uint32_t i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd c = estimations[i] - ground_truth[i];
    c = c.array() * c.array();
    rmse = rmse + c;    
  }

  // calculate the mean
  rmse = rmse / estimations.size();

  // squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // check division by zero
  if (px == 0 && py ==0)
  {
    std::cout << "CalculateJacobian () - Error - Division By Zero" << std::endl;
    return Hj; 
  }
  
  // compute the Jacobian matrix

  // values for demoninators of Jacobian matrix 
  float d1 = px*px + py*py;
  float d2 = sqrt(d1);
  float d3 = d1*d2;
  

  Hj <<       px/d2,              py/d2,            0,     0,
              py/d1,              px/d1,            0,     0,
        py*(vx*py-vy*px)/d3, px*(vy*px-vx*py)/d3, px/d2, py/d2;
  
  return Hj;

}
