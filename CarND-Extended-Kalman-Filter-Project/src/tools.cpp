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
    VectorXd c(4);
    rmse << 0,0,0,0;

    if (estimations.size()==0||estimations.size()!=ground_truth.size()){

        std::cout<<"All estimations not present"<<endl;
        return rmse;}
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        c = estimations[i]-ground_truth[i];
        c = (c.array())*(c.array());
		rmse = rmse + c;
	}
	//calculate the mean
	rmse = rmse /estimations.size();

	//calculate the squared root
    rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);

    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //Initializing Hj matrix
    Hj << 0,0,0,0,
          0,0,0,0,
          0,0,0,0;

    //check division by zero
    if(px*px + py*py==0)
    {
        std::cout<<"CalculateJacobian - Error - Division by Zero"<<endl;
        return Hj;
    }

    //compute the Jacobian matrix
    float pxy = px*px + py*py;
    float sqrpxy = sqrt(pxy);
    float thrsqrpxy = pxy*sqrpxy;

    Hj << px/sqrpxy,py/sqrpxy,0,0,
         -py/pxy,px/pxy,0,0,
          py*(vx*py-vy*px)/thrsqrpxy,px*(-vx*py+vy*px)/thrsqrpxy,px/sqrpxy,py/sqrpxy;
    
    return Hj;
}
