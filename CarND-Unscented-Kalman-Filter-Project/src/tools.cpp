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
  /**
   *   TODO:
   *       * Calculate the RMSE here.
   *         */
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

