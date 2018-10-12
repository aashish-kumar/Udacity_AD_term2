#ifndef PID_H
#define PID_H
#include <iostream>
#include <queue>
class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  std::queue<double> q1;
  double cte_p;
  int fir = 1;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
