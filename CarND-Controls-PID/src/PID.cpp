#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	this->p_error = 0;
        this->i_error = 0;
        this->cte_p = 0;
        this->d_error = 0;
	this->fir = 1;
	}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void PID::UpdateError(double cte) {
       // if(q1.size()!=50)
	q1.push(cte);
        i_error = i_error+cte;
        if(q1.size()>50){
          i_error = i_error - q1.front();
          q1.pop();}
        if(this->fir)
	{
		this->cte_p = cte;
                this->fir = 0;
        }
	this->p_error = cte;
	//this->i_error = i_error + cte;
	this->d_error = cte - this->cte_p;
	this->cte_p = cte;
}

double PID::TotalError() {
	return Kp*p_error + Ki*i_error + Kd*d_error; 
}

