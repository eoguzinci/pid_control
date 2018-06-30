#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  L2_error = 0.0;

  prev_cte = 0.0;

}

void PID::UpdateError(double cte) {
	if (cte-p_error) {
	// to get correct initial d_error
    d_error = cte - p_error;
  	p_error = cte;
  	i_error += cte;
  }
  


}

double PID::TotalError() {
	L2_error += (p_error * p_error);
	return L2_error;
}

void PID::Run(double &cte, double &steer_value, double &throttle, double& speed, double &current_error){

  UpdateError(cte);
  current_error = TotalError();

  steer_value = - (Kp * p_error) - (Ki * i_error) - (Kd * d_error);

  // Limit the steering angle between [-1, 1]
  if (steer_value > 1.0) {
    steer_value = 1.0;
  } else if (steer_value < -1.0) {
    steer_value = -1.0;
  }

  // throttle is a function of steering value and speed to avoid going out of the track
  if(speed<25){
    throttle = 0.3; // constant speed 0.3
  // throttle = 0.8 - fabs(steer_value); // linear model with 0.3 average
  // throttle = -0.45*pow(steer_value,2)+0.45; // quadratic model with 0.3 average  
  }
  else{
    throttle = 0.0;
  }
  

}
