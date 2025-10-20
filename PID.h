#ifndef _PID_H_
#define _PID_H_

#include <math.h>


class PID{
    private: 
        double _e;
        double _u;
        double _e_i;
        double _e_d;
        double _e_prev;
        double _delta;
        double _Kp;
        double _Ki;
        double _Kd;
    public: 
        PID(double soglia, double Kp, double Ki, double Kd );//Definizione Costruttore
        double  signal(double X_des, double X_mis, double Sec);
        double error(double a1, double a2);
};

PID::PID(double soglia,  double Kp, double Ki, double Kd){//COSTRUTTORE implementazione
  _e = 0.00;
  _u = 0.00;
  _e_i=0.00;
  _e_d=0.00;
  _e_prev = 0.0;
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _delta = soglia;
  
}

double PID::signal(double X_des, double X_mis, double Sec){//COSTRUTTORE implementazione
  _e = error(X_des, X_mis);
  _e_i += _e * Sec;
  _e_d = (_e - _e_prev) / Sec;
  _u = _e * _Kp + _e_i * _Ki+ + _Kd * _e_d ;
  if (_u > 1) {
    _u = 1;
    _e_i -= _e * Sec;
  }
  if (_u < - 1) {
    _u = -1;
    _e_i -= _e * Sec;
  }
  _e_prev = _e;
  return _u;
}



double PID::error(double a1, double a2) {
  a1 = a1;
  a2 = a2;
  double err = 0.00;
  if (a1 > a2) {
    err = a1 - a2;
  } else if (a1 < a2) {
    err = (a2 - a1) * -1;
  }
  if (abs(err) < _delta) err = 0.000;
  return err;
}



#endif // _PID_H_
