#ifndef _FILTER_H_
#define _FILTER_H_
#include <math.h>
class LPF{
    private: 
        double _y_old;
        double _y;
        double _alfa;
    public: 
        LPF(double f_c, double T_s);//Definizione Costruttore
        double update(double x_raw);
        void set_initial_condition (double y_0);
};
LPF::LPF(double f_c, double T_s){//COSTRUTTORE implementazione
    double tau_c= 1.0/(2.0*PI*f_c); //const. di tempo (data frequenza di taglio(_c = cut))
    _alfa = (T_s ) / (tau_c + T_s );
    _y = 0.0;
    _y_old=_y;
}

void LPF::set_initial_condition(double y_0){
    _y=y_0;
    _y_old=_y;
}
double LPF::update(double x_raw) {
    _y = ((1 - _alfa) * _y_old) + ( _alfa * x_raw);
    _y_old = _y;
    return _y;
}
#endif // _FILTER_H_
 