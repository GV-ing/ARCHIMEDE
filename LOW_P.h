#ifndef _LOW_P_
#define _LOW_P_
#include <math.h>
#include <LowPower.h>
class LOW_P{
    private: 
        int _T; // secondi
    public: 
        LOW_P(int timer);//Definizione Costruttore 
        void ACTIVE ();
        
};
LOW_P::LOW_P(int timer){//COSTRUTTORE implementazione
  _T=timer;
  
}
void LOW_P::ACTIVE (){
   for (int t=0; t<_T; t++){
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    }
}

#endif // _LOW_P_
