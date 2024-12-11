#include <math.h>                   
#include <SolarCalculator.h>        
#include <TimeLib.h>                
#include <Wire.h>                   
#include <QMC5883LCompass.h>        
#include <EEPROM.h>                 
#include <LowPower.h>          
#include <JY901.h> 
#include "DLPF.h" 
#include "PID.h"
#include "LOW_P.h"


#define     PIN_LED         3
#define     PIN_TARGET      8
#define     GRAD_TO_RAD     PI/180.0 
#define     RAD_TO_GRAD     180.0/PI  
#define     F_C_MEAS        50.0 //Hz
#define     F_C_DES         2.0 //Hz
#define     T_S_CTRL        1.0/250.0 //s-->1/Hz
#define     T_S_SUN         10.0 //s
#define     T_S_TARGET      1.0/250.0
#define     T_S_LOW_P       45.00 //s
#define     SOGLIA          0.001 
#define     KP_AZ           0.01
#define     KI_AZ           0.003
#define     KP_EL           0.008
#define     KI_EL           0.001
#define     KD              0.00
#define     LOW_P_TIME      60*2 //s

/*Oggetti*/
LPF LPF_az_mis(F_C_MEAS/60, T_S_CTRL);
LPF LPF_el_mis(F_C_MEAS, T_S_CTRL);
LPF LPF_az_des(F_C_DES, T_S_CTRL);
LPF LPF_el_des(F_C_DES, T_S_CTRL);

PID PID_AZ (SOGLIA, KP_AZ , KI_AZ, KD);
PID PID_EL (SOGLIA, KP_EL, KI_EL, KD);

LOW_P LOW_P(LOW_P_TIME);

QMC5883LCompass compass;

/*PARAMETRI*/
const int DX_el=5, SX_el=6, DX_az= 10, SX_az=11; 
const double latitude = 40.828660 /*°*/, longitude = 14.189816 /*°*/;
const int address_az = 0, address_el = 100;

const double T_s_tsrget = T_S_TARGET;
const double T_s_sun = T_S_SUN * 1000; //mS
const double T_s_ctrl = T_S_CTRL* 1000;  //mS
const double T_s_low_p = T_S_LOW_P*1000;   //ms
//

/*VARIABILI GLOBALI*/
double  t_sun_old = 0.00,  t_ctrl_old = 0.00, t_target_old = 0.00, t_lowp_old = 0.00;
double az_sun, el_sun, az_target, el_target, az_des, el_des, az_mis, el_mis, s_az, s_el;
bool stato= HIGH;



void setup() {
  delay (1000);
  EEPROM.get(address_az, az_target);
  EEPROM.get(address_el, el_target);
  compass.init();
  delay(500);
  JY901.StartIIC();
  Serial.begin(115200);
  setTime(toUtc(compileTime()));
  time_t utc = now();
  delay(500);
  pinMode(PIN_TARGET, INPUT_PULLUP);
  pinMode(4, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  calcHorizontalCoordinates(utc, latitude, longitude, az_sun, el_sun);
    
  LPF_az_des.set_initial_condition((az_sun + az_target) / 2.00);
  LPF_el_des.set_initial_condition((el_sun + el_target) / 2.00);
  LPF_az_mis.set_initial_condition(compass.getAzimuth()+180);
  LPF_el_mis.set_initial_condition((float)JY901.stcAngle.Angle[0]/32768*180);
  }

  
void loop() {

  if (millis() - t_sun_old >= T_s_sun) {
    time_t utc = now();
    calcHorizontalCoordinates(utc, latitude, longitude, az_sun, el_sun);
    while (el_sun<0){
      LOW_P.ACTIVE();
      delay(1000);
      calcHorizontalCoordinates(utc, latitude, longitude, az_sun, el_sun);
    }    
    t_sun_old = millis();
  }
  
  if (millis() - t_ctrl_old >= T_s_ctrl ) {
    compass.read();
    az_mis= LPF_az_mis.update(map(compass.getAzimuth(), -180, 180, 0, 360));
    JY901.GetAngle();
    el_mis= LPF_el_mis.update((float)JY901.stcAngle.Angle[0]/32768*180);
    az_des =LPF_az_des.update (((az_sun + az_target) / 2.00));
    el_des =LPF_el_des.update(((el_sun + el_target) / 2.00)); //grad
    s_az= PID_AZ.signal (az_des, az_mis, T_s_ctrl/1000);
    s_el= PID_EL.signal (el_des, el_mis, T_s_ctrl/1000);
    Controllo_az(s_az);
    Controllo_el(s_el);
  Serial.print(millis());
  Serial.print(", ");
  Serial.print(az_mis);
  Serial.print(", ");
  Serial.println(map(compass.getAzimuth(), -180, 180, 0, 360));
    //stato = !stato;
    //digitalWrite(4, stato);
    t_ctrl_old = millis();
  }  

  if (millis() -t_lowp_old >= T_s_low_p and (abs(az_des-az_mis)<=0.5) and (abs(el_des-el_mis)<=1)){
    
    analogWrite(DX_az, 0);
    analogWrite(SX_az, 0);
    analogWrite(DX_el, 0);
    analogWrite(SX_el, 0);
    LOW_P.ACTIVE();
    delay(1000);
    analogWrite(DX_az, 0);
    analogWrite(SX_az, 0);
    analogWrite(DX_el, 0);
    analogWrite(SX_el, 0);
    compass.read();
    az_mis= LPF_az_mis.update(map(compass.getAzimuth(), -180, 180, 0, 360));
    if (az_mis==360) az_mis=0;
    JY901.GetAngle();
    el_mis= LPF_el_mis.update((float)JY901.stcAngle.Angle[0]/32768*180);
    t_lowp_old=millis();
  }


  if (millis() - t_target_old >= T_s_tsrget){
    controllo_taget();
    digitalWrite(PIN_LED,digitalRead(PIN_TARGET));
    t_lowp_old=millis();    
    t_target_old = millis();
  } 
  //PRINT();
  

}





void controllo_taget(){
  if(digitalRead(PIN_TARGET)){
    
    switch (lettura_pulsante()){
      case 0:
        analogWrite(DX_az, 80);
        analogWrite(SX_az, 0);
        analogWrite(SX_el, 0);
        analogWrite(DX_el, 0);
        break;
      case 1:
        analogWrite(DX_az, 0);
        analogWrite(SX_az, 80);
        analogWrite(SX_el, 0);
        analogWrite(DX_el, 0);
        break;
      case 2:
        analogWrite(DX_az, 0);
        analogWrite(SX_az, 0);
        analogWrite(DX_el, 50);
        analogWrite(SX_el, 0);
        break;
      case 3:
        analogWrite(DX_az, 0);
        analogWrite(SX_az, 0);
        analogWrite(DX_el, 0);
        analogWrite(SX_el, 50);
        break;
      case 4:
        analogWrite(DX_az, 0);
        analogWrite(SX_az, 0);
        analogWrite(SX_el, 0);
        analogWrite(DX_el, 0);
        break;
    }
    
        compass.read();
        az_mis= LPF_az_mis.update(map(compass.getAzimuth(), -180, 180, 0, 360));
        az_des=az_mis;
        az_target= (2*az_des)-az_sun;
        JY901.GetAngle();
        el_mis=LPF_el_mis.update((float)JY901.stcAngle.Angle[0]/32768*180);
        el_des=el_mis;
        el_target= (2*el_des)-el_sun;
        
        EEPROM.put(address_az, az_target);
        EEPROM.put(address_el, el_target);

  } 

  
}

void Controllo_az(double S){
  if (S>0){
   analogWrite(DX_az, S*255);
   analogWrite(SX_az, 0);
  }else if (S<0){
    analogWrite(SX_az, -S*255);
    analogWrite(DX_az, 0);
  }else if (S==0){
    analogWrite(SX_az, 0);
    analogWrite(DX_az, 0);
  }
  
}
void Controllo_el(double S){
  if (S>0){
   analogWrite(DX_el, S*255);
   analogWrite(SX_el, 0);
  }else if (S<0){
    analogWrite(SX_el, -S*255);
    analogWrite(DX_el, 0);
  }else if (S==0){
    analogWrite(SX_el, 0);
    analogWrite(DX_el, 0);
  }
  
}


/*-------------------------------------------CALCOLO DATA E ORA---------------------------------------------*/
time_t compileTime() {
  const uint8_t COMPILE_TIME_DELAY = 8;
  const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char chMon[4], *m;
  tmElements_t tm;
  strncpy(chMon, compDate, 3);
  chMon[3] = '\0';
  m = strstr(months, chMon);
  tm.Month = ((m - months) / 3 + 1);
  tm.Day = atoi(compDate + 4);
  tm.Year = atoi(compDate + 7) - 1970;
  tm.Hour = atoi(compTime);
  tm.Minute = atoi(compTime + 3);
  tm.Second = atoi(compTime + 6);
  time_t t = makeTime(tm);
  return t + COMPILE_TIME_DELAY;
}
/*----------------------------------------------------------------------------------------------------------*/

/*-----------------------------CONVERSIONE IN UTC------------------------------*/
time_t toUtc(time_t local) {
  int utc_offset = 2;
  return local - utc_offset * 3600L;
}
/*-----------------------------------------------------------------------------*/


int lettura_pulsante()
{
  const int Az_up = 819, Az_dw =  612, El_up = 408 , El_dw= 203, delta = 10;
  int Value = analogRead(A3);
  if (Value <Az_up+delta && Value >Az_up-delta) return 0;
  if (Value <Az_dw+delta && Value >Az_dw-delta) return 1;
  if (Value <El_up+delta && Value >El_up-delta) return 2;
  if (Value <El_dw+delta && Value >El_dw-delta) return 3;
  if (Value <10+delta && Value >10-delta) return 4;
}

void PRINT() {
  /*Serial.print(millis());
  Serial.print(", ");
  Serial.print(az_sun);
  Serial.print(", ");
  Serial.print(el_sun);
  Serial.print(", ");
  Serial.print(az_des);
  Serial.print(", ");
  Serial.print(el_des);
  Serial.print(", ");*/
  Serial.print((float)JY901.stcAngle.Angle[0]/32768*180);
  Serial.print(", ");
  Serial.print(el_mis);
  Serial.print(", ");
  Serial.print(az_mis);
  Serial.print(", ");
  Serial.println(map(compass.getAzimuth(), -180, 180, 0, 360));


}
