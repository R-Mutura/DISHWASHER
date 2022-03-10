#include <Arduino.h>

//and optimize code serial printing
#define DEBUG 1

#if     DEBUG == 1
#define debug(x)      Serial.print(x)
#define debugln(x,y)  Serial.println(x,y)
#define debugln(x)    Serial.println(x)
#else
#define debug(x)
#define debugln(x,y) 
#define debugln(x)
#endif


//INITIALIZING ALL PINS AND THEIR STATESS HERE
#define pressure_sw_1 49 //PL0
#define pressure_sw_2 48 //pl1

#define door_sensor 18

#define  rtd_cs    53
#define rtd_drdy   10
#define  rtd_cs_1  11
#define rtd_drdy_1 12

#define speaker 13

#define screen_rst A6 //pf6

#define irq  19 //pd2 this is the acrylic interrupt
#define water_lvl_low  34//PC3 
#define water_lvl_high 33//PC4 

#define zero_cross_detect 2 //pe4 int4

#define solenoid 35//pc2
#define perilistic_pump_1 37 //pc0
#define perilistic_pump_2 36//pc1

#define drain_pump 17 //PH0 
#define wash_pump 16//PH1  

#define tank_heater 6//ph3

#define boiler_heater_l1 7//ph4
#define boiler_heater_l2 8 //ph5
#define boiler_heater_l3 9 //ph6

#define rgb_onoff_blu A0 //pf0
#define rgb_onoff_red A1//pf1
#define rgb_onoff_grn A2//pf2

#define start_blu  PF3 //A3//pf3
#define start_red  PF4 //A4//pf4
#define start_grn  PF5 //A5//pf5

//temperature reading pins
#define tank_rtd    A8  //pk0
#define boiler_rtd  A9 //pk1


void pin_init(){
pinMode(pressure_sw_1, INPUT);
pinMode(pressure_sw_2, INPUT);
pinMode(door_sensor, INPUT);
pinMode(door_sensor, INPUT);

pinMode(water_lvl_low, INPUT);
pinMode(water_lvl_high, INPUT);
//pinMode(zero_cross_detect, INPUT); //attached interrupt


pinMode(rtd_cs, OUTPUT);
pinMode(rtd_cs_1, OUTPUT);
pinMode(rtd_drdy, OUTPUT);
pinMode(rtd_drdy_1, OUTPUT);

pinMode(speaker, OUTPUT);

pinMode(screen_rst, OUTPUT);

pinMode(irq, INPUT);


pinMode(solenoid, OUTPUT);
pinMode(perilistic_pump_1 , OUTPUT);
pinMode(perilistic_pump_2, OUTPUT);
pinMode(drain_pump , OUTPUT);
pinMode(wash_pump , OUTPUT);

pinMode(tank_heater, OUTPUT);

pinMode(boiler_heater_l1 , OUTPUT);
pinMode(boiler_heater_l2, OUTPUT);
pinMode(boiler_heater_l3, OUTPUT);

pinMode(rgb_onoff_blu , OUTPUT);
pinMode(rgb_onoff_red, OUTPUT);
pinMode(rgb_onoff_grn , OUTPUT);

pinMode(tank_rtd, INPUT);
pinMode(boiler_rtd, INPUT);

//we ll use register manipulation for these pins which will used inside an ISR ROUTINE
//pinMode(start_blu , OUTPUT);
//pinMode(start_red , OUTPUT);
//pinMode(start_grn , OUTPUT);
//START LEDS IN REGISTER SETTING AS OUTPUT
/*#define start_blu A3//pf3
#define start_red  A4//pf4
#define start_grn  A5//pf5*/
DDRF |=(1<<start_blu); //SETTING PFx PIN AS OUTPUT in the DDRF register
DDRF |=(1<<start_red); //SETTING PFx PIN AS OUTPUT in the DDRF register
DDRF |=(1<<start_grn); //SETTING PFx PIN AS OUTPUT in the DDRF register

PORTF &=(0<<start_blu); //initialize the pin to low
PORTF &=(0<<start_red); //initialize the pin to low
PORTF &=(0<<start_grn); //initialize the pin to low

}


const float vt_factor=1.88;
const float offset =0;

float tempRead(int pin)
{
  int sensorvalue=0;
  for(int i=0; i<=5; i++)
  {
      sensorvalue+=analogRead(pin);
      i++;
  }
  sensorvalue=sensorvalue/7;
  float value = sensorvalue * (5.0 / 1023.0);
  float temp_c = (((value * 100) / vt_factor) + offset);
  debug("Temp");
  if(DEBUG == 1){ Serial.println(temp_c,2);}
  
  
  return temp_c;
}