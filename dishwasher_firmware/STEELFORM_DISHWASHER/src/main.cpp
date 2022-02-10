#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include "Adafruit_MPR121.h"

//and optimize code serial printing
#define DEBUG 1
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#include "Pin_init.h"

//temperature measurement using MAX31865 parameter initialization
Adafruit_MAX31865 tank_rtd = Adafruit_MAX31865(rtd_cs); //temp sensor 1
Adafruit_MAX31865 boiler_rtd = Adafruit_MAX31865(rtd_cs_1); //temp sensor 2
#define RREF 4300.0
#define RNOMINAL 1000
//end of temp initializations

//initializing touch panel
#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif
#define panel_add 0X5a
//matching electrode with the buttons on the acrylic panel
#define on_off 0
#define menu   1
#define back   2
#define left   3 
#define set    4 
#define right  5
#define start  6

Adafruit_MPR121 panel = Adafruit_MPR121();
//end of screen initializatin


//FUNCTION PROTOTYPES
void getNumber();


void setup() {
  // put your setup code here, to run once:
  
  pin_init();
  //start the the temp measurement function
  tank_rtd.begin(MAX31865_3WIRE);
  boiler_rtd.begin(MAX31865_3WIRE);
 //initializing ACRYLIC PANEL COMPONENTS
  if (!panel.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  debugln("MPR121 found!");
  //WE ATTACH AN INTERRUPT TO THE MPR121 IRQ PIN SO THAT WHEN THE PANEL IS TOUCHED THE ARDUINO KNOWS(NO POLLING)
  attachInterrupt(4,getNumber,LOW); //PD2 == 19 IS INTERRUPT NUMBER 4
 //DONE WITH ACRYLIC TOUCH PANEL 

}



void loop() {

  // put your main code here, to run repeatedly:
  float tank_temp=tank_rtd.temperature(RNOMINAL, RREF);
  float boiler_temp=boiler_rtd.temperature(RNOMINAL, RREF);

}

void getNumber(){

  int touchNumber = 0;
  uint16_t touchstatus;
  char digits;

  touchstatus = panel.touched();
  for (int j=0; j<12; j++)  // Check how many electrodes were pressed
  {
    if ((touchstatus & (1<<j))) //AND BIT WISE OPERATION IF THERE IS A 1 IN THE RECEIVED DATA THEN THE TOUCH NUMBER WILL INCREMENT
      touchNumber++; //GIVES THE NUMBER OF PRESSED ELECTRODES IN THE TOUCH PANEL

  }
  //NOW TO DO SOME OPERATIONS DEPENDING ON THE PRESSED BUTTON
  //THIS LINES ARE DEPENDEDNT ON THE TOUCH NUMBER ABOVE
  if (touchNumber == 1)
  {//if only one key i presssed then we check what key is it and perform our task accordingly
   if(touchstatus & (1<<on_off))
   {
     //TO KEEP THIS ISR CALL ROUTINE SHORT AND MANAGEABLE - WE SET FLAGS IN HERE WHICH WILL BE HANDLED IN THE VOID LOOP

     //perform the on off function eg decommissioning
     //check if the machine was a sleep and wake up if it was not asleep then continue with the decommissioning process
     //the decomissioninf and any other process function is called here

   } 
   else if(touchstatus & (1<<menu))
   {

   }
   else if(touchstatus & (1<<back))
   {

   }
   else if(touchstatus & (1<<left))
   {

   }
   else if(touchstatus & (1<<set))
   {

   }
   else if(touchstatus & (1<<right))
   {

   }
   else if(touchstatus & (1<<start))
   {

   }
  }
 else if(touchNumber == 2)
 {//if two buttons are pressed then we proceed to do the relevant check of which buttons are they
   if ((touchstatus & (1<<on_off)) && (touchstatus & (1<<start)))
   {
     // GO TO THE INSTALLER MENU

   }
   

 }
  


}