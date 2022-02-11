#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include "Adafruit_MPR121.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1305.h>



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
//end of touch panel initializatin
//screen initialization
#define OLED_RESET 9

// software SPI
//Adafruit_SSD1305 display(128, 64, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
// hardware SPI - use 7Mhz (7000000UL) or lower because the screen is rated for 4MHz, or it will remain blank!
//Adafruit_SSD1305 display(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS, 7000000UL);

// I2C
Adafruit_SSD1305 mydisplay(128, 64, &Wire, OLED_RESET);
/*end of screen */
//*********************************************************************************************
/*FLAGS FOR INTERRUPTS*/
volatile int installer_menu_flag = 0;
volatile int start_flag = 0;
volatile int on_off_flag = 0;
volatile int menu_flag = 0;
volatile int back_flag = 0;

/**/
volatile int commissioning_flag=0;
volatile int filling_flag=0;
//...........................................................................................
//FUNCTION PROTOTYPES
void getNumber();
void water_filling_process();
void commissioning();
void door_open_ISR();

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
  attachInterrupt(digitalPinToInterrupt(irq),getNumber,LOW); //PD2 == 19 IS INTERRUPT NUMBER 4
 //DONE WITH ACRYLIC TOUCH PANEL 

//ADDING A DOOR INTERRUPT SO THAT WHEN IT IS OPENED AND A PROCESS IS ON GOING, THE PROCESS IS INTERRUPTED.
 attachInterrupt(digitalPinToInterrupt(door_sensor),door_open_ISR, HIGH);
// initializing mydisplay
if ( ! mydisplay.begin(0x3C) ) {
     Serial.println("Unable to initialize OLED");
     while (1) yield();
  }

  mydisplay.display(); // show splashscreen
  delay(1000);
  mydisplay.clearDisplay();   // clears the screen and buffer
  mydisplay.setTextSize(1);
    mydisplay.setTextColor(WHITE);
    mydisplay.setCursor(0,0);
    mydisplay.println("test test!");
    mydisplay.display();
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
  for (int j=0; j<6; j++)  // Check how many electrodes were pressed
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
    if(on_off_flag==0){
      on_off_flag=1;

    }
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
      if (start_flag==0 && on_off_flag==1)
    {
        start_flag=1;
    }
   
    
   }
  }
 else if(touchNumber == 2)
 {//if two buttons are pressed then we proceed to do the relevant check of which buttons are they
   if ((touchstatus & (1<<on_off)) && (touchstatus & (1<<start)))
   {
     // GO TO THE INSTALLER MENU
    
   }
 else
 {
   //do nothing
   return;
 }  
 
 }
  


}

void commissioning(){
commissioning_flag=1;
//implement timer software interrupt to turn on/off the led

//start filling process 
//THE TANK IS EMPTY
water_filling_process();


}

void water_filling_process(){
 filling_flag=1; //set to high to be used in the ISR routines
 //digitalRead(water_lvl_low);
 while(digitalRead(water_lvl_high))//initially the pin is high(1) => we wait for it to go low
 {
   //TURN ON THE SOLENOID UNTIL THE WATER LEVEL HIGH PROBE READS LOW(INDICATING THE TANK IS FULL)

   digitalWrite(solenoid, HIGH);//turn the solenoid on thus opening it

 }
 filling_flag=0;//reset the filling flag to low 
return;//once done we return
//end of filling process
}

void door_open_ISR()
{
  
  if(commissioning_flag==1 && filling_flag==1)

  {//if this routine is called while the commissioning process is on going it will stop everything and display door open
   
   digitalWrite(solenoid, LOW);//TURN OFF SOLENOID AND THUS THE FILLING PROCESS
   
    mydisplay.clearDisplay();
    mydisplay.setTextSize(2);
    mydisplay.setTextColor(WHITE);
    mydisplay.setCursor(20,10);
    mydisplay.println("DOOR OPEN!!");
    mydisplay.display();
    
  }
  while (digitalRead(door_sensor))
  {
    //wait for the door to be closed,,,,nothing else will be done until the door closes
  }
  //the we delay for 500mS after the door is closed
  long timeon = millis();
  while (millis()-timeon >=500)
  {
    
  }
  
  return; //break from the ISR and return to where the program left off
}
