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
//#include "processes.h"

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
/*FLAGS variables FOR INTERRUPTS*/
volatile int installer_menu_flag = 0;
volatile int start_flag = 0;
volatile int on_off_flag = 0;
volatile int menu_flag = 0;
volatile int back_flag = 0;
volatile int pump_counter=0;
volatile long mysecond =0;
volatile int last_CH1_state = 0;
volatile bool zero_cross_detected=false;

/**/
//variables to be used in function first
volatile int commissioning_flag=0;
volatile int filling_flag=0;
volatile int getting_ready_flag =0;

int full=0;
int pump_1_on=0;
int ready_flag=0;
int in_process_flag=0;
int rinse_aid_pump=0;
int done_refilling=0;
int rinsing=0;
int sensitive_wash_ready=0;

//constants to be set and read from permanent storage of the atmega
int setpoint_tank = 45; //degrees 
int setpoint_boiler =72; //degrees
float detergent_dose=5;//ml //this will be read from the eeprom memory and stored here to be used when calculating detergent time 
float rinse_aid_dose=5;//ml
int menu_screen_2 = 0; //will store the menu position selected(such that the menu in use is persistent even after power down)

//processes constant time
const int normal_time=90;
const int intensive_time=150;
const int cookware_time=270;
//...........................................................................................
//global variables
int detergent_time=0;
int rinse_aid_time=0;
float tank_temp = 0;
float boiler_temp=0;
//end of global vars
//TIMER3 FUNCTION CONSTANTS
const uint16_t t3_load=0;
const uint16_t t3_comp=62500;//this value will be loaded to the compare register of the timer. so the timer upon reachin this value an interrupt is generated.

// TRIAC CONTROL CONSTAnts
const int firing_delay = 7400;
const int maximum_firing_delay = 7400;

const float peristalitic_feedrate=0.4166667;//ml
//FUNCTION PROTOTYPES
void getNumber();
void water_filling_process();
void commissioning();
void door_open_ISR();
void getting_ready();
void zero_cross();
int pid_tank_control(int real_temperature, int setpoint);
int pid_boiler_control(int real_temperature, int setpoint);
void update_screen_2();
void basic_process(int timing);
void glassware();


void setup() {
  // put your setup code here, to run once:
  
  pin_init();
  //start the the temp measurement function

 //timer3 initialization
  
  TCCR3A =0; //re initialize the timer to its default state(set everything to 0)
  
  //BITS TO ENSURE THAT THE TIMER REGISTER TIMSK3 RESETS AFTER IT REACHES THE MAX VALUE IN THE COMPARE REGISTER
  TCCR3B &= ~(1<<WGM13); //SENDING A ZERO TO THE REGISTER POSITION WGM13 TO SENSURE ITS ZERO
  TCCR3B |= (1<<WGM32); //THIS WILL RESET THE TIMER TO 0 ONCE THE VALUE IN THE COMPARE REGISTER AND THE TIMER MATCH

  //to set our prescalar value of 256 (1 0 0 => cs32 cs31 cs30) located in the TCCR3B regiter
  TCCR3B |= (1<<CS32); //the or bit wise opration sets the register to at position cs32 to 1
  TCCR3B &= (1<<CS31); //and bitwise operation with zero = zero so the default state of zero is maintained in this registers
  TCCR3B &= (1<<CS30); //and bitwise operation with zero = zero so the default state of zero is maintained in this registers
  //prescalar set
  
  //reset timer and set the pcompare value register eith the value such that interrupts occur every 1 second
  TCNT3 = t3_load;
  OCR3A = t3_comp;
  
  //to set timer3 compare interrupt
  TIMSK3 |= (1<<OCIE3A);
 //ENABLE GLOBAL INTERRUPTS
 sei();

 // end of timer 3 init


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
  // zero crossing detection
  attachInterrupt(digitalPinToInterrupt(zero_cross_detect),zero_cross, CHANGE);

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
   tank_temp=tank_rtd.temperature(RNOMINAL, RREF);
   boiler_temp=boiler_rtd.temperature(RNOMINAL, RREF);
  //WE WRITE THE PID CONTROL
 if ( full==1 )
 {
  //check if the tank level is okay and then start the heating elements
   int tank_pid = pid_tank_control(tank_temp, setpoint_tank);
      if (zero_cross_detected)     
    {
      delayMicroseconds(maximum_firing_delay - tank_pid); //This delay controls the power
      //control tank heater
      digitalWrite(tank_heater,HIGH);
    }
   int boiler_pid =pid_boiler_control(boiler_temp, setpoint_boiler);
   //now to update the trigger in the heater element pins
   if (zero_cross_detected)     
    {
      delayMicroseconds(maximum_firing_delay - boiler_pid);
      //boiler heater
      digitalWrite(boiler_heater_l1,HIGH);
      digitalWrite(boiler_heater_l2,HIGH);
      digitalWrite(boiler_heater_l2,HIGH);
      delayMicroseconds(50);//wait for 1ooms the write them all low
      digitalWrite(tank_heater,LOW);
      //boiler heater
      digitalWrite(boiler_heater_l1,LOW);
      digitalWrite(boiler_heater_l2,LOW);
      digitalWrite(boiler_heater_l2,LOW);
      zero_cross_detected = false;
    } 
 }
}
 
 ISR(TIMER3_COMPA_vect)
 {//THIS IS THE FUNCTION THAT GETS CALLED EVERYTIME THE TIMER3 INTERRUPT OCCURS ACCORDING TI THE DATASHEET ITS ADDRESS IS $0040 PRIORITY NUMNER 33

 if (commissioning_flag ==1 && ready_flag==0)
 { //perform toggling of these pins everytime the isr is triggered using exclusive or bitwise operation
   PORTF ^= (1<<start_blu);
   PORTF ^= (1<<start_red);
   PORTF ^= (1<<start_grn);
 }
  if(getting_ready_flag ==1 && pump_1_on==1)
  { 
    if(pump_counter < detergent_time)
    {
      digitalWrite(perilistic_pump_1, HIGH);
      pump_counter++; //this will be used to check timing in the priming stage without causing polling action
    }
    else{
      digitalWrite(perilistic_pump_1, LOW);
      pump_1_on=0;//once done reset the pump 1 on flag to zero
      pump_counter=0;
    }
  }
  
  if(in_process_flag ==1 && rinse_aid_pump==1)
  { 
    if(pump_counter < rinse_aid_time)
    {
      digitalWrite(perilistic_pump_2, HIGH);
      pump_counter++; //this will be used to check timing in the priming stage without causing polling action
    }
    else{
      digitalWrite(perilistic_pump_2, LOW);
      rinse_aid_pump=0; //resest the rinse aid pump on flag
      pump_counter=0;

    }
  }
  mysecond++;
  //AUTOMATIC RETURN from interrupt
 }

void zero_cross()
{

  if(last_CH1_state == 0){       //If the last state was 0, then we have a state change...
      zero_cross_detected = true;  //We have detected a state change! We need both falling and rising edges
    }
  else if(last_CH1_state == 1){    //If pin 8 is LOW and the last state was HIGH then we have a state change      
    zero_cross_detected = true;    //We haev detected a state change!  We need both falling and rising edges.
    last_CH1_state = 0;            //Store the current state into the last state for the next loop
    }

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

void commissioning()
{
  commissioning_flag=1;
  //implement timer software interrupt to turn on/off the led

  //start filling process 
  //THE TANK IS EMPTY
  water_filling_process(); //process 1
  getting_ready();

 
}

void water_filling_process(){
 filling_flag=1; //set to high to be used in the ISR routines
 //set screen to filling
    
    mydisplay.clearDisplay();
    mydisplay.setTextSize(2);
    mydisplay.setTextColor(WHITE);
    mydisplay.setCursor(20,10);
    mydisplay.println("FILLING");
    mydisplay.display();

 while(digitalRead(water_lvl_high))//initially the pin is high(1) => we wait for it to go low
 {
   //TURN ON THE SOLENOID UNTIL THE WATER LEVEL HIGH PROBE READS LOW(INDICATING THE TANK IS FULL)

   digitalWrite(solenoid, HIGH);//turn the solenoid on thus opening it

 }
 filling_flag=0;//reset the filling flag to low 
 digitalWrite(solenoid, LOW);
 full=1;
 return;//once done we return
//end of filling process
}

void getting_ready()
{
  //getting ready process involves heating / maintainig water temperature in the tank and otherwise
  //feeding detergent into the tank
  getting_ready_flag=1;
  //TO CALCULATE THE TIME THE PUMP WILL STAY ON:  SPECS 3L/HR == 0.83333mL/SECOND
  detergent_time = (detergent_dose/0.833333); //thi is the timw the pump will stay on in seconds //usig int will automatically truncate the decimal giving an accurracy og 1 in dosing
  pump_1_on=1;
  if(tank_temp>=35 && boiler_temp >=70){
  //we read the temperature status of the tan and boiler
  ready_flag=1;
   PORTF |= (1<<start_blu);//setting these pin to 1 using bitwise OR
   PORTF |= (1<<start_red);  //setting these pin to 1 using bitwise OR
   PORTF |= (1<<start_grn);  //setting these pin to 1 using bitwise OR
  getting_ready_flag=0;
  }
  
 
 return;
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

//PID tank variables
float previous_error = 0;
float elapsedTime, Time, timePrev;
//PID constants
int kp = 203;   int ki= 7.2;   int kd = 1.04;

//PID Boiler variables
float previous_error_b = 0;
float elapsedTime_b, Time_b, timePrev_b;


//PID FUNCTION CONTROL THE HEATERS
int pid_tank_control(int real_temperature, int setpoint)
{
  //local variable declerations
  float PID_error = 0;
  int PID_value = 0;
  int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
    PID_error = setpoint - real_temperature;
    
    if(PID_error > 30)                              //integral constant will only affect errors below 30ºC             
    {
      PID_i = 0;
    }
    
   PID_p = kp * PID_error;                         //Calculate the P value
   PID_i = PID_i + (ki * PID_error);               //Calculate the I value
    timePrev_b = Time_b;                    // the previous time is stored before the actual time read
    Time_b = millis();                    // actual time read
    elapsedTime = (Time_b - timePrev_b) / 1000;   
    PID_d = kd*((PID_error - previous_error_b)/elapsedTime);  //Calculate the D value
    PID_value = PID_p + PID_i + PID_d;                      //Calculate total PID value
    
    //We define firing delay range between 0 and 7400. Read above why 7400!!!!!!!
    if(PID_value < 0)
    { 
           PID_value = 0; 
    }
    if(PID_value > 7400)
    {    
        PID_value = 7400; 
    }
    //Printe the values on the LCD
    
    debugln(" ");
    debug("Setpoint: ");
    debugln(setpoint);
    
    debug("Real temp: ");
    debugln(real_temperature);

    previous_error_b = PID_error; //Remember to store the previous error.
  


 return PID_value;
}

int pid_boiler_control(int real_temperature, int setpoint)
{
    //local variable declerations
  float PID_error = 0;
  int PID_value = 0;
  int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
    PID_error = setpoint - real_temperature;
    
    if(PID_error > 30)                              //integral constant will only affect errors below 30ºC             
    {
      PID_i = 0;
    }
    
   PID_p = kp * PID_error;                         //Calculate the P value
   PID_i = PID_i + (ki * PID_error);               //Calculate the I value
    timePrev = Time;                    // the previous time is stored before the actual time read
    Time = millis();                    // actual time read
    elapsedTime = (Time - timePrev) / 1000;   
    PID_d = kd*((PID_error - previous_error)/elapsedTime);  //Calculate the D value
    PID_value = PID_p + PID_i + PID_d;                      //Calculate total PID value
    
    //We define firing delay range between 0 and 7400. Read above why 7400!!!!!!!
    if(PID_value < 0)
    { 
           PID_value = 0; 
    }
    if(PID_value > 7400)
    {    
        PID_value = 7400; 
    }
    //Printe the values on the LCD
    
    debugln(" ");
    debug("Setpoint: ");
    debugln(setpoint);
    
    debug("Real temp: ");
    debugln(real_temperature);

    previous_error = PID_error; //Remember to store the previous error.
  


 return PID_value;

}

void update_screen_2()
{//FUNCION FOR SCREEN 2 PROCESS SELECTION
 //TO SHOW THE MENU AND SELECT THE DESIRED PROCESS FROM THE MENU.
 //this happens if the ready flag is set and will be handled(called) in the flag_handler function
 menu_screen_2 = 0;
 switch (menu_screen_2)
 {
 case 0: { /* constant-expression */
     menu_screen_2=0;
    mydisplay.clearDisplay();
    mydisplay.setTextSize(1);
    mydisplay.setTextColor(WHITE);
    mydisplay.setCursor(2,0);
    mydisplay.println("=> NORMAL");
    mydisplay.setCursor(2,1);
    mydisplay.println("INTENSIVE");
    mydisplay.setCursor(2,2);
    mydisplay.println("COOKWARE");
    mydisplay.setCursor(2,3);
    mydisplay.println("HIGH TEMP CYCLE");
    mydisplay.display();
    
  
   break;
  }
   case 1: { /* constant-expression */
      menu_screen_2=1;
    mydisplay.clearDisplay();
    mydisplay.setTextSize(1);
    mydisplay.setTextColor(WHITE);
    mydisplay.setCursor(2,0);
    mydisplay.println("NORMAL");
    mydisplay.setCursor(2,1);
    mydisplay.println("=> INTENSIVE");
    mydisplay.setCursor(2,2);
    mydisplay.println("COOKWARE");
    mydisplay.setCursor(2,3);
    mydisplay.println("HIGH TEMP CYCLE");
    mydisplay.display();
   
   break;
  }
   case 2: { /* constant-expression */
       menu_screen_2=2;
    mydisplay.clearDisplay();
    mydisplay.setTextSize(1);
    mydisplay.setTextColor(WHITE);
    mydisplay.setCursor(2,0);
    mydisplay.println("NORMAL");
    mydisplay.setCursor(2,1);
    mydisplay.println("INTENSIVE");
    mydisplay.setCursor(2,2);
    mydisplay.println("=> COOKWARE");
    mydisplay.setCursor(2,3);
    mydisplay.println("HIGH TEMP CYCLE");
    mydisplay.display();
  
   break;
  }
   case 3: { /* constant-expression */
       menu_screen_2=3;
    mydisplay.clearDisplay();
    mydisplay.setTextSize(1);
    mydisplay.setTextColor(WHITE);
    mydisplay.setCursor(2,0);
    mydisplay.println("NORMAL");
    mydisplay.setCursor(2,1);
    mydisplay.println("INTENSIVE");
    mydisplay.setCursor(2,2);
    mydisplay.println("COOKWARE");
    mydisplay.setCursor(2,3);
    mydisplay.println("=> HIGH TEMP CYCLE");
    mydisplay.display();
  
   break;
  }
  
  case 4: { /* constant-expression */
       menu_screen_2=4;
    mydisplay.clearDisplay();
    mydisplay.setTextSize(1);
    mydisplay.setTextColor(WHITE);
    mydisplay.setCursor(2,0);
    mydisplay.println("INTENSIVE");
    mydisplay.setCursor(2,1);
    mydisplay.println("COOKWARE");
    mydisplay.setCursor(2,2);
    mydisplay.println("HIGH TEMP CYCLE");
    mydisplay.setCursor(2,3);
    mydisplay.println("=> Water Exchange");
    mydisplay.display();
  
   break;
  }
  
  case 5: { /* constant-expression */
       menu_screen_2=5;
    mydisplay.clearDisplay();
    mydisplay.setTextSize(1);
    mydisplay.setTextColor(WHITE);
    mydisplay.setCursor(2,0);
    mydisplay.println("COOKWARE");
    mydisplay.setCursor(2,1);
    mydisplay.println("HIGH TEMP CYCLE");
    mydisplay.setCursor(2,2);
    mydisplay.println("Water Exchange");
    mydisplay.setCursor(2,3);
    mydisplay.println("=> Self Cleaning Cycle");
    mydisplay.display();
  
   break;
  }
 default:
  menu_screen_2=0;
   break;
 }
 return;
}
//all processes will be written in this section. then the flag handler will call them appropriately
void basic_process(int timing)//timing is in sconds
{  
   unsigned long current_time=0;
   unsigned long wash_time =(timing*1000); // to get timin in miliseconds=> *1000
   unsigned int wash_time_sec=timing;
  if ((ready_flag==1 && start_flag==2) || in_process_flag==1)
  {
   if(rinsing==0){
      in_process_flag=1;
      current_time=millis();
      mysecond=0;
      while(millis()-current_time<=wash_time)
      {//this will keep the wash pump on for 90 seconds
        digitalWrite(wash_pump, HIGH);
        unsigned int time_display = wash_time_sec-mysecond;
        mydisplay.clearDisplay();
        mydisplay.setTextSize(1);
        mydisplay.setTextColor(WHITE);
        mydisplay.setCursor(2,0);
        mydisplay.println("NORMAL");
        mydisplay.setTextSize(1);
        mydisplay.setCursor(2,1);
        mydisplay.println(time_display);
        mydisplay.display();
      }// once the washing is done
      //turn off wash pump
      digitalWrite(wash_pump,LOW);
      rinsing=1; //indicate start of rinsing process
    }
   else{
    //refill reheat and rinse
    //TURN SOLENOID VALVE ON
    
    ready_flag=0;
    current_time=millis();
    mysecond=0;
    if(done_refilling==0){
    while(millis()-current_time<=30000)
     {//wdo this for 30 seconds for the refilling process
        digitalWrite(solenoid, HIGH);
        mydisplay.clearDisplay();
        mydisplay.setTextSize(1);
        mydisplay.setTextColor(WHITE);
        mydisplay.setCursor(2,0);
        mydisplay.println("REFILLING");
        mydisplay.setTextSize(1);
        mydisplay.setCursor(2,1);
        mydisplay.println(30-mysecond);
        mydisplay.display();
       
      }
      digitalWrite(solenoid, LOW);
      //add the rinse aid
      rinse_aid_time= rinse_aid_dose/peristalitic_feedrate;
      rinse_aid_pump=1;//set the rinse pump flag on to be used in the ISR timer3
      done_refilling=1;
      
    }
    //AFTER EACH PROCESS CALL COMMISSIONING SO AS TO GET THE WATER LEVEL AND TEMPERATURES WITHI RANGES AGAIN

    
    
    //read temperature to ensure all is ready
    if((boiler_temp>=setpoint_boiler) && (tank_temp>=setpoint_tank) && (rinse_aid_pump==0))
    {
    ready_flag=1; //set ready flag to 1
    }
    //we check the ready flag to start the rinsing process
    if(ready_flag==1 && in_process_flag==1 && done_refilling==1)
    {
        current_time=millis();
      mysecond=0;
      if(done_refilling==0)
      {
        while(millis()-current_time<=30000)
        {
          digitalWrite(wash_pump,HIGH);
            mydisplay.clearDisplay();
            mydisplay.setTextSize(1);
            mydisplay.setTextColor(WHITE);
            mydisplay.setCursor(2,0);
            mydisplay.println("RINSING");
            mydisplay.setTextSize(2);
            mydisplay.setCursor(2,1);
            mydisplay.println(30-mysecond);
            mydisplay.display();
        }
            //NORMAL_DONE=1;
          in_process_flag=0;
          start_flag=1;//SIGNAL END OF PROCESS AND RETURN TO THE MAIN MENU SCREEN
      }
   }
  }
return;
}
}

void glassware()
{
   
   unsigned long current_time=0;
    long wash_time =150; // to get timin in miliseconds=> *1000
    wash_time *=1000;
   unsigned int wash_time_sec=150;
   float temp_boiler=0;
   int elapsed_seconds=0;
   int reheat_flag=0;
  //first we check temperature
   temp_boiler =boiler_rtd.temperature(RNOMINAL, RREF);
  if(temp_boiler>62 && sensitive_wash_ready==0 )
  {
    mysecond=0; //reset myseconds counter so as to save the length of the waiting loop
    do
    { //polling loop
      //we are doing nothing here...just waiting for the temperature to drop
      //this loop will go on waiting for the temperature to drops accordingly
      elapsed_seconds=mysecond;
    } while ((temp_boiler=boiler_rtd.temperature(RNOMINAL, RREF))>62 );

    sensitive_wash_ready=1; //once done set the ready flag
  }
  else if((temp_boiler<58 && sensitive_wash_ready==0) ||reheat_flag==1)
  {
     mysecond=0; //reset myseconds counter so as to save the length of the waiting loop
    do
    { 

      boiler_temp=0;
      int boiler_pid =pid_boiler_control(boiler_temp, setpoint_boiler);
      elapsed_seconds=mysecond;
   //now to update the trigger in the heater element pins
    if (zero_cross_detected)     
    {
      delayMicroseconds(maximum_firing_delay - boiler_pid); //This delay controls the power
      //control tank heater
      //boiler heater high
      digitalWrite(boiler_heater_l1,HIGH);
      digitalWrite(boiler_heater_l2,HIGH);
      digitalWrite(boiler_heater_l2,HIGH);
      delayMicroseconds(100);//wait for 1ooms the write them all low
       //boiler heater low
      digitalWrite(boiler_heater_l1,LOW);
      digitalWrite(boiler_heater_l2,LOW);
      digitalWrite(boiler_heater_l2,LOW);
      zero_cross_detected = false;
    } 
    } while (tank_temp<58);
    sensitive_wash_ready=1;
    reheat_flag=0;
  }
  else
 {
   sensitive_wash_ready=1;
 }
 (20-elapsed_seconds)<=0 ? delay(0):delay(20-elapsed_seconds); //is else to set the delay of the glass washing start of 20 seconds
 //proceeding to washing operations
 if (sensitive_wash_ready==1 )
 {
   //we begin the washing process
   if ((ready_flag==1 && start_flag==2) || in_process_flag==1)
  {
   if(rinsing==0)
   {
      in_process_flag=1;
      current_time=millis();
      mysecond=0;
      while(millis()-current_time<=wash_time)
      {//this will keep the wash pump on for 90 seconds
        digitalWrite(wash_pump, HIGH);
        unsigned int time_display = wash_time_sec-mysecond;
        mydisplay.clearDisplay();
        mydisplay.setTextSize(1);
        mydisplay.setTextColor(WHITE);
        mydisplay.setCursor(2,0);
        mydisplay.println("WASHING");
        mydisplay.setTextSize(1);
        mydisplay.setCursor(2,1);
        mydisplay.println(time_display);
        mydisplay.display();
      }// once the washing is done
      //turn off wash pump
      digitalWrite(wash_pump,LOW);
      rinsing=1; //indicate start of rinsing process
    }
  else
  {
   //RINSING PROCESS
       sensitive_wash_ready=0;
    current_time=millis();
    mysecond=0;
    if(done_refilling==0)
    {
    while(millis()-current_time<=30000)
     {//wdo this for 30 seconds for the refilling process
        digitalWrite(solenoid, HIGH);
        mydisplay.clearDisplay();
        mydisplay.setTextSize(1);
        mydisplay.setTextColor(WHITE);
        mydisplay.setCursor(2,0);
        mydisplay.println("REFILLING");
        mydisplay.setTextSize(1);
        mydisplay.setCursor(2,1);
        mydisplay.println(30-mysecond);
        mydisplay.display();
       
      }
      digitalWrite(solenoid, LOW);
      //add the rinse aid
      rinse_aid_time= rinse_aid_dose/peristalitic_feedrate;
      rinse_aid_pump=1;//set the rinse pump flag on to be used in the ISR timer3
      done_refilling=1;
      
    }
    
    
    
    //read temperature to ensure all is ready
    if((temp_boiler>=58) && (temp_boiler<=62) && (rinse_aid_pump==0))
    {
    ready_flag=1; //set ready flag to 1
    sensitive_wash_ready=1;
    }
    else if((temp_boiler<58) && (rinse_aid_pump==0))
    {//if not within the range and and tergent pump is on then we use gp to to go to the function for increasing temperature
     reheat_flag=1;
    }
    //we check the ready flag to start the rinsing process
    if(sensitive_wash_ready==1 && in_process_flag==1 && done_refilling==1 && reheat_flag==1)
    {
      current_time=millis();
     mysecond=0;
     if(done_refilling==0)
     {
        while(millis()-current_time<=30000)
        {
          digitalWrite(wash_pump,HIGH);
          mydisplay.clearDisplay();
          mydisplay.setTextSize(1);
          mydisplay.setTextColor(WHITE);
          mydisplay.setCursor(2,0);
          mydisplay.println("RINSING");
          mydisplay.setTextSize(2);
          mydisplay.setCursor(2,1);
          mydisplay.println(30-mysecond);
          mydisplay.display();
        }
        //NORMAL_DONE=1;
       in_process_flag=0;
       start_flag=1;//SIGNAL END OF PROCESS AND RETURN TO THE MAIN MENU SCREEN
    }
  }
  }
 }
  }
}
