/////////////////////////////////////////////////////////////////////////////////////////
//
// Saturn Power Manager sketch by Laurence Barker G8NJJ
// this sketch controls a DC power switch for the G2 radio
// functions are as follows:
// initial press turns on power; microcontroller asserts an outpu to hold power on
// on button release, normal execution continues
// after a long putton press, the microcontroller signals the AREM processor to shut down
// after it has powered off, the DC power switch is removed
//
// the code is written for an Atmel ATTINY412 using megatinycore for Arduino
//
// "main" file with setup() and loop()
// 10ms timer tick driven for real time operation
/////////////////////////////////////////////////////////////////////////////////////////
//

#if !( defined(MEGATINYCORE) )
  #error This is designed only for MEGATINYCORE megaAVR board! Please check your Tools->Board setting
#endif

// These define's must be placed at the beginning before #include "megaAVR_TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

// Select USING_FULL_CLOCK      == true for  20/16MHz to Timer TCBx => shorter timer, but better accuracy
// Select USING_HALF_CLOCK      == true for  10/ 8MHz to Timer TCBx => shorter timer, but better accuracy
// Select USING_250KHZ          == true for 250KHz to Timer TCBx => longer timer,  but worse  accuracy
// Not select for default 250KHz to Timer TCBx => longer timer,  but worse accuracy
#define USING_FULL_CLOCK      true
#define USING_HALF_CLOCK      false
#define USING_250KHZ          false         // Not supported now

// Try to use RTC, TCA0 or TCD0 for millis()
#define USE_TIMER_0           true          // Check if used by millis(), Servo or tone()
#define USE_TIMER_1           false         // Check if used by millis(), Servo or tone()

#if USE_TIMER_0
  #define CurrentTimer   ITimer0
#elif USE_TIMER_1
  #define CurrentTimer   ITimer1
#else
  #error You must select one Timer  
#endif


// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "ATtiny_TimerInterrupt.h"

#define TIMER1_INTERVAL_MS    10
int GTickCounter;                           // tick counter for 16ms tick
bool GTickTriggered;                        // true if a 16ms tick has been triggered
bool LEDOn;
int Counter = 0;
byte GBlinkRate = 50;                       // on/off periods in 10ms units


//
// define pins. 5 are used
//
#define VPBPIN 1                            // pushbutton input
#define VLEDPIN 3                           // front panel LED output
#define VARMACTIVEPIN 0                     // input from ARM signalling it is active
#define VARMSHUTDOWNPIN 2                   // open collector output to ARM requesting shutdown
#define VPWRONPIN 4                         // active high output to hold on the +12v switch
#define VARMCONFIGPIN 5                     // UPDI used as input


//
// power manager states
//
enum EPowerState
{ 
  ePowerOn,
  eNormalOperation, 
  eShutdownRequest,
  eShutdownInitiated,
  eShutdownWait,
  eShutdown
}; 
EPowerState GPowerState;


//
// pushbuttton events. Set by button handler; cleared by consumer
//
enum EButtonEvent
{ 
  eNone,                                      // no event
  ePressed,                                   // initial press event
  eLongPressed,                               // long press event
  eReleased                                   // button released event
}; 


#define VLONGPRESSCOUNT 200                   // 2 seconds
byte PBShift;                                 // shifted state of pushbutton input pin
byte ARMInShift;                              // shifted state of ARM input pin
byte PBLongCounter;                           // counter for long press
EButtonEvent GButtonEvent;                    // decoded pushbutton event
EButtonEvent GARMInEvent;                     // decoded ARM input event
unsigned int GPowerOffTimer;
unsigned int GARMOffTimer;
bool GARMOffReady = false;

#define VPOWEROFFDELAY 1000                   // 10 seconds before shutdown
#define VARMPOWEROFFDELAY 3000                // 30 seconds before input armed


//
// read and debounce pushbutton input
// called from 10ms tick handler
//
void ReadPushbutton(void)
{
  byte Input = 0;
  if(digitalRead(VPBPIN) == HIGH)
    Input = 1;

  PBShift = ((PBShift << 1) | (Input & 1)) & 0b00000111;           // most recent 3 samples
  if(PBShift == 0b00000100)                         // button press detected
  {
    PBLongCounter = VLONGPRESSCOUNT;                // set long press count
    GButtonEvent = ePressed;
  }
  else if (PBShift == 0b00000011)                   // button release detected
  {
    GButtonEvent = eReleased;
    PBLongCounter = 0;                              // clear long press count
  }
  else if(PBLongCounter != 0)                       // if button pressed, and long press not yet declared
  {
    if(--PBLongCounter == 0)
      GButtonEvent = eLongPressed;
  }
}



//
// read and debounce "ARM request power off" input
// called from 10ms tick handler
//
void ReadARMInput(void)
{
  byte Input = 0;
  if(digitalRead(VARMCONFIGPIN) == HIGH)
    Input = 1;

  ARMInShift = ((ARMInShift << 1) | (Input & 1)) & 0b00000111;           // most recent 3 samples
  if(ARMInShift == 0b00000100)                         // active low (trigger) detected
    GARMInEvent = ePressed;
}


//
// PowerManagerTick()
// execute sequencer code for state diagram
//
void PowerManagerTick(void)
{

  switch(GPowerState)
  {
    case ePowerOn:                                    // power up into this state
      if(digitalRead(VPBPIN) == LOW)                  // oly latch power if PB is pressed
      {
        digitalWrite(VPWRONPIN, HIGH);                // take control of 12V power, turning it on
        digitalWrite(VLEDPIN, HIGH);                  // front panel LED lit
      }
      if(GButtonEvent == eReleased)                   // state change when button released
        GPowerState = eNormalOperation;
      break;

    case eNormalOperation:                            // 12V power on; wait for shutdown press
      digitalWrite(VPWRONPIN, HIGH);                  // take control of 12V power, turning it on
      if(digitalRead(VARMCONFIGPIN) == LOW)
      {
        if (LEDOn)                                    // LED blinks if UPDI is zero
          digitalWrite(VLEDPIN, HIGH);
        else
          digitalWrite(VLEDPIN, LOW);
      }
      else
        digitalWrite(VLEDPIN, HIGH);                // front panel LED lit if UPDI is high
      if(GARMInEvent == ePressed)                   // if ARM has requested shutdown, do it
      {
        GPowerState = eShutdownInitiated;
        GBlinkRate = 25;                            // double LED gflash rate for ARM triggered shutdown
      }
      else if(GButtonEvent == eLongPressed)         // state change when button pressed for 2s
        GPowerState = eShutdownRequest;
      break;

    case eShutdownRequest:                          // shutdown requested; blink LED & wait for button released
      digitalWrite(VPWRONPIN, HIGH);                // take control of 12V power, turning it on
      if (LEDOn)                                    // LED blinks
        digitalWrite(VLEDPIN, HIGH);
      else
        digitalWrite(VLEDPIN, LOW);
      if(GButtonEvent == eReleased)                 // state change when button pressed for 2s
        GPowerState = eShutdownInitiated;
      break;

    case eShutdownInitiated:                        // blink LED; notify ARM
      digitalWrite(VPWRONPIN, HIGH);                // take control of 12V power, turning it on
      if (LEDOn)                                    // LED blinks
        digitalWrite(VLEDPIN, HIGH);
      else
        digitalWrite(VLEDPIN, LOW);
//
// assert ARM_shutdown as open drain
//
      digitalWrite(VARMSHUTDOWNPIN, LOW);           // pin logic level = 0
      pinMode(VARMSHUTDOWNPIN, OUTPUT);             // make an output, so now driven to logic 0
      if(digitalRead(VARMACTIVEPIN) == LOW)         // state change when ARM power off
      {
        GPowerState = eShutdownWait;                // go to final wait state with 10s count
        GPowerOffTimer = VPOWEROFFDELAY;
      }
      break;

    case eShutdownWait:                             // wait 10s before power off. LED still blinks
      if (LEDOn)                                    // LED blinks
        digitalWrite(VLEDPIN, HIGH);
      else
        digitalWrite(VLEDPIN, LOW);
      if(--GPowerOffTimer == 0)
        GPowerState = eShutdown;
      break;

    case eShutdown:                                 // remove 12V power and wait for power off
      digitalWrite(VLEDPIN, LOW);                   // LED off 
      digitalWrite(VPWRONPIN, LOW);                 // remove 12V power
      break;
  }
  GButtonEvent = eNone;
  GARMInEvent = eNone;
}




//
// Tick interrupt handler
// called every 10ms; invoke main tick code in loop()
//
void TickHandler(void)
{
  GTickTriggered = true;
}



//
// initialize digital pins, and tick timer
//
void setup()
{
  
  pinMode(VLEDPIN, OUTPUT);
  pinMode(VPWRONPIN, OUTPUT);
  pinMode(VARMSHUTDOWNPIN, INPUT);            // will become an output to make open collector driver
  pinMode(VARMACTIVEPIN, INPUT);
  pinMode(VPBPIN, INPUT_PULLUP);
  pinMode(VARMCONFIGPIN, INPUT_PULLUP);
  
  CurrentTimer.init();
  CurrentTimer.attachInterruptInterval(TIMER1_INTERVAL_MS, TickHandler);
  GARMOffTimer = VARMPOWEROFFDELAY;           // delay before pin used
}



//
// the loop function runs over and over again forever
// wait for tick interrupt then run loop once
//
void loop() 
{
  while (GTickTriggered)
  {
    GTickTriggered = false;

// set blinking LED state
    if (Counter == 0)
    {
      Counter= GBlinkRate;                   // 50 * 10ms
      LEDOn = !LEDOn;
    }
    else
      Counter--;
//
// normal loop code
//      
    if(GARMOffTimer != 0)                   // 30s delay before "ARM request power off" input enabled
      if(--GARMOffTimer == 0)
        GARMOffReady = true;
    ReadPushbutton();
    ReadARMInput();
    PowerManagerTick();
  }
}
