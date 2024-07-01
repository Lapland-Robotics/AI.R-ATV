// Copy of "Kelvin Nelson 24/07/2019" codes and little modified and simplified 06.2021
// Pulse Width Modulation (PWM) decoding of RC Receiver with failsafe

#include "ATV.h"
 
const int pwmPIN[]={34, 35}; // an array to identify the PWM input pins (the array can be any length), first pin is channel 1, second is channel 2...etc

//int RC_inputs = 2;       // The number of pins in pwmPIN that are connected to an RC receiver                                            

// fail safe positions

int RC_failsafe[] = {1500, 1500};

// The failsafe tolerances are: 10-330Hz & 500-2500us

//GLOBAL PWM DECODE VARIABLES
const int num_ch = RC_input_Count;              // number of input pins (or channels)
volatile int PW[RC_input_Count];                        // an array to store pulsewidth measurements
volatile boolean prev_pinState[RC_input_Count];         // an array used to determine whether a pin has gone low-high or high-low
volatile unsigned long pciTime;                 // the time of the current pin change interrupt
volatile unsigned long pwmTimer[RC_input_Count];        // an array to store the start time of each PWM pulse

volatile boolean pwmFlag[RC_input_Count];               // flag whenever new data is available on each pin
volatile boolean RC_data_rdy;                   // flag when all RC receiver channels have received a new pulse
volatile unsigned long pwmPeriod[RC_input_Count];       // period, mirco sec, between two pulses on each pin

byte pwmPIN_State;  // Current State                 


void IRAM_ATTR RC_PCM_isr(){                                  // this function will run if a pin change is detected
  pciTime = micros();                                         // Record the time of the PIN change in microseconds
  for (int i = 0; i < RC_input_Count; i++){                   // run through each of the channels
      pwmPIN_State = digitalRead(pwmPIN[i]);                  // Current state is
    if(prev_pinState[i] == 0 && pwmPIN_State){                // and the pin state has changed from LOW to HIGH (start of pulse)
        prev_pinState[i] = 1;                                 // record pin state
        pwmPeriod[i] = pciTime - pwmTimer[i];                 // calculate the time period, micro sec, between the current and previous pulse
        pwmTimer[i] = pciTime;                                // record the start time of the current pulse
    } else if (prev_pinState[i] == 1 && !pwmPIN_State){       // or the pin state has changed from HIGH to LOW (end of pulse)
        prev_pinState[i] = 0;                                 // record pin state
        PW[i] = pciTime - pwmTimer[i];                        // calculate the duration of the current pulse
        pwmFlag[i] = HIGH;                                    // flag that new data is available
        if(i+1 == RC_input_Count){
          RC_data_rdy = HIGH;                  
        }
    }
  }
}

// SETUP OF PIN CHANGE INTERRUPTS
void setup_pwmRead(){

  for (int i = 0; i < RC_input_Count; i++){
    pinMode(pwmPIN[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(pwmPIN[i]), &RC_PCM_isr, CHANGE);
  }
} 

// RC OUTPUT FUNCTIONS
boolean RC_avail(){
  boolean avail = RC_data_rdy;
  RC_data_rdy = LOW;               // reset the flag
  return avail;
}

int RC_decode(int i){ 
  int CH_output;
  if(FAILSAFE(i) == HIGH){           // If the RC channel is outside of failsafe tolerances (10-330hz and 500-2500uS)
    CH_output = RC_failsafe[i];      // set the failsafe position 
  } else {                           // If the RC signal is valid
    CH_output = PW[i];               // 
  }
  return CH_output;                                 
}

// Basic Receiver FAIL SAFE
// check for 500-2500us and 10-330Hz (same limits as pololu)

boolean FAILSAFE(int i){
   boolean failsafe_flag = LOW;
   if(pwmFlag[i] == 1){                           // if a new pulse has been measured
       pwmFlag[i] = 0;                            // set flag to zero
       if(pwmPeriod[i] > 100000){                 // if time between pulses indicates a pulse rate of less than 10Hz   
         failsafe_flag = HIGH;                       
       } else if(pwmPeriod[i] < 3000){            // or if time between pulses indicates a pulse rate greater than 330Hz   
         failsafe_flag = HIGH;                             
       }
     if(PW[i] < 500 || PW[i] > 2500){             // if pulswidth is outside of the range 500-2500ms
       failsafe_flag = HIGH;                        
     }   
   } else if (micros() - pwmTimer[i] > 100000){   // if there is no new pulswidth measurement within 100ms (10hz)
     failsafe_flag = HIGH;                      
   }
return failsafe_flag;   
}

//Quick print function of Rx channel input
void print_RCpwm(){                             // display the raw RC Channel PWM Inputs
  for (int i = 0; i < RC_input_Count; i++){
    //Serial.print(" ch");Serial.print(i+1);
    Serial.print("  ");
    if(PW[i] < 1000) Serial.print(" ");
    Serial.print(PW[i]);
  }
  Serial.println("");
}
