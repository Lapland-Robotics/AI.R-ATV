#ifndef PWMREAD_HPP
#define PWMREAD_HPP

#include <Arduino.h>
#include <ATV.h>

void IRAM_ATTR RC_PCM_isr();
void setup_pwmRead();
boolean RC_avail();
int RC_decode(int i);
boolean FAILSAFE(int i);
void print_RCpwm();

#endif // PWMREAD_HPP
