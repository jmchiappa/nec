/*
  nec.cpp - nec library v1.0.0 - 2021-03-27
  Copyright (c) 2021 Jean-Marc Chiappa.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

// nec Protocol decoder Arduino code

#include "nec.h"

enum etat {
  START,
  LEAD_PULSE_START,
  LEAD_SPACE_START,
  GET_PULSE
};

/**
  Debug helper
*/

// #define DEBUG

#ifdef DEBUG

  typedef struct elm {
    uint16_t time;
    uint8_t nec_state;
    uint8_t nb_j;
    uint8_t pinstate;
    char comment[10];
  };  

  elm buffer[1024];
  uint32_t ptrpusher=0;
  uint32_t ptrpuller=0;

  #define Debug_print(a,b)      {Serial.print(a);Serial.print(" : ");Serial.print(b);}
  #define Debug_println(a,b)    {Serial.print(a);Serial.print(" : ");Serial.println(b);}

  inline void pushtrace(uint16_t time,uint8_t state,uint8_t j,uint8_t pin, char *comment){
      buffer[ptrpusher] = {           
        .time = time,                   
        .nec_state = state,             
        .nb_j = j,                      
        .pinstate = pin
      };
      strcpy(buffer[ptrpusher].comment,comment);
      ptrpusher++;
      ptrpusher = ptrpusher % 1024;     
  }

  inline void poptrace(void) {
    if(ptrpusher!=ptrpuller) {
      Debug_print("time", buffer[ptrpuller].time);
      Debug_print("\t", buffer[ptrpuller].comment);
      Debug_print("\tnec state", buffer[ptrpuller].nec_state);
      Debug_print("\tj", buffer[ptrpuller].nb_j);
      Debug_println("\tpin state", buffer[ptrpuller].pinstate);
      ptrpuller++;
      ptrpuller = ptrpuller % 1024;
    }
  }

#else

  #define Debug_print(a,b)      {}
  #define Debug_println(a,b)    {}
  inline void pushtrace(uint16_t time,uint8_t state,uint8_t j,uint8_t pin, char *comment){}
  inline void poptrace()        {}

#endif

/**
 Local variables shared with ISR
*/
  
uint8_t j;
HardwareTimer *tim;
boolean nec_ok;
uint32_t timer_value;
uint8_t nec_state=0;
uint32_t nec_code;
uint8_t pinReceiver;
uint8_t repeated;
nec::nec(uint8_t pin_receiver, TIM_TypeDef *instance){
  pinReceiver = pin_receiver;
  htimer = instance;
};


void nec::begin() {
  if(htimer==NULL) {
    htimer = TIM6;
  }
  tim = new HardwareTimer(htimer);
  tim->setPrescaleFactor(10);
  tim->setOverflow(10000,MICROSEC_FORMAT); 
  tim->pause();
  tim->setCount(0);
  tim->attachInterrupt(overflow);
  attachInterrupt(pinReceiver, nec_read, CHANGE);          // Enable external interrupt (INT0)

}

void nec_read() {
  if(!nec_ok){
    // if(nec_state != BEGIN_START){
      timer_value = tim->getCount(MICROSEC_FORMAT);                         // Store Timer1 value
      tim->setCount(0);
      // pushtrace(timer_value,nec_state,j,0xFF,"ITEntry");
    // }
  // #if 0
    switch(nec_state){
     case START :
        pushtrace(0,nec_state,j,0xFF,"start");
        tim->resume();
        nec_state = LEAD_PULSE_START;
        break;
     case LEAD_PULSE_START :
      pushtrace(0,nec_state,j,0xFF,"in_lp");
      if( timer_value<repeated_min || timer_value > lead_pulse_max ){         // Invalid interval ==> stop decoding and reset
        nec_state = START;                             // Reset decoding process
        tim->pause();
        pushtrace(timer_value,nec_state,j,0xFF,"Errlp");
        break;
      }
      repeated=0;
      pushtrace(timer_value,nec_state,j,0xFF,"to_ls");
      nec_state = LEAD_SPACE_START;                               // Next state: end of mid1
      break;
     case LEAD_SPACE_START :                                      // End of mid1 ==> check if we're at the beginning of start1 or mid0
      pushtrace(0,nec_state,j,0xFF,"in_ls");
      if( timer_value<repeated_min || timer_value > lead_space_max ){         // Invalid interval ==> stop decoding and reset
        nec_state = START;                             // Reset decoding process
        tim->pause();
        pushtrace(timer_value,nec_state,j,0xFF,"Errls");
        break;
      }
      if(timer_value < lead_space_min) {
        // this is a repeated key, assuem the previous code was ok
        pushtrace(timer_value,nec_state,j,0xFF,"repeated");
        nec_state = START;
        nec_ok=1;
        repeated=1;
        break;
      }
      pushtrace(timer_value,nec_state,j,1,"togetpulse");
      nec_state = GET_PULSE;
      // reset sample here
      j = 0;      
      nec_code=0;
     break;
     case GET_PULSE :
      pushtrace(timer_value,nec_state,j,0xFE,"in_get");
      if(!digitalRead(pinReceiver)) {
        if((timer_value > high_max) || (timer_value < low_min)){
          nec_state = START;                             // Reset decoding process
          tim->pause();
          pushtrace(timer_value,nec_state,j,0xFF,"ErrTiming");
          break;
        }
        uint8_t state = (timer_value<low_max ? 0 : 1);
        nec_code |= state<<j;
        pushtrace(timer_value,nec_code,j,state,"savepulse");
        j++;
        if(j==32) {
          nec_ok=1;
        }
      }
     break;
    }
  }
}

static void overflow() {
  pushtrace(tim->getCount(MICROSEC_FORMAT),0,0,0xFF,"overflow");  
  nec_state = START;
  tim->pause();
} 

boolean nec::codeReceived() {
  return nec_ok!=0;
}

uint16_t nec::rawCode() {
  update();
  nec_ok = 0;
  return nec_code;
}

boolean nec::newKeyPressed() {
  return repeated==0;
}

uint8_t nec::Command() {
  update();
  nec_ok = 0;
  return command;
}

uint8_t nec::Address() {
  update();
  return address;
};

void nec::skipThisCode() {
  nec_ok = 0;
};

void nec::update() {
  uint8_t a,ai,c,ci;
  poptrace();
  if(nec_ok){                                    // If the mcu receives nec message with successful
    nec_state = START;
    tim->pause();
    a  =   nec_code      & 0xFF;
    ai = ~(nec_code>>8)  & 0xFF;
    c  =  (nec_code>>16) & 0xFF;
    ci = ~(nec_code>>24) & 0xFF;
    address = (a==ai) ? a : 0xFF;
    command = (c==ci) ? c : 0xFF; 
  }
}
