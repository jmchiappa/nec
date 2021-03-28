/*
  nec.h - nec library v1.0.0 - 2021-03-27
  Copyright (c) 2021 Jean-Marc Chiappa.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

// nec Protocol decoder Arduino code
#ifndef nec_h
#define nec_h

#include "Arduino.h"
 
// Define number of Timer1 ticks (with a prescaler of 1/8)
#define lead_pulse_min     8800
#define lead_pulse_max     9200
#define lead_space_min     4400
#define lead_space_max     4600
#define high_max           (560-2250)+100
#define low_max            (1120-560)+100
#define low_min            (1120-560)-100
#define repeated_min       2200

static void nec_read();
static void overflow();

class nec
{
public:
  nec(uint8_t pin_receiver, TIM_TypeDef *instance);
  void begin();
  boolean codeReceived();
  uint16_t rawCode();
  boolean newKeyPressed();
  uint8_t Command();
  uint8_t Address();
  void skipThisCode();
private:
  void update(void);
  TIM_TypeDef *htimer=NULL;
  uint8_t pin;
  uint8_t address;
  uint8_t command;
};

#endif // nec_h