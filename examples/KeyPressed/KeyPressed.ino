/*
  Copyright (c) 2021 Jean-Marc Chiappa.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

// sirc Protocol decoder Arduino code

#include "nec.h"

nec receiver(D2,TIM6);

void setup() {
  Serial.begin(115200);
  receiver.begin();
}

void loop() {
  if( receiver.codeReceived() ){
    if(receiver.newKeyPressed()) {
      Serial.print("Address : ");
      Serial.print(receiver.Address());
      Serial.print("\tCommand : ");
      Serial.println(receiver.Command());
      delay(100);
    }else{
      receiver.skipThisCode();
    }
  }
}
