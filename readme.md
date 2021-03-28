# IR NEC protocol decoder

This library works ONLY with stm32 MCU.
It uses a hardware timer to catch spaces and pulses duration. This library is in non-blocking mode. It works under the hood, so it just checks if a new code is available.
Once a new valid code is catched, it waits until it's read or skipped.

## Requirements:
- stm32duino > 1.6.1 https://github.com/stm32duino
- (optional) Arduino IDE
- NEC compatible remote
- IR receiver like https://arduinomodules.info/ky-022-infrared-receiver-module/
- (optional) STM32 Nucleo board 

## Tested:
- Nucleo L476RG https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-mpu-eval-tools/stm32-mcu-mpu-eval-tools/stm32-nucleo-boards/nucleo-l476rg.html
- stm32duino 1.9.0

Event if it has been tested only on the L476, there's no reason to fail with another MCU. Just keep in mind that you must use an existing timer resource (ex: TIM3, TIM4). Refer to the example to point the used instance for L476.

## Limitations

IT works ONLY for NEC protocol. Indeed, the timings for other protocol are totally differents.
Sometimes, the output is inverted regarding the specification. So you would adjust by inverting the input test `if(digitalRead(pinReceiver)...` in nec.cpp file.

## API

class name : nec

### constructor : sirc instance( arduino pin number, Timer instance )

Take care to use an existing timer instance. refer to the mcu datasheet to find an existing timer instance
pin number is expressed as arduino pin i.e. Dxx. Any Nucleo pin can be assigned.

### instance.begin

Initialize all required hardware resources

### instance.codeReceived

Check if a validated code has been received. Once one has been decoded, this code is protected against a new one until you read it or you skip it.
TRUE : a new code is available

### instanece.newKeyPressed

Check if the new code is the repeated previous one.
TRUE: the key is still pressed

### instance.rawCode

Return the 14 bits code including the start bit. It will let you decodeing all sirc fields by yourself

### instance.Address

Return the address field. It doesn't rearm the state machine. So no new code can be catched. The address field depends on the length of the sequence. Known lengths are 12, 15 or 20 bits.
12 bits : return an adress on 6 bits length
15 bits : return an adress on 8 bits length
20 bits : return an adress on 13 bits length

### instance.Command

Return the command field. It restarts the sampling state machine

### instance.skipThisCode

Skip the current code and rearm the sequencer to catch the next one

## Debug

Uncomment in `nec.cpp` the `#define DEBUG` line. As the sequencer works in real time, it is better to use a trace buffer. 2 helper functions can be used :
- `pushtrace` : store some variables into the stack trace
- `poptrace` : remove from stack and display on serial interface.

Take care to initialize in your setup the serial interface  before using traces

## License

Copyright (c) 2021 Jean-Marc Chiappa.  All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

See file LICENSE.txt for further informations on licensing terms.
