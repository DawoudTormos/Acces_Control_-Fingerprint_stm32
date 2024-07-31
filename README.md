# Fingerprint Access Control System with STM32

## Goal
An access control system to allow access through a door that automatically opens after the right fingerprint. A handle is released after a capacitive sensor detects the user is near the door  Security type is a fingerprint. The system can be configured through Bluetooth and an app could be made for this purpose. 

## Uses
 Access control to a room, vault or closet
 
## What is in the repo.
You get the code for programming the STM32f103c8t6 (Blue Pill). You need experience with STM microcontrollers.

## What do you need more
- Extra Hardware
- Basic Electronics understanding
- A diagram for connections. There is no one at the current time but it can be pulled out from the code.


## Hardware
 - Electronics modules: motor driver for 2 motors - HC05 - LM7805 - Capacitors - LEDs - AM160 Fingerprint reader
 - Door lock that could open with power
 - Actuator or whatever for opening/closing the door through the motor driver.
 - Note: the motor driver should be able to give the current needed for the lock and actuator. Also be capable of the needed voltage and be controlled with 3.3v from STM32f103c8t6 


## Scalability
 - This code contains a library for using a AM160 Fingerprint reader. It's a cheap fingerprint module with good enough performance and reliability.
 - You could make it communicate to a REST API through: a serial interface u love -> esp32 -> wifi -> Rest API
 - You could use a lot of electric lock and actuators through controlling their power (from inside of the room or vaultto not be easily hacked).
 - I will be adding a wireless Button for a secretary to open the door in the day. The button could be as far as you need. It send a Fixed Code over 433mhz or 315mhz to a receiver. I could go to rolling code (more secure) but that isn't needed for my use.
