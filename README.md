# rplidar-a3-arduino

Arduino/Teensy library for non-blocking communication with RPLidar A3. 

## Hardware 

Library needs:
- RPLidar A3 (tested with [firmware](https://www.slamtec.com/en/Support#rplidar-a3) TODO)
- Arduino compatible MCU (tested with [Teensy](https://www.pjrc.com/teensy/) 3.5)
- 5V power supply (tested with DFRobot [DC-DC Power Module 25W](https://www.dfrobot.com/product-752.html) powered from 2S 7,4V LiPo)

## Purpose

- speed motor control
- non-blocking communication
- raw measurement packets
- precise timestamps

Note - this library doesn't decode the packets. It starts the lidar, controls the motor, gets the packets synchronizing to boundraries and checking CRC. You may add decoding on your own (hints below). I use this library for precisise synchronization of multiple sensors and do decoding on CPU.

## State

Work in progress.

Currently working:
- PID motor control
- starting lidar scanning (hardcoded mode)
- retrieveing raw measurements data (raw data of rplidar_sdk [rplidar_response_ultra_capsule_measurement_nodes_t](https://github.com/Slamtec/rplidar_sdk/blob/8291e232af614842447a634b6dbd725b81f24713/sdk/sdk/include/rplidar_cmd.h#L197))

Both library interface and implementation may change.

## Dependencies 

rpliar-a3-arduino uses [Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library.git).

## Installation

- in Arduino IDE Sketch -> Include Library -> Manage Libraries... -> PID -> Install
- copy or clone `rplidar-a3-arduino` directory to your sketchbook `libraries` directory

## Using

```C++
#include <rplidar.h>

const int pwmPin = 36;
const int rpm=600; 

//Use Teensy 3.5 hardware serial
RPLidar lidar(Serial1, pwmPin); 
RPLidarPacket packet;

void setup()
{
  lidar.setup(rpm);
  lidar.start();
}
  
void loop()
{
  //this function doesn't block waiting for the packet
  //but it expects the same from the rest of your code
  if( lidar.processAvailable(&packet) )
  {
    //do something with the packet
  }
}
```

## Packet decoding notes

As was already mentioned this library doesn't decode the packets.

If you want to add the decoding:
- raw packets correspond to [rplidar_response_ultra_capsule_measurement_nodes_t](https://github.com/Slamtec/rplidar_sdk/blob/8291e232af614842447a634b6dbd725b81f24713/sdk/sdk/include/rplidar_cmd.h#L197) in [rplidar_sdk](https://github.com/Slamtec/rplidar_sdk)
- decoding should do the same as [_ultraCapsuleToNormal](https://github.com/Slamtec/rplidar_sdk/blob/master/sdk/sdk/src/rplidar_driver.cpp#L1137) in [rplidar_sdk](https://github.com/Slamtec/rplidar_sdk)

rplidar_sdk needs 2 consecutive packets to do the decoding. Peeking into the SDK code you may probably decode everything but the last reading (or 4 readings) without the second packet.

For checking if the packets follow one another use the `sequence` field returned by the library (e.g. i, i+1 or uint8_t max, 0.

## License

Library is licensed under Mozilla Public License, v. 2.0

This is similiar to LGPL but more permissive:

- you can use it as LGPL in prioprietrary software
- unlike LGPL you may compile it statically with your code

Like in LGPL, if you modify this library, you have to make your changes available. Making a github fork of the library with your changes satisfies those requirements perfectly.
