#include <FastTransfer.h>
#include "Arduino.h"
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include "PozyxWrapper.h"

/*
 *  TO-DO:
 *   -Implement Gyro  
 *   -Clean up and combine a few leftover functions
 */
#define DEBUG
//#define FASTTRANSFER
PozyxWrapper Poz;
FastTransfer Send;
int receiveArray[3];

void setup() {
  Serial.begin(115200);
  Poz.PozyxBoot();
  Send.begin(Details(receiveArray), 8, false, &Serial);
}


void loop() {
  #ifdef DEBUG
  Serial.println(" ");
  Serial.println("===================================");  //Just for clean output
  Serial.println(" ");
  #endif
  Poz.updateStatus(); //Need to run this to collect distance data.
  Poz.calculateCenter();   //try using this within updateStatus
  
  Poz.updateHeading();
  
  //Poz.printBasicXY(); //Outputs raw X1, X2 vals (can be modified to print Y1, Y2 as well)
  Poz.printCH();
}
