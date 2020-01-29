#include <FastTransfer.h>
#include "Arduino.h"
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

//#define DEBUG

#define FASTTRANSFER

#include "PozyxWrapper.h"

/*
 *  TO-DO:
 *   -Clean up and combine a few leftover functions
 */

PozyxWrapper Poz;


void setup() {
  Serial.begin(115200);
  Poz.PozyxBoot();
  Poz.calibrateGyro();  //initialize gyro calibration
}


void loop() {
  #ifdef DEBUG
  Serial.println(" ");
  Serial.println("===================================");  //Just for clean output
  Serial.println(" ");
  #endif
  
  Poz.updateStatus(); //Need to run this to collect distance data.
  Poz.updateCoordinates();
  Poz.calculateCenter();   //try using this within updateStatus
  
  Poz.updateHeading();
  Poz.adjustHeading();  //Adjust heading w/ Gyro
  
  //Poz.printBasicXY(); //Outputs raw X1, X2 vals (can be modified to print Y1, Y2 as well)
  Poz.printCH();
}
