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

PozyxWrapper Poz;

void setup() {
  Serial.begin(115200);
  Poz.PozyxBoot();
}


void loop() {
  
  Serial.println(" ");
  Serial.println("===================================");  //Just for clean output
  Serial.println(" ");
  
  Poz.updateStatus(); //Need to run this to collect distance data.
  Poz.calculateCenter();   //try using this within updateStatus
  
  Poz.updateHeading();
  
  //Poz.printBasicXY(); //Outputs raw X1, X2 vals (can be modified to print Y1, Y2 as well)
  Poz.printCH();
}
