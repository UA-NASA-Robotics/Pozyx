 #include "Arduino.h"
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include "PozyxWrapper.h"

void setup() {
  Serial.begin(115200);
}

void loop() {
  PozyxWrapper Poz;  //run constructor, this will boot up the Pozyx system
  Poz.updateStatus(); //Need to run this to collect distance data.
  Serial.println("===================================");  //For debugging purposes
  //Poz.calculateCenter();
  //Poz.updateHeading();
  Poz.printBasicXY(); //Outputs raw X1, X2 vals (can be modified to print Y1, Y2 as well)
}
