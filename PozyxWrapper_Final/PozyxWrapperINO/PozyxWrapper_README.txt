Latest Revision: 12/22/2019
by Alexander Ulery

==============================
======== POZYXWRAPPER ========
==============================

Introduction:
  PozyxWrapper includes all of the calculations and methods for computing and sending the centroid position
of the robot as well as the header (vector/facing angle) to the CANbus system. CANbus I/O will be implemented
when the code is rewritten for the PIC32 MCU.

  The goal of PozyxWrapper is to improve the dynamic memory allocation of the Pozyx system, as well as keep
the main driver code simple and clean. Everything is object-oriented, hence the driver code should primarily
consist of function calls.

  Keep in mind that PozyxWrapper runs in an object-oriented fashion. Everything runs through a 'PozyxWrapper' object
that is constructed in the global scope. i.e. 'PozyxWrapper Poz;'   and then run the functions off of Poz.function();

(for object name 'X')
  Unless debugging, simply use X.PozyxBoot() after Serial.begin(115200), and then run the following cycle:

  X.updateStatus();	  //Gathers position data
  X.calculateCenter();    //Calculates center from data
  X.updateHeading();      //Calculates heading from data
  X.printCH();            //Outputs data; replace with fastTransfer down the road.

 

  Keep in mind that a lot of the calculations/code have been either taken from previous renditions of the
Pozyx setup, and that I do not take credit for all of the code written. -Alex

=======================
=== Included files: ===
=======================

 -PozyxWrapperINO    (quick demo/main file demonstrating functionality)
 -PozyxWrapper.h     (Header file for PozyxWrapper)
 -PozyxWrapper.cpp   (Implementation file for PozyxWrapper)


===========================
=== Notable components: ===
===========================

Class: PozyxWrapper


Functions:

 void PozyxBoot();             //Boots up Pozyx system, run this in setup!
 double calculateX1Position(); //Prints position of X coord for shield/Ard on robot
 double calculateX2Position(); //Prints position of X coord for Pozyx beacon on robot
 double calculateY1Position(); //Returns val of shield on arduino
 double calculateY2Position(); //Returns position of remote beacon
 void printBasicXY();          //Prints raw X1/X2 values, mostly used for debugging and accuracy testing
 void printCH();               //This is the printing method of choice; returns center X/Y and header values
 void updateStatus();	       //RUN THIS EVERY CYCLE; Updates raw X1/X2 positions
 void calculateCenter();       //RUN THIS EVERY CYCLE; Calculates/updates center value
 void updateHeading();	       //RUN THIS EVERY CYCLE; Calculates/updates heading value

 void BufferAddVal(uint32_t *, uint8_t *, uint32_t val); //This runs in the background in a few functions;
                                                         //adds new values to circular buffer

 **Note: Let me know if any of my descriptions are incorrect.


To add:

 void updateCoordinates();  //Will compress Calculate[..] functions here to simplify things down the road.

  *Implementing Gyro functions from 'functionalranging' as soon as possible
 


==============
=== TO DO: ===
==============
 -Clean up/combine a few leftover functions if need-be; however, everything works
 -Gyro



