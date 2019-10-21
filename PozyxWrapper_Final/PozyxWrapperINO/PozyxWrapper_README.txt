Latest Revision: 10/21/2019
by Alexander Ulery

==============================
======== POZYXWRAPPER ========
==============================

Introduction:
  PozyxWrapper includes all of the calculations and methods for computing and sending the centroid position
of the robot as well as the header (vector/facing angle) to the CANbus system. CANbus I/O has not yet been
implemented.

  The goal of PozyxWrapper is to improve the dynamic memory allocation of the Pozyx system, as well as keep
the main driver code simple and clean. Everything is object-oriented, hence the driver code should primarily
consist of function calls. The constructor will boot the Pozyx system (i.e. PozyxWrapper Poz; )


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

 PozyxWrapper();  //Constructor for Pozyx system; includes Pozyx device boot.
 double calculateX1Position(); //Prints position of X coord for shield/Ard on robot
 double calculateX2Position(); //Prints position of X coord for Pozyx beacon on robot
 double calculateY1Position(); //Returns val of shield on arduino
 double calculateY2Position(); //Returns position of remote beacon
 void printBasicXY();          //Currently only printing X1 and X2
 void updateStatus();	       //RUN THIS EVERY CYCLE; updates X1/X2 positions
 void updateHeading();	       //Calculates/updates heading value

 void BufferAddVal(uint32_t *, uint8_t *, uint32_t val); //This runs in the background in a few functions;
                                                         //adds new values to circular buffer

 **Note: Let me know if any of my descriptions are incorrect.


To add:

 void updateCoordinates();  //Will compress Calculate[..] functions here to simplify things down the road.

 


==============
=== TO DO: ===
==============
 URGENT:
   -Get remote (currently only bouncing off of shield; this is a fault of my understanding and will be
                               the next thing I do as soon as possible)


 -Be able to send header and centroid data to CANbus
 -Clean up and optimize performance where applicable
 -Compress some hanging doubles into structs; i.e. remote_pos_X, remote_pos_Y  etc.  (easy, just gotta
  do it)
 -Gyro



