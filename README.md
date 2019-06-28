# Pozyx

Specifications for Pozyx Sensor fusion with Arduino

Date: 2/4/2019

Author: Seth Carpenter

>>  Specification Overview: 

This code on the arduino will be the conduit to the Pozyx sensors. the Pozyx Sensors are a time of flight based sensor that will used for localization of the robot in the arena. Each sensor has the ability to get the range of its self to another device or rather every sensor can get the range of every sensor between any sensor. The data on the arduino will be aquisitioned by the master controller via fasttransfer. 


Code needs to be written for two cases:
Make cases changeable with a

#define DUAL_POZYX

#ifdef DUAL_POZYX

#else

#endif

What will be standard for both cases is that there will be two anchors attached collection bin at a set distance. This distance can be hard coded using a #define ANCHOR_BASE_DIST 1500 for now. 

NOTE: all measurements are calculated using mm not cm

First (one Pozyx Tag) :

One Tag will be attached to the robot and a triangle will be established between the three sensors.

Second (Two Pozyx Tag) :

One Tag and one anchor will be attached to the robot and two triangle will be established between the four sensors. The distance between the anchor and the tag will also be a set distance that can be used for calculating values
