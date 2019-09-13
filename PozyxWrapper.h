// Interface file for Pozyx functions I will create

#ifndef PozyxWrapper_h
#define PozyxWrapper_h

#include "Arduino.h"

class PozyxWrapper
{
    public:
        PozyxWrapper(); //default constructor
        void boot(); //do I need this?  constructor can do this
        void updateDistances();
        void calculateXYPos();
        void updateHeading();
        
    private:
        uint16_t leftAnchorBeaconAddress;
        uint16_t rightAnchorBeacon;
        uint16_t remoteBeacon; //left side of robot, REMOTE
        uint16_t rightRemoteBeacon; //right side of robot, DEVICE
        uint8_t ranging_protocol;
        int leftRemoteBeacon
        int deviceLeftStatus, deviceRightStatus, remoteLeftStatus, remoteRightStatus;
    
        //put in the internal array buffers in here too!
        int bufferCount;
        uint32_t deviceLeftDistanceBuffer[bufferCount];
        uint32_t deviceRightDistanceBuffer[bufferCount];
        uint32_t remoteLeftDistanceBuffer[bufferCount];
        uint32_t remoteRightDistanceBuffer[bufferCount];
};

#endif 
