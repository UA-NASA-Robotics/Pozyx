// Interface file for Pozyx functions I will create
#ifndef PozyxWrapper_h
#define PozyxWrapper_h

#include "Arduino.h"

#define isRemote true
#define RadToPi 57.2957795131
#define ANCHORDISPLACEMENT  1600 //predefined distance between anchors on collection bin, in mm
                                //1700
#define MID_DIST 300.0 //set distance to center of robot
#define TAG_DIST 450.0 //was 460



///////////////////////////////////////////////////////////////////////////////////////////
//
//	TODO:
//	-Transfer functions of newTestArena_COPY into PozyxWrapper where applicable
//	-Transfer #defines, and defined global vars as private member attributes of header
//
///////////////////////////////////////////////////////////////////////////////////////////


class PozyxWrapper
{
    public:
        PozyxWrapper(); //default constructor
        //void boot(); //do I need this?  constructor can do this
		void printStatus();  //for debugging
        void updateDistances();		//Needs written under .cpp
        void updateCoordinates();
        void updateHeading();
        void enableGyroAssistance();	//Needs written under .cpp
        void BufferAddVal(uint32_t *buff, uint8_t *head, uint32_t val);
        uint32_t getBuffAvg(uint32_t *buff);		//Needs written under .cpp
		//
		//
		//
        //return buffer averages of the distances 
        uint32_t getDeviceLeftDistance();	//Needs written under .cpp
        uint32_t getDeviceRightDistance();	//Needs written under .cpp
        uint32_t getRemoteLeftDistance();	//Needs written under .cpp
        uint32_t getRemoteRightDistance();	//Needs written under .cpp
		void printXYposition();
        
		
    private:
        uint16_t leftAnchorBeaconAddress;
        uint16_t rightAnchorBeacon;
        uint16_t remoteBeacon; //left side of robot, REMOTE
        uint16_t rightRemoteBeacon; //right side of robot, DEVICE
        uint8_t ranging_protocol;
        int leftRemoteBeacon;
        uint8_t Head_1, Head_2, Head_3, Head_4;
		
		//Calculations, mathematical laws, etc. go here
		unsigned long powerOfTwo(unsigned long x);
		double lawOfCOS( uint32_t a, uint32_t b, uint32_t c);
		

        struct status{
            int deviceToLeft;
            int deviceToRight;
            int remoteToLeft;
            int remoteToRight;
        };

        typedef struct{
            double X;
            double Y;
        }remotePos, devicePos, centerPos;
        //int deviceLeftStatus, deviceRightStatus, remoteLeftStatus, remoteRightStatus;
    
        device_range_t deviceLeftRange;
        device_range_t deviceRightRange;
        device_range_t remoteLeftRange;
        device_range_t deviceLeftRange;

        //put in the internal array buffers in here too!
        int bufferCount;
        uint32_t deviceLeftDistanceBuffer[bufferCount];
        uint32_t deviceRightDistanceBuffer[bufferCount];
        uint32_t remoteLeftDistanceBuffer[bufferCount];
        uint32_t remoteRightDistanceBuffer[bufferCount];

        struct bufferHead{
            uint8_t deviceLeft;
            uint8_t deviceRight;
            uint8_t remoteLeft;
            uint8_t remoteRight;
        };
};

#endif 
