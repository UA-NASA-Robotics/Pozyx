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
#define DUAL_POZYX
#define DEBUG
#define stdTesting
#define XYpos
#define AVERAGEAMOUNT 50
#define LeftAnchor 0
#define RightAnchor 1
#define MID_DIST 300.0
#define TAG_DIST 450.0
#define RadToPi 57.2957795131

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
		S
        //return buffer averages of the distances 
		/*/////
		
		    Not needed currently.
        uint32_t getDeviceLeftDistance();	//Needs written under .cpp
        uint32_t getDeviceRightDistance();	//Needs written under .cpp
        uint32_t getRemoteLeftDistance();	//Needs written under .cpp
        uint32_t getRemoteRightDistance();	//Needs written under .cpp
		*//////
		
		
		bool isWithinFloat(double, double, double);	//add to .cpp
		void printXYposition();
		
    private:
        uint16_t leftAnchorBeaconAddress;
        uint16_t rightAnchorBeacon;
        uint16_t remoteBeacon; //left side of robot, REMOTE
        uint16_t rightRemoteBeacon; //right side of robot, DEVICE
        uint8_t ranging_protocol;
        int leftRemoteBeacon;
        uint8_t Head_1, Head_2, Head_3, Head_4;
		
		/////////////////////////////////////////////
		/////////////////////////////////////////////    Taken straight from newTestArena_COPY; needs cleaned
		/*
		uint16_t destination_id_1 = 0x6719; //device id of left anchor on collection bin
		uint16_t destination_id_2 = 0x6e21; //device id of right anchor on collection bin
		signed int range_step_mm = 1000;      // every 1000mm in range, one LED less will be giving light.
		uint8_t ranging_protocol = POZYX_RANGE_PROTOCOL_PRECISION; // ranging protocol of the Pozyx.
		uint16_t remote_id = 0x6751;// the network ID of the remote device
		//uint16_t remote_id2 = 0x6715;  
		bool remote = true;      // whether to use the given remote device for ranging
		uint32_t DistanceVals1[AVERAGEAMOUNT];
		uint8_t Head_1 = 0;
		uint32_t DistanceVals2[AVERAGEAMOUNT];
		uint8_t Head_2 = 0;
		
		uint16_t AnchorDisplacment = 0;
		device_range_t deviceLeftRange; //change to deviceLeftRange
		device_range_t deviceRightRange; //change to deviceRightRange
		int deviceLeftStatus = 0, deviceRightStatus = 0; //status for connection between device and left and right anchors on collection bin
		//double deviceX, deviceY;

		unsigned long startMillis; 
		unsigned long currentMillis;
		unsigned long lastFTMillis;
		const unsigned long period = 500; //one second
		*/
		//////////////////////////////////////////////
		//////////////////////////////////////////////
		
		struct deviceXY{
		double X;
		double Y;
		};
		deviceXY device_pos;

		double heading;
		int quadrant = 0;


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
		
		/*
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
		*/
		
        struct bufferHead{
            uint8_t deviceLeft;
            uint8_t deviceRight;
            uint8_t remoteLeft;
            uint8_t remoteRight;
        };
};

#endif 
