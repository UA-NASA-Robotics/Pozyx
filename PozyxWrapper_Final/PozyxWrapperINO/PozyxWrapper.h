// Interface file for Pozyx functions I will create
#ifndef PozyxWrapper_h
#define PozyxWrapper_h

#include "Arduino.h"

#define DUAL_POZYX  //define if using remote + shield; else comment out!
#define isRemote true
#define RadToPi 57.2957795131
#define ANCHORDISPLACEMENT  1600 //predefined distance between anchors on collection bin, in mm (was 1700)
#define MID_DIST 300.0 //set distance to center of robot
#define TAG_DIST 450.0 //was 460
#define AVERAGEAMOUNT 50//changed, needed more memory

/*
 *  I PLAN ON WRITING A README FILE THAT BRIEFLY EXPLAINS WHAT EACH FUNCTION DOES AND PUSH IT ONTO THE REPO -Alex 
 * 
 */
 
class PozyxWrapper
{
    public:
        void PozyxBoot();           //Boots up Pozyx System, RUN THIS IN SETUP.
        void updateCoordinates();   //Mesh together calculateX1, X2, Y1 and Y2 methods together here  -> not in use quite yet (uses typedef struct; allocate to a namespace?)
        double calculateX1Position();
        double calculateX2Position();
        double calculateY1Position();
        double calculateY2Position();
        void printBasicXY();
        void printCH();
        void updateStatus();
        void updateHeading();
        void BufferAddVal(uint32_t *buff, uint8_t *head, uint32_t val);
        
        int updateTagAngles(uint32_t distanceVals1, uint32_t distanceVals2, bool remote_flag);
        uint32_t getBuffAvg(uint32_t *buff);
        
        void calculateCenter();
        
    private:
        uint16_t remote_id = 0x6751;// the network ID of the remote device
        uint16_t destination_id_1 = 0x6719; //device id of left anchor on collection bin
        uint16_t destination_id_2 = 0x6e21; //device id of right anchor on collection bin
        uint8_t ranging_protocol = POZYX_RANGE_PROTOCOL_PRECISION; // ranging protocol of the Pozyx.
        uint32_t DistanceVals1[AVERAGEAMOUNT];
        uint32_t DistanceVals2[AVERAGEAMOUNT];
        bool remote = true;
        
        uint8_t Head_1 = 0;
        uint8_t Head_2 = 0;
        
        int deviceLeftStatus;
        int deviceRightStatus;
        
        device_range_t deviceLeftRange;
        device_range_t deviceRightRange;
        
        device_range_t remoteLeftRange;
        device_range_t remoteRightRange;

        //Temporary until I can store in struct
        double device_pos_X;
        double device_pos_Y;
        double remote_pos_X;
        double remote_pos_Y;
        double center_X;
        double center_Y;
        double mid_X;
        double mid_Y;

        double heading;
        int quadrant = 0;

        #ifdef DUAL_POZYX
          //Param for remote device
          uint32_t DistanceVals3[AVERAGEAMOUNT];
          uint32_t DistanceVals4[AVERAGEAMOUNT];
          uint8_t Head_3 = 0;
          uint8_t Head_4 = 0;
          //Statuses for connection between remote device and left and right anchors on collection bin
          int remoteLeftStatus;
          int remoteRightStatus;

          double slope, A, B, m;
          double remoteLeftAngle, remoteRightAngle; //global var to store calculation for updateTagAngles function
          
        #endif

        unsigned long powerOfTwo(unsigned long x);
        double lawOfCOS( uint32_t a, uint32_t b, uint32_t c);
};

#endif 
