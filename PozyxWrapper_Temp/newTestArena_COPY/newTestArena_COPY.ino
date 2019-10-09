//#include <FastTransfer.h>
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
/*
 * Beacon and Anchor tags are interchangeable 
 * Beacon <=> Anchors
 */
////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

/* These five lines are #defines for either debugging or runtime
#define DUAL_POZYX
//#define DEBUG //comment this OUT when not in DEBUG (FASTTRANSFER IS defined if DEBUG is undefined)
//#define FASTTRANSFER //not defined when we want to debug (i.e. have Serial.print working for debug purposes)
//#define testAngles //this should only be defined IF FASTTRANSFER is NOT defined
#define stdTesting //defined IF we want to print out the distances for analysis 
        (std means standard deviation, I was taking the std. deviation to ensure Pozyx was working at a reasonable accuracy)

(FASTRANSFER) XOR (DEBUG OR testAngles OR stdTesting)
*/
#define DUAL_POZYX
#define DEBUG 
//#define FASTTRANSFER 
//#define testAngles 
#define stdTesting 
#define XYpos
#define AVERAGEAMOUNT 50//changed, needed more memory
#define   LeftAnchor  0
#define   RightAnchor 1
#define ANCHORDISPLACEMENT  1600 //predefined distance between anchors on collection bin, in mm
                                //1700
#define MID_DIST 300.0 //set distance to center of robot
#define TAG_DIST 450.0 //was 460
#define RadToPI 57.2957795131

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

struct deviceXY{
  double X;
  double Y;
};
deviceXY device_pos;

double heading;
int quadrant = 0;






//Prototypes for functions:
bool isWithinFloat(double sample, double lowBound, double highBound);
double filterAngle(double Pozyx, double Angle);
void calebrateGyro();
double lawOfCOS(uint32_t a,uint32_t b,uint32_t c);
uint32_t getBuffAvg(uint32_t *buff);
void BufferAddVal(uint32_t *buff, uint8_t *head, uint32_t val);
void pozyxBoot(); //initial setup of Pozyx devices
void printStatus(); //print out the status
void updateStatus(); //update current tag distances/status
void printXYposition();
double returnAngle(int, int, String);
void calculateCenter();
double deviceLeftAngle, deviceRightAngle;//global variable for storing calculations in updateTagAngles function 

#ifdef DUAL_POZYX
  //Parameters for remote device: 
  uint32_t DistanceVals3[AVERAGEAMOUNT];
  uint8_t Head_3 = 0;
  uint32_t DistanceVals4[AVERAGEAMOUNT];
  uint8_t Head_4 = 0;
  device_range_t remoteLeftRange;
  device_range_t remoteRightRange;
  int remoteLeftStatus = 0, remoteRightStatus = 0; //status for connection between remote device and left and right anchors on collection bin
  //double remoteX, remoteY;
  deviceXY remote_pos, mid, center; //remote device XY position, midpoint and center XY position
  double slope, A, B, m;
  double remoteLeftAngle, remoteRightAngle;  //global variable for storing calculations in updateTagAngles function 
#endif

//timing:
unsigned long startMillis; 
unsigned long currentMillis;
unsigned long lastFTMillis;
const unsigned long period = 500; //one second


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(115200);
  pozyxBoot();
}

// long lastMillis;
// double yAngle = 0;
// double lastyAngle = 0;
// bool flag = 1;
// unsigned long previousMillis = 0;
// unsigned long interval = 50;
// double currentHeading = 0;
// int count = 0;

void loop() {
      updateStatus(); //maybe have this run less?
      //calculateCenter();
      //updateHeading();
      Serial.println("X1 position:");
      Serial.println(calculateX1Position());
      Serial.println("X2 position:");
      Serial.println(calculateX2Position());
      
      #ifdef DEBUG

        //Serial.println(center.X);
        //Serial.println(center.Y);
        //Serial.println(heading);
      #endif
      
}




///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
