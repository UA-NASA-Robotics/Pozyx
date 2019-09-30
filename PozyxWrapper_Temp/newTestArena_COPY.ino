//test arena 9/16/19
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

//updateStatus
void updateStatus()
{
    //perform loop operations (put in one void function)
    // let's perform ranging with the destination

    //RANGING WITH DEVICE (BEACON #1 ON THE ROBOT)

    deviceLeftStatus = Pozyx.doRanging(destination_id_1, &deviceLeftRange);
    deviceRightStatus = Pozyx.doRanging(destination_id_2, &deviceRightRange);
    
    if (deviceLeftStatus == POZYX_SUCCESS && deviceRightStatus == POZYX_SUCCESS)
    { 
      //Updating the buffers
      BufferAddVal(DistanceVals1, &Head_1, deviceLeftRange.distance); 
      BufferAddVal(DistanceVals2, &Head_2, deviceRightRange.distance);
      //Update tag angle, x, y pos
      updateTagAngles(getBuffAvg(DistanceVals1), getBuffAvg(DistanceVals2), false); //passing 0 because it's not the remote device
      //updateTagAngles(deviceLeftRange.distance, deviceRightRange.distance, 0);
    }
    //RANGING WITH THE REMOTE DEVICE (BEACON #2 ON THE ROBOT)
    #ifdef DUAL_POZYX

    remoteLeftStatus = Pozyx.doRemoteRanging(remote_id, destination_id_1, &remoteLeftRange);
    remoteRightStatus = Pozyx.doRemoteRanging(remote_id, destination_id_2, &remoteRightRange);
    
    if(remoteLeftStatus == POZYX_SUCCESS && remoteRightStatus == POZYX_SUCCESS)
    {
      //Updating the buffers
      BufferAddVal(DistanceVals3, &Head_3, remoteLeftRange.distance);
      BufferAddVal(DistanceVals4, &Head_4, remoteRightRange.distance);
      
      //Update tag angle, x, y pos
      updateTagAngles(getBuffAvg(DistanceVals3), getBuffAvg(DistanceVals4), true); //passing 1 because it's the remote device
      //updateTagAngles(remoteLeftRange.distance, remoteRightRange.distance, 1);
    }
#endif
}

void updateHeading()
{
     //**HEADING CALCULATIONS: *****************************************************************************************************
     
      double tan_num = (double)(device_pos.Y) - (double)(remote_pos.Y);
      double tan_den = (double)(remote_pos.X) - (double)(device_pos.X);
      heading = atan(tan_num/tan_den)*RadToPI;
     
      //Serial.print(heading);
      //Quadrant 1
      if(device_pos.X > remote_pos.X && (device_pos.Y < remote_pos.Y)){// || (device_pos.Y - remote_pos.Y < 200))){
        
        //heading = abs(heading -90);
        //heading = abs(heading -90)+45;
        //heading = abs(heading) + 90;
        //heading = heading - 90; //update range for 360 degrees
        heading = abs(90 - heading);
        heading = heading - 10;
        quadrant = 1;
      }
      //Quadrant 2 (
      else if(device_pos.X > remote_pos.X && (device_pos.Y > remote_pos.Y))//|| remote_pos.Y - device_pos.Y < 180) && (device_pos.X - remote_pos.X > 180))
      {
        
        //heading = 0 - (90 - abs(heading)) -90; //decreases when I need it to increase
        //Serial.println(heading);
        heading = abs(heading) + 90;   //working right now 4//11/19    
        quadrant = 2;     
      }
      
      //Quadrant 3 
      else if((device_pos.X < remote_pos.X ) && device_pos.Y > remote_pos.Y){       //(device_pos.X < remote_pos.X || (device_pos.X - remote_pos.X < 80)
       
        heading = heading + 180;
        //heading = 0-heading;  //need absolute negative
        //heading = heading + 102;
        //heading = heading+180;
        quadrant = 3;
      }
      //Quadrant 4 
      else if(device_pos.X < remote_pos.X && device_pos.Y < remote_pos.Y){
        
        heading = abs(heading);
        heading = heading + 10;
        heading = heading + 270; //update quadrants
        if(heading >= 360)
        {
          heading = heading - 360;
          quadrant = 1;
        }
        else{
          quadrant = 4;
        }
      }
}



#define RadToPi 57.2957795131
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
int updateTagAngles (uint32_t distanceVals1, uint32_t distanceVals2, bool remote_flag)
{
  double leftAngle = lawOfCOS(distanceVals1, ANCHORDISPLACEMENT, distanceVals2); 
  if (remote_flag)
  {
    remote_pos.X = ((double)distanceVals1 * cos(leftAngle)); 
    remote_pos.Y = ((double)distanceVals1 * sin(leftAngle));
  }
  else
  {
    device_pos.X = ((double)distanceVals1 * cos(leftAngle));
    device_pos.Y = ((double)distanceVals1 * sin(leftAngle));
  }
}

void calculateCenter()
{
  // centerpoint between POZYX sensors on robot
  mid.X = (device_pos.X + remote_pos.X) / 2;
  mid.Y = (device_pos.Y + remote_pos.Y) / 2;

  // compute unit vector in direction of robot heading
  double x_component = remote_pos.X - device_pos.X;
  double y_component = remote_pos.Y - device_pos.Y;
  double magnitude = sqrt(pow(x_component, 2) + pow(y_component, 2));
  double unit_x_component = x_component / magnitude;
  double unit_y_component = y_component / magnitude;
  double const VEC_ANGLE = 270;
  double heading_unit_vector_x = cos(VEC_ANGLE) * unit_x_component - sin(VEC_ANGLE) * unit_y_component;
  double heading_unit_vector_y = sin(VEC_ANGLE) * unit_x_component + cos(VEC_ANGLE) * unit_y_component;

  // compute robot centroid
  center.X = mid.X + MID_DIST * heading_unit_vector_x;
  center.Y = mid.Y + MID_DIST * heading_unit_vector_y;
}


bool isWithinFloat(double sample, double lowBound, double highBound)
{
    return (sample > lowBound && sample < highBound);
}

double calculateX1Position()
{
  //double a = getBuffAvg(DistanceVals1);
  double b = deviceRightRange.distance;
  //Serial.println(getBuffAvg(DistanceVals1));
  //double c = getBuffAvg(DistanceVals3);
  double d = deviceLeftRange.distance;
//  Serial.println("DeviceLeftRange:");
//  Serial.println(d);
  //Serial.println(getBuffAvg(DistanceVals2));
  double u = ANCHORDISPLACEMENT;
  double w = TAG_DIST;
  
  unsigned long squareThis = ((b*b)-(u*u)-(d*d))/(-2*u);
  unsigned long squared = powerOfTwo(squareThis);
//  Serial.print("before square: ");
//  Serial.println(squareThis);
//  Serial.print("after squaring: ");
//  Serial.println(squared);
//  Serial.print("D*D: ");Serial.println((d*d));
//  Serial.print("result: ");
  unsigned long secVal = -1*squared+(unsigned long)(d*d);
//  Serial.println(secVal);
  Serial.println(sqrt(-1*powerOfTwo(((b*b)-(u*u)-(d*d))/(-2*u))+(unsigned long)(d*d)));
  return sqrt(secVal);
}

double calculateX2Position()
{ 
  double a = remoteRightRange.distance;
  //Serial.println(a);
  double c = remoteLeftRange.distance;
  //Serial.println(c);
  double u = ANCHORDISPLACEMENT;
  double w = TAG_DIST;

  unsigned long squareThis = ((a*a)-(u*u)-(c*c))/(-2*u);
  //Serial.println(squareThis);
  unsigned long squared = powerOfTwo(squareThis);
  //Serial.println(squared);
  unsigned long secVal = -1*squared+(unsigned long)(c*c);
  //Serial.println(secVal);
  //Serial.println("testing equation in one line: ");
  //Serial.println(sqrt(-1*powerOfTwo(((a*a)-(u*u)-(c*c))/(-2*u))+(unsigned long)(c*c)));
  Serial.println(sqrt(-1*powerOfTwo(((a*a)-(u*u)-(c*c))/(-2*u))+(unsigned long)(c*c)));
  return sqrt(secVal);
}
