#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <PozyxWrapper.h>

int bufferCount;
uint32_t deviceLeftDistanceBuffer[bufferCount];
uint32_t deviceRightDistanceBuffer[bufferCount];
uint32_t remoteLeftDistanceBuffer[bufferCount];
uint32_t remoteRightDistanceBuffer[bufferCount];
        
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

void setup() {
 Serial.begin(115200);
 

}

void loop() {
  
}
