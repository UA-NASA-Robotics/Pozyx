#include <FastTransfer.h>
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
//#define DEBUG 
//#define FASTTRANSFER 
//#define testAngles 
#define stdTesting 
#define XYpos
#define AVERAGEAMOUNT 50//changed, needed more memory
#define   LeftAnchor  0
#define   RightAnchor 1
#define ANCHORDISPLACEMENT  1600 //predefined distance between anchors on collection bin, in mm
#define MID_DIST 300.0 //set distance to center of robot
#define TAG_DIST 460.0
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

//GYRO CODE
#define Samples 50
#define SCALING_GYRO .9//0.0151
int16_t gyro_raw[3];
long offsetG_Y;
int gyroYDPS;
int highG_y = -5000,lowG_y = 0;

#ifdef FASTTRANSFER
//fast transfer:
FastTransfer Send;
int receiveArray[5]; //receive buffer, not really used but still need to define
#endif

// Function for calculating variance 
int variance(uint32_t a[], int n) 
{ 
    // Compute mean (average of elements) 
    int sum = 0; 
    for (int i = 0; i < n; i++) 
        sum += a[i]; 
    double mean = (double)sum /  
                  (double)n; 
  
    // Compute sum squared  
    // differences with mean. 
    double sqDiff = 0; 
    for (int i = 0; i < n; i++)  
        sqDiff += (a[i] - mean) *  
                  (a[i] - mean); 
    return sqDiff / n; 
} 

void setup() {
  Serial.begin(115200);
  pozyxBoot();
  calebrateGyro();
    //Pozyx.regWrite(POZYX_POS_FILTER, FILTER_TYPE_MOVINGAVERAGE | 0x10, 8); //changed from moving average to moving median
  Pozyx.regWrite(POZYX_POS_FILTER, FILTER_TYPE_MOVINGMEDIAN | B11110000, 8); //works better, B11110000 (equivalent to 15 base 10=>range is 0-15 for filter strength) OR FILTER_TYPE_MOVINGAVERAGE 
  //Pozyx.regWrite(POZYX_POS_FILTER, FILTER_TYPE_NONE | B11110000, 8);
  //set the pozyx position filter
  //#ifdef DEBUG
  //Serial.println("REG WRITE SUCCESS");
  //int val = 0;
  //***Serial.println(Pozyx.regWrite(POZYX_POS_FILTER, val,8));//FILTER_TYPE_MOVINGAVERAGE | 0x10, 8));
  //#endif
  //Serial.println(Pozyx.regWrite(POZYX_POS_FILTER, 0xF3, 8));
  //Serial.println(Pozyx.regWrite(POZYX_POS_FILTER, 0xF4, 8));

 //if FASTTRANSFER defined, then Fast Transfer code runs
  #ifdef FASTTRANSFER
  startMillis = millis(); //initial start time
  lastFTMillis = 0;
  Send.begin(Details(receiveArray), 9, false, &Serial); //6 is the address
  #endif
}

long lastMillis;
double yAngle = 0;
double lastyAngle = 0;
bool flag = 1;
unsigned long previousMillis = 0;
unsigned long interval = 50;
double currentHeading = 0;
int count = 0;

void loop() {

      #ifdef stdTesting
         
////        Serial.println("RL, RR, DL, DR in that order: "); //remote tag distance to left beacon (anchor), 
////        Serial.print("RL: ");
//        Serial.println(remoteLeftRange.distance);
////        Serial.print("RR: ");
//        Serial.println(remoteRightRange.distance);
////        Serial.print("DL: ");
//        Serial.println(deviceLeftRange.distance);
////        Serial.print("DR: ");
//         Serial.println(deviceRightRange.distance);
//
        Serial.println("CALCULATED HEADING");
        Serial.println(heading);
        Serial.println("GYRO ASSISTED HEADING");
        Serial.println(currentHeading);
        Serial.print("Quadrant: ");
        Serial.println(quadrant);

        printXYposition();
        Serial.println("RL, RR, DL, DR in that order: ");
        Serial.println(getBuffAvg(DistanceVals3));//remote
        Serial.println(getBuffAvg(DistanceVals4));
        Serial.println(getBuffAvg(DistanceVals1));//device
        Serial.println(getBuffAvg(DistanceVals2));
        
      #endif
      
      #ifdef testAngles
      //remoteLeftAngle remoteRightAngle deviceLeftAngle deviceRightAngle:
        Serial.print("remote Left Angle: ");
        Serial.println(remoteLeftAngle);

        Serial.print("remote Right Angle: ");
        Serial.println(remoteRightAngle);

        Serial.print("device Left Angle: ");
        Serial.println(deviceLeftAngle);

        Serial.print("device right Angle: ");
        Serial.println(deviceRightAngle);

        if(remoteLeftAngle > deviceLeftAngle){
             Serial.println("***Remote left angle GREATER THAN Device left angle");
  
        }else{
             Serial.println("***Device left angle GREATER THAN Remote left angle");
        }
  
        if(remoteRightAngle > deviceRightAngle)
        {
             Serial.println("***Remote Right angle GREATER THAN Device Right angle");
        }else{
             Serial.println("***Device Right angle GREATER THAN Remote Right angle");
        }
        
      #endif
      
      if(count > 100)
      {
        //Serial.println(currentHeading);
        //Serial.println("SYSTEM RESET******************");
        Pozyx.resetSystem(); //keep?
        count = 0;
      }
      count++;
      updateStatus(); //maybe have this run less?
      calculateCenter();
      #ifdef DEBUG
      printXYposition();
      #endif
      //update the pozyx heading every second
      //unsigned long currentMillis = millis();
      if (abs(millis() - previousMillis) > interval) 
      {
          //update pozyx angle
          updateHeading();
           if(isnan(heading))
           {
           flag = 0;
           }
           else{
              flag = 1;
           }
          //reset gyro offset
          if(flag)
          {
            yAngle = 0;
          }
          previousMillis = millis();
      } 

      lastyAngle = yAngle;
      //obtain the gyro offset
      if((abs(lastMillis - millis())) > 10)
      {
          Pozyx.regRead(POZYX_GYRO_X, (uint8_t*)&gyro_raw, 3*sizeof(int16_t));
          gyroYDPS =  gyro_raw[1]- offsetG_Y;
          if(isWithinFloat(gyroYDPS,lowG_y*1.1,highG_y*1.1))
             gyroYDPS=0;  
          yAngle += ((double)gyroYDPS*SCALING_GYRO * ((millis()-lastMillis)/1000.0)/16);
          
          lastMillis = millis();
      } 
       //add the gyro offset to the current heading
      if(flag){
      
         currentHeading = heading + yAngle;
      }
      else{
         #ifdef DEBUG
         Serial.println("currentHEADING UPDATED");
         Serial.println(currentHeading);
         Serial.println("yAngle");
         Serial.println(yAngle);
         #endif
         currentHeading = currentHeading + yAngle; //update the heading with only the gyro offset
      }
      #ifdef DEBUG
      Serial.println("flag");
      Serial.println(flag);
      Serial.print("CURRENT HEADING: ");
      Serial.println(currentHeading);
      Serial.print("calculated heading: ");
      Serial.println(heading); //we're good!
      #endif

      //fastransfer send data to master
      // master address is 4
      // to load data in to FT send buffer use toSend(what index, data)
      // to send the data use sendData(address of receiver)
      // we need to send this info at 1 sec interval
      // use an if statemnet that will compare to current time to last time we send data
      // you want to use millis()
      #ifdef FASTTRANSFER
      if (abs(millis()-lastFTMillis) > 500)
      {
        //Send.ToSend(1, (int)(center.X/10)); //divide by 10 because we want to send in centimeters
        //Send.ToSend(2, (int)(center.Y/10)); //divide by 10 because we want to send in centimeters
        //Send.ToSend(3, (int)(currentHeading));
        Send.ToSend(1, (int)(mid.X));
        Send.ToSend(2, (int)(mid.Y));
        Send.ToSend(3, (int)(currentHeading));
        Send.sendData(5);
        lastFTMillis = millis();
      }
      #endif
}

//Calculating angle based on three sides of the triangle
double lawOfCOS( uint32_t a, uint32_t b, uint32_t c)
{
/*                c
 *       A *-----------* B      Cos(C) = (a^2 + b^2 -c^2)/(2*a*b)
 *          \         /         Cos(A) = (b^2 + c^2 -a^2)/(2*b*c)
 *           \       /          Cos(B) = (c^2 + a^2 -b^2)/(2*c*a)
 *          b \     /  a
 *             \   /
 *              \ /
 *               C
 */           
  long num = (a*a) + (b*b) - (c*c);
  long  den = (2*a*b);

   return acos((double)num/(double)den);
}

//function for adding new value to the buffer (circular buffer)
void BufferAddVal(uint32_t *buff, uint8_t *head, uint32_t val) //updates the value of buff
{
  buff[(*head)++] = val;
  if ( *head >= AVERAGEAMOUNT)
  {
    *head = 0;
  }
}

//passes in pointer to buffer
//returns average of the buffer (dependent on AverageAmount)
uint32_t getBuffAvg(uint32_t *buff)
{
  unsigned long long sum = 0;
  for (int i = 0; i < AVERAGEAMOUNT; i++)
  {
    sum += buff[i];
  }
  sum = sum / AVERAGEAMOUNT;
  return sum;
}

//void function to boot up the Pozyx, should be called in Setup
void pozyxBoot()
{
   if (Pozyx.begin() == POZYX_FAILURE) {
    #ifdef DEBUG
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    #endif
    delay(100);
    abort();
  }
  // setting the remote_id back to NULL will use the local Pozyx
  if (!remote) {
    remote_id = NULL;
  }
#ifdef DEBUG
  Serial.println("------------POZYX RANGING V1.1------------");
  Serial.println("NOTES:");
  Serial.println("- Change the parameters:");
  Serial.println("\tdestination_id (target device)");
  Serial.println("\trange_step (mm)");
  Serial.println();
  Serial.println("- Approach target device to see range and");
  Serial.println("led control");
  Serial.println("------------POZYX RANGING V1.1------------");
  Serial.println();
  Serial.println("START Ranging:");
#endif
  // make sure the pozyx system has no control over the LEDs, we're the boss
  uint8_t led_config = 0x0;
  Pozyx.setLedConfig(led_config, remote_id);
  // do the same with the
  Pozyx.setLedConfig(led_config, destination_id_1);
  // do the same with the
  Pozyx.setLedConfig(led_config, destination_id_2);
  // set the ranging protocol
  Pozyx.setRangingProtocol(ranging_protocol, remote_id);

  Pozyx.setSensorMode(0, remote_id);
  // Get the distance between the Anchors
}

//updateStatus
void updateStatus()
{
    //perform loop operations (put in one void function)
    // let's perform ranging with the destination
    //ranging with device
    uint32_t sumLeft = 0;
    uint32_t sumRight = 0;
    int avgCount = 4;
    for(int i = 0; i < avgCount; i++)
    {
      deviceLeftStatus = Pozyx.doRanging(destination_id_1, &deviceLeftRange);
      deviceRightStatus = Pozyx.doRanging(destination_id_2, &deviceRightRange);
      sumLeft = sumLeft + deviceLeftRange.distance;
      sumRight = sumRight + deviceRightRange.distance;

    }


    deviceLeftStatus = Pozyx.doRanging(destination_id_1, &deviceLeftRange);
    deviceRightStatus = Pozyx.doRanging(destination_id_2, &deviceRightRange);
    //deviceLeftStatus = Pozyx.doRemoteRanging(remote_id2, destination_id_1, &deviceLeftRange);
    //deviceRightStatus = Pozyx.doRemoteRanging(remote_id2, destination_id_2, &deviceRightRange);
    if (deviceLeftStatus == POZYX_SUCCESS && deviceRightStatus == POZYX_SUCCESS)
    { //not getting here
      
      //Updating the buffers
      BufferAddVal(DistanceVals1, &Head_1, (sumLeft/avgCount)); 
      BufferAddVal(DistanceVals2, &Head_2, (sumRight/avgCount));
      //Update tag angle, x, y pos
      updateTagAngles(getBuffAvg(DistanceVals1), getBuffAvg(DistanceVals2), false); //passing 0 because it's not the remote device
      //updateTagAngles(deviceLeftRange.distance, deviceRightRange.distance, 0);
    }
    //ranging with remote device
    #ifdef DUAL_POZYX
    sumLeft = 0;
    sumRight = 0;
    for(int j = 0; j < avgCount; j++)
    {
      remoteLeftStatus = Pozyx.doRemoteRanging(remote_id, destination_id_1, &remoteLeftRange);
      remoteRightStatus = Pozyx.doRemoteRanging(remote_id, destination_id_2, &remoteRightRange);
      sumLeft = sumLeft + remoteLeftRange.distance;
      sumRight = sumRight + remoteRightRange.distance;
    }

   
    if(remoteLeftStatus == POZYX_SUCCESS && remoteRightStatus == POZYX_SUCCESS)
    {
      //Updating the buffers
      BufferAddVal(DistanceVals3, &Head_3, sumLeft/avgCount);
      BufferAddVal(DistanceVals4, &Head_4, sumRight/avgCount);
      
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
        #ifdef DEBUG
        Serial.println("DEVICE X > REMOTE X;  DEVICE Y < REMOTE Y");
        Serial.println("Quadrant 1");
        #endif
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
        #ifdef DEBUG
        Serial.println("DEVICE X > REMOTE X;  DEVICE Y > REMOTE Y");
        Serial.println("Quadrant 2");
        #endif
        //heading = 0 - (90 - abs(heading)) -90; //decreases when I need it to increase
        //Serial.println(heading);
        heading = abs(heading) + 90;   //working right now 4//11/19    
        quadrant = 2;     
      }
      
      //Quadrant 3 
      else if((device_pos.X < remote_pos.X ) && device_pos.Y > remote_pos.Y){       //(device_pos.X < remote_pos.X || (device_pos.X - remote_pos.X < 80)
        #ifdef DEBUG
        Serial.println("DEVICE X < REMOTE X;  DEVICE Y > REMOTE Y");
        Serial.println("Quadrant 3");
        #endif
        heading = heading + 180;
        //heading = 0-heading;  //need absolute negative
        //heading = heading + 102;
        //heading = heading+180;
        quadrant = 3;
      }
      //Quadrant 4 
      else if(device_pos.X < remote_pos.X && device_pos.Y < remote_pos.Y){
        #ifdef DEBUG
        Serial.println("DEVICE X < REMOTE X;  DEVICE Y < REMOTE Y");
        Serial.println("Quadrant 4");
        #endif
        
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

void printStatus()
{
#ifdef DEBUG
  if (deviceLeftStatus == POZYX_SUCCESS && deviceRightStatus == POZYX_SUCCESS)
    {
      Serial.print("Range: ");
      Serial.print("\tLeft ");
      Serial.print(getBuffAvg(DistanceVals1));
      Serial.print("\tRight ");
      Serial.print(getBuffAvg(DistanceVals2));
      Serial.print(" ");
      Serial.print(" ");
      Serial.print("X pos: ");
      Serial.print(device_pos.X);
      Serial.print(" ");
      Serial.print("Y pos: ");
      Serial.print(device_pos.Y);
      Serial.println(" ");
    }
    else{
        Serial.print("device error, connection to left and/or right anchor error");
      }
    //Check if we're using two pozyx tags on the robot:
#ifdef DUAL_POZYX
    if (remoteLeftStatus == POZYX_SUCCESS && remoteRightStatus == POZYX_SUCCESS)
    {
      Serial.print("Remote: ");
      Serial.print("Left: ");
      Serial.print(getBuffAvg(DistanceVals3));
      Serial.print("Right: ");
      Serial.print(getBuffAvg(DistanceVals4));
      Serial.print(" ");
      Serial.print("X pos: ");
      Serial.print(remote_pos.X);
      Serial.print(" ");
      Serial.print("Y pos: ");
      Serial.print(remote_pos.Y);
      Serial.print(" ");
      Serial.print("Heading: ");
      Serial.println(heading);
    }
#endif
#endif
}
#define RadToPi 57.2957795131

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

void printXYposition()
{
#ifdef XYpos
  Serial.print("Device X: ");
  Serial.print(device_pos.X);
  Serial.print("  Y: ");
  Serial.println(device_pos.Y);
  Serial.print("Remote X: ");
  Serial.print(remote_pos.X);
  Serial.print("  Y: ");
  Serial.println(remote_pos.Y);
  Serial.print("Center X: ");
  Serial.print(center.X);
  Serial.print(" Y: ");
  Serial.println(center.Y);
#endif
}

void calebrateGyro()
{
  long sumY = 0;
  for(int i = 0; i< Samples; i++)
  {
    Pozyx.regRead(POZYX_GYRO_X, (uint8_t*)&gyro_raw, 3*sizeof(int16_t));
    sumY += gyro_raw[1];
    //Keep track of the highest values
    
    if(gyro_raw[1]>highG_y)      highG_y=gyro_raw[1];
    //Keep track of the lowest values
    if(gyro_raw[1]<lowG_y)       lowG_y=gyro_raw[1];
   
  }
  offsetG_Y = sumY / Samples;
  highG_y -= offsetG_Y;
  lowG_y  -= offsetG_Y;
}

bool isWithinFloat(double sample, double lowBound, double highBound)
{
    return (sample > lowBound && sample < highBound);
}
