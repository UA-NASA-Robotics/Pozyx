/*
    PozyxWrapper.cpp - Custom implementation file for interacting 
    with Pozyx device
*/

#include "Arduino.h"
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <FastTransfer.h>
#include "PozyxWrapper.h"
#include <math.h>
//#define DEBUG
#define FASTTRANSFER
FastTransfer Send;

int receiveArray[3];
double headingAvg[10];
void PozyxWrapper::updateHeading()
{
  // CALCULATIONS FOR HEADING: CHECK WITH PEERS TO MAKE SURE THIS IS THE CORRECT METHOD

    double tan_num = (double)(device_pos_X)- (double)(remote_pos_X);
    double tan_den = (double)(device_pos_Y)- (double)(remote_pos_Y);
    heading =  (atan2(tan_num,tan_den) * RadToPi);  //RadToPi defined in PozyxWrapper.
    if(heading > 0)heading = 360 - heading;
    else heading = abs(heading);

}


void PozyxWrapper::PozyxBoot()
{
  #ifdef FASTTRANSFER
  Send.begin(Details(receiveArray), 8, false, &Serial);
  #endif
    if (Pozyx.begin() == POZYX_FAILURE) {
        #ifdef DEBUG
        Serial.println("ERROR: Unable to connect to POZYX shield");
        Serial.println("Reset required");
        #endif
        delay(100);
        abort();
    }
    // setting the remote_id back to NULL will use the local Pozyx
    if (!isRemote) {
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
}


void PozyxWrapper::updateStatus()
{
  deviceLeftStatus = Pozyx.doRanging(destination_id_1, &deviceLeftRange);
  deviceRightStatus = Pozyx.doRanging(destination_id_2, &deviceRightRange);

  if(deviceLeftStatus == POZYX_SUCCESS && deviceRightStatus == POZYX_SUCCESS)
  {
    BufferAddVal(DistanceVals1, &Head_1, deviceLeftRange.distance);
    BufferAddVal(DistanceVals2, &Head_2, deviceRightRange.distance);
  }


  remoteLeftStatus = Pozyx.doRemoteRanging(remote_id, destination_id_1, &remoteLeftRange);
  remoteRightStatus = Pozyx.doRemoteRanging(remote_id, destination_id_2, &remoteRightRange);

  if(remoteLeftStatus == POZYX_SUCCESS && remoteRightStatus == POZYX_SUCCESS)
  {
    BufferAddVal(DistanceVals3, &Head_3, remoteLeftRange.distance);
    BufferAddVal(DistanceVals4, &Head_4, remoteRightRange.distance);

    //updateTagAngles(getBuffAvg(DistanceVals3), getBuffAvg(DistanceVals4), true);
    
  }
  else
  {
    #ifdef DEBUG
    Serial.println("Error in 'UPDATESTATUS' for REMOTE. Cause: remoteLeftStatus a/or remoteRightStatus failure");
    #endif
  }

}

uint32_t PozyxWrapper::getBuffAvg(uint32_t *buff)
{
  unsigned long long sum = 0;
  for (int i = 0; i < AVERAGEAMOUNT; ++i)
  {
    sum += buff[i];
  }

  return (sum / AVERAGEAMOUNT);
}

void PozyxWrapper::BufferAddVal(uint32_t *buff, uint8_t *head, uint32_t val) //updates the value of buff
{
  buff[(*head)++] = val;
  if ( *head >= AVERAGEAMOUNT)
  {
    *head = 0;
  }
}

void PozyxWrapper::calculateCenter()
{
  //Code originally from David's 'newTestArena' code:
  //mid-point between Pozyx sensors on robot
  mid_X = (device_pos_X + remote_pos_X) / 2;
  mid_Y = (device_pos_Y + remote_pos_Y) / 2;

  //compute unit vector in direction of robot heading
  double x_component = (remote_pos_X - device_pos_X);     
  double y_component = (remote_pos_Y - device_pos_Y);
  
  center_X =  mid_X - ((MID_DIST * y_component) / TAG_DIST);
  center_Y =  mid_Y + ((MID_DIST * x_component) / TAG_DIST);
  
}


void PozyxWrapper::updateCoordinates()    //Needs re-worked. Will try something out once header + buffer working as intended
{
    double a = getBuffAvg(DistanceVals4);//remoteRightRange.distance;
    double b = getBuffAvg(DistanceVals2);//deviceRightRange.distance;
    double c = getBuffAvg(DistanceVals3);//remoteLeftRange.distance;
    double d = getBuffAvg(DistanceVals1);//deviceLeftRange.distance; 
    double u = ANCHORDISPLACEMENT;
    double w = TAG_DIST;

    //calculate Y1 position (Y coordinate of Pozyx shield on Arduino device on robot)
    device_pos_Y = sqrt(-1*powerOfTwo(((b*b)-(u*u)-(d*d))/(-2*u))+(unsigned long)(d*d));
    //calculate Y2 position (Y coordinate of remote Pozyx beacon on robot)
    remote_pos_Y = (sqrt(-1*powerOfTwo(((a*a)-(u*u)-(c*c))/(-2*u))+(unsigned long)(c*c)));
    //calculate X1 position (shield on arduino)
    device_pos_X = ((d*d) - (b*b) + (u*u))/(2*u);
    //calculate X2 position (remote beacon)
    remote_pos_X = ((c*c)-(a*a)+(u*u))/(2*u);
}

double PozyxWrapper::calculateX1Position()
{
  double b = deviceRightRange.distance;
  double d = deviceLeftRange.distance;
  double u = ANCHORDISPLACEMENT;
  double w = TAG_DIST;

  unsigned long squareThis = ((b*b)-(u*u)-(d*d))/(-2*u);
  unsigned long squared = powerOfTwo(squareThis);
  unsigned long secVal = -1*squared+(unsigned long)(d*d);
  #ifdef DEBUG
  Serial.println(sqrt(-1*powerOfTwo(((b*b)-(u*u)-(d*d))/(-2*u))+(unsigned long)(d*d)));
  #endif
  return sqrt(secVal);
}

double PozyxWrapper::calculateX2Position()
{ 
  double a = remoteRightRange.distance;
  double c = remoteLeftRange.distance;
  double u = ANCHORDISPLACEMENT;
  double w = TAG_DIST;
  
  unsigned long squareThis = ((a*a)-(u*u)-(c*c))/(-2*u);
  unsigned long squared = powerOfTwo(squareThis);
  unsigned long secVal = -1*squared+(unsigned long)(c*c);
#ifdef DEBUG
  Serial.println(sqrt(-1*powerOfTwo(((a*a)-(u*u)-(c*c))/(-2*u))+(unsigned long)(c*c)));
  #endif
  return sqrt(secVal);
  
}

double PozyxWrapper::calculateY1Position()
{
  double d = deviceLeftRange.distance;  
  double b = deviceRightRange.distance;
  double u = ANCHORDISPLACEMENT;
  
  return(((d*d) - (b*b) + (u*u))/(2*u));
}

double PozyxWrapper::calculateY2Position()
{
  double a = remoteRightRange.distance;
  double c = remoteLeftRange.distance;
  double u = ANCHORDISPLACEMENT;
  
  return(((c*c)-(a*a)+(u*u))/(2*u));
}


unsigned long PozyxWrapper::powerOfTwo(unsigned long x)
{
  return x*x;  
}


void PozyxWrapper::printGyro()
{
  Serial.print("GYRO ASSISTED HEADING: ");
  Serial.println(currentHeading);
}

// GYRO IMPLEMENTATION //

void PozyxWrapper::calibrateGyro()
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
unsigned int lastFTMillis;
void PozyxWrapper::adjustHeading()
{
  if (abs(millis() - previousMillis) > interval) 
      {
          //update pozyx angle
          updateHeading();
          previousMillis = millis();
      

      lastyAngle = yAngle;
      //obtain the gyro offset
      if((abs(lastMillis - millis())) > 10)
      {
          Pozyx.regRead(POZYX_GYRO_X, (uint8_t*)&gyro_raw, 3*sizeof(int16_t));
          gyroYDPS =  gyro_raw[1]- offsetG_Y;
          if(isWithinFloat(gyroYDPS,lowG_y*1.1,highG_y*1.1)){
             gyroYDPS=0;  
             //yAngle = heading;
          }
          yAngle = ((double)gyroYDPS*SCALING_GYRO * ((millis()-lastMillis)/1000.0)/16);
          
          lastMillis = millis();
      } 
       //add the gyro offset to the current heading
      currentHeading = (yAngle) + heading;
      #ifdef DEBUG
       Serial.print("Y-Val: ");
      Serial.println(yAngle);
      Serial.print("CURRENT HEADING: ");
      Serial.println(currentHeading);
      Serial.print("calculated heading: ");
      Serial.println(heading); //we're good!
      #endif 
      }

}
int lastTime = 0;
void PozyxWrapper::printCH()
{
  
  #ifdef DEBUG
  //printBasicXY();
  Serial.print("Center X: ");
  Serial.println(center_X);
  Serial.print("Center Y: ");
  Serial.println(center_Y);
  Serial.print("HEADING: ");
  Serial.println(heading);
  Serial.print("GYRO ASSISTED HEADING: ");
  Serial.println(currentHeading);
  #endif
        //fastransfer send data to master
      // master address is 4
      // to load data in to FT send buffer use toSend(what index, data)
      // to send the data use sendData(address of receiver)
      // we need to send this info at 1 sec interval
      // use an if statemnet that will compare to current time to last time we send data
      // you want to use millis()

  
  #ifdef FASTTRANSFER
   
  //Sending the message periodically
  if(abs(lastTime - millis()) > 200)
  {
    // Loading the info for fasttransfer
    Send.ToSend(1, center_X);
    Send.ToSend(2, center_Y);
    Send.ToSend(3, currentHeading);
    // Sending....
    Send.sendData(5);
    lastTime = millis();
  }
  #endif
}

bool PozyxWrapper::isWithinFloat(double sample, double lowBound, double highBound)
{
    return (sample > lowBound && sample < highBound);
}

//NOT REALLY NEEDED CURRENTLY, REMOVE SOON IF NO USAGE
int variance(uint32_t a[], int n) 
{ 
    // Compute mean (average of elements) 
    int sum = 0; 
    for (int i = 0; i < n; i++) 
        sum += a[i]; 
    double mean = (double)sum / (double)n; 
    
    // Compute sum squared  
    // differences with mean. 
    double sqDiff = 0; 
    for (int i = 0; i < n; i++)  
        sqDiff += (a[i] - mean) *  
                  (a[i] - mean); 
    return sqDiff / n; 
}


void PozyxWrapper::printBasicXY()
{
  //Two methods of printing; the first one is faster
  //calculateX1Position();  //Values of X1, X2 are printed during these function calls. The returned values are the 
  //double X2 = calculateX2Position();

  /*Serial.print("X1: ");
  calculateX1Position();
  Serial.print("X2: ");
  calculateX2Position();
  */
  #ifdef DEBUG
  Serial.print("Device X: ");
  Serial.print(device_pos_X);
  Serial.print("  Y: ");
  Serial.println(device_pos_Y);
  Serial.print("Remote X: ");
  Serial.print(remote_pos_X);
  Serial.print("  Y: ");
  Serial.println(remote_pos_Y);
  Serial.print("Center X: ");
  Serial.print(center_X);
  Serial.print(" Y: ");
  Serial.println(center_Y); 
  #endif
  
}

/*
void PozyxWrapper::updateDistances()
{
    deviceLeftStatus = Pozyx.doRanging(destination_id_1, &deviceLeftRange);
    deviceRightStatus = Pozyx.doRanging(destination_id_2, &deviceRightRange);
    remoteLeftStatus = Pozyx.doRemoteRanging(remote_id, destination_id_1, &remoteLeftRange);
    remoteRightStatus = Pozyx.doRemoteRanging(remote_id, destination_id_2, &remoteRightRange);
    if (deviceLeftStatus == POZYX_SUCCESS && deviceRightStatus == POZYX_SUCCESS)
    {
        //Updating the buffers
        BufferAddVal(deviceLeftDistanceBuffer, &bufferHead.deviceLeft, deviceLeftRange.distance); 
        BufferAddVal(deviceRightDistanceBuffer, &bufferHead.deviceRight, deviceRightRange.distance);
    }
#ifdef isRemote
    if (remoteLeftStatus == POZYX_SUCCESS && remoteRightStatus == POZYX_SUCCESS)
    {
        BufferAddVal(remoteLeftDistanceBuffer, &bufferHead.remoteLeft, remoteLeftRange.distance);
        BufferAddVal(remoteRightDistanceBuffer, &bufferHead.remoteRight, remoteRightRange.distance);
    }
#endif 
}
*/
