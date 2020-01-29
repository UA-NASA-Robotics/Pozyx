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
//  double tan_num = (double)(center_X) - (double)(mid_X);
//  double tan_den = (double)(center_Y) - (double)(mid_Y);
    double tan_num = (double)(device_pos_X)- (double)(remote_pos_X);
    double tan_den = (double)(device_pos_Y)- (double)(remote_pos_Y);
    heading = atan(tan_num / tan_den) * (180/PI);  //RadToPi defined in PozyxWrapper.
//    double sum = 0;
//    int i;
//    for(i=0;i<9;i++){
//        sum +=headingAvg[i];
//        headingAvg[i] = headingAvg[i+1];
//    }
//    headingAvg[9] = heading;
//    sum += headingAvg[9];
//    heading = sum/10;
    
  //Quad 1 -> 4
  if(device_pos_X > remote_pos_X && (device_pos_Y < remote_pos_Y))
  {

    heading = 180 + abs(heading);
    //heading = heading + 10;
//    if(heading >= 360)
//    {
//      heading = heading - 360;
//      quadrant = 1;
//    }
//    else
//    {
//      quadrant = 4;
//    }
//    
    quadrant = 3;
  }
  //Quad 2 -> 1
  else if(device_pos_X > remote_pos_X && (device_pos_Y > remote_pos_Y))
  {
    heading = 270 +(90 -heading); //quad1
    //heading = heading + 10;
    quadrant = 4;
  }
  //Quad 3 -> 2
  else if(device_pos_X < remote_pos_X && (device_pos_Y > remote_pos_Y))
  {
      heading = abs(heading);
      //heading = heading + 10;
      quadrant = 1;
  }
  //Quad 4 -> 3
  else if(device_pos_X < remote_pos_X && (device_pos_Y < remote_pos_Y))
  {
    heading = 90 - heading + 90 ;
  //heading = heading + 10;
    
    quadrant = 3;
  }
  Serial.print("Quad: ");Serial.println(quadrant);
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

    updateTagAngles(getBuffAvg(DistanceVals1), getBuffAvg(DistanceVals2), false);
  }

#ifdef DUAL_POZYX

  remoteLeftStatus = Pozyx.doRemoteRanging(remote_id, destination_id_1, &remoteLeftRange);
  remoteRightStatus = Pozyx.doRemoteRanging(remote_id, destination_id_2, &remoteRightRange);

  if(remoteLeftStatus == POZYX_SUCCESS && remoteRightStatus == POZYX_SUCCESS)
  {
    BufferAddVal(DistanceVals3, &Head_3, remoteLeftRange.distance);
    BufferAddVal(DistanceVals4, &Head_4, remoteRightRange.distance);

    updateTagAngles(getBuffAvg(DistanceVals3), getBuffAvg(DistanceVals4), true);
  }
  else
  {
    #ifdef DEBUG
    Serial.println("Error in 'UPDATESTATUS' for REMOTE. Cause: remoteLeftStatus a/or remoteRightStatus failure");
    #endif
  }


#endif
}

uint32_t PozyxWrapper::getBuffAvg(uint32_t *buff)
{
  unsigned long long sum = 0;
  for (int i = 0; i < AVERAGEAMOUNT; ++i)
  {
    sum += buff[i];
  }
  //sum = sum / AVERAGEAMOUNT;
  return(sum / AVERAGEAMOUNT);
}

int PozyxWrapper::updateTagAngles(uint32_t distanceVals1, uint32_t distanceVals2, bool remote_flag)
{
  
  double leftAngle = lawOfCOS(distanceVals1, ANCHORDISPLACEMENT, distanceVals2);
  
  if(remote_flag)
  {
    remote_pos_X = ((double)distanceVals1 * cos(leftAngle));
    remote_pos_Y = ((double)distanceVals1 * sin(leftAngle));
  }
  else
  {
    device_pos_X = ((double)distanceVals1 * cos(leftAngle));
    device_pos_Y = ((double)distanceVals1 * sin(leftAngle));
  }
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



  /*  OLD METHOD, for reference

  double magnitude = sqrt(pow(x_component, 2) + pow(y_component, 2));
  Now defined in Header

  
  double unit_x_component = x_component / magnitude;
  double unit_y_component = y_component / magnitude;
  
  double const VEC_ANGLE = 270; //FIX THIS; needs to be dynamic value;    270 deg = 4.71239 rad  for reference

  
  double heading_unit_vector_x = cos(VEC_ANGLE) * unit_x_component - sin(VEC_ANGLE) * unit_y_component;
  double heading_unit_vector_y = sin(VEC_ANGLE) * unit_x_component + cos(VEC_ANGLE) * unit_y_component;

  //compute robot centroid
  center_X = mid_X + MID_DIST * heading_unit_vector_x;
  center_Y = mid_Y + MID_DIST * heading_unit_vector_y;
  
  */
  
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

void PozyxWrapper::updateCoordinates()    //Needs re-worked. Will try something out once header + buffer working as intended
{
    double a = remoteRightRange.distance;
    double b = deviceRightRange.distance;
    double c = remoteLeftRange.distance;
    double d = deviceLeftRange.distance; 
    double u = ANCHORDISPLACEMENT;
    double w = TAG_DIST;


    /*
    //calculate X1 position (X coordinate of Pozyx shield on Arduino device on robot)
    devicePos.X = sqrt(-1*powerOfTwo(((b*b)-(u*u)-(d*d))/(-2*u))+(unsigned long)(d*d));
    //calculate X2 position (X coordinate of remote Pozyx beacon on robot)
    remotePos.X = (sqrt(-1*powerOfTwo(((a*a)-(u*u)-(c*c))/(-2*u))+(unsigned long)(c*c)));
    //calculate Y1 position (shield on arduino)
    devicePos.Y = ((d*d) - (b*b) + (u*u))/(2*u);
  
    //calculate Y2 position (remote beacon)
    remotePos.Y = ((c*c)-(a*a)+(u*u))/(2*u);
    */
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

double PozyxWrapper::lawOfCOS( uint32_t a, uint32_t b, uint32_t c)
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

int lastTime = 0;
void PozyxWrapper::printCH()
{
  #ifdef DEBUG
  Serial.print("Center X: ");
  Serial.println(center_X);
  Serial.print("Center Y: ");
  Serial.println(center_Y);
  Serial.print("HEADING: ");
  Serial.println(heading);
  Serial.print("GYRO ASSISTED HEADING: ");
  Serial.println(currentHeading);
  #endif
  
  // Transmitting the information to the master Controller
  
  #ifdef FASTTRANSFER
   
  //Sending the message periodically
  if(abs(lastTime - millis()) > 500)
  {
    // Loading the info for fasttransfer
    Send.ToSend(1, center_X);
    Send.ToSend(2, center_Y);
    Send.ToSend(3, heading);
    // Sending....
    Send.sendData(5);
    lastTime = millis();
  }
  #endif
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

void PozyxWrapper::adjustHeading()
{
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
