/*
    PozyxWrapper.cpp - Custom implementation file for interacting 
    with Pozyx device
*/

#include "Arduino.h"
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include "PozyxWrapper.h"

PozyxWrapper::PozyxWrapper()
{
        //Bootup protocol
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
}

PozyxWrapper::updateDistances()
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

void PozyxWrapper::updateHeading()
{
	//Calculations for heading
    double tan_num = (double)(device_pos.Y) - (double)(remote_pos.Y);
    double tan_den = (double)(remote_pos.X) - (double)(device_pos.X);
    heading = atan(tan_num/tan_den)*RadToPI;
	
	//Update quadrant
	//Q1
	if(device_pos.X > remote_pos.X && (device_pos.Y < remote_pos.Y))
	{
		heading = abs(90 - heading);
		heading = heading - 10;
		quadrant = 1;
	}
	//Q2
	else if (device_pos.X > remote_pos.X && (device_pos.Y > remote_pos.Y))
	{
		heading = abs(heading) + 90;
		quadrant = 2;
	}
	//Q3
	else if (device_pos.X < remote_pos.X && (device_pos.Y > remote_pos.Y))
	{
		heading = heading + 180;
		quadrant = 3;
	}
	//Q4 (or Q1)
	else if(device_pos.X < remote_pos.X && (device_pos.Y < remote_pos.Y))
	{
		heading = abs(heading);
		heading = heading + 10;
		heading = heading + 270;
		if(heading >= 360)
		{
			heading = heading - 360;
			quadrant = 1;
		}
		else
		{
			quadrant = 4;
		}
	}
}

//function for adding new value to the buffer (circular buffer)
void PozyxWrapper::BufferAddVal(uint32_t *buff, uint8_t *head, uint32_t val) //updates the value of buff
{
  buff[(*head)++] = val;
  if ( *head >= AVERAGEAMOUNT)
  {
    *head = 0;
  }
}

//Passes in pointer to buffer
//Returns avg of the buffer (from AverageAmount)
uint32_t PozyxWrapper::getBuffAvg(uint32_t *buff)
{
  unsigned long long sum = 0;
  for (int i = 0; i < AVERAGEAMOUNT; i++)
  {
    sum += buff[i];
  }
  sum = sum / AVERAGEAMOUNT;
  return sum;
}

void PozyxWrapper::updateCoordinates()
{
    double a = remoteRightRange.distance;
    double b = deviceRightRange.distance;
    double c = remoteLeftRange.distance;
    double d = deviceLeftRange.distance; 
    double u = ANCHORDISPLACEMENT;
    double w = TAG_DIST;

    //calculate X1 position (X coordinate of Pozyx shield on Arduino device on robot)
    devicePos.X = sqrt(-1*powerOfTwo(((b*b)-(u*u)-(d*d))/(-2*u))+(unsigned long)(d*d));

    //calculate X2 position (X coordinate of remote Pozyx beacon on robot)
    remotePos.X = (sqrt(-1*powerOfTwo(((a*a)-(u*u)-(c*c))/(-2*u))+(unsigned long)(c*c)));

	//calculate Y1 position (shield on arduino)
	devicePos.Y = ((d*d) - (b*b) + (u*u))/(2*u);
	
	//calculate Y2 position (remote beacon)
	remotePos.Y = ((c*c)-(a*a)+(u*u))/(2*u);
	
}

void updateStatus()
{

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

/////////////////////////////////////////////////////////
//Stuff that prints to terminal; statuses, coords etc.
/////////////////////////////////////////////////////////

void PozyxWrapper::printStatus()
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

void PozyxWrapper::printXYposition()
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


/////////////////////////////////////////////
//Calculations, mathematical laws, etc. here
/////////////////////////////////////////////

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

unsigned long PozyxWrapper::powerOfTwo(unsigned long x)
{
	return(x*x);
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










		

