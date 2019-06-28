//functional_ranging.h
#ifndef FUNCTIONAL_RANGING
#define FUNCTIONAL_RANGING

//**************************************************************************************************
//void function to boot up the Pozyx, should be called in Setup
void pozyxBoot(bool remote, bool remote_id, uint16_t destination_id_1, uint16_t destination_id_2, uint8_t ranging_protocol)
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
}
//**********************************************************************************************

//passes in pointer to buffer
//returns average of the buffer (dependent on AverageAmount)
uint32_t getBuffAvg(uint32_t *buff, int average_amount)
{
  unsigned long long sum = 0;
  for (int i = 0; i < average_amount; i++)
  {
    sum += buff[i];
  }
  sum = sum / average_amount;
  return sum;
}

//updateStatus
void updateStatus()
{
    //perform loop operations (put in one void function)
    // let's perform ranging with the destination
    //ranging with device
    deviceLeftStatus = Pozyx.doRanging(destination_id_1, &deviceLeftRange);
    deviceRightStatus = Pozyx.doRanging(destination_id_2, &deviceRightRange);
    //deviceLeftStatus = Pozyx.doRemoteRanging(remote_id2, destination_id_1, &deviceLeftRange);
    //deviceRightStatus = Pozyx.doRemoteRanging(remote_id2, destination_id_2, &deviceRightRange);
    if (deviceLeftStatus == POZYX_SUCCESS && deviceRightStatus == POZYX_SUCCESS)
    { //not getting here
      
      //Updating the buffers
      BufferAddVal(DistanceVals1, &Head_1, deviceLeftRange.distance); 
      BufferAddVal(DistanceVals2, &Head_2, deviceRightRange.distance);
      //Update tag angle, x, y pos
      updateTagAngles(getBuffAvg(DistanceVals1), getBuffAvg(DistanceVals2), false); //passing 0 because it's not the remote device
      //updateTagAngles(deviceLeftRange.distance, deviceRightRange.distance, 0);
    }
    //ranging with remote device
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

//not really needed currently, just helpful to have in case I want to print out the variance
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

//not really needed currently, just helpful to have in case I want to print out the std. dev.
int standardDeviation(uint32_t arr[],  
                         int n) 
{ 
    return sqrt(variance(arr, n)); 
} 
#endif
