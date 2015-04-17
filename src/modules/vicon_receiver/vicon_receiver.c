/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
 
/** 
 * @file vicon_receiver.c
 * 
 * Receives Vicon position/attitude data through SERIAL 5.
 *
 * Publishes VISION_POSITION_ESTIMATE topic via uORB.
 *
 * @author Daniel McArthur <mcarthur.dr@gmail.com>
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/vision_position_estimate.h>
#include <drivers/drv_hrt.h>

__EXPORT int vicon_receiver_main(int argc, char *argv[]);

int vicon_receiver_main(int argc, char *argv[])
{
  // Publish Vicon Position in vision_position_estimate topic
  struct vision_position_position_s vicon_position;
  memset(&vicon_position, 0, sizeof(vicon_position));
  int vicon_pub_fd = 0;

  // Variables to store Vicon position/velocity data
  int scanCheck = 0;        // Validate incoming data
  float x = 0;
  float y = 0;
  float z = 0;
  float vx = 0;
  float vy = 0;
  float vz = 0;   
  float roll = 0;
  float pitch = 0;
  float yaw = 0;            
  float prevX = 0;
  float prevY = 0;
  float prevZ = 0;
  char id;          // ID of this vehicle
  char buffer[80];  // Maximum incoming message size (in bytes)

  // Variables for time keeping
  uint64_t prevT = 0;
  uint64_t curT = 0;    // units: microseconds
  float dT = 0;         // units: seconds
  bool firstTime = true;

  // DEBUG Velocity calculations (make sure have good Hz)
  uint64_t lastCheck = hrt_absolute_time();
  uint64_t thisCheck = hrt_absolute_time();
  int loopCount = 0;
  int xCount = 0;
  int yCount = 0;
  int zCount = 0;

  // Main Loop (Read from serial 5 and publish GPS/Vicon data)
  while (1)
  {
    // Read from serial 5 port (get Vicon data through XBee)
    gets(buffer);

    // DEBUG - Print out 
    thisCheck = hrt_absolute_time();
    if((thisCheck - lastCheck) > 5000000) // update every 5 seconds
    {
      lastCheck = thisCheck;
      printf("Valid data points received in last 5 seconds: %d\n(X,Y,Z): (%d,%d,%d)\n",
          loopCount,xCount,yCount,zCount);
      loopCount = 0;
      xCount = 0;
      yCount = 0;
      zCount = 0;
    }

    // ************* Check for initialization message: 'BEGIN' *************
    if(strcmp(buffer,"BEGIN") == 0)
    {
      //printf("%s\n",buffer);  // Establish 1st communication
      continue;
    }
    else if(strcmp(buffer,"NODATA") == 0)
    {
      //printf("NO DATA!\n");
      continue;        // Ignore packets with no useful data
    }
    else
    {
      // Parse Vicon/GPS data from XBee into variables
      //scanCheck = sscanf(buffer,"%c,%ld,%ld,%ld,%f,%f,%f",
      //        &id,&lat,&lon,&alt,&x,&y,&yaw); // Read gps/vicon coordinates

      // Parse Vicon data from XBee into variables
      scanCheck = sscanf(buffer,"%c,%f,%f,%f,%f,%f,%f",
              &id,&x,&y,%z,&roll,&pitch,&yaw); // Read vicon coordinates

      //DEBUG - return original
      //printf("%1c,%11ld,%11ld,%5ld,%6.3f,%6.3f,%6.3f,%6.3f\n",
      //    id,lat,lon,alt,(double)x,(double)y,(double)z,(double)yaw);

      //printf("%1c,%11ld,%11ld,%5ld\n",id,lat,lon,alt); //DEBUG - return original
      if(scanCheck < 7)
      {
        printf("sscanf failed on \"%s\"! (returned: %d)\n",buffer,scanCheck);
        continue; // Don't use bad data!
      }
      if(id != 'A')
      {
        // Do nothing unless the message is for this Pixhawk
        continue;
      }

      //DEBUG
      loopCount++;  // Count how many valid data points received

      //DEBUG - Validate input from sscanf
      float maxPos = 7000;  // will never be more than 7 meters from center
      float maxAlt = 5000;  // will never be more than 5 meters above ground
      if(x < -maxPos || x > maxPos)
      {
        printf("X OUT OF RANGE: %.4f\n",(double)x);
        continue;
      }
      if(y < -maxPos || y > maxPos)
      {
        printf("Y OUT OF RANGE: %.4f\n",(double)y);
        continue;
      }
      if(z < -maxAlt || z > maxAlt)
      {
        printf("Z OUT OF RANGE: %.4f\n",(double)z);
        continue;
      }

      // Convert position from mm to meters (and convert to NED)
      x /= 1000.0f;
      y /= -1000.0f;  // Y is west... need to invert to get 'East' for NED
      z /= -1000.0f;  // Z is down in NED frame

      // Convert attitude from VICON_XYZ frame to NED frame
      pitch = -pitch;
      yaw = -yaw;

      curT = hrt_absolute_time(); // Get current timestamp

      // Calculate velocity AFTER first pass
      if(!firstTime)
      {
        dT = (curT - prevT) / 1000000.0f;
        if(dT < .000001f)
        {
          // Something went wrong... prevent divide by 0
          printf("ERROR: dT == 0!!!\n"); // DEBUG
          vx = 0;
          vy = 0;
          vz = 0;
        }
        else
        {
          // Calculate velocity with a backward difference approximation
          vx = (x - prevX) / dT;
          vy = (y - prevY) / dT;
          vz = (z - prevZ) / dT;
        }
        //DEBUG - Count how many times threshold velocity is exceeded in each direction
        float tv = 1; // threshold velocity
        if(vx > tv || vx < -1*tv)
        {
          xCount++;
          //printf("X velocity: %.3f\n",(double)vx);
          //printf("x: %.4f\tprevX: %.4f\ndT: %.7f\n",(double)x,(double)prevX,(double)dT);
        }
        if(vy > tv || vy < -1*tv)
        {
          yCount++;
          //printf("Y velocity: %.3f\n",(double)vy);
          //printf("y: %.4f\tprevY: %.4f\ndT: %.7f\n",(double)y,(double)prevY,(double)dT);
        }
        if(vz > tv || vz < -1*tv)
        {
          zCount++;
          //printf("Z velocity: %.3f\n",(double)vz);
          //printf("z: %.4f\tprevZ: %.4f\ndT: %.7f\n",(double)z,(double)prevZ,(double)dT);
        }
      }
      else
      {
        // Initialize variables on first pass through
        firstTime = false;
        prevT = curT;
        vx = 0;
        vy = 0;
        vz = 0;
      }

      // Set previous variables for next iteration
      prevT = curT;
      prevX = x;
      prevY = y;
      prevZ = z;

      // Use the component ID to identify the vision sensor
      vicon_position.id = unsigned int 123; // A BOGUS ID

      // Not sure what the difference is between timestamps
      vicon_position.timestamp_boot = hrt_absolute_time(); // Synced time
      vicon_position.timestamp_computer = hrt_absolute_time(); 
      
      // Position
      vicon_position.x = x;
      vicon_position.y = y;
      vicon_position.z = z;

      // Velocity
      vicon_position.vx = vx;
      vicon_position.vy = vy;
      vicon_position.vz = vz;

      // Attitude
      math::Quaternion q;
      q.from_euler(roll, pitch, yaw);

      vicon_position.q[0] = q(0);
      vicon_position.q[1] = q(1);
      vicon_position.q[2] = q(2);
      vicon_position.q[3] = q(3);

      // Publish/Advertise Vicon Position (VISION_POSITION_ESTIMATE)
      if (vicon_pub_fd != 0)
      {
        orb_publish( ORB_ID(vision_position_estimate), vicon_pub_fd, &vicon_position);
      }
      else // On the first pass, advertise
      {
        vicon_pub_fd = orb_advertise( ORB_ID(vision_position_estimate), &vicon_position);
      }

    }
  } // end while(1)


  return 0;
}