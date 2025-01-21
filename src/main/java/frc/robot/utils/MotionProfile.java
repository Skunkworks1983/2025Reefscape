// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;

// Visulisation of the motion profile based on maxVelocity and acceleration
//        ┄┄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄┄┄ max velocity
// vel ↑️   ▟████████████████████▙ 
// time → ▟██████████████████████▙ slope = max acceleration
//        └┬┘└────────┬───────┘└┬┘
//    acceleration cruise deceleration
//
// If there is not enough time to fully acclerate and decelerate, it will not
// cruise at maximum velocity.
//
//     ┄┄┄┄┄┄┄┄┄ Max velocity
// vel ↑️   ▟▙ 
// time → ▟██▙ 

public class MotionProfile {
  double maxVelocity;
  double acceleration;
  double timeToAccelerate;
  double distanceToAccelerate;

  // All velocity is in meters/second
  // All acceleration is in meters/second/second
  public MotionProfile(double maxVelocity, double acceleration) {
    this.maxVelocity = maxVelocity;
    this.acceleration = acceleration;

    // seconds = (velocity) / (velocity/second)
    // If the time required to get to maximum velocity is greater than
    // the time that has passed, the robot is not at max velocity.
    // Time to accelerate is equal to the time to decelerate.
    timeToAccelerate = maxVelocity / acceleration;

    // meters = (meters/second) * seconds. Area is width * height, so
    // meters is the area under the curve of the trapazoidal motion profile.
    // In geometry, area of the triangle is width * height / 2. In the context
    // of this of the motion profile, meters = timeToAccelerate * maxVelocity.
    distanceToAccelerate = timeToAccelerate * maxVelocity;
  }

  // This function calculates position based on position using the trapazoidal
  // motion profile. In other words, it finds the area between x=0 and timeElapsed.
  public double calculatePosition(double timeElapsed, double totalDistance) {

    // If the profile will not even reach max velocity, the following
    // two values will be negetive. It will instead be the distance or time
    // that would be required to travel in order to reach max velocity.
    double distanceAtMaxVelocity = totalDistance - distanceToAccelerate * 2;
    
    // (meters) / (meters/second) = seconds
    double timeAtMaxVelocity = distanceAtMaxVelocity / maxVelocity;
    boolean willReachMaxSpeed = distanceAtMaxVelocity < 0.0;
    double totalTime = timeToAccelerate * 2 + timeAtMaxVelocity;

    if(willReachMaxSpeed) {
      double timeAtReachMaxVelocity = timeToAccelerate;
      double timeAtLeaveMaxVelocity = totalTime - timeToAccelerate;
      
      if(timeElapsed < timeAtReachMaxVelocity) {
        return Math.pow(timeElapsed, 2.0) * .5 * acceleration; 
      } else if(timeElapsed < timeAtLeaveMaxVelocity) {
        return distanceToAccelerate + maxVelocity * 
          (timeElapsed - timeAtReachMaxVelocity);
      } else {
        return distanceToAccelerate + distanceAtMaxVelocity + maxVelocity - 
          Math.pow(timeElapsed - timeAtLeaveMaxVelocity, 2.0) 
          * .5 * acceleration;
      }
    }
    else {
      if (timeElapsed < .5 * totalTime) {
        return Math.pow(timeElapsed, 2.0) * .5 * acceleration; 
      } else {
        return Math.pow(totalTime * .5, 2.0) * .5 * acceleration 
          + maxVelocity - Math.pow((timeElapsed - totalTime * .5) * .5, 2.0)
          * .5 * acceleration;  
      }
    }
  }
}
