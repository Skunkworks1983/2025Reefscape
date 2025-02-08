// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import frc.robot.constants.Constants;

/** Add your docs here. */
public class Pheonix6Odometry {

  BaseStatusSignal[] signals;
  Thread thread = new Thread(this::run);

  public void run() {
    StatusCode status = BaseStatusSignal.waitForAll(
        Constants.Pheonix6Odometry.updatesPerSecond, 
        signals
      );
  }

  public Pheonix6Odometry(){

  }

  public void addSignal(StatusSignal<Double> signal){
    
      BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.Pheonix6Odometry.updatesPerSecond, signals);
  }
}
