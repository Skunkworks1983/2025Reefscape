// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry;

import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;
import java.util.function.Consumer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.Constants;

/** Add your docs here. */
public class Pheonix6Odometry {

  StatusSignal<Angle>[] signals;
  // Maps signals the 
  Map<StatusSignal<Angle>,Consumer<Double>> signalConsumerMap;
  Thread thread = new Thread(this::run);
  public AtomicBoolean isRunning;
  int failedUpdates;
  int vaildUpdates;
  public ReentrantReadWriteLock stateLock = new ReentrantReadWriteLock();

  public void run() {
    while(isRunning.get()){

      // Notes: waitForAll uses signals as an out param.
      StatusCode status = BaseStatusSignal.waitForAll(
        Constants.Pheonix6Odometry.updatesPerSecond,
        signals
      );

      if(status.isOK()) {
        vaildUpdates++;
      } else {
        failedUpdates++;
      }
    }
  }

  public Pheonix6Odometry(List<StatusSignal<Angle>> signals){
    this.signals = signals.toArray(this.signals);

    BaseStatusSignal.setUpdateFrequencyForAll(
      Constants.Pheonix6Odometry.updatesPerSecond, 
      this.signals
    );
  }

  // This function registers the signal (ensuring it always updates at the same 
  // time as other signals). This funciton returns a supplier that autmatically
  // returns the most recent value.
  public Supplier<Double> registerSignal(StatusSignal statusSignal) {
    addSignal(statusSignal);
    double currentValue = 0;
    return () -> currentValue;
    // BaseStatusSignal.getLatencyCompensatedValue(statusSignal, null /*TODO*/).baseUnitMagnitude(); // TODO: check baseUnitMagnitude
  }

  private void addSignal(StatusSignal statusSignal) {


  }
}
