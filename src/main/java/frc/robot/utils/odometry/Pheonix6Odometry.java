// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry;

import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import frc.robot.constants.Constants;

public class Pheonix6Odometry {

  // Measuer<Unit> just means that signals can be a measurment of any unit 
  // (AngularVelocity, Angle, Position, etc.)
  StatusSignal<Measure<Unit>>[] signals;
  // Maps signals the 
  Map<StatusSignal<Measure<Unit>>,Double> signalToPositionMap;
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

      for (StatusSignal<Measure<Unit>> signal : signalToPositionMap.keySet()) {
        Double position = BaseStatusSignal.getLatencyCompensatedValue(
          signal, null /*TODO*/).magnitude();
        signalToPositionMap.put(signal, position);
      }
    }
  }

  public Pheonix6Odometry(){}

  // This function registers the signal (ensuring it always updates at the same 
  // time as other signals). This funciton returns a supplier that autmatically
  // returns the most recent value. Critically, Getting the supplier value of other 
  // registered signals will always be sampled from the same position in time.
  public Supplier<Double> registerSignal(StatusSignal<Measure<Unit>> statusSignal) {
    signalToPositionMap.put(statusSignal, statusSignal.getValueAsDouble());
    return () -> signalToPositionMap.get(statusSignal);
  }
}
