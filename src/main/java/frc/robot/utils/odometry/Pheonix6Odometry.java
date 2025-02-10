// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry;

import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import frc.robot.constants.Constants;
import frc.robot.utils.MutableTriplet;

public class Pheonix6Odometry {

  // Maps signals the 

  // Contains signal, signal rate of change, and measurement
  List<Pair<StatusSignal<Measure<Unit>>,Double>> signalValuePairList;

  // Contains signal, signal rate of change, and measurement
  List<MutableTriplet<StatusSignal<Measure<Unit>>,StatusSignal<Measure<PerUnit<Unit, TimeUnit>>>, Double>> 
    compensatedSignalValueGroup;
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

      for (MutableTriplet<StatusSignal<Measure<Unit>>,StatusSignal<Measure<PerUnit<Unit, TimeUnit>>>, Double>
        triplet : compensatedSignalValueGroup) {
        Double position = BaseStatusSignal.getLatencyCompensatedValue(
          triplet.getFirst(),  triplet.getSecond()).magnitude();
        triplet.setThird(position);
      }
    }
  }

  public Pheonix6Odometry(){}

  // This function registers the signal (ensuring it always updates at the same 
  // time as other signals). This funciton returns a supplier that autmatically
  // returns the most recent value. Critically, Getting the supplier value of other 
  // registered signals will always be sampled from the same position in time.
  public Supplier<Double> registerSignal(StatusSignal<Measure<Unit>> statusSignal) {
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.Pheonix6Odometry.updatesPerSecond, statusSignal);
    signalToPositionMap.put(statusSignal, statusSignal.getValueAsDouble());
    return () -> signalToPositionMap.get(statusSignal);
  }


  public Supplier<Double> registerSignalWithLatencyCompensation(
    StatusSignal<Measure<Unit>> statusSignal,
    StatusSignal<Measure<PerUnit<Unit, TimeUnit>>> statusSignalRateOfChange) {
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.Pheonix6Odometry.updatesPerSecond, statusSignal);

    MutableTriplet<
        StatusSignal<Measure<Unit>>,
        StatusSignal<Measure<PerUnit<Unit, TimeUnit>>>,
        Double
      > triplet = 
      new MutableTriplet<> (
        statusSignal, 
        statusSignalRateOfChange, 
        statusSignal.getValueAsDouble()
      );
    compensatedSignalValueGroup.add(
      triplet
    );

    return () -> triplet.getThird();
  }
}
