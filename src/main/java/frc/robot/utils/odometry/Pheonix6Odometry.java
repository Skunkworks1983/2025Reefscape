// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import frc.robot.constants.Constants;
import frc.robot.utils.SignalValue;

public class Pheonix6Odometry {

  // Maps signals the 

  // Contains signal, signal rate of change, and measurement
  // List<Pair<StatusSignal<? extends Unit>,Double>> signalValuePairList;

  // Contains signal, signal rate of change, and measurement
  List<SignalValue<?,?,?,?>> 
    compensatedSignalValueGroup = new ArrayList<>();
  Thread thread = new Thread(this::run);
  public AtomicBoolean isRunning;
  int failedUpdates;
  int vaildUpdates;
  public ReentrantReadWriteLock stateLock = new ReentrantReadWriteLock();


  public void run() {
    while(isRunning.get()){

      // Notes: waitForAll uses signals as an out param.
      List<BaseStatusSignal> allSignals = new ArrayList<>();
      for(SignalValue<?,?,?,?> signalValue : compensatedSignalValueGroup){
        allSignals.add(signalValue.getStatusSignal());
        StatusSignal<?> signal = signalValue.getStatusSignal();
        if(signal!=null) allSignals.add(signal);

      }
      StatusCode status = BaseStatusSignal.waitForAll(
        Constants.Pheonix6Odometry.updatesPerSecond,
        (BaseStatusSignal[])allSignals.toArray()
      );

      if(status.isOK()) {
        vaildUpdates++;
      } else {
        failedUpdates++;
      }

      // Not sure how to resolve warning
      for (SignalValue
        triplet : compensatedSignalValueGroup) { //<Unit, PerUnit<Unit, TimeUnit>, Measure<Unit>, Measure<PerUnit<Unit, TimeUnit>>>
        if(triplet.getstatusSignalRate() == null){
          Double position = triplet.getStatusSignal().getValueAsDouble();
          triplet.setValue(position);
        } else{ 
          Double position = BaseStatusSignal.getLatencyCompensatedValue(
          triplet.getStatusSignal(), triplet.getstatusSignalRate()).magnitude();
          triplet.setValue(position);
        }
      }
    }
  }

  public Pheonix6Odometry(){}

  // This function registers the signal (ensuring it always updates at the same 
  // time as other signals). This funciton returns a supplier that autmatically
  // returns the most recent value. Critically, Getting the supplier value of other 
  // registered signals will always be sampled from the same position in time.
  /*
  public Supplier<Double> registerSignal(StatusSignal<Measure<Unit>> statusSignal) {
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.Pheonix6Odometry.updatesPerSecond, statusSignal);
    signalToPositionMap.put(statusSignal, statusSignal.getValueAsDouble());
    return () -> signalToPositionMap.get(statusSignal);
  } 
  */


  public 
    <U extends Unit, U_PER_SEC extends PerUnit<U, TimeUnit>, MEAS extends Measure<U>, MEAS_PER_SEC extends Measure<U_PER_SEC>>
  Supplier<Double> registerSignalWithLatencyCompensation(
    StatusSignal<MEAS> statusSignal,
    StatusSignal<MEAS_PER_SEC> statusSignalRateOfChange) {
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.Pheonix6Odometry.updatesPerSecond, statusSignal);

    SignalValue<U, U_PER_SEC, MEAS, MEAS_PER_SEC> triplet = 
      new SignalValue<> (
        statusSignal, 
        statusSignalRateOfChange, 
        statusSignal.getValueAsDouble()
      );

    compensatedSignalValueGroup.add(triplet);

    return triplet::getValue;
  }

  public 
    <U extends Unit, U_PER_SEC extends PerUnit<U, TimeUnit>, MEAS extends Measure<U>, MEAS_PER_SEC extends Measure<U_PER_SEC>>
  Supplier<Double> registerSignal(
    StatusSignal<MEAS> statusSignal
  ) {
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.Pheonix6Odometry.updatesPerSecond, statusSignal);

    SignalValue<U, U_PER_SEC, MEAS, MEAS_PER_SEC> triplet = 
      new SignalValue<> (
        statusSignal, 
        null, 
        statusSignal.getValueAsDouble()
      );

    compensatedSignalValueGroup.add(triplet);

    return triplet::getValue;
  }
}
