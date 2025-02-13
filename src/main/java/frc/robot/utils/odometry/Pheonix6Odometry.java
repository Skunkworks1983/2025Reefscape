// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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

  // For information on what a SignalValue is, see the comment in the SignalValue class.
  List<SignalValue<? extends Unit>> 
    SignalValueGroup = new ArrayList<>();
  Thread thread = new Thread(this::run);
  public AtomicBoolean isRunning;
  int failedUpdates;
  int vaildUpdates;
  public ReentrantReadWriteLock stateLock = new ReentrantReadWriteLock();


  // WARNNING: Due to Java generics issues, there must be at least one unchecked cast
  // (from <? extends Unit> to <Unit>). Three different mentors have been unable to solve this.
  // As long as we do not do anything crazy somewhere else, the unchecked cast /should/ never
  // result in a crash (according to Thomas).
  @SuppressWarnings("unchecked")
  public void run() {
    while(isRunning.get()){

      // Notes: waitForAll uses signals as an out param.
      List<BaseStatusSignal> signalsGroup = new ArrayList<>();
      for(SignalValue<? extends Unit> signalValue : SignalValueGroup){
        signalsGroup.add(signalValue.getStatusSignal());
        Optional<? extends StatusSignal<? extends Measure<? extends PerUnit<?, TimeUnit>>>> 
          optionalSignalSlope = signalValue.getStatusSignalSlope();

        if(optionalSignalSlope.isPresent()) {
          signalsGroup.add(optionalSignalSlope.get());
        }

      }
      StatusCode status = BaseStatusSignal.waitForAll(
        Constants.Pheonix6Odometry.updatesPerSecond,
        (BaseStatusSignal[])signalsGroup.toArray()
      );

      if(status.isOK()) {
        vaildUpdates++;
      } else {
        failedUpdates++;
      }

      for (SignalValue<? extends Unit> signalValue : SignalValueGroup) {
        // Note: This cast is unchecked! (This should never result in a crash)
        SignalValue<Unit> uncheckedSignalValue = (SignalValue<Unit>)signalValue;

        if(uncheckedSignalValue.getStatusSignalSlope().isPresent()) {
          double position = BaseStatusSignal.getLatencyCompensatedValue(
              uncheckedSignalValue.getStatusSignal(), uncheckedSignalValue.getStatusSignalSlope().get()
          ).magnitude();
          uncheckedSignalValue.setValue(position);
        } else { 
          double position = uncheckedSignalValue.getStatusSignal().getValueAsDouble();
          uncheckedSignalValue.setValue(position);
        }
      }
    }
  }

  // This function registers the signal and the rate of change of the first signal
  // (to predict the curent) position based on the change in time (ensuring it always
  // updates at the same time as other signals). This funciton returns a supplier that autmatically
  // returns the most recent sampled value. Importantly, getting the supplier value of other 
  // registered signals will always be sampled from the same position in time.
  public <U extends Unit> Supplier<Double> registerSignalWithLatencyCompensation(
    StatusSignal<Measure<U>> statusSignal,
    StatusSignal<Measure<PerUnit<U,TimeUnit>>> statusSignalRateOfChange) {
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.Pheonix6Odometry.updatesPerSecond, statusSignal);

    SignalValue<U> triplet = 
      new SignalValue<> (
        statusSignal, 
        Optional.of(statusSignalRateOfChange), 
        statusSignal.getValueAsDouble()
      );

    SignalValueGroup.add(triplet);

    return triplet::getValue;
  }

  // This function registers the signal (ensuring it always updates at the same 
  // time as other signals). This funciton returns a supplier that autmatically
  // returns the most recent sampled value. Importantly, getting the supplier value of other 
  // registered signals will always be sampled from the same position in time.
  public <U extends Unit> Supplier<Double> registerSignal(StatusSignal<Measure<U>> statusSignal) {
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.Pheonix6Odometry.updatesPerSecond, statusSignal);

    SignalValue<U> triplet = 
      new SignalValue<> (
        statusSignal, 
        Optional.empty(), 
        statusSignal.getValueAsDouble()
      );

    SignalValueGroup.add(triplet);
    return triplet::getValue;
  }
}
