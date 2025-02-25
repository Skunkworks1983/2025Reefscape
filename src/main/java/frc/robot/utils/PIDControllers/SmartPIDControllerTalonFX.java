// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.PIDControllers;

import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

/** Makes it super easy to tune, allows to change K values from smart dashboard*/
/** Makes it super easy to tune controllers, allows programmers to change constants from smart dashboard */

public class SmartPIDControllerTalonFX implements SmartPIDInterface {

  public String name;
  public boolean smart;
  public TalonFX motor;
  public double lastKpValue;
  public double lastKiValue;
  public double lastKdValue;
  public double lastKfValue;
  public Optional<Double> lastKvValue = Optional.empty();
  public Optional<Double> lastKaValue = Optional.empty();
  public Optional<Double> lastKsValue = Optional.empty();


  private SmartPIDControllerTalonFX(double kp, double ki, double kd, double kf, Optional<Double> kv, 
  Optional<Double> ka, Optional<Double> ks, String name, boolean smart, TalonFX motor) {
    this.motor = motor;
    this.name = name;
    this.smart = smart;

    lastKpValue = kp;
    lastKiValue = ki;
    lastKdValue = kd;
    lastKfValue = kf;

    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = lastKpValue;
    slot0Configs.kI = lastKiValue;
    slot0Configs.kD = lastKdValue;
    slot0Configs.kG = lastKfValue;

    if(kv.isPresent()) {
      putValueSmartDashboard(name, "kv Value", kv.get());
      slot0Configs.kV = kv.get();
      lastKvValue = kv;
    }

    if(ka.isPresent()) {
      putValueSmartDashboard(name, "ka Value", ka.get());
      slot0Configs.kA = ka.get();
      lastKaValue = ka;
    }

    if(ks.isPresent()) {
      putValueSmartDashboard(name, "ks Value", ks.get());
      slot0Configs.kS = ks.get();
      lastKsValue = ks;
    }

    motor.getConfigurator().apply(slot0Configs);

    putValueSmartDashboard(name, "kp Value", kp);
    putValueSmartDashboard(name, "ki Value", ki);
    putValueSmartDashboard(name, "kd Value", kd);
    putValueSmartDashboard(name, "kf Value", kf);
  }

  public SmartPIDControllerTalonFX(double kp, double ki, double kd, double kf, String name,
    boolean smart, TalonFX motor) {

    this(kp, ki, kd, kf, Optional.empty(), Optional.empty(), Optional.empty(), name, smart, motor);
  }

  public SmartPIDControllerTalonFX(double kp, double ki, double kd, double kf, double kv, double ka, double ks,  String name,
    boolean smart, TalonFX motor) {

    this(kp, ki, kd, kf, Optional.of(kv), Optional.of(ka), Optional.of(ks), name, smart, motor);
  }

  public void updatePID() {
    // if we pass this test, we are smart, so we can save some bandwith by only
    // grabing the k values once
    if (!smart || !Constants.Drivebase.PIDs.SMART_PID_ENABLED) {
      return;
    }

    double currentKpValue = getValueFromSmartDashboard(name, "kp Value", lastKpValue);
    double currentKiValue = getValueFromSmartDashboard(name, "ki Value", lastKiValue);
    double currentKdValue = getValueFromSmartDashboard(name, "kd Value", lastKdValue);
    double currentKfValue = getValueFromSmartDashboard(name, "kf Value", lastKfValue);

    Slot0Configs slot0Configs = new Slot0Configs();

    Optional<Double> currentKv = Optional.empty();
    if(lastKvValue.isPresent()) {
      currentKv = Optional.of(getValueFromSmartDashboard(name, "kv Value", lastKvValue.get()));
      if(currentKv.get() != lastKvValue.get()) {
        lastKvValue = currentKv;
        slot0Configs.kV = currentKv.get();
      }
    }

    Optional<Double> currentKa = Optional.empty();
    if(lastKaValue.isPresent()) {
      currentKa = Optional.of(getValueFromSmartDashboard(name, "ka Value", lastKaValue.get()));
      if(currentKa.get() != lastKaValue.get()) {
        lastKaValue = currentKa;
        slot0Configs.kA = currentKa.get();
      }
    }

    Optional<Double> currentKs = Optional.empty();
    if(lastKsValue.isPresent()) {
      currentKs = Optional.of(getValueFromSmartDashboard(name, "ks Value", lastKsValue.get()));
      if(currentKs.get() != lastKsValue.get()) {
        lastKsValue = currentKs;
        slot0Configs.kS = currentKs.get();
      }
    }

    if (currentKpValue != lastKpValue || currentKiValue != lastKiValue
        || currentKdValue != lastKdValue || currentKfValue != lastKfValue) {

      lastKpValue = currentKpValue;
      lastKiValue = currentKiValue;
      lastKdValue = currentKdValue;
      lastKfValue = currentKfValue;
    }

    slot0Configs.kP = lastKpValue;
    slot0Configs.kI = lastKiValue;
    slot0Configs.kD = lastKdValue;
    slot0Configs.kV = lastKfValue;

    motor.getConfigurator().apply(slot0Configs);

    putValueSmartDashboard(name, "Error", motor.getClosedLoopError().getValueAsDouble());
  }
}
