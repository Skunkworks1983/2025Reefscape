// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

/** Makes it super easy to tune, allows to change K values from smart dashboard*/
/** Makes it super easy to tune controllers, allows programmers to change constants from smart dashboard */

public class SmartPIDControllerTalonFX {

  public String name;
  public boolean smart;
  public TalonFX motor;
  public double lastKpValue;
  public double lastKiValue;
  public double lastKdValue;
  public double lastKfValue;

  public SmartPIDControllerTalonFX(double kp, double ki, double kd, double kf, String name,
    boolean smart, TalonFX motor) {

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
    slot0Configs.kV = lastKfValue;

    motor.getConfigurator().apply(slot0Configs);

    SmartDashboard.putNumber(name + " kp Value", kp);
    SmartDashboard.putNumber(name + " ki Value", ki);
    SmartDashboard.putNumber(name + " kd Value", kd);
    SmartDashboard.putNumber(name + " kf Value", kf);
  }

  public void updatePID() {
    // if we pass this test, we are smart, so we can save some bandwith by only
    // grabing the k values once
    if (!smart || !Constants.Drivebase.PIDs.SMART_PID_ENABLED) {
      return;
    }

    double currentKpValue = SmartDashboard.getNumber(name + " kp Value", lastKpValue);
    double currentKiValue = SmartDashboard.getNumber(name + " ki Value", lastKiValue);
    double currentKdValue = SmartDashboard.getNumber(name + " kd Value", lastKdValue);
    double currentKfValue = SmartDashboard.getNumber(name + " kf Value", lastKfValue);

    if (currentKpValue != lastKpValue || currentKiValue != lastKiValue
        || currentKdValue != lastKdValue || currentKfValue != lastKfValue) {

      lastKpValue = currentKpValue;
      lastKiValue = currentKiValue;
      lastKdValue = currentKdValue;
      lastKfValue = currentKfValue;

      Slot0Configs slot0Configs = new Slot0Configs();

      slot0Configs.kP = lastKpValue;
      slot0Configs.kI = lastKiValue;
      slot0Configs.kD = lastKdValue;
      slot0Configs.kV = lastKfValue;

      motor.getConfigurator().apply(slot0Configs);
    }

    SmartDashboard.putNumber(name + " Error", motor.getClosedLoopError().getValueAsDouble());
  }
}
