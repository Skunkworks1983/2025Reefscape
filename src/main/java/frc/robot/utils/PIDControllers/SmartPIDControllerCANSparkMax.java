// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.PIDControllers;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.Constants;

/** Add your docs here. */
public class SmartPIDControllerCANSparkMax implements SmartPIDBase {

  public String name;
  public boolean smart;
  public SparkMax motor;
  public double lastKpValue;
  public double lastKiValue;
  public double lastKdValue;
  public double lastKfValue;
  public SparkMaxConfig config;

  public SmartPIDControllerCANSparkMax(double kp, double ki, double kd, double kf, String name,
      boolean smart, SparkMax motor) {

    this.motor = motor;
    this.name = name;
    this.smart = smart;
    config = new SparkMaxConfig();

    lastKpValue = kp;
    lastKiValue = ki;
    lastKdValue = kd;
    lastKfValue = kf;

    config.closedLoop.pidf(kp, ki, kd, kf);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    putValueSmartDashboard(name, "kp Value", kp);
    putValueSmartDashboard(name, "ki Value", ki);
    putValueSmartDashboard(name, "kd Value", kd);
    putValueSmartDashboard(name, "kf Value", kd);
  }

  public void updatePID() {
    // if we pass this test, we are smart, so we can save some bandwith by only
    // grabing the k values once
    if (!smart || !Constants.Testing.SMART_PID_ENABLED) {
      return;
    }

    double currentKpValue = getValueFromSmartDashboard(name, "kp Value", lastKpValue);
    double currentKiValue = getValueFromSmartDashboard(name, "ki Value", lastKiValue);
    double currentKdValue = getValueFromSmartDashboard(name, "kd Value", lastKdValue);
    double currentKfValue = getValueFromSmartDashboard(name, "kf Value", lastKfValue);

    if (currentKpValue != lastKpValue || currentKiValue != lastKiValue
        || currentKdValue != lastKdValue || currentKfValue != lastKfValue) {

      lastKpValue = currentKpValue;
      lastKiValue = currentKiValue;
      lastKdValue = currentKdValue;
      lastKfValue = currentKfValue;

      config.closedLoop.pidf(lastKpValue, lastKiValue, lastKdValue, lastKfValue);
      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }
}
