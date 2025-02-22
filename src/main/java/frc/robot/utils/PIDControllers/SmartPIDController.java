// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.PIDControllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

/** Add your docs here. */
public class SmartPIDController extends PIDController implements SmartPIDInterface {

  public String name;
  public boolean smart;
  // Wpilib has no kf value in its PID controller, since kf is commonly used, i added a option to use kf in this PID controller
  private double kf;
  private boolean iskf;

  public SmartPIDController(double kp, double ki, double kd, String name, boolean smart) {
    super(kp, ki, kd);

    this.name = name;
    this.smart = smart;
    this.kf = 0.0;
    this.iskf = false;

    putValueSmartDashboard(name, "kp Value", kp);
    putValueSmartDashboard(name, "ki Value", ki);
    putValueSmartDashboard(name, "kd Value", kd);

  }

  public SmartPIDController(double kp, double ki, double kd, double kf, String name, boolean smart) {
    super(kp, ki, kd);

    this.name = name;
    this.smart = smart;
    this.kf = kf;
    this.iskf = true;

    putValueSmartDashboard(name, "kp Value", kp);
    putValueSmartDashboard(name, "ki Value", ki);
    putValueSmartDashboard(name, "kd Value", kd);
    putValueSmartDashboard(name, "kf Value", kf);

  }

  @Override
  public double calculate(double measurement) {

    if (smart && Constants.Drivebase.PIDs.SMART_PID_ENABLED) {
      super.setP(getValueFromSmartDashboard(name, "kp Value", super.getP()));
      super.setI(getValueFromSmartDashboard(name, "ki Value", super.getI()));
      super.setD(getValueFromSmartDashboard(name, "kd Value", super.getD()));
      updateKf();
    }

    double calculate = super.calculate(measurement) + kf;
    putValueSmartDashboard(name, "Measurement", measurement);
    putValueSmartDashboard(name, "Error", getPositionError());
    putValueSmartDashboard(name, "Setpoint", getSetpoint());
    putValueSmartDashboard(name, "Calculated Value", calculate);

    // Adding the kf value onto the calculation
    return calculate;
  }

  @Override
  public double calculate(double measurement, double setpoint) {

    if (smart && Constants.Drivebase.PIDs.SMART_PID_ENABLED) {
      super.setP(getValueFromSmartDashboard(name, "kp Value", super.getP()));
      super.setI(getValueFromSmartDashboard(name, "ki Value", super.getI()));
      super.setD(getValueFromSmartDashboard(name, "kd Value", super.getD()));
      updateKf();
    }

    // Adding the kf value onto the calculation
    double calculate = super.calculate(measurement, setpoint) + kf;

    putValueSmartDashboard(name, "Measurement", measurement);
    putValueSmartDashboard(name, "Error", getPositionError());
    putValueSmartDashboard(name, "Setpoint", getSetpoint());
    putValueSmartDashboard(name, "Calculated Value", calculate);

    return calculate;
  }

  private void updateKf() {
    if(iskf) {
      kf = getValueFromSmartDashboard(name, "kf Value", kf);
    }
  }

  public double getF() {
    return kf;
  }
}
