// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.PIDControllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

/** Add your docs here. */
public class SmartPIDController extends PIDController {

    public String name;
    public boolean smart;
    public String setpointLabel;
    public String errorLabel;

    public SmartPIDController(double kp, double ki, double kd, String name, boolean smart) {
        super(kp, ki, kd);

        this.name = name;
        this.smart = smart;

        SmartDashboard.putNumber(name + " kp Value", kp);
        SmartDashboard.putNumber(name + " ki Value", ki);
        SmartDashboard.putNumber(name + " kd Value", kd);

        setpointLabel = name + " Setpoint";
        errorLabel = name + " Error";
    }

    @Override
    public double calculate(double measurement) {

        if (smart && Constants.Drivebase.PIDs.SMART_PID_ENABLED) {
            super.setP(SmartDashboard.getNumber(name + " kp Value", super.getP()));
            super.setI(SmartDashboard.getNumber(name + " ki Value", super.getI()));
            super.setD(SmartDashboard.getNumber(name + " kd Value", super.getD()));
        }

        double calculate = super.calculate(measurement);
        SmartDashboard.putNumber(name + " Measurement", measurement);
        SmartDashboard.putNumber(errorLabel, getPositionError());
        SmartDashboard.putNumber(setpointLabel, getSetpoint());
        SmartDashboard.putNumber(name + " Calculated Value", calculate);
        
        return calculate;
    }

    @Override
    public double calculate(double measurement, double setpoint) {

        if (smart && Constants.Drivebase.PIDs.SMART_PID_ENABLED) {
            super.setP(SmartDashboard.getNumber(name + " kp Value", super.getP()));
            super.setI(SmartDashboard.getNumber(name + " ki Value", super.getI()));
            super.setD(SmartDashboard.getNumber(name + " kd Value", super.getD()));
        }

        SmartDashboard.putNumber(errorLabel, getPositionError());
        SmartDashboard.putNumber(setpointLabel, getSetpoint());

        return super.calculate(measurement, setpoint);
    }
}
