// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.PIDControllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.ConditionalSmartDashboard;

/** Add your docs here. */
public interface SmartPIDInterface {

  public default void putValueSmartDashboard(String PIDName, String name, double value) {
    ConditionalSmartDashboard.putNumber("SmartPid/" + PIDName + "/" + name, value);
  }

  public default double getValueFromSmartDashboard(String PIDName, String name, double defaultValue) {
    return SmartDashboard.getNumber("SmartPid/" + PIDName + "/" + name, defaultValue);
  }

}
