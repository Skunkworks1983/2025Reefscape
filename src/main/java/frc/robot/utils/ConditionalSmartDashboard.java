// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ConditionalSmartDashboard {

  private static boolean smartdashboardEnabled = true;
  private static boolean robotIsDisabledOrIsTest;

  // Use instead of SmartDashboard.putNumber
  public static boolean putNumber(String key, double value) {
    if(checkConditions()) {
      return SmartDashboard.putNumber(key, value);
    }
    else {
      return false;
    }
  }

  // Use instead of SmartDashboard.putBoolean
  public static boolean putBoolean(String key, Boolean value) {
    if(checkConditions()) {
      return SmartDashboard.putBoolean(key, value);
    }
    else {
      return false;
    }
  }

  // Use instead of SmartDashboard.putData
  public static void putData(String key, Sendable data) {
    if(checkConditions()) {
      SmartDashboard.putData(key, data);
    }
  }

  // Only Robot should be calling this as of now
  public static void setSmartdashboardEnabled(boolean isEnabled) {
    smartdashboardEnabled = isEnabled;
  }

  public static boolean isSmartdashboardEnabled() {
    return smartdashboardEnabled;
  }

  private static boolean checkConditions() {
    return smartdashboardEnabled || robotIsDisabledOrIsTest;
  }

  public static void updateConditions() {
    robotIsDisabledOrIsTest = DriverStation.isDisabled() || DriverStation.isTestEnabled();
  }
}
