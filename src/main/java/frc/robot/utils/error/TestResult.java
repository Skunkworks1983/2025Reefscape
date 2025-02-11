// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.error;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestResult {
  public String name;
  // true if the test failed, false if the test passed
  public boolean errorStatus;
  public SubsystemBase subsystem;
  // tell someone what your test did
  public String testDescription;
  public TestResult(String name, boolean errorStatus, SubsystemBase subsystem, String testDescription) {
    this.name = name;
    this.errorStatus = errorStatus;
    this.subsystem = subsystem;
    this.testDescription = testDescription;
  }
}
