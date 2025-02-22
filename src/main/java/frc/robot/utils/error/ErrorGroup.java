// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.error;
import java.util.Set;
import java.util.TreeSet;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public class ErrorGroup {

  private Set<TestResult> testList = new TreeSet<TestResult>();

  public ErrorGroup() {}

  public void addTestSetEntry(TestResult test) {
    // checks the list to see if there is a duplicate, and if either is an error, the final becomes an error
    for(TestResult testResult : testList) {
      if(testResult.name.equals(test.name) && testResult.subsystem == test.subsystem) {
        if(test.errorStatus == AlertType.kWarning) {
          setTestStatus(test.name, test.subsystem, AlertType.kWarning);
        }
        System.out.println("Duplicate entry in list, please check to see what is doing this. Subsystem: " + test.subsystem.toString() + " Error: " + test.name);
        return;
      }
    }
    testList.add(test);
    putTestToSmartdashboard(test);
  }

  public void setTestStatus(String entryName, Subsystem subsystem, AlertType error) {
    for(TestResult testResult : testList) {
      if(testResult.name.equals(entryName) && testResult.subsystem.equals(subsystem)) {
        testResult.errorStatus = error;
        putTestToSmartdashboard(testResult);
        return;
      }
    }
  }

  public void setTestStatusUsingTestResult(TestResult test) {
    setTestStatus(test.name, test.subsystem, test.errorStatus);
  }

  public AlertType getTestStatus(String entryName, Subsystem subsystem) {
    for(TestResult testResult : testList) {
      if(testResult.name.equals(entryName) && testResult.subsystem == subsystem) {
        return testResult.errorStatus;
      }
    }
    System.out.println("getErrorStatus couldn't find the error " + entryName + " connected to subsystem " + subsystem.toString());
    // returns true because id rather get an error if it couldn't find the requested error than not an error
    return AlertType.kError;
  }

  public void clearAllTest() {
    testList = new TreeSet<TestResult>();
  }

  public void putTestToSmartdashboard(TestResult test) {
    SmartDashboard.putBoolean("Tests/" + test.name + " " + test.subsystem.toString(), test.errorStatus == AlertType.kInfo);
    System.out.println(test.name + " " + test.subsystem.toString());
  }

  public void putAllErrors() {
    for(TestResult testResult : testList) {
      if(testResult.errorStatus != AlertType.kInfo) {
        Alert alert = new Alert(testResult.name + " " + testResult.subsystem.toString(), testResult.errorStatus);
        alert.set(true);
      }
    }
  }
}
