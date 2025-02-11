// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.error;
import java.util.Set;
import java.util.TreeSet;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public class ErrorGroup {

  private Set<TestResult> testList = new TreeSet<TestResult>();

  public ErrorGroup() {}

  public void addTestMapEntry(TestResult test) {
    // checks the list to see if there is a duplicate, and if either is an error, the final becomes an error
    for(TestResult testResult : testList) {
      if(testResult.name.equals(test.name) && testResult.subsystem == test.subsystem) {
        if(test.errorStatus = true) {
          setTestStatus(test.name, test.subsystem, true);
        }
        System.out.println("Duplicate entry in list, please check to see what is doing this");
        return;
      }
    }
    testList.add(test);
  }

  public void setTestStatus(String entryName, Subsystem subsystem, boolean error) {
    for(TestResult testResult : testList) {
      if(testResult.name.equals(entryName)) {
        testResult.errorStatus = error;
        return;
      }
    }
  }

  public boolean getTestStatus(String entryName, Subsystem subsystem) {
    for(TestResult testResult : testList) {
      if(testResult.name.equals(entryName) && testResult.subsystem == subsystem) {
        return testResult.errorStatus;
      }
    }
    System.out.println("getErrorStatus couldn't find the error " + entryName + " connected to subsystem " + subsystem.toString());
    // returns true because id rather get an error if it couldn't find the requested error than not an error
    return true;
  }

  public void clearAllTest() {
    testList = new TreeSet<TestResult>();
  }

  public void putAllErrors() {
    for(TestResult testResult : testList) {
      Alert alert = new Alert(testResult.name + " " + testResult.subsystem.toString(), AlertType.kError);
      alert.set(testResult.errorStatus);
    }
  }
}
