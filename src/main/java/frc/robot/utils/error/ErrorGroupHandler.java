// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.error;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public class ErrorGroupHandler {

  private List<ErrorT> errorList = new LinkedList<ErrorT>();

  public ErrorGroupHandler() {

  }

  public void addErrorMapEntry(ErrorT error) {
    errorList.add(error);
  }

  public void setErrorStatus(String entryName, Subsystem subsystem, boolean error) {
    for(ErrorT errorT : errorList) {
      if(errorT.name.equals(entryName)) {
        errorT.errorStatus = error;
        return;
      }
    }
  }

  public boolean getErrorStatus(String entryName, Subsystem subsystem) {
    for(ErrorT errorT : errorList) {
      if(errorT.name.equals(entryName) && errorT.subsystem == subsystem) {
        return errorT.errorStatus;
      }
    }
    //returns true because id rather get an error if it couldn't find the requested error than not an error
    return true;
  }

  public void clearAllErrors() {
    errorList = new LinkedList<ErrorT>();
  }

  public void putAllErrors() {
    for(ErrorT errorT : errorList) {
      Alert alert = new Alert(errorT.name + " " + errorT.subsystem.toString(), AlertType.kError);
      alert.set(errorT.errorStatus);
    }
  }
}
