// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/** Add your docs here. */
public class ErrorGroupHandler {

  private List<ErrorT> errorList = new LinkedList<ErrorT>();

  public ErrorGroupHandler() {

  }

  public void addErrorMapEntry(ErrorT error) {
    errorList.add(error);
  }

  public void setErrorStatus(String entryName, boolean error) {
    for(ErrorT errorT : errorList) {
      if(errorT.name.equals(entryName)) {
        errorT.errorStatus = error;
        return;
      }
    }
  }

  public boolean getErrorStatus(String entryName) {
    for(ErrorT errorT : errorList) {
      if(errorT.name.equals(entryName)) {
        return errorT.errorStatus;
      }
    }
    return true;
  }

  public void putAllErrors() {
    for(ErrorT errorT : errorList) {
      Alert alert = new Alert(errorT.name, AlertType.kError);
      alert.set(errorT.errorStatus);
    }
  }
}
