// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedTests;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.error.TestResult;

public class TestModuleComponentsConnection extends Command {
  
  SwerveModule swerveModule;
  Consumer<TestResult> addTest;
  Consumer<TestResult> setTest;

  TestResult encoderConnectedTest;

  TestResult turnMotorConnectedTest;

  TestResult driveMotorConnectedTest;

  // test command to test the connection of swerve module components
  public TestModuleComponentsConnection(
    Consumer<TestResult> addTest,
    Consumer<TestResult> setTest,
    SwerveModule swerveModule
  ) {
    this.addTest = addTest;
    this.setTest = setTest;
    this.swerveModule = swerveModule;

    encoderConnectedTest = new TestResult(
      "Turn Encoder Not Connected", 
      AlertType.kWarning, 
      swerveModule,
      "Checks the connection status of the turn encoder"
    );

    turnMotorConnectedTest = new TestResult(
      "Turn Motor Not Connected", 
      AlertType.kWarning, 
      swerveModule,
      "Checks the connection status of the turn motor"
    );

    driveMotorConnectedTest = new TestResult(
      "Drive Motor Not Connected",
      AlertType.kWarning, 
      swerveModule,
      "Checks the connection status of the drive motor"
    );

    addTest.accept(encoderConnectedTest);

    addTest.accept(turnMotorConnectedTest);

    addTest.accept(driveMotorConnectedTest);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {

    // Testing the Modules Encoder connection status
    if(swerveModule.isEncoderConnected()) {
      encoderConnectedTest.errorStatus = AlertType.kInfo;
    }
    else {
      encoderConnectedTest.errorStatus = AlertType.kError;
    }
    setTest.accept(encoderConnectedTest);

    // Testing the Modules Turn Motor connection status
    if(swerveModule.isTurnMotorConnected()) {
      turnMotorConnectedTest.errorStatus = AlertType.kInfo;
    }
    else {
      turnMotorConnectedTest.errorStatus = AlertType.kError;
    }
    setTest.accept(turnMotorConnectedTest);

    // Testing the Modules Drive Motor connection status
    if(swerveModule.isDriveMotorConnected()) {
      driveMotorConnectedTest.errorStatus = AlertType.kInfo;
    }
    else {
      driveMotorConnectedTest.errorStatus = AlertType.kError;
    }
    setTest.accept(driveMotorConnectedTest);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
