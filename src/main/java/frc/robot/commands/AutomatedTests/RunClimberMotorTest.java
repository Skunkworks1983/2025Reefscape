// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedTests;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.utils.error.TestResult;

public class RunClimberMotorTest extends Command {

  Climber climber;
  Consumer<TestResult> addTest;
  Consumer<TestResult> setTest;
  double startingPos;
  double maxCurrent;
  boolean currentExceeds;

  TestResult motorDidNotRunTest;

  TestResult climberCurrentTooHigh;

  public RunClimberMotorTest(
    Consumer<TestResult> addTest,
    Consumer<TestResult> setTest,
    Climber climber
  ) {
    this.addTest = addTest;
    this.setTest = setTest;
    this.climber = climber;
    addRequirements(climber);

    motorDidNotRunTest = new TestResult(
      "Climb Motor Did Not Run", 
      AlertType.kWarning, 
      climber,
      "checks if motor ran"
    );

    climberCurrentTooHigh = new TestResult(
      "Climber Current Exeeds Tolerance", 
      AlertType.kWarning,  
      climber,
      "checks if Current exeeds its tolerance"
    );

    addTest.accept(motorDidNotRunTest);
    addTest.accept(climberCurrentTooHigh);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPos = climber.getHeight();
    maxCurrent = climber.getCurrent();
    currentExceeds = false;
    climber.setClimberSetPoint(startingPos + Constants.Testing.CLIMBER_HEIGHT_CHANGE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climber.getCurrent() > maxCurrent){
      maxCurrent = climber.getCurrent();
    }
    if(maxCurrent > Constants.Testing.CLIMBER_CURRENT_TOLERANCE) //makes sure that the climber current has not exeeded a pre-determained level
    {
      currentExceeds = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(climber.getHeight() == startingPos) {
      motorDidNotRunTest.setErrorStatus(AlertType.kError);
    }
    else {
      motorDidNotRunTest.setErrorStatus(AlertType.kInfo);
    }
    setTest.accept(motorDidNotRunTest);

    if(currentExceeds) {
      climberCurrentTooHigh.setErrorStatus(AlertType.kError);
    }
    else {
      climberCurrentTooHigh.setErrorStatus(AlertType.kInfo);
    }
    setTest.accept(climberCurrentTooHigh);

    climber.setClimberSetPoint(startingPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.isAtSetpoint();
  }
}
