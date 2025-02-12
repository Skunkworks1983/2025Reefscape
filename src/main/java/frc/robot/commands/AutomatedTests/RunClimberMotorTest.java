// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedTests;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.utils.error.TestResult;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunClimberMotorTest extends Command {
  /** Creates a new RunClimberMotorTest. */
  Climber climber;
  Consumer<TestResult> alert;
  double startingPos;
  public RunClimberMotorTest(
    Consumer<TestResult> alert,
    Climber climber
  ) {
    this.alert = alert;
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPos = climber.getHeight();
    climber.setClimberSetPoint(startingPos + 0.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    alert.accept(
          new TestResult(
            "Motor Did Not Run", 
            climber.getHeight() != startingPos, 
            climber,
            "checks if motor ran"
          )
        );
    climber.setClimberSetPoint(startingPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.isAtSetpoint();
  }
}
