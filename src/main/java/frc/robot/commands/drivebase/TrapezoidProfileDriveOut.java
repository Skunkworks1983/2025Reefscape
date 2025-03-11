// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.Drivebase;

public class TrapezoidProfileDriveOut extends Command {

  Drivebase drivebase;
  Timer timeElasped = new Timer();
  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(2.0, 1.0));
  State startState = new State();
  State goalState;

  public TrapezoidProfileDriveOut(Drivebase drivebase) {
    this.drivebase = drivebase;
    addRequirements(drivebase);
    SmartDashboard.putNumber("distanceToTravel",0.0);
  }

  double cachedHeadingForCommand;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cachedHeadingForCommand = drivebase.getCachedGyroHeading().getDegrees();

    goalState = new State(SmartDashboard.getNumber("distanceToTravel", 0.0), 0.0);
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      drivebase.resetGyroHeading(Rotation2d.fromDegrees(180));
      cachedHeadingForCommand = 180;
    }
    else {
      drivebase.resetGyroHeading(Rotation2d.fromDegrees(0));
      cachedHeadingForCommand = 0;
    }
    
    timeElasped.reset();
    timeElasped.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = profile.calculate(timeElasped.get(), startState, goalState).velocity;
    drivebase.drive(xVelocity, 0.0, drivebase.headingController.calculate(
            drivebase.getCachedGyroHeading().getDegrees(),
            cachedHeadingForCommand)
    , false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeElasped.get() > profile.totalTime();
  }
}
