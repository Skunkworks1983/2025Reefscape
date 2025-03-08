// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.Drivebase;

public class DeadReckoningDriveOut extends Command {

  Drivebase drivebase;
  Timer timeElasped = new Timer();
  final double totalTime = 1.0;

  public DeadReckoningDriveOut(Drivebase drivebase) {
    this.drivebase = drivebase;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeElasped.reset();
    timeElasped.start();
    drivebase.drive(1.0, 0.0, 0.0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeElasped.get() > totalTime;
  }
}
