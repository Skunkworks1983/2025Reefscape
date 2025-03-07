// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectorBloopSpeedCommand extends Command {
  /** Creates a new CollectorBloopSpeedCommand. */
  Collector collector;
  double speed;
  public CollectorBloopSpeedCommand(Collector collector, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    this.speed = speed;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.setCollectorSpeeds(speed, speed * Constants.Collector.Speeds.SPEED_MULIPILER_LEFT);
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
    return false;
  }
}
