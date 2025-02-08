// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.TestTurnMotorAndEncoder;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;
import frc.robot.utils.error.ErrorCommandGenerator;
import frc.robot.utils.error.ErrorGroupHandler;
import frc.robot.utils.error.SubsystemError;

public class Robot extends TimedRobot {

  Drivebase drivebase = new Drivebase();
  OI oi = new OI();
  ErrorGroupHandler errorGroupHandler = new ErrorGroupHandler();

  public Robot() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() { 
    drivebase.getSwerveTeleopCommand(
      oi::getInstructedXMetersPerSecond,
      oi::getInstructedYMetersPerSecond,
      oi::getInstructedRotationPerSecond,
      true
    ).schedule();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    errorGroupHandler.putAllErrors();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    errorGroupHandler.clearAllErrors();
    ErrorCommandGenerator.getErrorCommand(
      errorGroupHandler,
      new SubsystemError[] {drivebase}
    );
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
