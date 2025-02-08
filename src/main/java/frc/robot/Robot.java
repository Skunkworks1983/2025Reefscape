// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.TestTurnMotorAndEncoder;
import frc.robot.utils.error.ErrorCommandGenerator;
import frc.robot.utils.error.ErrorGroupHandler;
import frc.robot.utils.error.SubsystemError;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

  // replace subsystem with Optional.empty() for testing
  // ENSURE_COMPETITION_READY_SUBSYSTEMS must be false for testing.
  Optional<Drivebase> drivebase = Optional.of(new Drivebase());
  Optional<Elevator> elevator = Optional.of(new Elevator());
  Optional<Collector> collector = Optional.of(new Collector());

  OI oi = new OI( 
    elevator,
    collector
  );
  Drivebase drivebase = new Drivebase();
  OI oi = new OI();
  ErrorGroupHandler errorGroupHandler = new ErrorGroupHandler();

  public Robot() {
    if(Constants.Testing.ENSURE_COMPETITION_READY_SUBSYSTEMS) {
      assert drivebase.isPresent();
      assert collector.isPresent();
      assert elevator.isPresent();
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() { 
    if(drivebase.isPresent()) {
      drivebase.get().getSwerveTeleopCommand(
        oi::getInstructedXMetersPerSecond,
        oi::getInstructedYMetersPerSecond,
        oi::getInstructedDegreesPerSecond,
        true
      ).schedule();
    }
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
