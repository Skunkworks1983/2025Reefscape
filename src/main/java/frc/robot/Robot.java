// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.error.ErrorCommandGenerator;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.error.DiagnosticSubsystem;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivebase.Drivebase;

public class Robot extends TimedRobot {

  // replace subsystem with Optional.empty() for testing
  // ENSURE_COMPETITION_READY_SUBSYSTEMS must be false for testing.
  Optional<Drivebase> drivebase = Optional.of(new Drivebase());
  Optional<Elevator> elevator = Optional.empty(); //Optional.of(new Elevator());
  Optional<Collector> collector = Optional.empty(); //Optional.of(new Collector());

  OI oi = new OI( 
    elevator,
    collector
  );
  ErrorGroup errorGroup = new ErrorGroup();

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
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

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
  public void teleopPeriodic() {
  }

  @Override
  public void disabledPeriodic() {}

  public void disabledInit() {
    errorGroup.putAllErrors();
  }

  @Override
  public void testPeriodic() {}

  public void testInit() {
    errorGroup.clearAllTest();

    //we provide the errorCommandGenerator with the error group and a array of subsystems to get commands from
    if(drivebase.isPresent()) {
      ErrorCommandGenerator.getErrorCommand(
        errorGroup,
        new DiagnosticSubsystem[] {drivebase.get()}
      ).schedule();
    }
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

}
