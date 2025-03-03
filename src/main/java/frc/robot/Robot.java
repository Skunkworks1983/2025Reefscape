// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.error.ErrorCommandGenerator;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.error.DiagnosticSubsystem;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivebase.Drivebase;

public class Robot extends TimedRobot {

  // replace subsystem with Optional.empty() when you do not wish to use add all
  // subsystems. ENSURE_COMPETITION_READY_SUBSYSTEMS must be false for testing.

  Optional<Drivebase> drivebase = Optional.ofNullable(null);
  Optional<Elevator> elevator = Optional.ofNullable(null);
  Optional<Collector> collector = Optional.ofNullable(null);
  Optional<Wrist> wrist = Optional.ofNullable(null);
  Optional<Climber> climber = Optional.ofNullable(null);
  Optional<Funnel> funnel = Optional.ofNullable(null);

  OI oi = new OI( 
    elevator,
    collector,
    wrist,
    climber,
    drivebase,
    funnel
  );
  
  ErrorGroup errorGroup = new ErrorGroup();

  //ultra sonic sensors to aid alignmment to reef pipes
  private AnalogInput leftUtraSonic;
  private AnalogInput rightUtraSonic;


  public Robot() {
    DataLogManager.start();

    if(Constants.Testing.ENSURE_COMPETITION_READY_SUBSYSTEMS) {
      if(drivebase.isEmpty()) {
        throw new IllegalStateException("Drivebase not present");
      }
      if (collector.isEmpty()) {
        throw new IllegalStateException("Collector not present");
      }
      if (elevator.isEmpty()) {
        throw new IllegalStateException("Elevator not present");
      }
      if (wrist.isEmpty()) {
        throw new IllegalStateException("Wrist not present");
      }
      if (climber.isEmpty()) {
        throw new IllegalStateException("Climber not present");
      }
    }

    //set up ultra sonic sensors
    leftUtraSonic = new AnalogInput(0);
    rightUtraSonic = new AnalogInput(1);


    if(drivebase.isPresent()) {
      drivebase.get().setDefaultCommand(
        drivebase.get().getSwerveCommand(
          oi::getInstructedXMetersPerSecond,
          oi::getInstructedYMetersPerSecond,
          oi::getInstructedDegreesPerSecond,
          true
        )
      );
    }
  }

  @Override 
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ConditionalSmartDashboard.updateConditions();

    double leftSensorValue = leftUtraSonic.getVoltage();
    double rightSensorValue = rightUtraSonic.getVoltage();
    final double scaleFactor = 1/(5./512.); //scale converting voltage to distance
    double leftDistance = 5*leftSensorValue*scaleFactor;
    double rightDistance = 5*rightSensorValue*scaleFactor;

    SmartDashboard.putNumber("left ultra sonic", leftDistance); 
    SmartDashboard.putNumber("right ultra sonic", rightDistance); 
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() { 
    if(drivebase.isPresent()) {
      drivebase.get().getSwerveCommand(
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
  public void disabledPeriodic() {}

  public void disabledInit() {
    errorGroup.putAllErrors();
  }

  @Override
  public void testPeriodic() {}

  public void testInit() {
    errorGroup.clearAllTest();

    // We provide the errorCommandGenerator with the error group and a array of subsystems to get commands from
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
