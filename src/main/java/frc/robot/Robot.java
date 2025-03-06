// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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

  

  Optional<Drivebase> drivebase = Optional.empty();// Optional.of(new Drivebase());
  Optional<Elevator> elevator = Optional.empty();//Optional.of(new Elevator());
  Optional<Collector> collector = Optional.empty();//Optional.of(new Collector());
  Optional<Wrist> wrist = Optional.empty();//Optional.of(new Wrist());
  Optional<Climber> climber = Optional.empty();//Optional.of(new Climber());
  Optional<Funnel> funnel = Optional.empty();//Optional.of(new Funnel());

  OI oi = new OI( 
    elevator,
    collector,
    wrist,
    climber,
    drivebase,
    funnel
  );
  
  ErrorGroup errorGroup = new ErrorGroup();

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
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  Counter lidar = new Counter(1);
  DigitalOutput output = new DigitalOutput(0);
  @Override
  public void teleopInit() { 
    output.set(true);
    lidar.setMaxPeriod(1.0);
    lidar.setSemiPeriodMode(true);
    lidar.reset();
    t.reset();
  }

  Timer t = new Timer();
  @Override
  public void teleopPeriodic() {
    /*
    if(t.get() > 1.0) {
      t.reset();
      output.set(false);
    }
    else {
      output.set(true);
    }
    */
    double dist;
    double v = lidar.getPeriod();
    if(lidar.get() < 1){
      dist = 0;
    }
    else {
      dist = v * 1000000.0 / 10.0;
    }
      SmartDashboard.putNumber("ALocation:", dist);
      SmartDashboard.putNumber("Avalue:", v);
      System.out.println(v);
    /*
    oi.putRotationJoystickToSmartDashboard();
    oi.putTranslationJoystickToSmartDashboard();
    */
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
