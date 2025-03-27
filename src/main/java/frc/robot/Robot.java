// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.error.ErrorCommandGenerator;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.error.DiagnosticSubsystem;
import frc.robot.commands.MoveEndEffector;
import frc.robot.commands.drivebase.TrapezoidProfileDriveOut;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivebase.Drivebase;

public class Robot extends TimedRobot {

  // replace subsystem with Optional.empty() when you do not wish to use add all
  // subsystems. ENSURE_COMPETITION_READY_SUBSYSTEMS must be false for testing.

  Optional<Drivebase> drivebase = Optional.empty();
  Optional<Elevator> elevator = Optional.empty();
  Optional<Collector> collector = Optional.empty();
  Optional<Wrist> wrist = Optional.empty();
  Optional<Climber> climber = Optional.of(new Climber());
  Optional<Funnel> funnel = Optional.empty();

  private SendableChooser<Command> autoChooser;

  OI oi = new OI( 
    elevator,
    collector,
    wrist,
    climber,
    drivebase,
    funnel
  );

  ErrorGroup errorGroup = new ErrorGroup();
  Command automatedVisionMountTest;

  Command deadReckoningDriveOut;
  Command trapezoidProfileDriveOut;

  public Robot() {
    DataLogManager.start();

    if (Constants.Testing.ENSURE_COMPETITION_READY_SUBSYSTEMS) {
      if (drivebase.isEmpty()) {
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
      if (climber.isPresent()) {
        throw new IllegalStateException("Climber is present"); // Climber will not be part of our robot in our first match
      }
      if (funnel.isPresent()) {
        throw new IllegalStateException("Funnel is present"); // Funnel will not be part of our robot in our first match
      }
      if (Constants.Testing.ROBOT != Constants.Testing.Robot.Comp2025) {
        throw new IllegalStateException("Using 2024 drivebase constants! Change to 2025 (Constants.Testing.ROBOT)");
      }
      if (Constants.Testing.SMART_PID_ENABLED) {
        throw new IllegalStateException("Global Smartpid Enabled! (Constants.Drivebase.PIDS.SMART_PID_ENABLED)");
      }
    }

    if (drivebase.isPresent()) {
      drivebase.get().setDefaultCommand(
        drivebase.get().getSwerveCommand(
          oi::getInstructedXMetersPerSecond,
          oi::getInstructedYMetersPerSecond,
          oi::getInstructedDegreesPerSecond,
          true
        )
      );

      trapezoidProfileDriveOut = new TrapezoidProfileDriveOut(drivebase.get());
    }

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
    // CameraServer.startAutomaticCapture();
  }

  @Override 
  public void robotInit() {
    if (elevator.isPresent() && wrist.isPresent()) {

      // move to pos coral 
      NamedCommands.registerCommand("Coral to L4",
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.CORAL_L4));
      
      NamedCommands.registerCommand("Coral to L3", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.CORAL_L3));

      NamedCommands.registerCommand("Coral to L2", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.CORAL_L2));

      NamedCommands.registerCommand("Coral to L1", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.CORAL_L1));

      NamedCommands.registerCommand("Coral to Ground", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.CORAL_GROUND));

      NamedCommands.registerCommand("Coral to Stow", 
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.CORAL_STOW));

      // move to pos Algae
      NamedCommands.registerCommand("Algae to L2 ", 
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.ALGAE_L2));

      NamedCommands.registerCommand("Algae to L3", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.ALGAE_L3));

      NamedCommands.registerCommand("Algae to Ground", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.ALGAE_GROUND));

      NamedCommands.registerCommand("Algae to Processor", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.ALGAE_PROCESSOR));
      
      NamedCommands.registerCommand("Algea to Stow", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.ALGAE_STOW));

      // Collector 
      NamedCommands.registerCommand("Expel Coral", collector.get().expelCoralCommand(true, elevator.get()::getEndEffectorSetpoint));

      NamedCommands.registerCommand("Expel Algae",collector.get().expelAlgaeCommand(true));

      NamedCommands.registerCommand("Intake Coral", collector.get().intakeCoralCommand(true));

      NamedCommands.registerCommand("Intake Algae ", collector.get().intakeAlgaeCommand(true));

    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ConditionalSmartDashboard.updateConditions();
  }

  @Override
  public void autonomousInit() {
    Command autoCommand = autoChooser.getSelected();
    if (autoCommand != null) {
      autoCommand.schedule();
    }

    // trapezoidProfileDriveOut.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}
  
  @Override
  public void teleopPeriodic() {
    oi.putRotationJoystickToSmartDashboard();
    oi.putTranslationJoystickToSmartDashboard();
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
    if (drivebase.isPresent()) {
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
