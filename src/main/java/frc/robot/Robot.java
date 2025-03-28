// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.DoubleSupplier;

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
import frc.robot.commands.AutomatedScoring.AutomatedLidarScoring;
import frc.robot.commands.drivebase.OdometryFreeScoreAuto;
import frc.robot.commands.drivebase.OdometryFreeScoreAutoCenter;
import frc.robot.commands.drivebase.TrapezoidProfileDriveStraight;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivebase.Drivebase;

public class Robot extends TimedRobot {

  // replace subsystem with Optional.empty() when you do not wish to use add all
  // subsystems. ENSURE_COMPETITION_READY_SUBSYSTEMS must be false for testing.

  
  Optional<Elevator> elevator;
  Optional<Collector> collector;
  Optional<Wrist> wrist;
  Optional<Climber> climber;
  Optional<Funnel> funnel;
  Optional<Drivebase> drivebase;

  private SendableChooser<Command> autoChooser;

  OI oi;

  ErrorGroup errorGroup = new ErrorGroup();
  Command automatedVisionMountTest;

  Command deadReckoningDriveOut;
  Command trapezoidProfileDriveOut;
  Command scoreCoralNoOdometryLeft;
  Command scoreCoralNoOdometryRight;
  Command scoreCoralNoOdometryCenter;

  public Robot() {
    DataLogManager.start();

    elevator = Optional.of(new Elevator());
    collector = Optional.of(new Collector());
    wrist = Optional.of(new Wrist());
    climber = Optional.empty();
    funnel = Optional.empty();

    if (elevator.isPresent() && wrist.isPresent()) {
      // move to pos coral 
      NamedCommands.registerCommand("Coral to L4",
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.CORAL_L4, ()->0.0));
      
      NamedCommands.registerCommand("Coral to L3", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.CORAL_L3, ()->0.0));
  
      NamedCommands.registerCommand("Coral to L2", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.CORAL_L2, ()->0.0));
  
      NamedCommands.registerCommand("Coral to L1", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.CORAL_L1, ()->0.0));
  
      NamedCommands.registerCommand("Coral to Ground", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.CORAL_GROUND, ()->0.0));
  
      NamedCommands.registerCommand("Coral to Stow", 
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.CORAL_STOW, ()->0.0));
  
      // move to pos Algae
      NamedCommands.registerCommand("Algae to L2 ", 
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.ALGAE_L2, ()->0.0));
  
      NamedCommands.registerCommand("Algae to L3", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.ALGAE_L3, ()->0.0));
  
      NamedCommands.registerCommand("Algae to Ground", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.ALGAE_GROUND, ()->0.0));
  
      NamedCommands.registerCommand("Algae to Processor", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.ALGAE_PROCESSOR, ()->0.0));
      
      NamedCommands.registerCommand("Algea to Stow", 
        new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.ALGAE_STOW, ()->0.0));
  
      // Collector 
      NamedCommands.registerCommand("Expel Coral", collector.get().expelCoralCommand(true, elevator.get()::getEndEffectorSetpoint));
  
      NamedCommands.registerCommand("Expel Algae",collector.get().expelAlgaeCommand(true));
  
      NamedCommands.registerCommand("Intake Coral", collector.get().intakeCoralCommand(true, elevator.get()::getEndEffectorSetpoint));
  
      NamedCommands.registerCommand("Intake Algae ", collector.get().intakeAlgaeCommand(true, elevator.get()::getEndEffectorSetpoint));
  
    }

    drivebase = Optional.of(new Drivebase());

    oi = new OI( 
      elevator,
      collector,
      wrist,
      climber,
      drivebase,
      funnel
    );

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

      trapezoidProfileDriveOut = new TrapezoidProfileDriveStraight(drivebase.get(), 1.0, true);
    }

    if(drivebase.isPresent() && elevator.isPresent() && wrist.isPresent() && collector.isPresent()) {
      scoreCoralNoOdometryLeft = 
        new OdometryFreeScoreAuto(
          drivebase.get(), 
          elevator.get(), 
          wrist.get(), 
          collector.get(), 
          true
        );

    
    }

    if(drivebase.isPresent() && elevator.isPresent() && wrist.isPresent() && collector.isPresent()) {
      scoreCoralNoOdometryRight = 
        new OdometryFreeScoreAuto(
          drivebase.get(), 
          elevator.get(), 
          wrist.get(), 
          collector.get(), 
          false
        );
    }

    if(drivebase.isPresent() && elevator.isPresent() && wrist.isPresent() && collector.isPresent()) {
      scoreCoralNoOdometryCenter = 
        new OdometryFreeScoreAutoCenter(
          drivebase.get(), 
          elevator.get(), 
          wrist.get(), 
          collector.get()
        );
    }

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Score Coral No Odometry Right", scoreCoralNoOdometryRight);
    autoChooser.addOption("Score Coral No Odometry Left", scoreCoralNoOdometryLeft);
    autoChooser.addOption("Score Coral No Odometry Center", scoreCoralNoOdometryCenter);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putNumber("Auto wait seconds", 2.0);
    CameraServer.startAutomaticCapture();
  }

  @Override 
  public void robotInit() {}

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
  public void teleopInit() {
    System.out.print("Alliance Count " + drivebase.get().allianceCount[0]);
  }
  
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
