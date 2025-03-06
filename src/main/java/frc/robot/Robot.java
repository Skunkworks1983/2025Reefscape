// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.error.ErrorCommandGenerator;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.error.DiagnosticSubsystem;
import frc.robot.commands.MoveEndEffector;
import frc.robot.commands.funnel.MoveFunnelToSetpoint;
import frc.robot.commands.tests.JoystickElevatorVelocity;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivebase.Drivebase;

public class Robot extends TimedRobot {

  // replace subsystem with Optional.empty() when you do not wish to use add all
  // subsystems. ENSURE_COMPETITION_READY_SUBSYSTEMS must be false for testing.

  Optional<Drivebase> drivebase = Optional.empty();
  Optional<Elevator> elevator = Optional.of(new Elevator());
  Optional<Collector> collector = Optional.of(new Collector());
  Optional<Wrist> wrist = Optional.of(new Wrist());
  Optional<Climber> climber = Optional.empty();
  Optional<Funnel> funnel = Optional.empty();
  Optional<Climber> climber = Optional.of(new Climber());
  Optional<Funnel> funnel = Optional.of(new Funnel());

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

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override 
  public void robotInit() {
    if (elevator.isPresent() && wrist.isPresent()){

    //move to pos coral 
    NamedCommands.registerCommand("Coral to L4",
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.coralL4));
    
    NamedCommands.registerCommand("Coral to L3", 
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.coralL3));

    NamedCommands.registerCommand("Coral to L2", 
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.coralL2));

    NamedCommands.registerCommand("Coral to L1", 
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.coralL1));

    NamedCommands.registerCommand("Coral to Ground", 
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.coralGround));

    NamedCommands.registerCommand("Coral to Stow ", 
    new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.coralStow));

    // move to pos Algae
    NamedCommands.registerCommand("Algae to L2 ", 
    new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.algaeL2));

    NamedCommands.registerCommand("Algae to L3", 
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.algaeL3));

    NamedCommands.registerCommand("Algae to Ground", 
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.algaeGround));

    NamedCommands.registerCommand("Algae to Processor", 
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.algaeProcessor));
    
    NamedCommands.registerCommand("Algea to Stow", 
      new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.algaeStow));

    // funnel 
    NamedCommands.registerCommand("Funnel to Station", 
      new MoveFunnelToSetpoint(funnel.get(), Constants.Funnel.FUNNEL_POSITION_LOW_CONVERTED));
    
    NamedCommands.registerCommand("Funnel to up pos",
      new MoveFunnelToSetpoint(funnel.get(), Constants.Funnel.FUNNEL_POSITION_HIGH_CONVERTED));

    // Collector 
    NamedCommands.registerCommand("Expel Coral", collector.get().expelCoral(isAutonomous()));

    NamedCommands.registerCommand("Expel Algae",collector.get().expelAlgaeCommand(isAutonomous()));

    NamedCommands.registerCommand("Intake Coral", collector.get().intakeCoralCommand(isAutonomous()));

    NamedCommands.registerCommand("Intake Algae ", collector.get().intakeAlgaeCommand(isAutonomous()));

    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ConditionalSmartDashboard.updateConditions();
  }

  @Override
  public void autonomousInit() {
     Command currentAutonomousCommand = autoChooser.getSelected();
    if (currentAutonomousCommand != new MoveEndEffector(elevator.get(), wrist.get(), Constants.EndEffectorSetpoints.algaeL2)) {
      currentAutonomousCommand.schedule();
    }
  }


  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() { 
    // if(drivebase.isPresent()) {
    //   drivebase.get().getSwerveCommand(
    //     oi::getInstructedXMetersPerSecond,
    //     oi::getInstructedYMetersPerSecond,
    //     oi::getInstructedDegreesPerSecond,
    //     true
    //   ).schedule();
    // }
    new JoystickElevatorVelocity(elevator.get(), oi::getYrotationStick).schedule();
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
