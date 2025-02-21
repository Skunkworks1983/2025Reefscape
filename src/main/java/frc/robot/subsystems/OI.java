// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.Drivebase.FieldTarget;
import frc.robot.constants.Constants.OI.LIMITS;
import frc.robot.commands.Wrist.MoveWristToSetpoint;
import frc.robot.commands.elevator.*;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Elevator.Setpoints;
import frc.robot.constants.Constants.OI.IDs.Buttons;
import frc.robot.commands.elevator.*;
import frc.robot.constants.Constants.OI.IDs.Joysticks;

public class OI {
  private Joystick rotationJoystick = new Joystick(Joysticks.ROTATION_JOYSTICK_ID);
  private Joystick translationJoystick = new Joystick(Joysticks.TRANSLATION_JOYSTICK_ID);
  private Joystick buttonJoystick = new Joystick(Joysticks.BUTTON_STICK_ID);

  // Input to the function could be x or y axis.
  private DoubleFunction<Double> joystickToMetersPerSecond = (
      axisInput) -> Math.pow(axisInput, Constants.OI.AXIS_INPUT_EXPONENT)
          * LIMITS.MAX_INSTRUCTED_METERS_PER_SECOND;

  private DoubleFunction<Double> joystickToDegreesPerSecond = (
      xInput) -> Math.pow(xInput, Constants.OI.AXIS_INPUT_EXPONENT) * LIMITS.MAX_INSTRUCTED_DEGREES_PER_SECOND;

  // Input to the function could be x or y axis.
  // Deadband is applied on each axis individually. This might not be desirable.
  // This function uses the ternary operator ("?") to select between two options
  // in a single expression.
  public DoubleFunction<Double> applyDeadband = (axisInput) -> Math.abs(axisInput) < Constants.OI.AXIS_DEADBAND
      ? 0.0
      : axisInput;

  public OI(
    Optional<Elevator> optionalElevator, 
    Optional<Collector> optionalCollector,
    Optional<Wrist> optionalWrist,
    Optional<Climber> optionalClimber,
    Optional<Drivebase> optionalDrivebase) {

    JoystickButton algaeToggle = new JoystickButton(buttonJoystick, Buttons.ALGAE_TOGGLE);
    JoystickButton gotoPosition1 = new JoystickButton(buttonJoystick, Buttons.GOTO_POSITION_1);
    JoystickButton gotoPosition2 = new JoystickButton(buttonJoystick, Buttons.GOTO_POSITION_2);
    JoystickButton gotoPosition3 = new JoystickButton(buttonJoystick, Buttons.GOTO_POSITION_3);
    JoystickButton gotoPosition4 = new JoystickButton(buttonJoystick, Buttons.GOTO_POSITION_4);
    JoystickButton gotoGround = new JoystickButton(buttonJoystick, Buttons.GOTO_GROUND);
    JoystickButton gotoStow = new JoystickButton(buttonJoystick, Buttons.GOTO_STOW);

    // TODO: add wrist to all commands in which the elevator moves.
    if(optionalElevator.isPresent() && optionalWrist.isPresent()) {
      Elevator elevator = optionalElevator.get();
      Wrist wrist = optionalWrist.get();

      // Coral mode 
      Trigger coralToggle = algaeToggle.negate();
      gotoStow.and(coralToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.STOW_CORAL));
      gotoGround.and(coralToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.GROUND_AGLAE));
      gotoPosition1.and(coralToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.L1_POSITION_CORAL));
      gotoPosition2.and(coralToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.L2_POSITION_CORAL));
      gotoPosition3.and(coralToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.L3_POSITION_CORAL));
      gotoPosition4.and(coralToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.L4_POSITION_CORAL));
      gotoPosition4.and(coralToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.L4_POSITION_CORAL));

      // Algae mode 
      gotoStow.and(algaeToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.STOW_AGLAE));
      gotoGround.and(algaeToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.GROUND_AGLAE));
      gotoPosition1.and(algaeToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.PROCESSOR_POSITION));
      gotoPosition2.and(algaeToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.L2_POSITION_ALGAE));
      gotoPosition3.and(algaeToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.L3_POSITION_ALGAE));
      gotoPosition4.and(algaeToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.NET_POSITION));
    }

    if (optionalCollector.isPresent()) {
      Collector collector = optionalCollector.get();
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.ROTATE_PIECE)
        .whileTrue(collector.rotateCoralCommand());
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.INTAKE)
        .whileTrue(collector.waitAfterCatchPieceCommand());
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.EXPELL)
        .whileTrue(collector.scorePieceCommand());
    }

    if (optionalDrivebase.isPresent()) {
      Drivebase drivebase = optionalDrivebase.get();

      Command targetCommand = drivebase.getSwerveHeadingCorrected(
          this::getInstructedXMetersPerSecond,
          this::getInstructedYMetersPerSecond,
          // TODO: define an actual target point
          (Supplier<Rotation2d>) () -> drivebase.getTargetingAngle(FieldTarget.REEF_RED),
          true);

      targetCommand.addRequirements(drivebase);

      new JoystickButton(rotationJoystick, Constants.OI.IDs.TARGET_REEF_BUTTON)
          .whileTrue(targetCommand);
    }

    if(optionalClimber.isPresent()){
      Climber climber = optionalClimber.get();

      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.CLIMBER_GOTO_MAX)
        .onTrue(climber.goToPositionAfterMagnetSensor(Constants.ClimberIDs.CLIMBER_MAX));
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.CLIMBER_GOTO_MIN)
        .onTrue(climber.goToPositionAfterMagnetSensor(Constants.ClimberIDs.CLIMBER_MIN));
    }
  }

  public double getInstructedXMetersPerSecond() {

    return joystickToMetersPerSecond.apply(
        // X and Y are flipped because the joysticks' coordinate system is different
        // from the field
        applyDeadband.apply(translationJoystick.getY()));
  }

  public double getInstructedYMetersPerSecond() {

    return joystickToMetersPerSecond.apply(
        // X and Y are flipped because the joysticks' coordinate system is different
        // from the field
        applyDeadband.apply(translationJoystick.getX()));
  }

  public double getInstructedDegreesPerSecond() {

    return joystickToDegreesPerSecond.apply(
        applyDeadband.apply(-rotationJoystick.getX()));
  }
}