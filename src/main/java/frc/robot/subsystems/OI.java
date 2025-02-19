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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.Constants.OI.LIMITS;
import frc.robot.constants.Constants;
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
      Optional<Drivebase> optionalDrivebase) {

    if (optionalElevator.isPresent()) {
      Elevator elevator = optionalElevator.get();
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Elevator.GOTO_FLOOR_POSITION)
          .onTrue(new MoveToPositionCommand(elevator, Constants.Elevator.Setpoints.FLOOR_POSITION));
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Elevator.GOTO_L1)
          .onTrue(new MoveToPositionCommand(elevator, Constants.Elevator.Setpoints.L1_POSITION));
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Elevator.GOTO_L2)
          .onTrue(new MoveToPositionCommand(elevator, Constants.Elevator.Setpoints.L2_POSITION));
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Elevator.GOTO_L3)
          .onTrue(new MoveToPositionCommand(elevator, Constants.Elevator.Setpoints.L3_POSITION));
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Elevator.GOTO_L4)
          .onTrue(new MoveToPositionCommand(elevator, Constants.Elevator.Setpoints.L4_POSITION));
    }

    if (optionalCollector.isPresent()) {
      Collector collector = optionalCollector.get();
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Collector.ROTATE_CORAL)
          .whileTrue(collector.rotateCoralCommand());
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Collector.COLLECT_CORAL)
          .whileTrue(collector.waitAfterCatchPieceCommand());
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Collector.SCORE_CORAL)
          .whileTrue(collector.scorePieceCommand());
    }

    if (optionalDrivebase.isPresent()) {
      Drivebase drivebase = optionalDrivebase.get();

      Command c = drivebase.getSwerveHeadingCorrected(
          this::getInstructedXMetersPerSecond,
          this::getInstructedYMetersPerSecond,
          // TODO: define an actual target point
          (Supplier<Rotation2d>) () -> drivebase.getTargetingAngle(new Translation2d(0.0, 0.0)),
          true);

      c.addRequirements(drivebase);

      new JoystickButton(rotationJoystick, Constants.OI.IDs.Buttons.TARGET_REEF)
          .whileTrue(c);
    }
  }

  public double getInstructedXMetersPerSecond() {
    SmartDashboard.putNumber("Instructed X", joystickToMetersPerSecond.apply(
        // X and Y are flipped because the joysticks' coordinate system is different
        // from the field
        applyDeadband.apply(translationJoystick.getY())));

    return joystickToMetersPerSecond.apply(
        // X and Y are flipped because the joysticks' coordinate system is different
        // from the field
        applyDeadband.apply(translationJoystick.getY()));
  }

  public double getInstructedYMetersPerSecond() {

    SmartDashboard.putNumber("Instructed Y", joystickToMetersPerSecond.apply(
        // X and Y are flipped because the joysticks' coordinate system is different
        // from the field
        applyDeadband.apply(translationJoystick.getX())));

    return joystickToMetersPerSecond.apply(
        // X and Y are flipped because the joysticks' coordinate system is different
        // from the field
        applyDeadband.apply(translationJoystick.getX()));
  }

  public double getInstructedDegreesPerSecond() {

    SmartDashboard.putNumber("Instructed Rot", joystickToDegreesPerSecond.apply(
        applyDeadband.apply(-rotationJoystick.getX())));

    return joystickToDegreesPerSecond.apply(
        applyDeadband.apply(-rotationJoystick.getX()));
  }
}