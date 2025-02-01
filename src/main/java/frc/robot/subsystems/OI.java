// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.Constants.OI.LIMITS;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OI.IDs.Joysticks;

public class OI extends SubsystemBase {

  Joystick rotationJoystick = new Joystick(Joysticks.ROTATION_JOYSTICK_ID);
  Joystick translationJoystick = new Joystick(Joysticks.TRANSLATION_JOYSTICK_ID);
  Joystick buttonJoystick = new Joystick(Joysticks.BUTTON_STICK_ID);

  // Input to the function could be x or y axis.
  DoubleFunction<Double> joystickToMetersPerSecond = 
    (axisInput) -> Math.pow(axisInput, Constants.OI.AXIS_INPUT_EXPONENT) 
    * LIMITS.MAX_INSTRUCTED_METERS_PER_SECOND;

  DoubleFunction<Double> joystickToDegreesPerSecond = 
    (xInput) -> 
      Math.pow(xInput, Constants.OI.AXIS_INPUT_EXPONENT) * LIMITS.MAX_INSTRUCTED_DEGREES_PER_SECOND;


  // Input to the function could be x or y axis. 
  // Deadband is applied on each axis individually. This might not be desirable.
  // This function uses the turnary operator ("?") to select between two options 
  // in a single expression.
  public DoubleFunction <Double> applyDeadband =
    (axisInput) -> Math.abs(axisInput) < Constants.OI.AXIS_DEADBAND 
      ? 0.0 : axisInput;

  public OI(Optional<Elevator> optionalElevator, Optional<Collector> optionalCollector) {
    // There is repetition here but not enough to warrant a different aproach

    if(optionalElevator.isPresent()) {
      Elevator elevator = optionalElevator.get();
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Elevator.GOTO_FLOOR_POSITION)
        .onTrue(elevator.getMoveToPositionCommand(Constants.Elevator.Setpoints.FLOOR_POSITION_METERS));

      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Elevator.GOTO_L1)
        .onTrue(elevator.getMoveToPositionCommand(Constants.Elevator.Setpoints.L1_POSITION_METERS));

      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Elevator.GOTO_L2)
        .onTrue(elevator.getMoveToPositionCommand(Constants.Elevator.Setpoints.L2_POSITION_METERS));

      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Elevator.GOTO_L3)
        .onTrue(elevator.getMoveToPositionCommand(Constants.Elevator.Setpoints.L3_POSITION_METERS));

      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Elevator.GOTO_L4)
        .onTrue(elevator.getMoveToPositionCommand(Constants.Elevator.Setpoints.L4_POSITION_METERS));
    }

    if(optionalCollector.isPresent()) {
      Collector collector = optionalCollector.get();
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Collector.ROTATE_CORAL)
        .whileTrue(collector.rotateCoral());
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Collector.INTAKE_CORAL)
        .whileTrue(collector.intakeCoral());
    }
  }

  @Override
  public void periodic() {}

  public double getInstructedXMetersPerSecond() {
    return joystickToMetersPerSecond.apply(
      applyDeadband.apply(translationJoystick.getX())
    );
  }

  public double getInstructedYMetersPerSecond() {
    return joystickToMetersPerSecond.apply(
      applyDeadband.apply(translationJoystick.getY())
    );
  }

  public double getInstructedDegreesPerSecond() {
    return joystickToDegreesPerSecond.apply(
      applyDeadband.apply(rotationJoystick.getY())
    );
  }
}
