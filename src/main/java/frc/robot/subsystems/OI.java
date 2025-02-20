// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.OI.LIMITS;
import frc.robot.commands.elevator.*;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Elevator.Setpoints;
import frc.robot.constants.Constants.OI.IDs.Buttons;
import frc.robot.constants.Constants.OI.IDs.Joysticks;

public class OI {
  private Joystick rotationJoystick = new Joystick(Joysticks.ROTATION_JOYSTICK_ID);
  private Joystick translationJoystick = new Joystick(Joysticks.TRANSLATION_JOYSTICK_ID);
  private Joystick buttonJoystick = new Joystick(Joysticks.BUTTON_STICK_ID);

  // Input to the function could be x or y axis.
  private DoubleFunction<Double> joystickToMetersPerSecond = 
    (axisInput) -> Math.pow(axisInput, Constants.OI.AXIS_INPUT_EXPONENT) 
    * LIMITS.MAX_INSTRUCTED_METERS_PER_SECOND;

  private DoubleFunction<Double> joystickToDegreesPerSecond = 
    (xInput) -> 
      Math.pow(xInput, Constants.OI.AXIS_INPUT_EXPONENT) * LIMITS.MAX_INSTRUCTED_DEGREES_PER_SECOND;


  // Input to the function could be x or y axis. 
  // Deadband is applied on each axis individually. This might not be desirable.
  // This function uses the ternary operator ("?") to select between two options 
  // in a single expression.
  public DoubleFunction <Double> applyDeadband =
    (axisInput) -> Math.abs(axisInput) < Constants.OI.AXIS_DEADBAND 
      ? 0.0 : axisInput;

  public OI(Optional<Elevator> optionalElevator, Optional<Collector> optionalCollector) {

    JoystickButton algaeToggle = new JoystickButton(buttonJoystick, Buttons.ALGAE_TOGGLE);
    JoystickButton gotoPosition1 = new JoystickButton(buttonJoystick, Buttons.GOTO_POSITION_1);
    JoystickButton gotoPosition2 = new JoystickButton(buttonJoystick, Buttons.GOTO_POSITION_2);
    JoystickButton gotoPosition3 = new JoystickButton(buttonJoystick, Buttons.GOTO_POSITION_3);
    JoystickButton gotoPosition4 = new JoystickButton(buttonJoystick, Buttons.GOTO_POSITION_4);

    // TODO: add wrist
    if(optionalElevator.isPresent() /* && optionalWrist.isPresent() */) {
      Elevator elevator = optionalElevator.get();

      // Coral mode 
      Trigger coralToggle = algaeToggle.negate();
      gotoPosition1.and(coralToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.FLOOR_POSITION));
      gotoPosition2.and(coralToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.FLOOR_POSITION));
      gotoPosition3.and(coralToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.FLOOR_POSITION));
      gotoPosition4.and(coralToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.FLOOR_POSITION));

      // Algae mode 
      gotoPosition1.and(algaeToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.PROCESSOR_POSITION));
      gotoPosition2.and(algaeToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.L2_POSITION_ALGAE));
      gotoPosition3.and(algaeToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.L3_POSITION_ALGAE));
      gotoPosition4.and(algaeToggle).onTrue(new MoveToPositionCommand(elevator, Setpoints.NET_POSITION));
    }

    if(optionalCollector.isPresent()) {
      Collector collector = optionalCollector.get();

      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Collector.ROTATE_PIECE)
        .whileTrue(collector.rotateCoralCommand());
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Collector.INTAKE)
        .whileTrue(collector.waitAfterCatchPieceCommand());
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.Collector.EXPELL)
        .whileTrue(collector.scorePieceCommand());
    }
  }

  public double getInstructedXMetersPerSecond() {
    return joystickToMetersPerSecond.apply(
      // X and Y are flipped because the joysticks' coordinate system is different from the field
      applyDeadband.apply(translationJoystick.getY())
    );
  }

  public double getInstructedYMetersPerSecond() {
    return joystickToMetersPerSecond.apply(
      // X and Y are flipped because the joysticks' coordinate system is different from the field
      applyDeadband.apply(translationJoystick.getX())
    );
  }

  public double getInstructedDegreesPerSecond() {
    return joystickToDegreesPerSecond.apply(
      applyDeadband.apply(-rotationJoystick.getX())
    );
  }
}