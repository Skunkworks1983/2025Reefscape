// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleFunction;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.OI.LIMITS;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OI.IDS;

public class OI extends SubsystemBase {

  Joystick leftJoystick = new Joystick(IDS.LEFT_JOYSTICK_ID);
  Joystick rightJoystick = new Joystick(IDS.RIGHT_JOYSTICK_ID);
  Joystick buttonStick = new Joystick(IDS.BUTTON_STICK_ID);

  Joystick translationJoystick = leftJoystick;
  Joystick rotationJoystick = rightJoystick;

  // Input to the function could be x or y axis.
  DoubleFunction<Double> joystickToMetersPerSecond = 
    (axisInput) -> Math.pow(axisInput, Constants.OI.AXIS_INPUT_EXPONENT) 
    * LIMITS.MAX_INSTRUCTED_METERS_PER_SECOND;

  Function<Double,Rotation2d> joystickToRotationPerSecond = 
    (xInput) -> Rotation2d.fromDegrees(
      Math.pow(xInput, Constants.OI.AXIS_INPUT_EXPONENT) * LIMITS.MAX_INSTRUCTED_DEGREES_PER_SECOND
    );


  // Input to the function could be x or y axis. 
  // Deadband is applied on each axis individually. This might not be desirable.
  // This function uses the turnary opperator (?) to select between two options 
  // in a single expression.
  public DoubleFunction<Double> applyDeadband =
    (axisInput) -> Math.abs(axisInput) < Constants.OI.AXIS_DEADBAND 
      ? 0.0 : axisInput;

  public OI() {}

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

  public Rotation2d getInstructedRotationPerSecond() {
    return joystickToRotationPerSecond.apply(
      applyDeadband.apply(rotationJoystick.getY())
    );
  }
}
