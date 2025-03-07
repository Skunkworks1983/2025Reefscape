// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.EndEffectorSetpoints;
import frc.robot.constants.Constants.OI.LIMITS;
import frc.robot.commands.MoveEndEffector;
import frc.robot.commands.elevator.MoveElevatorToSetpointCommand;
import frc.robot.commands.funnel.MoveFunnelToSetpoint;
import frc.robot.commands.tests.JoystickEndEffectorPosition;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.drivebase.TeleopFeatureUtils;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OI.IDs.Buttons;
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

  public void putRotationJoystickToSmartDashboard() {
    double rotationJoystickAngleRadidans = Math.atan2(rotationJoystick.getY(), rotationJoystick.getX());
    double rotationJoystickAngleDegrees = Math.toDegrees(rotationJoystickAngleRadidans);
    SmartDashboard.putNumber("OI/ Rotation Joystick Angle", rotationJoystickAngleDegrees - 90);
  }
 
  public void putTranslationJoystickToSmartDashboard() {
    double translationJoystickAngleRadians = Math.atan2(translationJoystick.getY(), translationJoystick.getX());
    double translationJoystickAngleDegrees = Math.toDegrees(translationJoystickAngleRadians);
    SmartDashboard.putNumber("OI/ Translation Joystick Angle", translationJoystickAngleDegrees - 90);
  }
  

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
      Optional<Drivebase> optionalDrivebase,
      Optional<Funnel> optionalFunnel) {
    Trigger algaeToggle = new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.ALGAE_TOGGLE);
    Trigger coralToggle = algaeToggle.negate(); // If not in algae mode, we are using coral mode.

    if (optionalCollector.isPresent()) {
      Collector collector = optionalCollector.get();

      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.INTAKE)
          .and(coralToggle)
          .whileTrue(collector.intakeCoralCommand(true));

      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.EXPEL)
        .and(coralToggle)
        .whileTrue(collector.expelCoralCommand(true));

      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.INTAKE)
          .and(algaeToggle)
          .whileTrue(collector.intakeAlgaeCommand(true));

      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.EXPEL)
          .and(algaeToggle)
          .whileTrue(collector.expelAlgaeCommand(true));
    }

    if (optionalDrivebase.isPresent()) {
      Drivebase drivebase = optionalDrivebase.get();

      double alignSpeed = 3;

      Command targetCommand = drivebase.getSwerveHeadingCorrected(
          this::getInstructedXMetersPerSecond,
          this::getInstructedYMetersPerSecond,
          (Supplier<Rotation2d>) () -> 
            TeleopFeatureUtils.getPointAtReefFaceAngle(drivebase::getCachedEstimatedRobotPose),
          true);

      Command targetCoralStationCommand = drivebase.getSwerveHeadingCorrected(
          this::getInstructedXMetersPerSecond,
          this::getInstructedYMetersPerSecond,
          (Supplier<Rotation2d>) () -> 
            TeleopFeatureUtils.getPointAtCoralStationAngle(drivebase::getCachedEstimatedRobotPose),
          true);

      Command AlignCoralRightCommand = drivebase.getSwerveAlignCoral(
          this::getInstructedXMetersPerSecond,
          this::getInstructedYMetersPerSecond,
          -alignSpeed,
          /*goingRight=*/true
      );

      Command AlignCoralLeftCommand = drivebase.getSwerveAlignCoral(
          this::getInstructedXMetersPerSecond,
          this::getInstructedYMetersPerSecond,
          alignSpeed,
          /*goingRight=*/false
      );

      new JoystickButton(rotationJoystick, Constants.OI.IDs.Buttons.TARGET_REEF_BUTTON)
          .whileTrue(targetCommand);

      new JoystickButton(rotationJoystick, Constants.OI.IDs.Buttons.TARGET_CORAL_STATION_BUTTON)
          .whileTrue(targetCoralStationCommand);

      new JoystickButton(translationJoystick, 4)
          .whileTrue(AlignCoralRightCommand);

      new JoystickButton(translationJoystick, 5)
          .whileTrue(AlignCoralRightCommand);
    } 

    if(optionalClimber.isPresent()){
      Climber climber = optionalClimber.get();
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.CLIMBER_GOTO_MAX)
          .onTrue(climber.goToPositionAfterMagnetSensor(Constants.Climber.CLIMBER_MAX));
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.CLIMBER_GOTO_MIN)
          .onTrue(climber.goToPositionAfterMagnetSensor(Constants.Climber.CLIMBER_MIN));
    }

    if (optionalFunnel.isPresent()) {
      Funnel funnel = optionalFunnel.get();
      new JoystickButton(translationJoystick, Constants.OI.IDs.Buttons.FUNNEL_GO_TO_MAX)
          .onTrue(new MoveFunnelToSetpoint(funnel, Constants.Funnel.FUNNEL_POSITION_HIGH_CONVERTED));
      new JoystickButton(translationJoystick, Constants.OI.IDs.Buttons.FUNNEL_GO_TO_MIN)
          .onTrue(new MoveFunnelToSetpoint(funnel, Constants.Funnel.FUNNEL_POSITION_LOW_CONVERTED));
    }

    if (optionalElevator.isPresent() && optionalWrist.isPresent()) {
      Elevator elevator = optionalElevator.get();
      Wrist wrist = optionalWrist.get();

      JoystickButton elevatorTop = new JoystickButton(buttonJoystick, 23);
      JoystickButton elevatorBottom = new JoystickButton(buttonJoystick, 22);
      JoystickButton wristUp = new JoystickButton(buttonJoystick, 17);
      JoystickButton wristDown = new JoystickButton(buttonJoystick, 24);

      JoystickButton endEffectorButton = new JoystickButton(buttonJoystick, 9);
      // JoystickButton endEffectorToScoreLow = new JoystickButton(buttonJoystick, Buttons.GOTO_SCORE_LOW);
      // JoystickButton endEffectorToL2 = new JoystickButton(buttonJoystick, Buttons.GOTO_L2);
      // JoystickButton endEffectorToL3 = new JoystickButton(buttonJoystick, Buttons.GOTO_L3);
      // JoystickButton endEffectorToScoreHigh = new JoystickButton(buttonJoystick, Buttons.GOTO_SCORE_HIGH);

      elevatorTop.onTrue(new MoveElevatorToSetpointCommand(elevator, 39));
      elevatorBottom.onTrue(new MoveElevatorToSetpointCommand(elevator, 0.0));
      wristDown.onTrue(new MoveWristToSetpoint(wrist, 0.1441));
      wristUp.onTrue(new MoveWristToSetpoint(wrist, 0.0));

      endEffectorButton.whileTrue(new JoystickEndEffectorPosition(wrist, elevator, this::getYrotationStick, this::getYtranslationStick));

      // // Algae mode
      // endEffectorGround.and(algaeToggle).onTrue(
      //   new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.ALGAE_GROUND)
      // );

      // endEffectorStow.and(algaeToggle).onTrue(
      //   new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.ALGAE_STOW)
      // );

      // endEffectorToScoreLow.and(algaeToggle).onTrue(
      //   new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.ALGAE_PROCESSOR)
      // );

      // endEffectorToL2.and(algaeToggle).onTrue(
      //   new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.ALGAE_L2)
      // );

      // endEffectorToL3.and(algaeToggle).onTrue(
      //   new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.ALGAE_L3)
      // );

      // endEffectorToScoreHigh.and(algaeToggle).onTrue(
      //   new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.ALGAE_NET)
      // );

      // // Coral mode
      // endEffectorGround.and(coralToggle).onTrue(
      //   new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.CORAL_GROUND)
      // );

      // endEffectorStow.and(coralToggle).onTrue(
      //   new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.CORAL_STOW)
      // );

      // endEffectorToScoreLow.and(coralToggle).onTrue(
      //   new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.CORAL_L1)
      // );

      // endEffectorToL2.and(coralToggle).onTrue(
      //   new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.CORAL_L2)
      // );

      // endEffectorToL3.and(coralToggle).onTrue(
      //   new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.CORAL_L3)
      // );

      // endEffectorToScoreHigh.and(coralToggle).onTrue(
      //   new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.CORAL_L4)
      // );
    }
  }

  public double getInstructedXMetersPerSecond() {

    return joystickToMetersPerSecond.apply(
        // X and Y are flipped because the joysticks' coordinate system is different
        // from the field
        applyDeadband.apply(-translationJoystick.getY()));
  }

  public double getInstructedYMetersPerSecond() {

    return joystickToMetersPerSecond.apply(
        // X and Y are flipped because the joysticks' coordinate system is different
        // from the field
        applyDeadband.apply(-translationJoystick.getX()));
  }

  public double getInstructedDegreesPerSecond() {

    return joystickToDegreesPerSecond.apply(
        applyDeadband.apply(-rotationJoystick.getX()));
  }

  public double getYrotationStick() {
    return applyDeadband.apply(-rotationJoystick.getY());
  }

  public double getYtranslationStick() {
    return applyDeadband.apply(-translationJoystick.getY());
  }
}