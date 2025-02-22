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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.EndEffectorSetpoints;
import frc.robot.constants.Constants.Drivebase.FieldTarget;
import frc.robot.constants.Constants.OI.LIMITS;
import frc.robot.commands.MoveEndEffector;
import frc.robot.subsystems.drivebase.Drivebase;
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
    Optional<Funnel> optionalFunnel
  ) {
    JoystickButton algaeToggle = new JoystickButton(buttonJoystick, Buttons.ALGAE_TOGGLE);
    Trigger coralToggle = algaeToggle.negate();

    if (optionalCollector.isPresent()) {
      Collector collector = optionalCollector.get();
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.ROTATE_CORAL)
        .whileTrue(collector.rotateCoralCommand());

      JoystickButton intake = new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.INTAKE);
      JoystickButton expell = new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.EXPELL);
      intake.and(coralToggle)
        .whileTrue(collector.waitAfterCatchPieceCommand());

      // TODO: add intake code -- talk to lukas
      intake.and(algaeToggle).whileTrue(collector.intakeCoralCommand());
      intake.and(coralToggle).whileTrue(collector.intakeCoralCommand());

      expell.and(algaeToggle).whileTrue(collector.scorePieceCommand());
      expell.and(coralToggle).whileTrue(collector.scorePieceCommand());
    }

    if (optionalDrivebase.isPresent()) {
      Drivebase drivebase = optionalDrivebase.get();

      // TODO: find a better place for this command
      Command targetCommand = drivebase.getSwerveHeadingCorrected(
          this::getInstructedXMetersPerSecond,
          this::getInstructedYMetersPerSecond,
          // TODO: define an actual target point
          (Supplier<Rotation2d>) () -> drivebase.getTargetingAngle(FieldTarget.REEF_RED),
          true);

      targetCommand.addRequirements(drivebase);

      new JoystickButton(rotationJoystick, Constants.OI.IDs.Buttons.TARGET_REEF_BUTTON)
          .whileTrue(targetCommand);
    }

    if(optionalClimber.isPresent()){
      Climber climber = optionalClimber.get();
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.CLIMBER_GOTO_MAX)
        .onTrue(climber.goToPositionAfterMagnetSensor(Constants.Climber.CLIMBER_MAX));
      new JoystickButton(buttonJoystick, Constants.OI.IDs.Buttons.CLIMBER_GOTO_MIN)
        .onTrue(climber.goToPositionAfterMagnetSensor(Constants.Climber.CLIMBER_MIN));
    }

    if(optionalFunnel.isPresent()) {
      Funnel funnel = optionalFunnel.get();

      JoystickButton raiseFunnel = new JoystickButton(
        buttonJoystick,
        Constants.OI.IDs.Buttons.RAISE_FUNNEL_TOGGLE
      );

      raiseFunnel.whileTrue(funnel.goToPos(Constants.Funnel.Setpoints.RAISED_POSITION));
      raiseFunnel.whileFalse(funnel.goToPos(Constants.Funnel.Setpoints.LOWERED_POSITION));
    }

    if(optionalElevator.isPresent() && optionalWrist.isPresent()) {
      JoystickButton endEffectorToGround = new JoystickButton(buttonJoystick, Buttons.GOTO_GROUND);
      JoystickButton endEffectorToStow = new JoystickButton(buttonJoystick, Buttons.GOTO_STOW);
      JoystickButton endEffectorToPositionA = new JoystickButton(buttonJoystick, Buttons.GOTO_POSITION_A);
      JoystickButton endEffectorToPositionB = new JoystickButton(buttonJoystick, Buttons.GOTO_POSITION_B);
      JoystickButton endEffectorToPositionC = new JoystickButton(buttonJoystick, Buttons.GOTO_POSITION_C);
      JoystickButton endEffectorToPositionD = new JoystickButton(buttonJoystick, Buttons.GOTO_POSITION_D);

      Elevator elevator = optionalElevator.get();
      Wrist wrist = optionalWrist.get();

      // Algae mode
      endEffectorToGround.and(algaeToggle).onTrue(
        new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.algaeGround)
      );

      endEffectorToStow.and(algaeToggle).onTrue(
        new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.algaeStow)
      );

      endEffectorToPositionA.and(algaeToggle).onTrue(
        new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.algaeProcessor)
      );

      endEffectorToPositionB.and(algaeToggle).onTrue(
        new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.algaeL2)
      );

      endEffectorToPositionC.and(algaeToggle).onTrue(
        new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.algaeL3)
      );

      endEffectorToPositionD.and(algaeToggle).onTrue(
        new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.algaeNet)
      );

      // Coral mode
      endEffectorToGround.and(coralToggle).onTrue(
        new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.coralGround)
      );

      endEffectorToStow.and(coralToggle).onTrue(
        new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.coralStow)
      );

      endEffectorToPositionA.and(coralToggle).onTrue(
        new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.coralL1)
      );

      endEffectorToPositionB.and(coralToggle).onTrue(
        new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.coralL2)
      );

      endEffectorToPositionC.and(coralToggle).onTrue(
        new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.coralL3)
      );

      endEffectorToPositionD.and(coralToggle).onTrue(
        new MoveEndEffector(elevator, wrist, EndEffectorSetpoints.coralL4)
      );
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