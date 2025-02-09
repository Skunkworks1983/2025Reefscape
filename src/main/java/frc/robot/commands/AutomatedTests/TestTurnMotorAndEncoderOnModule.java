// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedTests;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.error.TestResult;

public class TestTurnMotorAndEncoderOnModule extends Command {

  double startPos;
  double encoderStartPos;
  SwerveModule swerveModule;
  Consumer<TestResult> alert;

  //This command spins a turn motor one full rotation of the wheel then extrapolates what data it can from that test
  public TestTurnMotorAndEncoderOnModule(
    Consumer<TestResult> alert,
    SwerveModule swerveModule
  ) {
    this.alert = alert;
    this.swerveModule = swerveModule;
    //Number of times to rotate the wheel for the test, currently one
    addRequirements(swerveModule);
  }

  @Override
  public void initialize() {
    //We need to turn off the Turn motors pid controller to run a manual test
    swerveModule.setTurnControllerActive(false);

    swerveModule.setTurnMotorSpeed(Constants.Testing.TURN_MOTOR_ROTATION_SPEED);
    startPos = swerveModule.getTurnMotorEncoderPosition();
    encoderStartPos = swerveModule.getRawEncoderValue();
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    swerveModule.setTurnMotorSpeed(0);
    double encoderDifference = swerveModule.getRawEncoderValue() - encoderStartPos;

    //An error is logged if the encoder and turn motor on a module report different values
    //this has a tolerance of 0.05 rotations
    alert.accept(
      new TestResult(
        "Turn Motor/Encoder Misaligned", 
        Math.abs(Math.abs(encoderDifference) - Constants.Testing.NUMBER_OF_MOTOR_ROTATIONS_FOR_MODULE_TEST) > Constants.Testing.TURN_MOTOR_AND_ENCODER_TOLERANCE, 
        swerveModule,
        "The encoder and motor reported two rotations different enough to get detected by this Error"
      )
    );
    //An error is logged if the turn motor did not move when Instructed to
    alert.accept(
      new TestResult(
        "Turn Motor did not move", 
        swerveModule.getTurnMotorEncoderPosition() == startPos, 
        swerveModule,
        "The motors position is the same as when it started, we can assume it did not move"
      )
    );
    //An error is logged if the encoder reports 0 in its start and end position. 
    //This happens when the encoder is unplugged or has other mechanical errors
    alert.accept(
      new TestResult(
        "Encoder is reporting 0", 
        encoderStartPos == 0.0 && swerveModule.getRawEncoderValue() == 0.0, 
        swerveModule,
        "The encoder reported a 0.0 position on startup and a 0.0 position on end."
      )
    );

    swerveModule.setTurnControllerActive(true);
  }

  @Override
  public boolean isFinished() {
    return Constants.Testing.NUMBER_OF_MOTOR_ROTATIONS_FOR_MODULE_TEST < Math.abs(swerveModule.getTurnMotorEncoderPosition() - startPos);
  }
}
