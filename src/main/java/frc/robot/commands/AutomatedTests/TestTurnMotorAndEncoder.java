// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedTests;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.error.ErrorT;

public class TestTurnMotorAndEncoder extends Command {

  double numOfTurns;
  double startPos;
  double encoderStartPos;
  SwerveModule swerveModule;
  Consumer<ErrorT> alert;

  public TestTurnMotorAndEncoder(
    Consumer<ErrorT> alert,
    SwerveModule swerveModule
  ) {
    this.alert = alert;
    this.swerveModule = swerveModule;
    numOfTurns = 1;
    addRequirements(swerveModule);
  }

  @Override
  public void initialize() {
    swerveModule.setTurnControllerActive(false);
    swerveModule.setTurnMotorSpeed(0.075);
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
    alert.accept(new ErrorT("Turn Motor/Encoder Misaligned", Math.abs(Math.abs(encoderDifference) - numOfTurns) > 0.05, swerveModule));

    //An error is logged if the turn motor did not move when Instructed to
    alert.accept(new ErrorT("Turn Motor did not move", swerveModule.getTurnMotorEncoderPosition() == startPos, swerveModule));

    //An error is logged if the encoder reports 0 in its start and end position. 
    //This happens when the encoder is unplugged or has other mechanical errors
    alert.accept(new ErrorT("Encoder is reporting 0", encoderStartPos == 0.0 && swerveModule.getRawEncoderValue() == 0.0, swerveModule));

    swerveModule.setTurnControllerActive(true);
  }

  @Override
  public boolean isFinished() {
    return numOfTurns < Math.abs(swerveModule.getTurnMotorEncoderPosition() - startPos);
  }
}
