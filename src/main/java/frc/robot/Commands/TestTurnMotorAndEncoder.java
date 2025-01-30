// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.ErrorT;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestTurnMotorAndEncoder extends Command {
  /** Creates a new testTurnMotorAndEncoder. */

  double numOfTurns;
  double startPos;
  double encoderStartPos;
  double highestMotorCurrent;
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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveModule.setTurnControllerActive(false);
    swerveModule.setTurnMotorSpeed(0.075);
    highestMotorCurrent = 0;
    startPos = swerveModule.getTurnMotorEncoderPosition();
    encoderStartPos = swerveModule.turnEncoder.getPosition().getValueAsDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnMotorCurrent = swerveModule.getTurnMotorCurrent();
    System.out.println(turnMotorCurrent);
    if(highestMotorCurrent < turnMotorCurrent) {
      highestMotorCurrent = turnMotorCurrent;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveModule.setTurnMotorSpeed(0);
    double encoderDifference = swerveModule.turnEncoder.getPosition().getValueAsDouble() - encoderStartPos;

    //tolerance of .05
    if(Math.abs(Math.abs(encoderDifference) - numOfTurns) > 0.05 ) {
      alert.accept(new ErrorT(swerveModule.moduleName + " Turn Motor/Encoder Misaligned", true, swerveModule));
    }
    else {
      alert.accept(new ErrorT(swerveModule.moduleName + " Turn Motor/Encoder Misaligned", false, swerveModule));
    }

    if(swerveModule.getTurnMotorEncoderPosition() == startPos) {
      alert.accept(new ErrorT(swerveModule.moduleName + " Turn Motor did not move", true, swerveModule));
    }
    else {
      alert.accept(new ErrorT(swerveModule.moduleName + " Turn Motor did not move", false, swerveModule));
    }

    System.out.println(highestMotorCurrent);

    if(highestMotorCurrent > 12) {
      alert.accept(new ErrorT(swerveModule.moduleName + " Turn Motor pulled too much Current", true, swerveModule));
    }
    else {
      alert.accept(new ErrorT(swerveModule.moduleName + " Turn Motor pulled too much Current", false, swerveModule));
    }

    if(encoderStartPos == 0.0 && swerveModule.turnEncoder.getPosition().getValueAsDouble() == 0.0) {
      alert.accept(new ErrorT(swerveModule.moduleName + " Encoder is reporting 0", true, swerveModule));
    }
    else {
      alert.accept(new ErrorT(swerveModule.moduleName + " Encoder is reporting 0", false, swerveModule));
    }

    swerveModule.setTurnControllerActive(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return numOfTurns < Math.abs(swerveModule.getTurnMotorEncoderPosition() - startPos);
  }
}
