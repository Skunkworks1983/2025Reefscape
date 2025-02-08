// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.errorCommands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.error.ErrorT;

public class TestModuleComponentsConnection extends Command {
  
  SwerveModule swerveModule;
  Consumer<ErrorT> alert;

  public TestModuleComponentsConnection(
    Consumer<ErrorT> alert,
    SwerveModule swerveModule
  ) {
    this.alert = alert;
    this.swerveModule = swerveModule;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {

    alert.accept(new ErrorT("Turn Encoder Not Connected", !swerveModule.isEncoderConnected(), swerveModule));

    alert.accept(new ErrorT("Turn Motor Not Connected", !swerveModule.isTurnMotorConnected(), swerveModule));

    alert.accept(new ErrorT("Drive Motor Not Connected", !swerveModule.isDriveMotorConnected(), swerveModule));
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
