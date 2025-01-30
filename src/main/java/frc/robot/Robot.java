// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

public class Robot extends TimedRobot {

  Drivebase drivebase = new Drivebase();
  OI oi = new OI();
  Vision vision = new Vision(
    drivebase::addVisionMeasurement,
    new VisionIOPhotonVision(
      VisionConstants.CAMERA_NAMES[0],
      VisionConstants.ROBOT_TO_CAMERA_TRANSFORM
    ),
    new VisionIOPhotonVision(
      VisionConstants.CAMERA_NAMES[1],
      VisionConstants.ROBOT_TO_CAMERA_TRANSFORM
    )
  );

  public Robot() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() { 
    drivebase.getSwerveTeleopCommand(
      oi::getInstructedXMetersPerSecond,
      oi::getInstructedYMetersPerSecond,
      oi::getInstructedRotationPerSecond,
      true
    ).schedule();
  }
  
  @Override
  public void teleopPeriodic() {
    System.out.println("Teleop Periodic Running");
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
