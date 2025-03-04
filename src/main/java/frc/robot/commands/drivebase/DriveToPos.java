package frc.robot.commands.drivebase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPos extends Command {

  @FunctionalInterface
  public interface DriveFunction<A, B, C, D> {
    public void apply(A xMetersPerSecond, B yMetersPerSecond, C degreesPerSecond, D fieldRelative);
  }

  DriveFunction<Double, Double, Double, Boolean> drive;
  PIDController xController = new PIDController(.01, 0, 0);
  PIDController yController = new PIDController(.01, 0, 0);
  PIDController rotController = new PIDController(.01, 0, 0);
  Pose2d goalPose;
  Supplier<Pose2d> getRobotPose;

  public DriveToPos(
      DriveFunction<Double, Double, Double, Boolean> drive, 
      Supplier<Pose2d> getEstimatedRobotPose,
      Pose2d position) {
    
    this.drive = drive;
    this.goalPose = position;
    this.getRobotPose = getEstimatedRobotPose;

    xController.setTolerance(.1);
    yController.setTolerance(.1);
    rotController.setTolerance(1);
  }

  @Override
  public void initialize() {
    xController.setSetpoint(goalPose.getX());
    yController.setSetpoint(goalPose.getY());
    rotController.setSetpoint(goalPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    Pose2d robotPose = getRobotPose.get();
    drive.apply(
      xController.calculate(robotPose.getX()), 
      yController.calculate(robotPose.getY()), 
      rotController.calculate(robotPose.getRotation().getDegrees()), true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
  }
}
