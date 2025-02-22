package frc.robot.commands.AutomatedTests;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.Drivebase;

public class PoseDeviationsTestDriveForward extends Command {

  public class StatePose {
    public final State x, y, rot;

    public StatePose(double x, double y, double rot) {
      this.x = new State(x, 0.0);
      this.y = new State(y, 0.0);
      this.rot = new State(rot, 0.0);
    }

    public StatePose() {
      this.x = new State();
      this.y = new State();
      this.rot = new State();
    }

    public Pose2d calculate() {
      return new Pose2d();
    }
  }

  private static final TrapezoidProfile motionProfile = new TrapezoidProfile(
    new Constraints(
      1.0,
      .1
    )
  );

  private StatePose startPose = new StatePose();
  private StatePose endPose = new StatePose(1.0, 0.0, 0.0);
  private final Timer timeElapsed = new Timer();
  Drivebase drivebase;

  public PoseDeviationsTestDriveForward(Drivebase drivebase) {
    this.drivebase = drivebase;
  }

  @Override
  public void initialize() {
    timeElapsed.start();
  }

  @Override
  public void execute() {
    State newStateX = motionProfile.calculate(timeElapsed.get(), startPose.x, endPose.x);
    State newStateY = motion
    drivebase.drive(newState.velocity, 0.0, 0.0, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
