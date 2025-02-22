package frc.robot.commands.AutomatedTests;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.Drivebase;

public class PoseDeviationsTestDriveForward extends Command {

  public class StatePose {
    public final State x, y, rot;

    public StatePose(State x, State y, State rot) {
      this.x = x;
      this.y = y;
      this.rot = rot;
    }

    public StatePose() {
      this.x = new State();
      this.y = new State();
      this.rot = new State();
    }
  }

  private static final TrapezoidProfile xProfile = new TrapezoidProfile(
    new Constraints(
      1.0,
      5.0
    )
  );

  private static final TrapezoidProfile yProfile = new TrapezoidProfile(
    new Constraints(
      1.0,
      5.0
    )
  );

  private static final TrapezoidProfile rotProfile = new TrapezoidProfile(
    new Constraints(
      1.0,
      5.0
    )
  );

  private StatePose startPose = new StatePose();
  private StatePose endPose = new StatePose(
    new State(0.0,0.0), 
    new State(1.0, 0.0), 
    new State(0.0, 0.0));

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
    State x = xProfile.calculate(timeElapsed.get(), startPose.x, endPose.x);
    State y = yProfile.calculate(timeElapsed.get(), startPose.y, endPose.y);
    State rot = rotProfile.calculate(timeElapsed.get(), startPose.rot, endPose.rot);
    drivebase.drive(x.velocity, y.velocity, rot.velocity, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
