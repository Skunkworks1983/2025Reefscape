package frc.robot.commands.AutomatedTests;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.Drivebase;

public class AutomatedVisionMountTest extends Command {

  /** Wrapper class containing a "pose" of 3 states, representing x, y, and rotation */
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
      .5,
      5.0
    )
  );

  private static final TrapezoidProfile yProfile = new TrapezoidProfile(
    new Constraints(
      .5,
      5.0
    )
  );

  private static final TrapezoidProfile rotProfile = new TrapezoidProfile(
    new Constraints(
      45.0,
      360.0
    )
  );
  
  private StatePose startPose = new StatePose();
  private StatePose endPose = new StatePose(
    new State(5.0,0.0), 
    new State(0.0, 0.0), 
    new State(0.0, 0.0));

  private final Timer timeElapsed = new Timer();
  Drivebase drivebase;

  public AutomatedVisionMountTest(Drivebase drivebase) {
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    System.out.println("Drive To Pos Commmand Start");
    timeElapsed.start();
  }

  @Override
  public void execute() {
    State x = xProfile.calculate(timeElapsed.get(), startPose.x, endPose.x);
    State y = yProfile.calculate(timeElapsed.get(), startPose.y, endPose.y);
    State rot = rotProfile.calculate(timeElapsed.get(), startPose.rot, endPose.rot);
    drivebase.drive(x.velocity, y.velocity, -45, true);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Drive To Pos Command End");
    timeElapsed.stop();
    timeElapsed.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
