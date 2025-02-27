package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.Drivebase;

/** Command that uses 3 trapezoid motion profiles to auto lineup to the reef scoring place. */
public class AutoDriveToPositionTrapezoidProfile extends Command {

  public class StatePose {

    State x, y, rot;

    public StatePose(
        double x,
        double y,
        double rot,
        double vx,
        double vy,
        double vrot) {

      this.x = new State(x, vx);
      this.y = new State(y, vy);
      this.rot = new State(rot, vrot);
    }

    public StatePose(
        State x, 
        State y, 
        State rot) {

      this.x = x;
      this.y = y;
      this.rot = rot;
    }
  }

  public class TripleProfile {

    TrapezoidProfile xProfile, yProfile, rotProfile;

    public TripleProfile(
        double maxVX,
        double maxAX,
        double maxVY,
        double maxAY,
        double maxVRot,
        double maxARot) {
    
      this.xProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVX, maxAX));
      this.yProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVY, maxAY));
      this.rotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVRot, maxARot));
    }

    public StatePose calculate(double t, StatePose initialPose, StatePose endPose) {
      State x = xProfile.calculate(t, initialPose.x, endPose.x);
      State y = yProfile.calculate(t, initialPose.y, endPose.y);
      State rot = rotProfile.calculate(t, initialPose.rot, endPose.rot);

      return new StatePose(x, y, rot);
    }
  }

  private final StatePose initialPose = 
    new StatePose(0.0,0.0,0.0,0.0,0.0,0.0);

  private final StatePose endPose = 
    new StatePose(0.0,0.0,0.0,0.0,0.0,0.0);

  private final TripleProfile motionProfile = 
    new TripleProfile(0.0, 0.0,0.0,0.0,0.0,0.0);
  
  private final Timer timeElapsed = new Timer();
  Drivebase drivebase;

  public AutoDriveToPositionTrapezoidProfile(Drivebase drivebase, Translation2d point) {
    this.drivebase = drivebase;
  }

  @Override
  public void initialize() {
    timeElapsed.start();
  }

  @Override
  public void execute() {
    StatePose statePose = motionProfile.calculate(timeElapsed.get(), initialPose, endPose);
    drivebase.drive(statePose.x.position, statePose.y.position, statePose.rot.position, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
