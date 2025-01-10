package frc.robot.subsystems.pathHandler;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class PathHandler extends MeasurableSubsystem {
  DriveSubsystem driveSubsystem;

  private PathStates currStates = PathStates.DONE;
  private List<String> NodeNames;
  private Timer timer = new Timer();
  private Trajectory<SwerveSample>[][] paths;
  private String[][] pathNames;

  PathHandler(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getMeasures'");
  }

  public enum PathStates {
    FETCH,
    DRIVE_FETCH,
    PLACE,
    DRIVE_PLACE,
    DONE
  }
}
