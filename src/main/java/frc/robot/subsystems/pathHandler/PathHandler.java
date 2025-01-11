package frc.robot.subsystems.pathHandler;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class PathHandler extends MeasurableSubsystem {
  DriveSubsystem driveSubsystem;

  private PathStates currstate = PathStates.DONE;
  private boolean isHandling = false;
  private List<Character> NodeNames;
  private Timer timer = new Timer();
  private List<Trajectory<SwerveSample>> fetchPaths;
  private List<Trajectory<SwerveSample>> placePaths;
  private String[][] pathNames = new String[12][2];
  private Trajectory<SwerveSample> currPath;
  private boolean runningPath = false;
  private boolean mirrorTrajectory = false;
  private boolean firstpath = true;
  private Character startNode = 'a';

  PathHandler(DriveSubsystem driveSubsystem, String[][] pathNames, List<Character> NodeNames,) {
    this.driveSubsystem = driveSubsystem;
    this.pathNames = pathNames;
    this.NodeNames = NodeNames;

    reassignAlliance();
  }

  public void startPathHandler() {
    firstpath = true;
  }

  public void reassignAlliance() {
    mirrorTrajectory = driveSubsystem.shouldFlip();
    for (int i = 0; i < 12; i++) {
      Optional<Trajectory<SwerveSample>> temp = Choreo.loadTrajectory(pathNames[i][0]);
      fetchPaths.add(temp.get());
    }
    for (int i = 0; i < 12; i++) {
      Optional<Trajectory<SwerveSample>> temp = Choreo.loadTrajectory(pathNames[i][1]);
      placePaths.add(temp.get());
    }
  }

  private void startPath(Trajectory<SwerveSample> path) {
    if (isHandling && path != null) {
      currPath = path;
      runningPath = true;
      timer.start();
      driveSubsystem.calculateController(currPath.getInitialSample(mirrorTrajectory).get());
      if (currstate == PathStates.FETCH) {
        currstate = PathStates.DRIVE_FETCH;
      } else if (currstate == PathStates.PLACE) {
        currstate = PathStates.DRIVE_PLACE;
      }
    }
  }

  private void drivePath() {
    if (isHandling && runningPath && currPath != null) {
      driveSubsystem.calculateController(currPath.sampleAt(timer.get(), mirrorTrajectory).get());
      if (timer.hasElapsed(currPath.getTotalTime())) {
        runningPath = false;
        timer.stop();
        timer.reset();
        driveSubsystem.calculateController(currPath.getFinalSample(mirrorTrajectory).get());
        if (currstate == PathStates.DRIVE_FETCH) {
          currstate = PathStates.FETCH;
        } else if (currstate == PathStates.DRIVE_PLACE) {
          currstate = PathStates.PLACE;
        }
      }
    }
  }

  private Trajectory<SwerveSample> nextPath() {
    if (NodeNames.size() > 0) {
      if (currstate == PathStates.DRIVE_FETCH) {
        return fetchPaths.get(NodeNames.get(0) - 'a');
      } else if (currstate == PathStates.DRIVE_PLACE) {
        return placePaths.get(NodeNames.get(0) - 'a');
      }
    } else {
      killPathHandler();
    }
    return null;
  }

  public void killPathHandler() {
    isHandling = false;
    currstate = PathStates.DONE;
    runningPath = false;
    timer.stop();
    timer.reset();
  }

  public void periodic() {
    switch (currstate) {
      case DRIVE_FETCH:
        startPath(nextPath());
        break;
      case FETCH:
        // align the robot
        // make the robot grab a piece
        break;
      case DRIVE_PLACE:
        startPath(nextPath());
        break;
      case PLACE:
        // align the robot
        // make the robot place a piece
        break;
      case DONE:
        isHandling = false;
        break;
      default:
        break;
    }
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
