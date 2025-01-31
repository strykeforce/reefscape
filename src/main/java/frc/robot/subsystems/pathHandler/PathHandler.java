package frc.robot.subsystems.pathHandler;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotStateConstants;
import frc.robot.constants.TagServoingConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import net.jafama.FastMath;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class PathHandler extends MeasurableSubsystem {
  DriveSubsystem driveSubsystem;

  private PathStates currState = PathStates.DONE;
  private boolean isHandling = false;
  private List<Character> NodeNames;
  private List<Integer> NodeLevels;
  private Timer timer = new Timer();
  private List<Trajectory<SwerveSample>> fetchPaths;
  private List<Trajectory<SwerveSample>> placePaths;
  private String[][] pathNames = new String[12][2];
  private Trajectory<SwerveSample> currPath;
  private String currPathString;
  private boolean runningPath = false;
  private boolean isServoing = false;
  private boolean mirrorTrajectory = false;
  private Character startNode = 'a';

  PathHandler(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    reassignAlliance();
  }

  PathHandler(
      DriveSubsystem driveSubsystem,
      String[][] pathNames,
      List<Character> NodeNames,
      List<Integer> NodeLevels,
      Character startNode) {
    this.driveSubsystem = driveSubsystem;
    this.pathNames = pathNames;
    this.NodeNames = NodeNames;
    this.NodeLevels = NodeLevels;
    this.startNode = startNode;
  }

  public void setPathNames(String[][] pathNames) {
    if (!isHandling) {
      this.pathNames = pathNames;
    }
  }

  public void setNodeNames(List<Character> NodeNames) {
    if (!isHandling) {
      this.NodeNames = NodeNames;
    }
  }

  public void setNodeLevels(List<Integer> NodeLevels) {
    if (!isHandling) {
      this.NodeLevels = NodeLevels;
    }
  }

  public void setStartNode(Character startNode) {
    if (!isHandling) {
      this.startNode = startNode;
    }
  }

  public void startPathHandler() {
    NodeNames.add(0, startNode);
    isHandling = true;
    currState = PathStates.DRIVE_FETCH;
  }

  public void reassignAlliance() {
    mirrorTrajectory = driveSubsystem.shouldFlip();
    for (int i = 0; i < 12; i++) {
      Optional<Trajectory<SwerveSample>> temp = Choreo.loadTrajectory(pathNames[i][0]);
      fetchPaths.add(temp.get());
      temp = Choreo.loadTrajectory(pathNames[i][1]);
      placePaths.add(temp.get());
    }
  }

  private void startPath(Trajectory<SwerveSample> path) {
    if (isHandling && path != null) {
      driveSubsystem.setAutoDebugMsg("Start " + currPathString);
      currPath = path;
      runningPath = true;
      timer.stop();
      timer.reset();
      timer.start();
      driveSubsystem.calculateController(currPath.getInitialSample(mirrorTrajectory).get());
      if (currState == PathStates.PLACE) {
        currState = PathStates.DRIVE_FETCH;
      } else if (currState == PathStates.FETCH) {
        currState = PathStates.DRIVE_PLACE;
      }
    }
  }

  private void drivePath() {
    if (isHandling && runningPath && currPath != null) {
      driveSubsystem.calculateController(currPath.sampleAt(timer.get(), mirrorTrajectory).get());
      if (timer.hasElapsed(currPath.getTotalTime())) {
        driveSubsystem.setAutoDebugMsg("End " + currPathString);
        runningPath = false;
        timer.stop();
        timer.reset();
        driveSubsystem.calculateController(currPath.getFinalSample(mirrorTrajectory).get());
        if (currState == PathStates.DRIVE_FETCH) {
          currState = PathStates.FETCH;
        } else if (currState == PathStates.DRIVE_PLACE) {
          currState = PathStates.PLACE;
        }
      } else if (shouldTransitionToServoing()) {
        currState = PathStates.DRIVE_PLACE_SERVO;
        isServoing = true;
      }
    }
  }

  private void drivePathServo() {
    if (isHandling && runningPath && currPath != null && isServoing) {
      driveSubsystem.calculateControllerServo(
          currPath.sampleAt(timer.get(), mirrorTrajectory).get(),
          0.0); // TODO: use tag servoing here when ready
      if (timer.hasElapsed(currPath.getTotalTime())) {
        driveSubsystem.setAutoDebugMsg("End " + currPathString);
        runningPath = false;
        timer.stop();
        timer.reset();
        driveSubsystem.calculateControllerServo(
            currPath.getFinalSample(mirrorTrajectory).get(), 0.0);
        if (currState == PathStates.DRIVE_FETCH) {
          currState = PathStates.FETCH;
        } else if (currState == PathStates.DRIVE_PLACE_SERVO) {
          isServoing = false;
          // RobotStateSubsystem.PlacePiece();
          currState = PathStates.PLACE;
        }
      }
    }
  }

  private Trajectory<SwerveSample> nextPath() {
    if (NodeNames.size() > 0) {
      if (currState == PathStates.DRIVE_FETCH) {
        currPathString = pathNames[NodeNames.get(0) - 'a'][0];
        return fetchPaths.get(NodeNames.get(0) - 'a');
      } else if (currState == PathStates.DRIVE_PLACE) {
        currPathString = pathNames[NodeNames.get(0) - 'a'][1];
        return placePaths.get(NodeNames.get(0) - 'a');
      }
    } else {
      killPathHandler();
    }
    return null;
  }

  private void advanceNodes() {
    if (NodeNames.size() > 0) {
      NodeNames.remove(0);
    }
  }

  public void killPathHandler() {
    isHandling = false;
    currState = PathStates.DONE;
    runningPath = false;
    timer.stop();
    timer.reset();
  }

  public void killPathHandlerAfterPath() {
    NodeNames.clear();
  }

  private boolean shouldTransitionToServoing() {
    boolean isCloseEnough = false;
    double preNormalizedAngle =
        FastMath.toRadians(
            RobotStateConstants.kNodeAngles[FastMath.floorToInt((NodeNames.get(0) - 'a') / 2)]);
    double goal =
        mirrorTrajectory
            ? FastMath.normalizeMinusPiPi(preNormalizedAngle + Math.PI)
            : preNormalizedAngle;
    double pos = driveSubsystem.getGyroRotation2d().getRadians();
    if (goal < -Math.PI / 2 || goal > Math.PI / 2) {
      isCloseEnough =
          FastMath.abs(FastMath.normalizeZeroTwoPi(pos) - FastMath.normalizeZeroTwoPi(goal))
              < FastMath.toRadians(TagServoingConstants.kAngleCloseEnough);
    } else {
      isCloseEnough =
          FastMath.abs(pos - goal) < FastMath.toRadians(TagServoingConstants.kAngleCloseEnough);
    }
    return currState == PathStates.DRIVE_PLACE
        && timer.hasElapsed(currPath.getTotalTime() - 1.0)
        && isCloseEnough;
  }

  public void periodic() {
    switch (currState) {
      case DRIVE_FETCH:
        if (!runningPath) {
          startPath(nextPath());
        }
        drivePath();
        break;
      case FETCH:
        // if (robotStateSubsystem.hasPiece) {
        advanceNodes();
        currState = PathStates.DRIVE_PLACE;
        // }
        break;
      case DRIVE_PLACE:
        if (!runningPath) {
          startPath(nextPath());
        }
        drivePath();
        break;
      case DRIVE_PLACE_SERVO:
        if (runningPath && isServoing) {
          drivePathServo();
        }
        break;
      case PLACE:
        // if (RobotStateSubsystem.State != RobotStateSubsystem.hasPiece) {
        currState = PathStates.DRIVE_FETCH;
        // }
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
    DRIVE_PLACE_SERVO,
    DONE
  }
}
