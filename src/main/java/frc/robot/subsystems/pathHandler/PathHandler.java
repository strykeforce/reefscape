package frc.robot.subsystems.pathHandler;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotStateConstants;
import frc.robot.constants.TagServoingConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import net.jafama.FastMath;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class PathHandler extends MeasurableSubsystem {
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private TagAlignSubsystem tagAlignSubsystem;
  private static final org.slf4j.Logger logger = LoggerFactory.getLogger(PathHandler.class);

  private PathStates curState = PathStates.DONE;
  private boolean isHandling = false;
  private List<Character> nodeNames;
  private List<Integer> nodeLevels;
  private Timer timer = new Timer();
  private List<Trajectory<SwerveSample>> fetchPaths;
  private List<Trajectory<SwerveSample>> placePaths;
  private String[][] pathNames = new String[12][2];
  private Trajectory<SwerveSample> currPath;
  private String currPathString;
  private boolean runningPath = false;
  private boolean isServoing = false;
  private boolean mirrorTrajectory = false;
  private boolean mirrorToProcessor = false;
  private Character startNode = 'a';

  public PathHandler(
      DriveSubsystem driveSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      RobotStateSubsystem robotStateSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.tagAlignSubsystem = tagAlignSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    // reassignAlliance();
  }

  public PathHandler(
      DriveSubsystem driveSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      String[][] pathNames,
      List<Character> NodeNames,
      List<Integer> NodeLevels,
      Character startNode,
      boolean mirrorToProcessor) {
    this.driveSubsystem = driveSubsystem;
    this.tagAlignSubsystem = tagAlignSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.pathNames = pathNames;
    this.nodeNames = nodeNames;
    this.nodeLevels = nodeLevels;
    this.startNode = startNode;
    this.mirrorToProcessor = mirrorToProcessor;
  }

  public void setPathNames(String[][] pathNames) {
    if (!isHandling) {
      this.pathNames = pathNames;
    }
  }

  public void setNodeNames(List<Character> NodeNames) {
    if (!isHandling) {
      this.nodeNames = NodeNames;
    }
  }

  public void setNodeLevels(List<Integer> NodeLevels) {
    if (!isHandling) {
      this.nodeLevels = NodeLevels;
    }
  }

  public void setStartNode(Character startNode) {
    if (!isHandling) {
      this.startNode = startNode;
    }
  }

  public void setMirrorToProcessor(boolean mirrorToProcessor) {
    if (!isHandling) {
      this.mirrorToProcessor = mirrorToProcessor;
    }
  }

  public void startPathHandler() {
    nodeNames.add(0, startNode);
    isHandling = true;
    robotStateSubsystem.setIsAutoPlacing(false);
    curState = PathStates.DRIVE_FETCH;
  }

  public void reassignAlliance() {
    mirrorTrajectory = driveSubsystem.shouldFlip();
    Optional<Trajectory<SwerveSample>> temp;
    for (int i = 0; i < 12; i++) {
      logger.info(i + "");
      temp = Choreo.loadTrajectory(pathNames[i][0]);
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
      driveSubsystem.calculateController(
          mirrorToProcessor(currPath.getInitialSample(mirrorTrajectory).get()));
      if (curState == PathStates.PLACE) {
        curState = PathStates.DRIVE_FETCH;
      } else if (curState == PathStates.FETCH) {
        curState = PathStates.DRIVE_PLACE;
      }
    }
  }

  private void drivePath() {
    if (isHandling && runningPath && currPath != null) {
      driveSubsystem.calculateController(
          mirrorToProcessor(currPath.sampleAt(timer.get(), mirrorTrajectory).get()));
      if (timer.hasElapsed(currPath.getTotalTime())) {
        driveSubsystem.setAutoDebugMsg("End " + currPathString);
        runningPath = false;
        timer.stop();
        timer.reset();
        driveSubsystem.calculateController(
            mirrorToProcessor(currPath.getFinalSample(mirrorTrajectory).get()));
        if (curState == PathStates.DRIVE_FETCH) {
          curState = PathStates.FETCH;
        } else if (curState == PathStates.DRIVE_PLACE) {
          curState = PathStates.PLACE;
        }
      } else if (shouldTransitionToServoing()) {
        curState = PathStates.DRIVE_PLACE_SERVO;
        isServoing = true;
      }
    }
  }

  private void drivePathServo() {
    if (isHandling && runningPath && currPath != null && isServoing) {
      driveSubsystem.calculateControllerServo(
          mirrorToProcessor(currPath.sampleAt(timer.get(), mirrorTrajectory).get()),
          0.0); // TODO: use tag servoing here when ready
      if (timer.hasElapsed(currPath.getTotalTime())) {
        driveSubsystem.setAutoDebugMsg("End " + currPathString);
        runningPath = false;
        timer.stop();
        timer.reset();
        driveSubsystem.calculateControllerServo(
            mirrorToProcessor(currPath.getFinalSample(mirrorTrajectory).get()), 0.0);
        if (curState == PathStates.DRIVE_FETCH) {
          curState = PathStates.FETCH;
        } else if (curState == PathStates.DRIVE_PLACE_SERVO) {
          isServoing = false;
          robotStateSubsystem.toPrepCoral();
          curState = PathStates.PLACE;
        }
      }
    }
  }

  private Trajectory<SwerveSample> nextPath() {
    if (nodeNames.size() > 0) {
      if (curState == PathStates.DRIVE_FETCH) {
        currPathString = pathNames[nodeNames.get(0) - 'a'][0];
        return fetchPaths.get(nodeNames.get(0) - 'a');
      } else if (curState == PathStates.DRIVE_PLACE) {
        currPathString = pathNames[nodeNames.get(0) - 'a'][1];
        return placePaths.get(nodeNames.get(0) - 'a');
      }
    } else {
      killPathHandler();
    }
    return null;
  }

  private void advanceNodes() {
    if (nodeNames.size() > 0) {
      nodeNames.remove(0);
    }
  }

  public void killPathHandler() {
    isHandling = false;
    curState = PathStates.DONE;
    runningPath = false;
    robotStateSubsystem.setIsAutoPlacing(true);
    timer.stop();
    timer.reset();
  }

  public void killPathHandlerAfterPath() {
    nodeNames.clear();
  }

  private boolean shouldTransitionToServoing() {
    boolean isCloseEnough = false;
    double preNormalizedAngle =
        FastMath.toRadians(
            RobotStateConstants.kNodeAngles[FastMath.floorToInt((nodeNames.get(0) - 'a') / 2)]);
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
    return curState == PathStates.DRIVE_PLACE
        && timer.hasElapsed(currPath.getTotalTime() - 1.0)
        // && TagAlignSubsystem.canSeeTag(desiredTag)
        && isCloseEnough;
  }

  private SwerveSample mirrorToProcessor(SwerveSample sample) {
    if (mirrorToProcessor) {
      sample =
          new SwerveSample(
              sample.t,
              sample.x,
              DriveConstants.kFieldMaxY - sample.y,
              sample.heading * -1,
              sample.vx,
              DriveConstants.kFieldMaxY - sample.vy,
              sample.omega * -1,
              sample.ax,
              DriveConstants.kFieldMaxY - sample.ay,
              sample.alpha * -1,
              sample.moduleForcesX(),
              new double[] {
                sample.moduleForcesY()[0] * -1,
                sample.moduleForcesY()[1] * -1,
                sample.moduleForcesY()[2] * -1,
                sample.moduleForcesY()[3] * -1
              });
    }
    return sample;
  }

  public boolean isFinished() {
    return !isHandling;
  }

  @Override
  public void periodic() {
    switch (curState) {
      case DRIVE_FETCH -> {
        if (!runningPath) {
          startPath(nextPath());
        }
        drivePath();
      }
      case FETCH -> {
        if (robotStateSubsystem.hasCoral()) {
          advanceNodes();
          curState = PathStates.DRIVE_PLACE;
        }
      }
      case DRIVE_PLACE -> {
        if (!runningPath) {
          startPath(nextPath());
        }
        drivePath();
      }
      case DRIVE_PLACE_SERVO -> {
        if (runningPath && isServoing) {
          tagAlignSubsystem.setup(
              robotStateSubsystem.getAllianceColor(),
              robotStateSubsystem.getScoreSide() == RobotStateSubsystem.ScoreSide.LEFT);
          drivePathServo();
        }
      }
      case PLACE -> {
        if (!robotStateSubsystem.hasCoral()) {
          curState = PathStates.DRIVE_FETCH;
        }
      }
      case DONE -> isHandling = false;
      default -> {}
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("State", () -> curState.ordinal()));
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
