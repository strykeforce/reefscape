package frc.robot.subsystems.pathHandler;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.PathHandlerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import java.util.ArrayList;
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

  // parameters
  private String[][] pathNames =
      PathHandlerConstants.kShallowPathNames; // the array of paths, in string form
  private List<Character> nodeNames = new ArrayList<>(); // the list of nodes, in order, to score on
  private List<Integer> nodeLevels = new ArrayList<>(); // the list of levels, in order, to score on
  private Character startNode = 'a'; // the node that the robot starts in front of
  private boolean mirrorToProcessor =
      false; // whether the robot starts and fetches on the processor side.

  private PathStates curState = PathStates.DONE;
  private boolean isHandling = false;
  private Timer pathTimer = new Timer();
  private Timer waitingTimer = new Timer();
  private List<Trajectory<SwerveSample>> fetchPaths = new ArrayList<>();
  private List<Trajectory<SwerveSample>> placePaths = new ArrayList<>();
  private Trajectory<SwerveSample> currPath;
  private String currPathString;
  private Pose2d currPathFinalPose = new Pose2d();
  private boolean runningPath = false;
  private boolean isServoing = false;
  private boolean mirrorTrajectory = false;

  public PathHandler(
      DriveSubsystem driveSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      RobotStateSubsystem robotStateSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.tagAlignSubsystem = tagAlignSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    reassignAlliance();
  }

  public PathHandler(
      DriveSubsystem driveSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      String[][] pathNames,
      List<Character> nodeNames,
      List<Integer> nodeLevels,
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
    reassignAlliance();
  }

  public void setNodeNames(List<Character> nodeNames) {
    if (!isHandling) {
      this.nodeNames = nodeNames;
    }
  }

  public void setNodeLevels(List<Integer> nodeLevels) {
    if (!isHandling) {
      this.nodeLevels = nodeLevels;
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
      temp = Choreo.loadTrajectory(pathNames[0][i]);
      if (!temp.isEmpty()) {
        fetchPaths.add(temp.get());
      } else {
        logger.error("path does not exist: {}", pathNames[0][i]);
      }
      temp = Choreo.loadTrajectory(pathNames[1][i]);
      if (!temp.isEmpty()) {
        placePaths.add(temp.get());
      } else {
        logger.error("path does not exist: {}", pathNames[0][i]);
      }
    }
  }

  private void startPath(Trajectory<SwerveSample> path) {
    waitingTimer.stop();
    waitingTimer.reset();
    if (isHandling && path != null) {
      driveSubsystem.setAutoDebugMsg("Start " + currPathString);
      logger.info("start Path:" + currPathString);
      currPath = path;
      currPathFinalPose = mirrorToProcessor(path.getFinalPose(mirrorTrajectory).get());
      runningPath = true;
      pathTimer.stop();
      pathTimer.reset();
      pathTimer.start();
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
          mirrorToProcessor(currPath.sampleAt(pathTimer.get(), mirrorTrajectory).get()));
      if (pathTimer.hasElapsed(currPath.getTotalTime() + AutonConstants.kAutoTimeout)
          || (FastMath.sqrt(
                      FastMath.pow(
                              driveSubsystem.getPoseMeters().getX() - currPathFinalPose.getX(), 2)
                          + FastMath.pow(
                              driveSubsystem.getPoseMeters().getY() - currPathFinalPose.getY(), 2))
                  < AutonConstants.kMaxPathErrorMeters
              && driveSubsystem.getHolonomicControllerOmegaErrorRadians()
                  < AutonConstants.kMaxOmegaErrorRadians)) {
        driveSubsystem.setAutoDebugMsg("End " + currPathString);
        runningPath = false;
        pathTimer.stop();
        pathTimer.reset();
        // driveSubsystem.calculateController(
        //     mirrorToProcessor(currPath.getFinalSample(mirrorTrajectory).get()));
        driveSubsystem.drive(0, 0, 0);
        if (curState == PathStates.DRIVE_FETCH) {
          curState = PathStates.FETCH;
          waitingTimer.start();
        } else if (curState == PathStates.DRIVE_PLACE) {
          curState = PathStates.PLACE;
          waitingTimer.start();
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
          mirrorToProcessor(currPath.sampleAt(pathTimer.get(), mirrorTrajectory).get()),
          tagAlignSubsystem.calculateAlignY());
      if (pathTimer.hasElapsed(currPath.getTotalTime())) {
        driveSubsystem.setAutoDebugMsg("End " + currPathString);
        runningPath = false;
        pathTimer.stop();
        pathTimer.reset();
        driveSubsystem.calculateControllerServo(
            mirrorToProcessor(currPath.getFinalSample(mirrorTrajectory).get()), 0.0);
        if (curState == PathStates.DRIVE_FETCH) {
          curState = PathStates.FETCH;
          waitingTimer.start();
        } else if (curState == PathStates.DRIVE_PLACE_SERVO) {
          isServoing = false;
          robotStateSubsystem.toPrepCoral();
          curState = PathStates.PLACE;
          waitingTimer.start();
        }
      }
    }
  }

  private Trajectory<SwerveSample> nextPath() {
    if (nodeNames.size() > 0) {
      int index;
      char nextNode = nodeNames.get(0);
      if (nextNode >= 97 && nextNode <= 108) {
        index = nextNode - 'a';
        if (curState == PathStates.DRIVE_FETCH) {
          currPathString = pathNames[0][index];
          return fetchPaths.get(index);
        } else if (curState == PathStates.DRIVE_PLACE) {
          currPathString = pathNames[1][index];
          return placePaths.get(index);
        }
      } else {
        logger.error("Bad node name! Check the node list for upper case and any letter past \"l\"");
        killPathHandler();
      }
    } else {
      killPathHandler();
    }
    return null;
  }

  private void advanceNodes() {
    if (nodeNames.size() > 0) {
      tagAlignSubsystem.setup(
          mirrorTrajectory ? Alliance.Red : Alliance.Blue, (nodeNames.get(0) - 'a') % 2 == 0);
      nodeNames.remove(0);
    }
  }

  public void killPathHandler() {
    isHandling = false;
    curState = PathStates.DONE;
    runningPath = false;
    robotStateSubsystem.setIsAutoPlacing(true);
    pathTimer.stop();
    pathTimer.reset();
  }

  public void killPathHandlerAfterPath() {
    nodeNames.clear();
  }

  private boolean shouldTransitionToServoing() {
    return false;
    // boolean isCloseEnough = false;
    // double preNormalizedAngle =
    //     FastMath.toRadians(
    //         RobotStateConstants.kNodeAngles[FastMath.floorToInt((nodeNames.get(0) - 'a') / 2)]);
    // double goal =
    //     mirrorTrajectory
    //         ? FastMath.normalizeMinusPiPi(preNormalizedAngle + Math.PI)
    //         : preNormalizedAngle;
    // double pos = driveSubsystem.getGyroRotation2d().getRadians();
    // if (goal < -Math.PI / 2 || goal > Math.PI / 2) {
    //   isCloseEnough =
    //       FastMath.abs(FastMath.normalizeZeroTwoPi(pos) - FastMath.normalizeZeroTwoPi(goal))
    //           < FastMath.toRadians(TagServoingConstants.kAngleCloseEnough);
    // } else {
    //   isCloseEnough =
    //       FastMath.abs(pos - goal) < FastMath.toRadians(TagServoingConstants.kAngleCloseEnough);
    // }
    // return curState == PathStates.DRIVE_PLACE
    //     && pathTimer.hasElapsed(currPath.getTotalTime() - 1.0)
    //     // && TagAlignSubsystem.canSeeTag(desiredTag)
    //     && isCloseEnough
    //     && false; // TODO remove, this is for testing
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
              sample.vy * -1,
              sample.omega * -1,
              sample.ax,
              sample.ay * -1,
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

  private Pose2d mirrorToProcessor(Pose2d pose) {
    if (mirrorToProcessor) {
      pose =
          new Pose2d(
              pose.getX(),
              DriveConstants.kFieldMaxY - pose.getY(),
              Rotation2d.fromDegrees(pose.getRotation().getDegrees() * -1));
    }
    return pose;
  }

  public boolean isFinished() {
    return !isHandling;
  }

  @Override
  public void periodic() {
    org.littletonrobotics.junction.Logger.recordOutput("PathHandler/State", curState);
    org.littletonrobotics.junction.Logger.recordOutput("PathHandler/curPath", currPathString);
    switch (curState) {
      case DRIVE_FETCH -> {
        if (!runningPath) {
          startPath(nextPath());
        }
        drivePath();
      }
      case FETCH -> {
        if (
        // robotStateSubsystem.hasCoral() ||
        // (robotStateSubsystem.hasCoralAuton() &&
        waitingTimer.hasElapsed(PathHandlerConstants.kWaitingTime)) {
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
        if (
        // !robotStateSubsystem.hasCoral() &&
        waitingTimer.hasElapsed(PathHandlerConstants.kWaitingTime)) {
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
