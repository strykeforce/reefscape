package frc.robot.subsystems.pathHandler;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.PathHandlerConstants;
import frc.robot.constants.TagServoingConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem.TagAlignStates;
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
  private List<ScoringLevel> nodeLevels =
      new ArrayList<>(); // the list of levels, in order, to score on
  private List<Double> lightDistances = new ArrayList<>();
  private List<Double> postOffsets = new ArrayList<>();
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
  private Pose2d alignTargetPose = new Pose2d();
  private int targetHexant = 0;
  private boolean runningPath = false;
  private boolean isServoing = false;
  private boolean mirrorTrajectory = false;
  private boolean proceedToNext = false;
  private boolean teleop = false;
  private boolean isPlacing = false;
  private boolean hasPreppedCoral = false;
  private boolean hasStagedAlgae = false;
  private boolean algaeOnLast = false;
  private boolean hasLEDsOn = false;

  private DigitalOutput lights = new DigitalOutput(3);

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
      List<ScoringLevel> nodeLevels,
      List<Double> lightDistances,
      List<Double> postOffsets,
      Character startNode,
      boolean mirrorToProcessor) {
    this.driveSubsystem = driveSubsystem;
    this.tagAlignSubsystem = tagAlignSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.pathNames = pathNames;
    this.nodeNames = nodeNames;
    this.nodeLevels = nodeLevels;
    this.lightDistances = lightDistances;
    this.postOffsets = postOffsets;
    this.startNode = startNode;
    this.mirrorToProcessor = mirrorToProcessor;

    setHeadlights(true);
  }

  public void setLightDistances(List<Double> lightDistances) {
    this.lightDistances = lightDistances;
  }

  public void setPostOffsets(List<Double> postOffsets) {
    this.postOffsets = postOffsets;
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

  public void addNode(Character nodeName, ScoringLevel nodeLevel) {
    nodeNames.add(nodeName);
    nodeLevels.add(nodeLevel);
  }

  public void setNodeLevels(List<ScoringLevel> nodeLevels) {
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

  public void setProceedToNext(boolean proceed) {
    this.proceedToNext = proceed;
  }

  public void setGetAlgaeLast(boolean algae) {
    this.algaeOnLast = algae;
  }

  public void startPathHandler() {
    teleop = false;
    nodeNames.add(0, startNode);
    nodeLevels.add(0, ScoringLevel.L4); // a dummy level
    postOffsets.add(0, 0.0); // dummy offset

    isHandling = true;
    robotStateSubsystem.setIsAutoPlacing(false);
    curState = PathStates.DRIVE_FETCH;
  }

  public void reassignAlliance() {
    fetchPaths.clear();
    placePaths.clear();
    mirrorTrajectory = driveSubsystem.shouldFlip();
    Optional<Trajectory<SwerveSample>> temp;
    for (int i = 0; i < 12; i++) {
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
    proceedToNext = false;
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
        robotStateSubsystem.setLEDLoadCoral(false);
        hasLEDsOn = false;
        // setHeadlights(true);
        curState = PathStates.DRIVE_PLACE;
      }
    }
  }

  private void drivePath() {
    if (isHandling && runningPath && currPath != null) {
      driveSubsystem.calculateController(
          mirrorToProcessor(currPath.sampleAt(pathTimer.get(), mirrorTrajectory).get()));
      if (pathTimer.hasElapsed(currPath.getTotalTime() + AutonConstants.kAutoTimeout)
          || (Math.sqrt(
                      FastMath.pow2(
                              driveSubsystem.getPoseMeters().getX() - currPathFinalPose.getX())
                          + FastMath.pow2(
                              driveSubsystem.getPoseMeters().getY() - currPathFinalPose.getY()))
                  < AutonConstants.kMaxPathErrorMeters
              && Math.abs(driveSubsystem.getHolonomicControllerOmegaErrorRadians())
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
      } else if (robotStateSubsystem.hasCoralAuton() && curState == PathStates.DRIVE_FETCH) {
        driveSubsystem.setAutoDebugMsg("End " + currPathString);
        runningPath = false;
        pathTimer.stop();
        pathTimer.reset();
        driveSubsystem.drive(0, 0, 0);
        advanceNodes();
        robotStateSubsystem.setLEDLoadCoral(false);
        hasLEDsOn = false;
        // setHeadlights(true);
        curState = PathStates.DRIVE_PLACE;

      } else if (shouldTransitionToServoing()) {
        tagAlignSubsystem.startAuto(
            mirrorTrajectory ? Alliance.Red : Alliance.Blue,
            robotStateSubsystem.getCoralLevel(),
            postOffsets.get(0),
            (nodeNames.get(0) - 'a') % 2 == 0,
            false);
        driveSubsystem.setAutoDebugMsg("Servo Start");
        if (!hasPreppedCoral) {
          if (algaeOnLast && nodeNames.size() == 1) {
            robotStateSubsystem.toReefAlignAlgaeAuto();
          } else {
            robotStateSubsystem.toPrepCoral();
          }
          hasPreppedCoral = true;
        }
        curState = PathStates.DRIVE_PLACE_SERVO;
        isServoing = true;
      }
    }
  }

  private void drivePathServo() {
    if (isHandling && isServoing) {
      if (tagAlignSubsystem.getState() == TagAlignStates.DONE) {
        isServoing = false;
        driveSubsystem.setAutoDebugMsg("End " + currPathString);
        runningPath = false;
        pathTimer.stop();
        pathTimer.reset();
        curState = PathStates.PLACE;
        driveSubsystem.stopDriving();
        waitingTimer.start();
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
          mirrorTrajectory ? Alliance.Red : Alliance.Blue,
          robotStateSubsystem.getCoralLevel(),
          (nodeNames.get(0) - 'a') % 2 == (mirrorToProcessor ? 1 : 0),
          false);
      nodeNames.remove(0);
      nodeLevels.remove(0);
      lightDistances.remove(0);
      postOffsets.remove(0);
    }
  }

  public void killPathHandler() {
    isHandling = false;
    curState = PathStates.DONE;
    runningPath = false;
    robotStateSubsystem.setLEDLoadCoral(false);
    robotStateSubsystem.setIsAutoPlacing(true);
    pathTimer.stop();
    pathTimer.reset();
  }

  public void killPathHandlerAfterPath() {
    nodeNames.clear();
  }

  private boolean shouldTransitionToServoing() {

    return tagAlignSubsystem.getCurRadius() < PathHandlerConstants.kServoRadius
        && curState == PathStates.DRIVE_PLACE;
    /*&& FastMath.abs(
        alignTargetPose
            .getTranslation()
            .minus(driveSubsystem.getPoseMeters().getTranslation())
            .rotateBy(
                Rotation2d.fromRadians(-TagServoingConstants.kAngleTarget[targetHexant]))
            .getY())
    < PathHandlerConstants.kMaxServoErrorY;*/
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

  private boolean shouldStageElevator() {
    return tagAlignSubsystem.getCurRadius() < AutonConstants.kElevatorStageRadius
        && curState == PathStates.DRIVE_PLACE
        && getCurError() < PathHandlerConstants.kMaxServoErrorY;
  }

  private double getCurError() {
    return FastMath.abs(
        alignTargetPose
            .getTranslation()
            .minus(driveSubsystem.getPoseMeters().getTranslation())
            .rotateBy(Rotation2d.fromRadians(-TagServoingConstants.kAngleTarget[targetHexant]))
            .getY());
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

  public void setHeadlights(boolean on) {
    lights.set(on);
  }

  @Override
  public void periodic() {
    org.littletonrobotics.junction.Logger.recordOutput("PathHandler/State", curState);
    org.littletonrobotics.junction.Logger.recordOutput("PathHandler/curPath", currPathString);
    org.littletonrobotics.junction.Logger.recordOutput(
        "PathHandler/curRadius", tagAlignSubsystem.getCurRadius());
    org.littletonrobotics.junction.Logger.recordOutput("PathHandler/hasLEDsOn", hasLEDsOn);
    org.littletonrobotics.junction.Logger.recordOutput(
        "PathHandler/hasStagedAlgae", hasStagedAlgae);

    switch (curState) {
      case DRIVE_FETCH -> {
        if (!runningPath) {
          startPath(nextPath());
        }
        double fetchDistance =
            Math.sqrt(
                FastMath.pow2(driveSubsystem.getPoseMeters().getX() - currPathFinalPose.getX())
                    + FastMath.pow2(
                        driveSubsystem.getPoseMeters().getY() - currPathFinalPose.getY()));
        org.littletonrobotics.junction.Logger.recordOutput(
            "PathHandler/fetchDistance", fetchDistance);
        if (fetchDistance < lightDistances.get(0) && !hasLEDsOn) {
          robotStateSubsystem.setLEDLoadCoral(true);
          hasLEDsOn = true;
          // setHeadlights(false);
        }
        drivePath();
      }
      case FETCH -> {
        if (robotStateSubsystem.hasCoral() || (robotStateSubsystem.hasCoralAuton())) {
          // waitingTimer.hasElapsed(PathHandlerConstants.kWaitingTime)
          // proceedToNext) {
          advanceNodes();

          robotStateSubsystem.setLEDLoadCoral(false);
          hasLEDsOn = false;
          // setHeadlights(true);
          curState = PathStates.DRIVE_PLACE;
        }
      }
      case DRIVE_PLACE -> {
        if (algaeOnLast
            && nodeNames.size() == 1
            && !hasStagedAlgae
            && robotStateSubsystem.hasCoral()) {
          robotStateSubsystem.toReefAlignAlgaeAuto();
          hasStagedAlgae = true;
        }
        if (
        /*tagAlignSubsystem.getCurRadius(robotStateSubsystem.getAllianceColor())
        <= AutonConstants.kElevatorStageRadius*/ shouldStageElevator()
            && !hasPreppedCoral
            && !hasStagedAlgae) {
          hasPreppedCoral = true;
          robotStateSubsystem.toPrepCoral();
        }
        if (!runningPath) {
          if (nodeNames.size() > 0) {
            char next = nodeNames.get(0);
            targetHexant = tagAlignSubsystem.computeFieldRelHexant(currPathFinalPose);
            alignTargetPose =
                tagAlignSubsystem.getTargetDrivePose((next - 'a') % 2 == 0, targetHexant);
            robotStateSubsystem.setScoringLevel(nodeLevels.get(0));
          }
          startPath(nextPath());
        }
        drivePath();
      }
      case DRIVE_PLACE_SERVO -> {
        if (
        /*tagAlignSubsystem.getCurRadius(
            driveSubsystem.shouldFlip() ? Alliance.Red : Alliance.Blue)
        <= AutonConstants.kElevatorStageRadius*/ shouldStageElevator()
            && !hasPreppedCoral) {
          hasPreppedCoral = true;
          if (algaeOnLast && nodeNames.size() == 1) {
            robotStateSubsystem.setScoringLevel(nodeLevels.get(0));
            robotStateSubsystem.toReefAlignAlgaeAuto();
          } else {
            robotStateSubsystem.toPrepCoral();
          }
        }
        if (runningPath && isServoing) {
          drivePathServo();
        }
      }
      case PLACE -> {
        if (algaeOnLast && nodeNames.size() == 1) {
          killPathHandler();
          return;
        }
        hasPreppedCoral = false;
        hasStagedAlgae = false;
        if (robotStateSubsystem.isElevatorFinished() && !isPlacing) {
          isPlacing = true;
          robotStateSubsystem.toPlaceCoralAuto();
        }
        if (!robotStateSubsystem.hasCoral()) {
          // waitingTimer.hasElapsed(PathHandlerConstants.kWaitingTime)
          // proceedToNext) {
          isPlacing = false;
          robotStateSubsystem.toFunnelLoad();
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
