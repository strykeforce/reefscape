package frc.robot.commands.auton;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.PathHandlerConstants;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem.TagAlignStates;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveAlgaeAutonServoCommand extends Command implements AutoCommandInterface {
  private final DriveSubsystem driveSubsystem;
  private final TagAlignSubsystem tagAlignSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private final VisionSubsystem visionSubsystem;

  private Trajectory<SwerveSample> trajectory;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);
  private boolean pathExists = false;
  private String trajectoryName;
  private boolean mirrorTrajectory = false;
  private boolean mirrorToProcessor = false;
  private boolean isServoing = false;

  private boolean resetOdometry;
  private boolean firstPath;
  private boolean lastPath;
  private boolean hasStaged = false;
  private boolean hasPreppedAlgae = false;

  private double yOffset;
  private RobotStateSubsystem.ScoringLevel algaeLevel;

  private SwerveSample desiredState;
  private Pose2d initialPose = new Pose2d();
  private Pose2d finalPose = new Pose2d();

  public DriveAlgaeAutonServoCommand(
      DriveSubsystem driveSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      VisionSubsystem visionSubsystem,
      String trajectoryName,
      boolean firstPath,
      boolean lastPath,
      boolean resetOdometry,
      double yOffset,
      RobotStateSubsystem.ScoringLevel algaeLevel) {

    addRequirements(driveSubsystem, elevatorSubsystem, biscuitSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.tagAlignSubsystem = tagAlignSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.visionSubsystem = visionSubsystem;

    this.resetOdometry = resetOdometry;
    this.firstPath = firstPath;
    this.lastPath = lastPath;
    this.trajectoryName = trajectoryName;
    this.yOffset = yOffset;
    this.algaeLevel = algaeLevel;

    if (trajectoryName != null) {
      Optional<Trajectory<SwerveSample>> tempTrajectory = Choreo.loadTrajectory(trajectoryName);
      if (tempTrajectory.isPresent()) {
        trajectory = tempTrajectory.get();
        pathExists = true;
      } else {
        logger.error("Trajectory {} not found", trajectoryName);
        pathExists = false;
      }
    } else {
      pathExists = false;
    }
    org.littletonrobotics.junction.Logger.recordOutput("Auto/mirrorToProcessor", mirrorToProcessor);
    org.littletonrobotics.junction.Logger.recordOutput("Auto/mirrorTrajectory", mirrorTrajectory);
    timer.start();
  }

  @Override
  public void reassignAlliance() {
    mirrorTrajectory = driveSubsystem.shouldFlip();
    if (pathExists) {
      initialPose = trajectory.getInitialPose(mirrorTrajectory).get();
      finalPose = trajectory.getFinalPose(mirrorTrajectory).get();

      if (resetOdometry) {
        driveSubsystem.prepForAuto(initialPose, initialPose.getRotation().getDegrees());
        driveSubsystem.resetHolonomicController(initialPose.getRotation().getRadians());
      }
    }
  }

  @Override
  public void initialize() {
    if (firstPath) {
      elevatorSubsystem.zero();
    } else {
      robotStateSubsystem.clearCoral();
    }

    visionSubsystem.setIsAuto(false);
    robotStateSubsystem.setAutoAlgaeLevel(algaeLevel);

    isServoing = false;
    hasStaged = false;
    hasPreppedAlgae = false;

    if (pathExists) {
      if (resetOdometry) {
        driveSubsystem.resetOdometry(initialPose);
      }
      driveSubsystem.setEnableHolo(true);
      driveSubsystem.setAutoDebugMsg("Initialize " + trajectoryName);

      driveSubsystem.grapherTrajectoryActive(true);
      timer.reset();

      desiredState = trajectory.sampleAt(timer.get(), mirrorTrajectory).get();

      driveSubsystem.calculateController(desiredState);
    }
  }

  @Override
  public void execute() {

    if (elevatorSubsystem.getState() == ElevatorStates.ZEROED
        && tagAlignSubsystem.getCurRadius() <= 3
        && !hasPreppedAlgae) {
      hasPreppedAlgae = true;
      robotStateSubsystem.toReefAlignAlgaeAuto();
    }

    if (pathExists) {
      if (!isServoing) {
        desiredState = trajectory.sampleAt(timer.get(), mirrorTrajectory).get();
        driveSubsystem.calculateController(desiredState);

        if (shouldTransitionToServoing()) {
          isServoing = true;

          visionSubsystem.setIsAuto(true);

          tagAlignSubsystem.startAuto(
              robotStateSubsystem.getAllianceColor(),
              robotStateSubsystem.getCoralLevel(),
              yOffset,
              true,
              true);
        }
      }
    }

    if (robotStateSubsystem.getState() == RobotStateSubsystem.RobotStates.REEF_ALIGN_CORAL) {
      robotStateSubsystem.toPlaceCoralAuto();
    }
  }

  private boolean shouldTransitionToServoing() {
    return tagAlignSubsystem.getCurRadius() < PathHandlerConstants.kServoRadius;
  }

  @Override
  public boolean isFinished() {
    if (!pathExists) {
      return true;
    }
    return ((timer.hasElapsed(trajectory.getTotalTime() + AutonConstants.kAutoTimeout)
            || tagAlignSubsystem.getState() == TagAlignStates.DONE && isServoing))
        && robotStateSubsystem.hasAlgae()
        && (!firstPath || !robotStateSubsystem.hasCoral());
    // || (FastMath.sqrt(
    //             FastMath.pow(driveSubsystem.getPoseMeters().getX() - finalPose.getX(), 2)
    //                 + FastMath.pow(
    //                     (driveSubsystem.getPoseMeters().getY() - finalPose.getY()), 2))
    //         < AutonConstants.kMaxPathErrorMeters)
    //     && driveSubsystem.getHolonomicControllerOmegaErrorRadians()
    //         < AutonConstants.kMaxOmegaErrorRadians);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setEnableHolo(false);

    if (!interrupted && !lastPath && pathExists) {
      driveSubsystem.calculateController(
          trajectory.sampleAt(trajectory.getTotalTime(), mirrorTrajectory).get());
    } else {
      driveSubsystem.drive(0, 0, 0);
    }

    tagAlignSubsystem.terminate();
    visionSubsystem.setIsAuto(true);

    driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory {}: {}", trajectoryName, timer.get());
    driveSubsystem.setAutoDebugMsg("End " + trajectoryName);
  }
}
