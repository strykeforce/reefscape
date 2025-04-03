package frc.robot.commands.auton;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.PathHandlerConstants;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem.TagAlignStates;
import java.util.Optional;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveAutonServoCommand extends Command implements AutoCommandInterface {
  private final DriveSubsystem driveSubsystem;
  private final TagAlignSubsystem tagAlignSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;

  private Trajectory<SwerveSample> trajectory;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);
  private boolean isTherePath = false;
  private String trajectoryName;
  private boolean mirrorTrajectory = false;
  private boolean mirrorToProcessor = false;
  private boolean isServoing = false;

  private boolean resetOdometry;
  private boolean lastPath;
  private boolean scoreLeft;
  private boolean hasStaged = false;
  private boolean hasPreppedCoral = false;

  private double yOffset;

  private SwerveSample desiredState;
  private Pose2d finalPose;
  private Pose2d initialPose = new Pose2d();

  public DriveAutonServoCommand(
      DriveSubsystem driveSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      String trajectoryName,
      boolean lastPath,
      boolean resetOdometry,
      boolean mirrorToProcessor,
      boolean scoreLeft,
      double yOffset) {

    addRequirements(driveSubsystem, elevatorSubsystem, biscuitSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.tagAlignSubsystem = tagAlignSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;

    this.resetOdometry = resetOdometry;
    this.lastPath = lastPath;
    this.trajectoryName = trajectoryName;
    this.mirrorToProcessor = mirrorToProcessor;
    this.scoreLeft = scoreLeft;
    this.yOffset = yOffset;
    Optional<Trajectory<SwerveSample>> tempTrajectory = Choreo.loadTrajectory(trajectoryName);
    if (tempTrajectory.isPresent()) {
      trajectory = tempTrajectory.get();
      isTherePath = true;
    } else {
      logger.error("Trajectory {} not found", trajectoryName);
      isTherePath = false;
    }
    org.littletonrobotics.junction.Logger.recordOutput("Auto/mirrorToProcessor", mirrorToProcessor);
    org.littletonrobotics.junction.Logger.recordOutput("Auto/mirrorTrajectory", mirrorTrajectory);
    timer.start();
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

  @Override
  public void reassignAlliance() {
    mirrorTrajectory = driveSubsystem.shouldFlip();
    if (isTherePath) {

      initialPose = mirrorToProcessor(trajectory.getInitialPose(mirrorTrajectory).get());
      // driveSubsystem.calculateController(trajectory.sampleAt(0, mirrorTrajectory).get());
      finalPose = mirrorToProcessor(trajectory.getFinalPose(mirrorTrajectory).get());
      if (resetOdometry) {
        driveSubsystem.prepForAuto(initialPose, initialPose.getRotation().getDegrees());
        // driveSubsystem.resetOdometry(initialPose);
        driveSubsystem.resetHolonomicController(initialPose.getRotation().getRadians());
      }
    }
  }

  @Override
  public void initialize() {
    // if (isTherePath) {
    //   driveSubsystem.setAutoDebugMsg("Initialize " + trajectoryName);
    //   Pose2d initialPose = new Pose2d();
    //   initialPose = mirrorToProcessor(trajectory.getInitialPose(mirrorTrajectory).get());
    //   if (resetOdometry) {
    //     driveSubsystem.resetOdometry(initialPose);
    // driveSubsystem.resetHolonomicController();
    //   }
    elevatorSubsystem.zero();
    isServoing = false;
    hasStaged = false;

    if (isTherePath) {
      driveSubsystem.setEnableHolo(true);
      // driveSubsystem.recordAutoTrajectory(trajectory);
      driveSubsystem.setAutoDebugMsg("Initialize " + trajectoryName);

      driveSubsystem.grapherTrajectoryActive(true);
      timer.reset();
      // logger.info("Begin Trajectory: {}", trajectoryName);
      desiredState = mirrorToProcessor(trajectory.sampleAt(timer.get(), mirrorTrajectory).get());
      driveSubsystem.calculateController(desiredState);
      if (resetOdometry) {
        driveSubsystem.resetOdometry(initialPose);
      }
    }
  }

  @Override
  public void execute() {
    if (elevatorSubsystem.getState() == ElevatorStates.ZEROED
        && !hasStaged
        && timer.hasElapsed(AutonConstants.kInitPathPrestageTime)) {
      hasStaged = true;
      robotStateSubsystem.toAutonPrestage();
    }
    if (tagAlignSubsystem.getCurRadius() <= AutonConstants.kElevatorStageRadiusPathOne
        && !hasPreppedCoral) {
      hasPreppedCoral = true;
      robotStateSubsystem.toPrepCoral();
    }
    if (isTherePath) {
      if (!isServoing) {
        desiredState = mirrorToProcessor(trajectory.sampleAt(timer.get(), mirrorTrajectory).get());
        driveSubsystem.calculateController(desiredState);

        if (shouldTransitionToServoing()) {
          isServoing = true;
          tagAlignSubsystem.startAuto(
              mirrorTrajectory ? Alliance.Red : Alliance.Blue,
              robotStateSubsystem.getCoralLevel(),
              yOffset,
              mirrorToProcessor ? !scoreLeft : scoreLeft,
              false);
        }
      }
    }
    org.littletonrobotics.junction.Logger.recordOutput("Auto/mirrorToProcessor", mirrorToProcessor);
    org.littletonrobotics.junction.Logger.recordOutput("Auto/mirrorTrajectory", mirrorTrajectory);
  }

  private boolean shouldTransitionToServoing() {
    return tagAlignSubsystem.getCurRadius() < PathHandlerConstants.kServoRadius;
  }

  @Override
  public boolean isFinished() {
    if (!isTherePath) {
      return true;
    }
    return ((timer.hasElapsed(trajectory.getTotalTime() + AutonConstants.kAutoTimeout)
            || tagAlignSubsystem.getState() == TagAlignStates.DONE && isServoing))
        && elevatorSubsystem.isFinished();
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
    // driveSubsystem.recordAutoTrajectory(null);

    if (!interrupted && !lastPath) {
      driveSubsystem.calculateController(
          mirrorToProcessor(
              trajectory.sampleAt(trajectory.getTotalTime(), mirrorTrajectory).get()));
    } else {
      driveSubsystem.drive(0, 0, 0);
    }
    isServoing = false;
    tagAlignSubsystem.terminate();

    driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory {}: {}", trajectoryName, timer.get());
    driveSubsystem.setAutoDebugMsg("End " + trajectoryName);
  }
}
