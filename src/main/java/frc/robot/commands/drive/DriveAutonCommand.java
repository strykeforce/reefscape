package frc.robot.commands.drive;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auton.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Optional;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveAutonCommand extends Command implements AutoCommandInterface {
  private final DriveSubsystem driveSubsystem;
  private Trajectory<SwerveSample> trajectory;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);
  private boolean isTherePath = false;
  private String trajectoryName;
  private boolean mirrorTrajectory = false;

  private boolean resetOdometry;
  private boolean lastPath;

  public DriveAutonCommand(
      DriveSubsystem driveSubsystem,
      String trajectoryName,
      boolean lastPath,
      boolean resetOdometry) {

    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.resetOdometry = resetOdometry;
    this.lastPath = lastPath;
    this.trajectoryName = trajectoryName;
    Optional<Trajectory<SwerveSample>> tempTrajectory = Choreo.loadTrajectory(trajectoryName);
    if (tempTrajectory.isPresent()) {
      trajectory = tempTrajectory.get();
      isTherePath = true;
    } else {
      logger.error("Trajectory {} not found", trajectoryName);
      isTherePath = false;
    }
    timer.start();
  }

  @Override
  public void reassignAlliance() {
    mirrorTrajectory = driveSubsystem.shouldFlip();
  }

  @Override
  public void initialize() {
    Pose2d initialPose = new Pose2d();
    if (trajectory.getInitialPose(mirrorTrajectory) != null) {
      initialPose = trajectory.getInitialPose(mirrorTrajectory).get();
    }
    if (resetOdometry && trajectory.getInitialPose(mirrorTrajectory) != null) {
      driveSubsystem.resetOdometry(initialPose);
    }
    driveSubsystem.setEnableHolo(true);
    // driveSubsystem.recordAutoTrajectory(trajectory);
    driveSubsystem.resetHolonomicController();
    driveSubsystem.grapherTrajectoryActive(true);
    timer.reset();
    logger.info("Begin Trajectory: {}", trajectoryName);
    SwerveSample desiredState = trajectory.sampleAt(timer.get(), mirrorTrajectory).get();
    driveSubsystem.calculateController(desiredState);
  }

  @Override
  public void execute() {
    SwerveSample desiredState = trajectory.sampleAt(timer.get(), mirrorTrajectory).get();
    driveSubsystem.calculateController(desiredState);
  }

  @Override
  public boolean isFinished() {
    if (isTherePath) {
      return false;
    }
    return timer.hasElapsed(trajectory.getTotalTime());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setEnableHolo(false);
    // driveSubsystem.recordAutoTrajectory(null);

    if (!interrupted && !lastPath) {
      driveSubsystem.calculateController(
          trajectory.sampleAt(trajectory.getTotalTime(), mirrorTrajectory).get());
    } else {
      driveSubsystem.drive(0, 0, 0);
    }

    driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory {}: {}", trajectoryName, timer.get());
  }
}
