package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Set;
import net.jafama.FastMath;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ClimbAlignSubsystem extends MeasurableSubsystem {
  private ClimbSubsystem climbSubsystem;
  private DriveSubsystem driveSubsystem;
  private ClimbAlignStates curState = ClimbAlignStates.DONE;

  private PIDController driveX;
  private PIDController driveY;
  private PIDController driveOmega;

  private Pose2d targetPose;
  private boolean directionPositive;
  private Timer delay = new Timer();

  public ClimbAlignSubsystem(ClimbSubsystem climbSubsystem, DriveSubsystem driveSubsystem) {
    this.climbSubsystem = climbSubsystem;
    this.driveSubsystem = driveSubsystem;

    this.driveX = new PIDController(4, 0, 0);
    this.driveY = new PIDController(4, 0, 0);
    this.driveOmega = new PIDController(6.0, 0, 0);

    Logger.recordOutput("ClimbAlignSubsystem/DriveXError", 0);
    Logger.recordOutput("ClimbAlignSubsystem/DriveYError", 0);
    Logger.recordOutput("ClimbAlignSubsystem/DriveOmegaError", 0);
    Logger.recordOutput("ClimbAlignSubsystem/Target Pose", new Pose2d());
  }

  public ClimbAlignStates getState() {
    return curState;
  }

  public void start(Alliance alliance) {
    driveSubsystem.setIgnoreSticks(true);
    driveX.reset();
    driveY.reset();
    driveOmega.reset();
    delay.reset();

    Pose2d starting = driveSubsystem.getPoseMeters();
    directionPositive = starting.getX() < ClimbConstants.kCenterX;

    double x = ClimbConstants.kCenterX;
    double y = ClimbConstants.kCenterY;
    Rotation2d yaw;

    if (directionPositive) {
      x -= ClimbConstants.kCageOffsetX;
      yaw = Rotation2d.fromDegrees(-90);
    } else {
      x += ClimbConstants.kCageOffsetX;
      yaw = Rotation2d.fromDegrees(90);
    }

    double climbOffset =
        directionPositive ? -ClimbConstants.kClimbRobotOffset : ClimbConstants.kClimbRobotOffset;

    double centerDist = FastMath.abs(starting.getY() + climbOffset - ClimbConstants.kCenterY);
    double minDist = FastMath.abs(centerDist - ClimbConstants.kCageOffsetY[0]);
    int index = 0;

    for (int i = 1; i < 3; i++) {
      double dist = FastMath.abs(centerDist - ClimbConstants.kCageOffsetY[i]);
      if (minDist > dist) {
        minDist = dist;
        index = i;
      }
    }

    switch (alliance) {
      case Blue -> {
        if (starting.getY() < ClimbConstants.kCenterY
            || FastMath.abs(starting.getX() - ClimbConstants.kCenterX) > ClimbConstants.kMaxDistX) {
          return;
        }
        y += ClimbConstants.kCageOffsetY[index] - climbOffset;
      }
      case Red -> {
        if (starting.getY() > ClimbConstants.kCenterY
            || FastMath.abs(starting.getX() - ClimbConstants.kCenterX) > ClimbConstants.kMaxDistX) {
          return;
        }
        y -= ClimbConstants.kCageOffsetY[index] - climbOffset;
      }
    }

    targetPose = new Pose2d(new Translation2d(x, y), yaw);

    Logger.recordOutput("ClimbAlignSubsystem/Target Pose", targetPose);

    curState = ClimbAlignStates.POSE_DRIVE;
    climbSubsystem.prepClimb();
  }

  public void terminate() {
    driveSubsystem.stopDriving();
    driveSubsystem.setIgnoreSticks(false);
    curState = ClimbAlignStates.DONE;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("ClimbAlignSubsystem/State", curState.toString());

    switch (curState) {
      case POSE_DRIVE -> {
        Pose2d current = driveSubsystem.getPoseMeters();

        double vX = driveX.calculate(current.getX(), targetPose.getX());
        double vY = driveY.calculate(current.getY(), targetPose.getY());
        double vOmega =
            driveOmega.calculate(
                current.getRotation().getRadians(), targetPose.getRotation().getRadians());

        Logger.recordOutput("ClimbAlignSubsystem/DriveXError", driveX.getError());
        Logger.recordOutput("ClimbAlignSubsystem/DriveYError", driveY.getError());
        Logger.recordOutput("ClimbAlignSubsystem/DriveOmegaError", driveOmega.getError());

        driveSubsystem.move(vX, vY, vOmega, true);

        if (FastMath.abs(driveX.getError()) < ClimbConstants.kCloseEnoughX
            && FastMath.abs(driveY.getError()) < ClimbConstants.kCloseEnoughY) {
          curState = ClimbAlignStates.FINAL_DRIVE;
          break;
        }
      }
      case FINAL_DRIVE -> {
        Pose2d current = driveSubsystem.getPoseMeters();

        double vX =
            directionPositive ? ClimbConstants.kFinalDriveVx : -ClimbConstants.kFinalDriveVx;
        double vY = driveY.calculate(current.getY(), targetPose.getY());
        double vOmega =
            driveOmega.calculate(
                current.getRotation().getRadians(), targetPose.getRotation().getRadians());

        Logger.recordOutput("ClimbAlignSubsystem/DriveXError", driveX.getError());
        Logger.recordOutput("ClimbAlignSubsystem/DriveYError", driveY.getError());
        Logger.recordOutput("ClimbAlignSubsystem/DriveOmegaError", driveOmega.getError());

        if (!delay.isRunning()) {
          driveSubsystem.move(vX, vY, vOmega, true);
        }

        double climbPos = climbSubsystem.getPosition().in(Rotations);
        if (climbPos > ClimbConstants.kClimbAngleGood) {
          if (!delay.isRunning()) {
            delay.start();
            driveSubsystem.stopDriving();
          }

          if (delay.hasElapsed(1)) {
            climbSubsystem.climb();
            delay.stop();
            terminate();
            break;
          }
        }
      }
      case DONE -> {}
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("State", () -> curState.ordinal()));
  }

  public enum ClimbAlignStates {
    POSE_DRIVE,
    FINAL_DRIVE,
    DONE
  }
}
