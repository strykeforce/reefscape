package frc.robot.subsystems.tagAlign;

import WallEye.Point;
import WallEye.WallEyeTagResult;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.TagServoingConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Set;
import net.jafama.FastMath;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class TagAlignSubsystem extends MeasurableSubsystem {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;

  private ProfiledPIDController driveX;
  private ProfiledPIDController driveY;
  private ProfiledPIDController driveOmega;

  private ProfiledPIDController alignX;
  private ProfiledPIDController alignY;
  private ProfiledPIDController alignOmega;

  private TagAlignStates curState = TagAlignStates.DONE;

  // Set by start()
  private Pose2d targetPose;
  private int targetCamId;
  private int targetTagId;
  private int fieldRelHexant;
  private Alliance alliance = Alliance.Blue;
  private double goalTargetDiag;

  private long startServoTime;

  public TagAlignSubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    // FIXME: need sane constants
    this.driveX = new ProfiledPIDController(3, 0, 0, new Constraints(1, 1.0));
    this.driveY = new ProfiledPIDController(5, 0, 0, new Constraints(2, 3));
    this.driveOmega = new ProfiledPIDController(5.0, 0, 0, new Constraints(1.0, 1.0));
    this.driveOmega.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    this.alignX = new ProfiledPIDController(0.0017, 0, 0, new Constraints(1.0, 1.0)); // 0.0015
    this.alignY = new ProfiledPIDController(0.0017, 0, 0, new Constraints(1.0, 1.0));
    this.alignOmega = new ProfiledPIDController(5.5, 0, 0, new Constraints(1.0, 1.0));
    this.alignOmega.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    Logger.recordOutput("TagAlignSubsystem/TargetDiag", -1);
    Logger.recordOutput("TagAlignSubsystem/TargetTag", -1);
    Logger.recordOutput("TagAlignSubsystem/TargetCenterX", -1);
    Logger.recordOutput("TagAlignSubsystem/Hexant", -1);
    Logger.recordOutput("TagAlignSubsystem/GoalTargetDiag", -1);
  }

  public int computeHexant(Alliance color) {
    Translation2d reefT =
        color == Alliance.Blue
            ? TagServoingConstants.kBlueReefPose
            : TagServoingConstants.kRedReefPose;
    double offset = Units.degreesToRadians(30);

    int hexant =
        (((int)
                    (FastMath.normalizeZeroTwoPi(
                            driveSubsystem
                                    .getPoseMeters()
                                    .getTranslation()
                                    .minus(reefT)
                                    .getAngle()
                                    .getRadians()
                                + offset)
                        / Units.degreesToRadians(60)))
                + (color == Alliance.Blue ? 3 : 0))
            % 6;

    Logger.recordOutput("TagAlignSubsystem/Hexant", hexant);

    return hexant;
  }

  // Red reef numbered like blue (red 0 is facing the same direction as blue 0)
  private int computeFieldRelHexant(Alliance color) {
    return (computeHexant(color) + (color == Alliance.Blue ? 0 : 3)) % 6;
  }

  private Pose2d getTargetDrivePose(Alliance color, boolean scoreLeft) {
    Translation2d reefT =
        color == Alliance.Blue
            ? TagServoingConstants.kBlueReefPose
            : TagServoingConstants.kRedReefPose;

    Translation2d offset =
        new Translation2d(
            TagServoingConstants.kInitialDriveRadius,
            Rotation2d.fromDegrees(computeFieldRelHexant(color) * 60 + 180));

    Translation2d sideOffset =
        new Translation2d(
            scoreLeft ? TagServoingConstants.kRightCamOffset : TagServoingConstants.kLeftCamOffset,
            Rotation2d.fromDegrees(computeFieldRelHexant(color) * 60 + 180 + 90));

    return new Pose2d(
        reefT.plus(offset).plus(sideOffset),
        Rotation2d.fromDegrees(computeFieldRelHexant(color) * 60));
  }

  private double getCurRadius(Alliance color) {
    Translation2d reefT =
        color == Alliance.Blue
            ? TagServoingConstants.kBlueReefPose
            : TagServoingConstants.kRedReefPose;

    Translation2d reefRelative = driveSubsystem.getPoseMeters().getTranslation().minus(reefT);
    return FastMath.hypot(reefRelative.getX(), reefRelative.getY());
  }

  public TagAlignStates getState() {
    return curState;
  }

  public void setup(Alliance alliance, boolean scoreLeft) {
    targetPose = getTargetDrivePose(alliance, scoreLeft);
    this.alliance = alliance;
    this.goalTargetDiag =
        scoreLeft
            ? TagServoingConstants.kRightCamDiagTarget
            : TagServoingConstants.kLeftCamDiagTarget;

    // Inverted, scoring left coral means aligning right camera
    targetCamId =
        !scoreLeft ? TagServoingConstants.kLeftServoCam : TagServoingConstants.kRightServoCam;
    targetTagId =
        alliance == Alliance.Blue
            ? TagServoingConstants.kBlueTargetTag[computeHexant(alliance)]
            : TagServoingConstants.kRedTargetTag[computeHexant(alliance)];

    fieldRelHexant = computeFieldRelHexant(alliance);

    Logger.recordOutput("TagAlignSubsystem/TargetTag", targetTagId);
    Logger.recordOutput("TagAlignSubsystem/GoalTargetDiag", goalTargetDiag);
    Logger.recordOutput("TagAlignSubsystem/TargetPose", targetPose);

    Pose2d current = driveSubsystem.getPoseMeters();

    driveX.reset(current.getX());
    driveY.reset(current.getY());
    driveOmega.reset(driveSubsystem.getGyroRotation2d().getRadians());

    alignX.reset(0);
    alignY.reset(0);
    alignOmega.reset(driveSubsystem.getGyroRotation2d().getRadians());
  }

  // CALL setup() first!! Do not use in internal state machine
  public double calculateAlignY() {
    WallEyeTagResult result = visionSubsystem.getLastResult(targetCamId);

    if (result == null) {
      return 2767;
    }

    int tagIndex = -1;
    int[] tags = result.getTagIDs();

    for (int i = 0; i < tags.length; i++) {
      if (tags[i] == targetTagId) {
        tagIndex = i;
        break;
      }
    }

    if (tagIndex == -1) {
      // Target tag not found
      return 2767;
    }

    Point center = result.getTagCenters().get(tagIndex);
    double diag = result.getTagDiags()[tagIndex];

    Logger.recordOutput("TagAlignSubsystem/TargetDiag", diag);
    Logger.recordOutput("TagAlignSubsystem/TargetCenterX", center.x());

    double vY = -alignY.calculate(TagServoingConstants.kHorizontalTarget - center.x(), 0);

    return vY;
  }

  public void start(Alliance alliance, boolean scoreLeft) {
    setup(alliance, scoreLeft);

    curState = TagAlignStates.DRIVE;
  }

  public void terminate() {
    driveSubsystem.move(0, 0, 0, false);
    driveSubsystem.drive(0, 0, 0);
    curState = TagAlignStates.DONE;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("TagAlignSubsystem/State", curState.toString());

    switch (curState) {
      case DRIVE -> {
        Pose2d current = driveSubsystem.getPoseMeters();

        double vX = driveX.calculate(current.getX(), targetPose.getX());
        double vY = driveY.calculate(current.getY(), targetPose.getY());
        double vOmega =
            driveOmega.calculate(
                current.getRotation().getRadians(), targetPose.getRotation().getRadians());

        Logger.recordOutput("TagAlignSubsystem/DriveXError", driveX.getPositionError());
        Logger.recordOutput("TagAlignSubsystem/DriveYError", driveY.getPositionError());
        Logger.recordOutput("TagAlignSubsystem/DriveOmegaError", driveOmega.getPositionError());

        Translation2d tagRelVel =
            new Translation2d(vX, vY)
                .rotateBy(
                    Rotation2d.fromRadians(-TagServoingConstants.kAngleTarget[fieldRelHexant]));

        double tagRelX = tagRelVel.getX();
        double tagRelY = tagRelVel.getY();

        double radius = getCurRadius(alliance);
        boolean ignoreX = radius < TagServoingConstants.kStopXDriveRadius;

        tagRelX = tagRelX < TagServoingConstants.kMinVelX ? TagServoingConstants.kMinVelX : tagRelX;

        if (radius < TagServoingConstants.kStopXDriveRadius) {
          tagRelX = 0;
        }

        Translation2d tagRelError =
            targetPose
                .minus(current)
                .getTranslation()
                .rotateBy(
                    Rotation2d.fromRadians(-TagServoingConstants.kAngleTarget[fieldRelHexant]));

        Logger.recordOutput("TagAlignSubsystem/Tag Rel Y Error", tagRelError.getY());

        Transform2d poseError = targetPose.minus(current);

        if (FastMath.abs(driveOmega.getPositionError()) < TagServoingConstants.kAngleCloseEnough
            && (FastMath.hypot(poseError.getX(), poseError.getY())
                    < TagServoingConstants.kDriveCloseEnough
                || ignoreX
                    && FastMath.abs(tagRelError.getY()) < TagServoingConstants.kDriveCloseEnough)) {
          alignX.reset(0);
          alignY.reset(0);
          alignOmega.reset(driveSubsystem.getGyroRotation2d().getRadians());

          curState = TagAlignStates.TAG_ALIGN;
          startServoTime = RobotController.getFPGATime();
          break;
        }

        Logger.recordOutput("TagAlignSubsystem/Tag Rel vX", tagRelX);
        Logger.recordOutput("TagAlignSubsystem/Tag Rel vY", tagRelY);

        Translation2d adjusted =
            new Translation2d(tagRelX, tagRelY)
                .rotateBy(
                    Rotation2d.fromRadians(TagServoingConstants.kAngleTarget[fieldRelHexant]));

        driveSubsystem.move(adjusted.getX(), adjusted.getY(), vOmega, true);
      }

      case TAG_ALIGN -> {
        WallEyeTagResult result = visionSubsystem.getLastResult(targetCamId);

        if (result == null) {
          return;
        }

        int tagIndex = -1;
        int[] tags = result.getTagIDs();

        for (int i = 0; i < tags.length; i++) {
          if (tags[i] == targetTagId) {
            tagIndex = i;
            break;
          }
        }

        if (tagIndex == -1) {
          return;
        }

        Point center = result.getTagCenters().get(tagIndex);
        double diag = result.getTagDiags()[tagIndex];

        Logger.recordOutput("TagAlignSubsystem/TargetDiag", diag);
        Logger.recordOutput("TagAlignSubsystem/TargetCenterX", center.x());

        double vX = -alignX.calculate(goalTargetDiag - diag, 0);
        double vY = -alignY.calculate(TagServoingConstants.kHorizontalTarget - center.x(), 0);

        Logger.recordOutput("TagAlignSubsystem/X Error", alignX.getPositionError());
        Logger.recordOutput("TagAlignSubsystem/Y Error", alignY.getPositionError());
        Logger.recordOutput(
            "TagAlignSubsystem/Last result delay",
            (RobotController.getFPGATime() - result.getTimeStamp()) / 1000);

        if (RobotController.getFPGATime() - result.getTimeStamp()
                > TagServoingConstants.kNoUpdateMicrosec
            && RobotController.getFPGATime() - startServoTime
                > TagServoingConstants.kNoUpdateMicrosec) {
          terminate();
          break;
        }

        if (FastMath.abs(goalTargetDiag - diag) < TagServoingConstants.kDiagCloseEnough
            || diag > goalTargetDiag) {
          vX = 0;

          if (FastMath.abs(TagServoingConstants.kHorizontalTarget - center.x())
              < TagServoingConstants.kHorizontalCloseEnough) {

            terminate();
            break;
          }
        }

        double vOmega =
            alignOmega.calculate(
                driveSubsystem.getGyroRotation2d().getRadians(),
                targetPose.getRotation().getRadians());

        driveSubsystem.move(vX, vY, vOmega, false);
      }

      case DONE -> {}
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("State", () -> curState.ordinal()));
  }

  public enum TagAlignStates {
    DRIVE,
    TAG_ALIGN,
    DONE
  }
}
