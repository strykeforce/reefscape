package frc.robot.subsystems.tagAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.TagServoingConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Set;
import net.jafama.FastMath;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class TagAlignSubsystem extends MeasurableSubsystem {
  private static final org.slf4j.Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;

  private PIDController driveX;
  private PIDController driveY;
  private PIDController driveOmega;

  private PIDController alignX;
  private PIDController alignY;

  private TagAlignStates curState = TagAlignStates.DONE;

  // Set by start()
  private Pose2d targetPose;
  private int fieldRelHexant;
  private Alliance alliance = Alliance.Blue;
  private double driveRadius = TagServoingConstants.kCoralInitialDriveRadius;
  private boolean algae = false;
  private double driveXCloseEnough = TagServoingConstants.kCoralDriveXCloseEnough;
  private double driveYCloseEnough = TagServoingConstants.kCoralDriveYCloseEnough;
  private boolean proceedToAlign = false;
  private boolean scoreLeft = true;
  private int currentThresCount = 0;
  private boolean finalDrive = false;
  private double xError = 2767;
  private double yError = 2767;
  private double yawError = 2767;
  private double coralOffset = 0;
  private double noProgressCounts = 0;

  public TagAlignSubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    this.driveX = new PIDController(4, 0, 0);
    this.driveY = new PIDController(4, 0, 0);
    this.driveOmega = new PIDController(6.0, 0, 0);
    this.driveOmega.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    this.alignX = new PIDController(4, 0, 0); // 0.0015
    this.alignY = new PIDController(4, 0, 0);

    Logger.recordOutput("TagAlignSubsystem/Hexant", -1);

    driveRadius = 1.338;
    for (int i = 0; i < 6; i++) {
      logger.info("Hexant {}, left and right", i);

      logger.info("{}, {}", getTargetDrivePose(true, i).getX(), getTargetDrivePose(true, i).getY());
      logger.info(
          "{}, {}", getTargetDrivePose(false, i).getX(), getTargetDrivePose(false, i).getY());
    }
  }

  public void setProceedToAlign(boolean proceed) {
    this.proceedToAlign = proceed;
  }

  public boolean yErrorSmall() {
    return curState == TagAlignStates.DONE
        || finalDrive
        || yError < TagServoingConstants.kSmallYThres;
  }

  public int computeHexant() {
    boolean blueSide = driveSubsystem.getPoseMeters().getX() < (DriveConstants.kFieldMaxX / 2);
    Translation2d reefT =
        blueSide ? TagServoingConstants.kBlueReefPose : TagServoingConstants.kRedReefPose;
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
                + (blueSide ? 3 : 0))
            % 6;

    Logger.recordOutput("TagAlignSubsystem/Hexant", hexant);

    return hexant;
  }

  public int computeHexant(Pose2d pose) {
    boolean blueSide = driveSubsystem.getPoseMeters().getX() < (DriveConstants.kFieldMaxX / 2);
    Translation2d reefT =
        blueSide ? TagServoingConstants.kBlueReefPose : TagServoingConstants.kRedReefPose;
    double offset = Units.degreesToRadians(30);

    int hexant =
        (((int)
                    (FastMath.normalizeZeroTwoPi(
                            pose.getTranslation().minus(reefT).getAngle().getRadians() + offset)
                        / Units.degreesToRadians(60)))
                + (blueSide ? 3 : 0))
            % 6;

    Logger.recordOutput("TagAlignSubsystem/Hexant", hexant);

    return hexant;
  }

  // Red reef numbered like blue (red 0 is facing the same direction as blue 0)
  private int computeFieldRelHexant() {
    return (computeHexant()
            + (driveSubsystem.getPoseMeters().getX() < (DriveConstants.kFieldMaxX / 2) ? 0 : 3))
        % 6;
  }

  public int computeFieldRelHexant(Pose2d pose) {
    return (computeHexant(pose)
            + (driveSubsystem.getPoseMeters().getX() < (DriveConstants.kFieldMaxX / 2) ? 0 : 3))
        % 6;
  }

  private Pose2d getTargetDrivePose(boolean scoreLeft) {
    boolean blueSide = driveSubsystem.getPoseMeters().getX() < (DriveConstants.kFieldMaxX / 2);
    Translation2d reefT =
        blueSide ? TagServoingConstants.kBlueReefPose : TagServoingConstants.kRedReefPose;

    Translation2d offset =
        new Translation2d(driveRadius, Rotation2d.fromDegrees(computeFieldRelHexant() * 60 + 180));

    Translation2d sideOffset =
        new Translation2d(
            scoreLeft
                ? TagServoingConstants.kRightCamOffset
                : TagServoingConstants.kLeftCamOffset + coralOffset,
            Rotation2d.fromDegrees(computeFieldRelHexant() * 60 + 180 + 90));

    return new Pose2d(
        reefT.plus(offset).plus(sideOffset), Rotation2d.fromDegrees(computeFieldRelHexant() * 60));
  }

  public Pose2d getTargetDrivePose(boolean scoreLeft, int hexant) {
    Translation2d reefT =
        driveSubsystem.getPoseMeters().getX() < (DriveConstants.kFieldMaxX / 2)
            ? TagServoingConstants.kBlueReefPose
            : TagServoingConstants.kRedReefPose;

    Translation2d offset =
        new Translation2d(driveRadius, Rotation2d.fromDegrees(hexant * 60 + 180));

    Translation2d sideOffset =
        new Translation2d(
            scoreLeft ? TagServoingConstants.kRightCamOffset : TagServoingConstants.kLeftCamOffset,
            Rotation2d.fromDegrees(hexant * 60 + 180 + 90));

    return new Pose2d(reefT.plus(offset).plus(sideOffset), Rotation2d.fromDegrees(hexant * 60));
  }

  public double getCurRadius() {
    Translation2d reefT =
        driveSubsystem.getPoseMeters().getX() < (DriveConstants.kFieldMaxX / 2)
            ? TagServoingConstants.kBlueReefPose
            : TagServoingConstants.kRedReefPose;

    Translation2d reefRelative = driveSubsystem.getPoseMeters().getTranslation().minus(reefT);

    return FastMath.hypot(reefRelative.getX(), reefRelative.getY());
  }

  public TagAlignStates getState() {
    return curState;
  }

  public double getErrX() {
    return xError;
  }

  public double getErrY() {
    return yError;
  }

  public double getErrYaw() {
    return yawError;
  }

  public boolean isFinalDrive() {
    return finalDrive;
  }

  public boolean stalled() {
    return noProgressCounts > TagServoingConstants.kMinStuckCounts;
  }

  public boolean fixableStuckCoral() {
    return FastMath.abs(getCurRadius() - TagServoingConstants.kCoralStuckRadius)
        < TagServoingConstants.kCoralStuckAllowence;
  }

  public boolean isAligned() {
    return FastMath.abs(yError) < TagServoingConstants.kCoralDriveYCloseEnough
        && FastMath.abs(yawError) < TagServoingConstants.kAngleCloseEnough;
  }

  public void setup(Alliance alliance, ScoringLevel level, boolean scoreLeft, boolean algae) {
    boolean blueSide = driveSubsystem.getPoseMeters().getX() < (DriveConstants.kFieldMaxX / 2);

    this.alliance = alliance;
    this.scoreLeft = scoreLeft;
    this.finalDrive = false;
    this.noProgressCounts = 0;

    this.driveRadius =
        algae
            ? TagServoingConstants.kAlgaeInitialDriveRadius
            : (level == ScoringLevel.L1
                ? TagServoingConstants.kL1CoralRadius
                : TagServoingConstants.kCoralInitialDriveRadius);
    this.algae = algae;
    this.driveXCloseEnough = TagServoingConstants.kInitialCloseEnough;
    this.driveYCloseEnough = TagServoingConstants.kInitialCloseEnough;
    this.currentThresCount = 0;

    this.targetPose = getTargetDrivePose(scoreLeft);

    this.fieldRelHexant = computeFieldRelHexant();

    this.coralOffset =
        blueSide
            ? TagServoingConstants.kBlueCoralOffset[fieldRelHexant][level.ordinal()][
                scoreLeft ? 0 : 1]
            : TagServoingConstants.kRedCoralOffset[(fieldRelHexant + 3) % 6][level.ordinal()][
                scoreLeft ? 0 : 1];
    this.proceedToAlign = false;
    this.yError =
        targetPose
            .getTranslation()
            .minus(driveSubsystem.getPoseMeters().getTranslation())
            .rotateBy(Rotation2d.fromRadians(-TagServoingConstants.kAngleTarget[fieldRelHexant]))
            .getY();

    Logger.recordOutput("TagAlignSubsystem/TargetPose", targetPose);
    Logger.recordOutput("TagAlignSubsystem/GettingAlgae", algae);

    driveX.reset();
    driveY.reset();
    driveOmega.reset();

    alignX.reset();
    alignY.reset();
  }

  public void start(Alliance alliance, ScoringLevel level, boolean scoreLeft, boolean algae) {
    setup(alliance, level, scoreLeft, algae);

    curState = TagAlignStates.DRIVE;
  }

  public void startAuto(Alliance alliance, ScoringLevel level, boolean scoreLeft, boolean algae) {
    setup(alliance, level, scoreLeft, algae);
    tagAlign();
  }

  private void tagAlign() {
    alignX.reset();
    alignY.reset();

    curState = TagAlignStates.TAG_ALIGN;

    this.driveXCloseEnough =
        algae
            ? TagServoingConstants.kAlgaeDriveXCloseEnough
            : TagServoingConstants.kCoralDriveXCloseEnough;

    this.driveYCloseEnough =
        algae
            ? TagServoingConstants.kAlgaeDriveYCloseEnough
            : TagServoingConstants.kCoralDriveYCloseEnough;

    this.driveRadius =
        algae ? TagServoingConstants.kAlgaeAlignRadius : TagServoingConstants.kCoralAlignRadius;
    targetPose = getTargetDrivePose(scoreLeft);
    Logger.recordOutput("TagAlignSubsystem/TargetPose", targetPose);
  }

  public void terminate() {
    driveSubsystem.stopDriving();
    curState = TagAlignStates.DONE;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("TagAlignSubsystem/State", curState.toString());
    Logger.recordOutput("TagAlignSubsystem/FinalDrive", finalDrive);
    Logger.recordOutput("TagAlignSubsystem/isAligned", isAligned());
    Logger.recordOutput("TagAlignSubsystem/stalled", stalled());
    Logger.recordOutput("TagAlignSubsystem/stuckCoral", fixableStuckCoral());

    switch (curState) {
      case DRIVE, TAG_ALIGN -> {
        Pose2d current = driveSubsystem.getPoseMeters();

        double vX = 0;
        double vY = 0;
        double vOmega =
            driveOmega.calculate(
                current.getRotation().getRadians(), targetPose.getRotation().getRadians());

        yawError = driveOmega.getError();

        Translation2d rotated =
            current
                .getTranslation()
                .rotateBy(
                    Rotation2d.fromRadians(-TagServoingConstants.kAngleTarget[fieldRelHexant]));
        Translation2d rotatedTarget =
            targetPose
                .getTranslation()
                .rotateBy(
                    Rotation2d.fromRadians(-TagServoingConstants.kAngleTarget[fieldRelHexant]));

        switch (curState) {
          case DRIVE -> {
            vX = driveX.calculate(rotated.getX(), rotatedTarget.getX());
            vY = driveY.calculate(rotated.getY(), rotatedTarget.getY());
            xError = driveX.getError();
            yError = driveY.getError();
          }
          case TAG_ALIGN -> {
            vX = alignX.calculate(rotated.getX(), rotatedTarget.getX());
            vY = alignY.calculate(rotated.getY(), rotatedTarget.getY());
            xError = alignX.getError();
            yError = alignY.getError();
          }
          default -> {}
        }
        Logger.recordOutput("TagAlignSubsystem/DriveXError", xError);
        Logger.recordOutput("TagAlignSubsystem/DriveYError", yError);

        if (vX > 2) vX = 2;
        if (vY > 2) vY = 2;

        Logger.recordOutput("TagAlignSubsystem/DriveOmegaError", yawError);

        // double radius = getCurRadius(alliance);
        boolean ignoreX = false; // radius < stopXRadius && !algae;

        // if (!algae || tagRelX > 0) {
        //   tagRelX =
        //       tagRelX < TagServoingConstants.kMinVelX ? TagServoingConstants.kMinVelX : tagRelX;
        // }

        // if (ignoreX) {
        //   tagRelX = 0;
        // }

        // Translation2d poseError = targetPose.getTranslation().minus(current.getTranslation());

        if (driveSubsystem.getAvgDriveCurrent() > TagServoingConstants.kEndDriveCurrentThreshold
            && driveSubsystem.getAvgRearDriveVel() < TagServoingConstants.kEndVelThreshold) {
          noProgressCounts++;
        } else {
          noProgressCounts = 0;
        }

        Logger.recordOutput("TagAlignSubsystem/noProgressCounts", noProgressCounts);
        Logger.recordOutput("TagAlignSubsystem/X Error Derivative", driveX.getErrorDerivative());

        if (finalDrive
            || FastMath.abs(driveOmega.getError()) < TagServoingConstants.kAngleCloseEnough) {
          switch (curState) {
            case DRIVE -> {
              if (FastMath.abs(driveX.getError()) < driveXCloseEnough
                      && FastMath.abs(driveY.getError()) < driveYCloseEnough
                  // || ignoreX && FastMath.abs(tagRelError.getY()) < driveCloseEnough
                  || FastMath.abs(driveY.getError()) < driveYCloseEnough) {
                tagAlign();
                break;
              }
            }
            case TAG_ALIGN -> {
              if (FastMath.abs(alignX.getError()) < driveXCloseEnough
                  && FastMath.abs(alignY.getError()) < driveYCloseEnough) {
                finalDrive = true;
              }
              if (finalDrive
                  && driveSubsystem.getAvgDriveCurrent()
                      > TagServoingConstants.kEndDriveCurrentThreshold
                  && driveSubsystem.getAvgRearDriveVel() < TagServoingConstants.kEndVelThreshold) {

                currentThresCount++;
                if (currentThresCount >= TagServoingConstants.kEndCountThreshold) {
                  terminate();
                  return;
                }
              } else {
                currentThresCount = 0;
              }
            }
          }
        }

        if (finalDrive) {
          vX = TagServoingConstants.kFinalDriveVel;
        }
        if (curState == TagAlignStates.WAITING) {
          break;
        }

        Logger.recordOutput("TagAlignSubsystem/Tag Rel Drive vX", vX);
        Logger.recordOutput("TagAlignSubsystem/Tag Rel Drive vY", vY);

        Translation2d fieldRelV =
            new Translation2d(vX, vY)
                .rotateBy(
                    Rotation2d.fromRadians(TagServoingConstants.kAngleTarget[fieldRelHexant]));

        driveSubsystem.move(fieldRelV.getX(), fieldRelV.getY(), vOmega, true);
      }

      case WAITING -> {
        if (proceedToAlign || !algae) {
          tagAlign();
        }
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
    WAITING,
    TAG_ALIGN,
    DONE
  }
}
