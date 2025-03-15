package frc.robot.subsystems.tagAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.TagServoingConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
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
  // private ProfiledPIDController alignOmega;

  // private ProfiledPIDController servoX;
  // private ProfiledPIDController servoY;

  private TagAlignStates curState = TagAlignStates.DONE;

  // Set by start()
  private Pose2d targetPose;
  // private int targetCamId;
  // private int targetTagId;
  private int fieldRelHexant;
  private Alliance alliance = Alliance.Blue;
  // private double goalTargetDiag;
  // private double stopXRadius = TagServoingConstants.kCoralStopXDriveRadius;
  private double driveRadius = TagServoingConstants.kCoralInitialDriveRadius;
  private boolean algae = false;
  private double driveXCloseEnough = TagServoingConstants.kCoralDriveXCloseEnough;
  private double driveYCloseEnough = TagServoingConstants.kCoralDriveYCloseEnough;
  private boolean proceedToAlign = false;
  private boolean scoreLeft = true;
  private int currentThresCount = 0;
  private boolean finalDrive = false;

  // private long startServoTime;

  public TagAlignSubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    this.driveX = new PIDController(4, 0, 0);
    this.driveY = new PIDController(4, 0, 0);
    this.driveOmega = new PIDController(6.0, 0, 0);
    this.driveOmega.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    this.alignX = new PIDController(4, 0, 0); // 0.0015
    this.alignY = new PIDController(4, 0, 0);
    // this.alignOmega =
    //     new ProfiledPIDController(6.0, 0, 0, TagServoingConstants.alignOmegaConstraints);
    // this.alignOmega.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    // If we revisit tag servoing later...
    // this.servoX =
    //     new ProfiledPIDController(4, 0, 0, TagServoingConstants.alignXConstraints);
    // this.servoY = new ProfiledPIDController(4, 0, 0, TagServoingConstants.alignYConstraints);

    Logger.recordOutput("TagAlignSubsystem/Hexant", -1);

    driveRadius = 1.223823;
    for (int i = 0; i < 6; i++) {
      logger.info("Hexant {}, left and right", i);

      logger.info(
          "{}, {}",
          getTargetDrivePose(Alliance.Blue, true, i).getX(),
          getTargetDrivePose(Alliance.Blue, true, i).getY());
      logger.info(
          "{}, {}",
          getTargetDrivePose(Alliance.Blue, false, i).getX(),
          getTargetDrivePose(Alliance.Blue, false, i).getY());
    }
  }

  public void setProceedToAlign(boolean proceed) {
    this.proceedToAlign = proceed;
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
            driveRadius, Rotation2d.fromDegrees(computeFieldRelHexant(color) * 60 + 180));

    Translation2d sideOffset =
        new Translation2d(
            scoreLeft ? TagServoingConstants.kRightCamOffset : TagServoingConstants.kLeftCamOffset,
            Rotation2d.fromDegrees(computeFieldRelHexant(color) * 60 + 180 + 90));

    return new Pose2d(
        reefT.plus(offset).plus(sideOffset),
        Rotation2d.fromDegrees(computeFieldRelHexant(color) * 60));
  }

  private Pose2d getTargetDrivePose(Alliance color, boolean scoreLeft, int hexant) {
    Translation2d reefT =
        color == Alliance.Blue
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

  public double getCurRadius(Alliance color) {
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

  public void setup(Alliance alliance, boolean scoreLeft, boolean algae) {
    this.alliance = alliance;
    this.scoreLeft = scoreLeft;
    this.finalDrive = false;
    // this.goalTargetDiag =
    //     scoreLeft
    //         ? TagServoingConstants.kRightCamDiagTarget
    //         : TagServoingConstants.kLeftCamDiagTarget;

    // this.stopXRadius =
    //     algae
    //         ? TagServoingConstants.kAlgaeStopXDriveRadius
    //         : TagServoingConstants.kCoralStopXDriveRadius;
    this.driveRadius =
        algae
            ? TagServoingConstants.kAlgaeInitialDriveRadius
            : TagServoingConstants.kCoralInitialDriveRadius;
    this.algae = algae;
    this.driveXCloseEnough = TagServoingConstants.kInitialCloseEnough;
    this.driveYCloseEnough = TagServoingConstants.kInitialCloseEnough;
    this.currentThresCount = 0;

    targetPose = getTargetDrivePose(alliance, scoreLeft);
    // Inverted, scoring left coral means aligning right camera
    // targetCamId =
    //     !scoreLeft ? TagServoingConstants.kLeftServoCam : TagServoingConstants.kRightServoCam;
    // targetTagId =
    //     alliance == Alliance.Blue
    //         ? TagServoingConstants.kBlueTargetTag[computeHexant(alliance)]
    //         : TagServoingConstants.kRedTargetTag[computeHexant(alliance)];

    fieldRelHexant = computeFieldRelHexant(alliance);
    proceedToAlign = false;

    // Logger.recordOutput("TagAlignSubsystem/TargetTag", targetTagId);
    // Logger.recordOutput("TagAlignSubsystem/GoalTargetDiag", goalTargetDiag);
    Logger.recordOutput("TagAlignSubsystem/TargetPose", targetPose);
    Logger.recordOutput("TagAlignSubsystem/GettingAlgae", algae);

    driveX.reset();
    driveY.reset();
    driveOmega.reset();

    alignX.reset();
    alignY.reset();
    // alignOmega.reset(driveSubsystem.getGyroRotation2d().getRadians());
  }

  // CALL setup() first!! Do not use in internal state machine
  public double calculateAlignY() {
    return 2767;

    // WallEyeTagResult result = visionSubsystem.getLastResult(targetCamId);

    // if (result == null) {
    //   return 2767;
    // }

    // int tagIndex = -1;
    // int[] tags = result.getTagIDs();

    // for (int i = 0; i < tags.length; i++) {
    //   if (tags[i] == targetTagId) {
    //     tagIndex = i;
    //     break;
    //   }
    // }

    // if (tagIndex == -1) {
    //   // Target tag not found
    //   return 2767;
    // }

    // Point center = result.getTagCenters().get(tagIndex);
    // double diag = result.getTagDiags()[tagIndex];

    // Logger.recordOutput("TagAlignSubsystem/TargetDiag", diag);
    // Logger.recordOutput("TagAlignSubsystem/TargetCenterX", center.x());

    // double vY = -alignY.calculate(TagServoingConstants.kHorizontalTarget - center.x(), 0);

    // return vY;
  }

  public void start(Alliance alliance, boolean scoreLeft, boolean algae) {
    setup(alliance, scoreLeft, algae);

    curState = TagAlignStates.DRIVE;
  }

  public void startAuto(Alliance alliance, boolean scoreLeft, boolean algae) {
    setup(alliance, scoreLeft, algae);
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

    // this.goalTargetDiag =
    //     scoreLeft
    //         ? TagServoingConstants.kRightCamDiagTarget
    //         : TagServoingConstants.kLeftCamDiagTarget;

    // this.stopXRadius =
    //     algae
    //         ? TagServoingConstants.kAlgaeStopXDriveRadius
    //         : TagServoingConstants.kCoralStopXDriveRadius;
    this.driveRadius =
        algae ? TagServoingConstants.kAlgaeAlignRadius : TagServoingConstants.kCoralAlignRadius;
    targetPose = getTargetDrivePose(alliance, scoreLeft);
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
    // Logger.recordOutput("TagAlignSubsystem/Hexant", computeHexant(alliance));

    switch (curState) {
      case DRIVE, TAG_ALIGN -> {
        Pose2d current = driveSubsystem.getPoseMeters();

        double vX = 0;
        double vY = 0;
        double vOmega =
            driveOmega.calculate(
                current.getRotation().getRadians(), targetPose.getRotation().getRadians());

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
            Logger.recordOutput("TagAlignSubsystem/DriveXError", driveX.getError());
            Logger.recordOutput("TagAlignSubsystem/DriveYError", driveY.getError());
          }
          case TAG_ALIGN -> {
            vX = alignX.calculate(rotated.getX(), rotatedTarget.getX());
            vY = alignY.calculate(rotated.getY(), rotatedTarget.getY());
            Logger.recordOutput("TagAlignSubsystem/DriveXError", alignX.getError());
            Logger.recordOutput("TagAlignSubsystem/DriveYError", alignY.getError());
          }
          default -> {}
        }

        if (vX > 2) vX = 2;
        if (vY > 2) vY = 2;

        Logger.recordOutput("TagAlignSubsystem/DriveOmegaError", driveOmega.getError());

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
                  break;
                }
              } else {
                currentThresCount = 0;
              }
            }
          }
        }

        if (finalDrive) {
          vX = 0.25;
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
