package frc.robot.subsystems.tagAlign;

import WallEye.Point;
import WallEye.WallEyeTagResult;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  public TagAlignSubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    // FIXME: need sane constants
    this.driveX = new ProfiledPIDController(0.0019, 0, 0, new Constraints(3.0, 3.0));
    this.driveY = new ProfiledPIDController(0.00001, 0, 0, new Constraints(2.0, 1.0));
    this.driveOmega = new ProfiledPIDController(5.0, 0, 0, new Constraints(1.0, 1.0));
    this.driveOmega.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    this.alignX = new ProfiledPIDController(0.0019, 0, 0, new Constraints(3.0, 3.0));
    this.alignY = new ProfiledPIDController(0.00001, 0, 0, new Constraints(2.0, 1.0));
    this.alignOmega = new ProfiledPIDController(5.0, 0, 0, new Constraints(1.0, 1.0));
    this.alignOmega.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    Logger.recordOutput("TagAlignSubsystem/TargetArea", -1);
    Logger.recordOutput("TagAlignSubsystem/TargetTag", -1);
    Logger.recordOutput("TagAlignSubsystem/TargetCenterX", -1);
  }

  // FIXME: uncomment when implemented
  public int computeHexant(/* Alliance color */ ) {
    Translation2d reefT = /* color == Alliance.Blue ? TagServoingConstants.kBlueReefPose : */
        TagServoingConstants.kRedReefPose;
    double offset = Units.degreesToRadians(30);

    return (((int)
                (FastMath.normalizeZeroTwoPi(
                            driveSubsystem
                                    .getPoseMeters()
                                    .getTranslation()
                                    .minus(reefT)
                                    .getAngle()
                                    .getRadians()
                                - offset)
                        / Units.degreesToRadians(60)
                    + offset))
            + 3)
        % 6;
  }

  // Red reef numbered like blue (red 0 is facing the same direction as blue 0)
  private int computeFieldRelHexant(/* Alliance color */ ) {
    return (computeHexant(/* color */ ) + /* color == Alliance.Blue ? 0 : */ 3) % 6;
  }

  private Pose2d getTargetDrivePose(/* Alliance color */ ) {
    Translation2d reefT = /* color == Alliance.Blue ? TagServoingConstants.kBlueReefPose : */
        TagServoingConstants.kRedReefPose;

    Translation2d offset =
        new Translation2d(
            TagServoingConstants.kInitialDriveRadius,
            Rotation2d.fromDegrees(computeFieldRelHexant() * 60));

    return new Pose2d(reefT.plus(offset), Rotation2d.fromDegrees(computeFieldRelHexant() * 60));
  }

  private double getCurRadius() {
    Translation2d reefT = /* color == Alliance.Blue ? TagServoingConstants.kBlueReefPose : */
        TagServoingConstants.kRedReefPose;
    return driveSubsystem.getPoseMeters().getTranslation().minus(reefT).getNorm();
  }

  public TagAlignStates getState() {
    return curState;
  }

  public void setup(Alliance alliance, boolean scoreLeft) {
    targetPose = getTargetDrivePose(/* color */ );

    // Inverted, scoring left coral means aligning right camera
    targetCamId =
        !scoreLeft ? TagServoingConstants.kLeftServoCam : TagServoingConstants.kRightServoCam;
    targetTagId = /*
                   * color == Alliance.Blue ? TagServoingConstants.kRedTargetTag[computeHexant()]
                   * :
                   */ TagServoingConstants.kRedTargetTag[computeHexant()];

    fieldRelHexant = computeFieldRelHexant();

    Logger.recordOutput("TagAlignSubsystem/TargetTag", targetTagId);

    Pose2d current = driveSubsystem.getPoseMeters();

    driveX.reset(current.getX());
    driveY.reset(current.getY());
    driveOmega.reset(driveSubsystem.getGyroRotation2d().getRadians());

    alignX.reset(current.getX());
    alignY.reset(current.getY());
    alignOmega.reset(driveSubsystem.getGyroRotation2d().getRadians());
  }

  // CALL setup() first!!
  // Do not use in internal state machine
  public double calculateAlignY() {
    WallEyeTagResult result = visionSubsystem.getLastResult(targetCamId);

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

    Logger.recordOutput("TagAlignSubsystem/TargetCenterX", center.x());

    double vY = alignY.calculate(center.x(), TagServoingConstants.kHorizontalTarget);

    return vY;
  }

  public void start(Alliance alliance, boolean scoreLeft) {
    setup(alliance, scoreLeft);

    curState = TagAlignStates.DRIVE;
  }

  public void terminate() {
    driveSubsystem.move(0, 0, 0, false);
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
            driveX.calculate(
                current.getRotation().getRadians(), targetPose.getRotation().getRadians());

        Translation2d tagRelVel =
            new Translation2d(vX, vY)
                .rotateBy(
                    Rotation2d.fromRadians(-TagServoingConstants.kAngleTarget[fieldRelHexant]));

        double tagRelX = tagRelVel.getX();
        double tagRelY = tagRelVel.getY();

        if (getCurRadius() < TagServoingConstants.kStopXDriveRadius) {
          tagRelX = 0;
        }

        if (Math.abs(driveOmega.getPositionError()) < TagServoingConstants.kAngleCloseEnough
            && targetPose.minus(current).getTranslation().getNorm()
                < TagServoingConstants.kDriveCloseEnough) {
          alignX.reset(current.getX());
          alignY.reset(current.getY());
          alignOmega.reset(driveSubsystem.getGyroRotation2d().getRadians());

          curState = TagAlignStates.TAG_ALIGN;
          break;
        }

        Translation2d adjusted =
            new Translation2d(tagRelX, tagRelY)
                .rotateBy(
                    Rotation2d.fromRadians(TagServoingConstants.kAngleTarget[fieldRelHexant]));

        driveSubsystem.move(adjusted.getX(), adjusted.getY(), vOmega, true);
      }

      case TAG_ALIGN -> {
        WallEyeTagResult result = (WallEyeTagResult) visionSubsystem.getLastResult(targetCamId);

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
        double area = result.getTagAreas()[tagIndex];

        Logger.recordOutput("TagAlignSubsystem/TargetArea", area);
        Logger.recordOutput("TagAlignSubsystem/TargetCenterX", center.x());

        double vX = alignX.calculate(area, TagServoingConstants.kAreaTarget);
        double vY = alignY.calculate(center.x(), TagServoingConstants.kHorizontalTarget);

        if (TagServoingConstants.kAreaTarget - area < TagServoingConstants.kAngleCloseEnough
            || area > TagServoingConstants.kAreaTarget) {
          vX = 0;

          if (Math.abs(TagServoingConstants.kHorizontalTarget - center.x())
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
