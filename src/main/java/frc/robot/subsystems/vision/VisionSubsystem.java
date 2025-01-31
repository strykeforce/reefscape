package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import WallEye.*;
import WallEye.UdpSubscriber;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Set;
import net.jafama.FastMath;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem {

  WallEyeCam[] cams;

  Translation2d[] camPositions = {
    VisionConstants.kCam1Pose.getTranslation().toTranslation2d(),
    VisionConstants.kCam2Pose.getTranslation().toTranslation2d(),
    VisionConstants.kCam3Pose.getTranslation().toTranslation2d(),
    VisionConstants.kCam4Pose.getTranslation().toTranslation2d(),
    VisionConstants.kCam5Pose.getTranslation().toTranslation2d()
  };

  Rotation2d[] camRotations = {
    VisionConstants.kCam1Pose.getRotation().toRotation2d(),
    VisionConstants.kCam2Pose.getRotation().toRotation2d(),
    VisionConstants.kCam3Pose.getRotation().toRotation2d(),
    VisionConstants.kCam4Pose.getRotation().toRotation2d(),
    VisionConstants.kCam5Pose.getRotation().toRotation2d()
  };

  private double[] camHeights = {
    VisionConstants.kCam1Pose.getMeasureZ().in(Meters),
    VisionConstants.kCam2Pose.getMeasureZ().in(Meters),
    VisionConstants.kCam3Pose.getMeasureZ().in(Meters),
    VisionConstants.kCam4Pose.getMeasureZ().in(Meters),
    VisionConstants.kCam5Pose.getMeasureZ().in(Meters)
  };

  String[] camNames = {
    VisionConstants.kCam1Name,
    VisionConstants.kCam2Name,
    VisionConstants.kCam3Name,
    VisionConstants.kCam4Name,
    VisionConstants.kCam5Name
  };

  String[] piNames = {
    VisionConstants.kPi1Name, VisionConstants.kPi2Name, VisionConstants.kPi3Name,
  };

  int[] camIndex = {
    VisionConstants.kCam1Idx,
    VisionConstants.kCam2Idx,
    VisionConstants.kCam3Idx,
    VisionConstants.kCam4Idx,
    VisionConstants.kCam5Idx
  };

  private Swerve swerve = new Swerve();
  private DriveSubsystem driveSubsystem = new DriveSubsystem(swerve);
  private Logger logger;
  private UdpSubscriber[] udpSubscriber;
  private Matrix adaptiveMatrix;
  private AprilTagFieldLayout field;
  private boolean updating = true;
  private int minTags;
  private CircularBuffer<Double> gyroBuffer =
      new CircularBuffer<Double>(VisionConstants.kCircularBufferSize);
  private double timeSinceLastUpdate;
  private int updatesToWheels;
  private ArrayList<Pair<WallEyeResult, Integer>> validResults = new ArrayList<>();

  public VisionSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    logger = LoggerFactory.getLogger("Vision");

    cams = new WallEyeCam[VisionConstants.kNumCams];
    udpSubscriber = new UdpSubscriber[VisionConstants.kNumPis];
    adaptiveMatrix = VisionConstants.kLocalMeasurementStdDevs.copy();
    try {
      field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch (IOException e) {
      logger.error("BAD NEWS BEARS NO APRIL TAG LAYOUT");
    }

    for (int i = 0; i < VisionConstants.kNumCams; i++) {
      cams[i] = new WallEyeCam(piNames[i], camIndex[i], -1);
    }

    udpSubscriber[0] = new UdpSubscriber(0, cams[0], cams[1]);
    udpSubscriber[1] = new UdpSubscriber(1, cams[2], cams[3]);
    udpSubscriber[2] = new UdpSubscriber(2, cams[4]);
  }
  // Setter Methods
  public void setVisionUpdating(boolean updating) {
    this.updating = updating;
  }

  public void setMinTags(int minTags) {
    this.minTags = minTags;
  }
  // Getter Methods
  public boolean isVisionUpdating() {
    return updating;
  }

  public boolean cameraConnected(int index) {
    return cams[index].isCameraConnected();
  }

  private double getSeconds() {
    return RobotController.getFPGATime();
  }

  private double minTagDistance(WallEyePoseResult result) {

    Translation2d camLoc = result.getCameraPose().getTranslation().toTranslation2d();
    int[] ids = result.getTagIDs();
    double minDistance = 2767;

    for (int id : ids) {
      Translation2d tagLoc = field.getTagPose(id).get().getTranslation().toTranslation2d();
      double dist = camLoc.getDistance(camLoc);

      if (dist < minDistance) {
        minDistance = dist;
      }
    }
    return minDistance;
  }

  private double averageTagDistance(WallEyePoseResult result) {

    Translation2d camLoc = result.getCameraPose().getTranslation().toTranslation2d();
    int[] ids = result.getTagIDs();
    double totalDistance = 0.0;

    for (int id : ids) {
      totalDistance +=
          camLoc.getDistance(field.getTagPose(id).get().getTranslation().toTranslation2d());
    }
    return totalDistance / ids.length;
  }

  // Filters
  private boolean camsAgreeWithWheels(Translation3d pose, WallEyeResult result) {

    ChassisSpeeds vel = driveSubsystem.getFieldRelSpeed();
    Pose2d curPose = driveSubsystem.getPoseMeters();

    Translation2d disp = (curPose.getTranslation().minus(pose.toTranslation2d()));

    double velMagnitude =
        Math.sqrt(Math.pow(vel.vxMetersPerSecond, 2) + Math.pow(vel.vyMetersPerSecond, 2));

    double dispMagnitude = Math.sqrt(Math.pow(disp.getX(), 2) + Math.pow(disp.getY(), 2));

    return result.getNumTags() >= minTags
        && dispMagnitude
            <= (velMagnitude * VisionConstants.kLinearCoeffOnVelFilter
                + VisionConstants.kOffsetOnVelFilter
                + Math.pow(velMagnitude * VisionConstants.kSquaredCoeffOnVelFilter, 2));
  }

  private boolean camsWithinField(Translation3d pose, WallEyePoseResult result) {

    return (result.getNumTags() >= 2 || result.getAmbiguity() < VisionConstants.kMaxAmbig)
        && pose.getMeasureX().in(Meters) < field.getFieldLength()
        && pose.getMeasureY().in(Meters) < field.getFieldWidth();
  }

  private double getStdDevFactor(double distance, int numTags, String camName) {
    switch (camName) {
      case "Upper Right":
      case "Upper Left":
        if (numTags == 1)
          return 1.0
              / VisionConstants.FOV58YUYVBaseTrust
              * FastMath.pow(
                  VisionConstants.baseNumber,
                  FastMath.pow(
                      VisionConstants.FOV58YUYVSingleTagCoeff * distance,
                      VisionConstants.FOV58YUYVPowerNumber));

        return 1
            / VisionConstants.FOV58YUYVBaseTrust
            * FastMath.pow(
                VisionConstants.baseNumber,
                FastMath.pow(
                    VisionConstants.FOV58YUYVMultiTagCoeff * distance,
                    VisionConstants.FOV58YUYVPowerNumber));

      case "Rear":
        if (numTags == 1)
          return 1
              / VisionConstants.FOV58YUYVBaseTrust
              * FastMath.pow(
                  VisionConstants.baseNumber,
                  FastMath.pow(
                      VisionConstants.FOV58MJPGSingleTagCoeff * distance,
                      VisionConstants.FOV58YUYVPowerNumber));

        return 1
            / VisionConstants.FOV58YUYVBaseTrust
            * FastMath.pow(
                VisionConstants.baseNumber,
                FastMath.pow(
                    VisionConstants.FOV58MJPGSingleTagCoeff * distance,
                    VisionConstants.FOV58YUYVPowerNumber));

      case "Servo Left":
      case "Servo Right":
        if (numTags == 1)
          return 1
              / VisionConstants.FOV75YUYVBaseTrust
              * FastMath.pow(
                  VisionConstants.baseNumber,
                  FastMath.pow(
                      VisionConstants.FOV75YUYVSingleTagCoeff * distance,
                      VisionConstants.FOV75YUYVPowerNumber));
        return 1
            / VisionConstants.FOV75YUYVBaseTrust
            * FastMath.pow(
                VisionConstants.baseNumber,
                FastMath.pow(
                    VisionConstants.FOV75YUYVMultiTagCoeff * distance,
                    VisionConstants.FOV75YUYVPowerNumber));

      default:
        if (numTags == 1)
          return 1
              / VisionConstants.baseTrust
              * FastMath.pow(
                  VisionConstants.baseNumber,
                  FastMath.pow(
                      VisionConstants.singleTagCoeff * distance, VisionConstants.powerNumber));
        return 1
            / VisionConstants.baseTrust
            * FastMath.pow(
                VisionConstants.baseNumber,
                FastMath.pow(
                    VisionConstants.multiTagCoeff * distance, VisionConstants.powerNumber));
    }
  }

  private Pose3d getCloserPose(Pose3d pose1, Pose3d pose2, double rotation) {
    if (Math.abs(new Rotation2d(rotation).minus(pose1.getRotation().toRotation2d()).getRadians())
        <= Math.abs(
            new Rotation2d(rotation).minus(pose2.getRotation().toRotation2d()).getRadians()))
      return pose1;
    else return pose2;
  }

  private Pose3d getCorrectPose(Pose3d pose1, Pose3d pose2, double time, int camIndex) {
    double dist1 = Math.abs(camHeights[camIndex] - pose1.getZ());
    double dist2 = Math.abs(camHeights[camIndex] - pose2.getZ());

    if (dist1 < dist2 && dist1 < 0.5 && dist1 > 0) {
      return pose1;
    }
    if (dist2 < dist1 && dist2 < 0.5 && dist2 > 0) {
      return pose2;
    }

    if (gyroBuffer.size() < VisionConstants.kCircularBufferSize) return pose1;

    double rotation =
        gyroBuffer.get(FastMath.floorToInt(((time / 1_000_000.0) / VisionConstants.kLoopTime)));
    return getCloserPose(pose1, pose2, rotation);
  }

  @Override
  public void periodic() {
    gyroBuffer.addFirst(
        FastMath.normalizeMinusPiPi(driveSubsystem.getGyroRotation2d().getRadians()));

    if (getSeconds() - timeSinceLastUpdate > VisionConstants.kMaxTimeNoVision) {
      updatesToWheels = 0;
    }

    validResults.clear();

    for (int i = 0; i < VisionConstants.kNumCams; i++) {
      if (cams[i].hasNewUpdate()) {
        timeSinceLastUpdate = getSeconds();
        validResults.add(new Pair<WallEyeResult, Integer>(cams[i].getResults(), i));
      }
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }
}
