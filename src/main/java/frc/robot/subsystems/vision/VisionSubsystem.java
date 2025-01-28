package frc.robot.subsystems.vision;

import WallEye.*;
import WallEye.UdpSubscriber;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;

import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import java.lang.Math;

public class VisionSubsystem extends MeasurableSubsystem {

  WallEyeCam[] cams;

  Translation2d[] camPositions = {
    VisionConstants.kCam1Pose.getTranslation().toTranslation2d(),
    VisionConstants.kCam2Pose.getTranslation().toTranslation2d(),
    VisionConstants.kCam3Pose.getTranslation().toTranslation2d(),
    VisionConstants.kCam4Pose.getTranslation().toTranslation2d()
  };

  Rotation2d[] camRotations = {
    VisionConstants.kCam1Pose.getRotation().toRotation2d(),
    VisionConstants.kCam2Pose.getRotation().toRotation2d(),
    VisionConstants.kCam3Pose.getRotation().toRotation2d(),
    VisionConstants.kCam4Pose.getRotation().toRotation2d()
  };

  String[] camNames = {
    VisionConstants.kCam1Name, VisionConstants.kCam2Name, VisionConstants.kCam3Name
  };

  String[] piNames = {
    VisionConstants.kPi1Name, VisionConstants.kPi2Name, VisionConstants.kPi3Name,
  };

  int[] camIndex = {
    VisionConstants.kCam1Idx,
    VisionConstants.kCam2Idx,
    VisionConstants.kCam3Idx,
    VisionConstants.kCam4Idx
  };

  private Swerve swerve = new Swerve();
  private DriveSubsystem driveSubsystem = new DriveSubsystem(swerve);
  private Logger logger;
  private UdpSubscriber[] udpSubscriber;
  private Matrix adaptiveMatrix;
  private AprilTagFieldLayout field;
  private boolean updating = true;
  private int minTags;

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

  public void setVisionUpdating(boolean updating) {
    this.updating = updating;
  }

  public boolean isVisionUpdating() {
    return updating;
  }

  public void setMinTags(int minTags) {
    this.minTags = minTags;
  }

  public boolean cameraConnected(int index) {
    return cams[index].isCameraConnected();
  }

  //Filters
  private boolean camsAgreeWithWheels(Translation3d pose, WallEyeResult result) {

      ChassisSpeeds vel = driveSubsystem.getFieldRelSpeed();
      Pose2d curPose = driveSubsystem.getPoseMeters();

      Translation2d disp = (curPose.getTranslation().minus(pose.toTranslation2d()));

      double velMagnitude = Math.sqrt(Math.pow(vel.vxMetersPerSecond,2)
      + Math.pow(vel.vyMetersPerSecond, 2));

      double dispMagnitude = Math.sqrt(Math.pow(disp.getX(),2)+
      Math.pow(disp.getY(),2));

      return result.getNumTags() >= minTags &&
      dispMagnitude <= (velMagnitude * VisionConstants.kLinearCoeffOnVelFilter 
      + VisionConstants.kOffsetOnVelFilter
      + Math.pow(velMagnitude * VisionConstants.kSquaredCoeffOnVelFilter, 2)
      );
  }

  private boolean camsWithinField(Translation3d pose, WallEyePoseResult result) {

    return (result.getNumTags() >= 2 
    || result.getAmbiguity() < VisionConstants.kMaxAmbig) 
    && pose.getMeasureX().in(Meters) < field.getFieldLength() 
    && pose.getMeasureY().in(Meters) < field.getFieldWidth();
  }

  


  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }
}
