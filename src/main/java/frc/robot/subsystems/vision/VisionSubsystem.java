package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.VisionConstants;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import frc.robot.subsystems.drive.DriveSubsystem;

public class VisionSubsystem extends MeasurableSubsystem {

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

  public VisionSubsystem(DriveSubsystem driveSubsystem) {

  }
  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }
}
