package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class TagServoingConstants {
  // Cameras
  public static final int kLeftServoCam = 0; // Score RIGHT coral
  public static final int kRightServoCam = 2; // Score LEFT coral

  // Offsets
  public static final double kLeftCamOffset = VisionConstants.kCam1Pose.getY();
  public static final double kRightCamOffset = VisionConstants.kCam2Pose.getY();

  // Targets
  public static final double kHorizontalTarget = 800;
  public static final double kLeftCamDiagTarget = 1180;
  public static final double kRightCamDiagTarget = 985;

  // Constraints
  public static final Constraints driveXConstraints = new Constraints(2, 2.0);
  public static final Constraints driveYConstraints = new Constraints(2, 3.0);
  public static final Constraints driveOmegaConstraints = new Constraints(1, 2.0);

  public static final Constraints alignXConstraints = new Constraints(1, 1.0);
  public static final Constraints alignYConstraints = new Constraints(1, 1.5);
  public static final Constraints alignOmegaConstraints = new Constraints(1, 1);

  public static final double[] kAngleTarget = {
    Units.degreesToRadians(0),
    Units.degreesToRadians(60),
    Units.degreesToRadians(120),
    Units.degreesToRadians(180),
    Units.degreesToRadians(240),
    Units.degreesToRadians(300)
  };

  // Tags
  public static final int[] kBlueTargetTag = {18, 17, 22, 21, 20, 19};
  public static final int[] kRedTargetTag = {7, 8, 9, 10, 11, 6};

  // Tag align
  public static final double kHorizontalCloseEnough = 20;
  public static final double kAngleCloseEnough = Units.degreesToRadians(1.0);
  public static final double kDiagCloseEnough = 20;
  public static final double kNoUpdateMicrosec = 500_000;

  // Drive
  public static final double kCoralInitialDriveRadius = 1.7; // 1.5
  public static final double kCoralStopXDriveRadius =
      kCoralInitialDriveRadius; // Should be closer to reef than target pose
  public static final double kAlgaeInitialDriveRadius = 1.6;
  public static final double kAlgaeStopXDriveRadius =
      kAlgaeInitialDriveRadius; // Should be closer to reef than target pose
  public static final double kCoralDriveCloseEnough = 0.1;
  public static final double kAlgaeDriveCloseEnough = 0.1;
  public static final double kMinVelX = 0.85;

  // Reef
  public static final Translation2d kBlueReefPose = new Translation2d(4.524, 4.033);

  public static final Translation2d kRedReefPose = new Translation2d(13.084, 4.033);
}
