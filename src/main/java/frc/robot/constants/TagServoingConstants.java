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
  public static final double kRightCamOffset = VisionConstants.kCam3Pose.getY();

  // Targets
  public static final double kHorizontalTarget = 800;
  public static final double kLeftCamDiagTarget = 1180;
  public static final double kRightCamDiagTarget = 985;

  // Constraints
  public static final Constraints driveXConstraints = new Constraints(2, 100000);
  public static final Constraints driveYConstraints = new Constraints(2, 100000);
  public static final Constraints driveOmegaConstraints = new Constraints(10000, 20000);

  public static final Constraints alignXConstraints = new Constraints(2, 100000);
  public static final Constraints alignYConstraints = new Constraints(2, 100000);
  // public static final Constraints alignOmegaConstraints = new
  // Constraints(10000, 20000);

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

  // Offsets
  public static final double[][][] kBlueCoralOffset = {
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}}
  }; // Alliance relative hexant, level, left/right
  public static final double[][][] kRedCoralOffset = {
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}}
  }; // Alliance relative hexant, level, left/right

  // Tag align
  public static final double kHorizontalCloseEnough = 20;
  public static final double kAngleCloseEnough = Units.degreesToRadians(1.0);
  public static final double kDiagCloseEnough = 20;
  public static final double kNoUpdateMicrosec = 500_000;

  // Drive
  public static final double kCoralInitialDriveRadius = 1.7; // 0.36 away from reef wall
  public static final double kCoralAlignRadius = 1.32; // 1.293823 is perfectly against the reef
  // public static final double kCoralStopXDriveRadius =
  // kCoralInitialDriveRadius; // Should be closer to reef than target pose
  public static final double kAlgaeInitialDriveRadius = kCoralInitialDriveRadius; // 1.75;
  public static final double kAlgaeAlignRadius = kCoralAlignRadius; // 1.34; // was 1.34
  public static final double kAlgaeStopXDriveRadius =
      kAlgaeInitialDriveRadius; // Should be closer to reef than target
  public static final double kL1CoralRadius = 1.5;
  // pose
  // public static final double kMinVelX = 0.85;

  // End conditions
  public static final double kInitialCloseEnough = 0.1;
  public static final double kCoralDriveXCloseEnough = 0.03;
  public static final double kCoralDriveYCloseEnough = 0.015; // was 0.025
  public static final double kAlgaeDriveXCloseEnough = 0.03;
  public static final double kAlgaeDriveYCloseEnough = kCoralDriveYCloseEnough;

  public static final double kSmallYThres = 0.3; // For elevator staging

  public static final double kEndDriveCurrentThreshold = 25;
  public static final int kEndCountThreshold = 1;
  public static final double kEndVelThreshold = 5;

  // Stuck coral
  public static final double kMaxStalledDer = 0.05;
  public static final double kMinStuckCounts = 5;

  // Reef
  public static final Translation2d kBlueReefPose = new Translation2d(4.489323, 4.0259);
  public static final Translation2d kRedReefPose = new Translation2d(13.058902, 4.0259);
}
