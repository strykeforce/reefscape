package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class AutonConstants {
  public static final double kAutoTimeout = 0.5;
  public static final double kMaxPathErrorMeters = 0.05;
  public static final double kMaxOmegaErrorDegrees = 0.5;
  public static final double kMaxOmegaErrorRadians = Units.degreesToRadians(kMaxOmegaErrorDegrees);
  public static final int kSwitchStableCounts = 3;
  public static final double kElevatorStageRadius = 2.3;
  public static final double kElevatorStageRadiusPathOne = 2.1; // 1.8
  public static final double kInitPathPrestageTime = 0.6;

  public static final double kStageBargeDistance = 2.5;
  public static final double kBargeScoreMinTime = 3;

  // Start Poses
  public static final Pose2d kNonProcessorShallow =
      new Pose2d(7.1, 5.076, Rotation2d.fromDegrees(180));
  public static final Pose2d kNonProcessorMid =
      new Pose2d(7.1, 6.1663548, Rotation2d.fromDegrees(180));
  public static final Pose2d kProcessorShallow =
      new Pose2d(7.1, 2.9718, Rotation2d.fromDegrees(180));
  public static final Pose2d kProcessorMid = new Pose2d(7.1, 1.8818, Rotation2d.fromDegrees(180));
  public static final Pose2d kNonProcessorDeep =
      new Pose2d(7.1, 7.257, Rotation2d.fromDegrees(180));
}
