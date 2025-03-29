package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class BargeAlignConstants {
  public static final double kXSpeed = 1.0;

  public static final double kBlueEjectAlgaeX = 6.0;
  public static final double kRedEjectAlgaeX = 6.0;
  public static final double kBlueRaiseElevatorX = 5.0;
  public static final double kRedRaiseElevatorX = 5.0;

  public static final Rotation2d kBlueDesiredYaw = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d kRedDesiredYaw = Rotation2d.fromDegrees(180.0);
}
