package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class BargeAlignConstants {
  public static final double kXSpeed = 0.45;

  public static final double kBlueEjectAlgaeX = 7.67; // 7.72
  public static final double kRedEjectAlgaeX = 9.86; // 9.81
  public static final double kBlueRaiseElevatorX = 7.02; // 7.22
  public static final double kRedRaiseElevatorX = 10.51; // 10.31

  public static final Rotation2d kBlueDesiredYaw = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d kRedDesiredYaw = Rotation2d.fromDegrees(180.0);
}
