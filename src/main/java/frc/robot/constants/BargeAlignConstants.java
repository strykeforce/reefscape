package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class BargeAlignConstants {
  public static final double kXSpeed = 0.35;

  public static final double kBlueEjectAlgaeX = 7.72;
  public static final double kRedEjectAlgaeX = 9.81;
  public static final double kBlueRaiseElevatorX = 6.72;
  public static final double kRedRaiseElevatorX = 10.81;

  public static final Rotation2d kBlueDesiredYaw = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d kRedDesiredYaw = Rotation2d.fromDegrees(180.0);
}
