package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class BargeAlignConstants {
  public static final double kXSpeed = 0.45;
  public static final double kXRevSpeed = 2;

  public static final double kBlueEjectAlgaeX = 7.67; // 7.72
  public static final double kRedEjectAlgaeX = 9.86; // 9.81
  public static final double kBlueRaiseElevatorX = 6.82; // 7.02
  public static final double kRedRaiseElevatorX = 10.71; // 10.51

  public static final double kBlueRevDoneX = kBlueRaiseElevatorX + 0.7; // +0.25
  public static final double kRedRevDoneX = kRedRaiseElevatorX - 0.7; // -0.25

  public static final double kBlueUnsafeX = 7.02;
  public static final double kRedUnsafeX = 10.51;

  public static final Rotation2d kBlueDesiredYaw = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d kRedDesiredYaw = Rotation2d.fromDegrees(180.0);
}
