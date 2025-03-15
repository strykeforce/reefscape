package frc.robot.constants;

public class RobotStateConstants {
  public static final double[] kNodeAngles = {0.0, 60.0, 120.0, 180.0, -120.0, -60.0};
  public static final double kAlgaeRetreatDistance = 0;

  public static final double kBlueBargeSafeX = 7.6;
  public static final double kRedBargeSafeX = DriveConstants.kFieldMaxX - kBlueBargeSafeX;

  public static final double kCoralEjectTimer = 0.25;
  public static final double kAlgaeEjectTimer = 0.5;

  public static final double kProcessorStowRadius = 0.5;

  // Elevator good for climb, tolerates stuck coral
  public static final double kElevatorClimbMax = 11.4;

  // Climb LED Thresholds
  public static final double kClimbAngleSmall = -0.245;
  public static final double kClimbAngleBig = -0.215;
}
