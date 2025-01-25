package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class TagServoingConstants {
    // Cameras
    public static final int kLeftServoCam = 0; // Score RIGHT coral
    public static final int kRightServoCam = 1; // Score LEFT coral

    // Tags
    public static final int[] kBlueTargetTag = { 18, 17, 22, 21, 20, 19 };
    public static final int[] kRedTargetTag = { 7, 8, 9, 10, 11, 6 };

    // Targets
    public static final double kHorizontalTarget = 800;
    public static final double kAreaTarget = 800;

    public static final double[] kAngleTarget = { Units.degreesToRadians(0), Units.degreesToRadians(60),
            Units.degreesToRadians(120), Units.degreesToRadians(180),
            Units.degreesToRadians(240), Units.degreesToRadians(300) };

    // Tag align
    public static final double kAreaCloseEnough = 0;
    public static final double kHorizontalCloseEnough = 0;
    public static final double kAngleCloseEnough = Units.degreesToRadians(3.0);

    // Drive
    public static final double kInitialDriveRadius = 1.5;
    public static final double kStopXDriveRadius = 1.2;
    public static final double kDriveCloseEnough = 0.3;

    // Reef
    public static final Translation2d kBlueReefPose = new Translation2d(Units.inchesToMeters(223.5),
            Units.inchesToMeters(158.5));

    public static final Translation2d kRedReefPose = kBlueReefPose
            .plus(new Translation2d(Units.inchesToMeters(337.39), 0));
}
