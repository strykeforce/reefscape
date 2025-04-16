package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public class RobotStateConstants {
  public static final double[] kNodeAngles = {0.0, 60.0, 120.0, 180.0, -120.0, -60.0};
  public static final double kAlgaeRetreatDistance = 0;

  public static final double kBlueBargeSafeX = 7; // 7.6
  public static final double kRedBargeSafeX = DriveConstants.kFieldMaxX - kBlueBargeSafeX;

  public static final double kCoralEjectTimer = 0.25;
  public static final double kAlgaeEjectTimer = 0.5;

  public static final double kProcessorStowRadius = 0.5;
  public static final double kL1CoralStowRadius = 1.766;

  public static final double kElevatorWaitRadius = 1.5;

  // Super Cycle Constants
  public static final double kBiscuitSuperCycleSafeFastThres = 8; // 6 is max
  public static final double kBiscuitSuperCycleSafeThres = 6; // 6 is max

  // Elevator good for climb, tolerates stuck coral
  public static final double kElevatorClimbMax = 11.4;

  // Climb LED Thresholds
  public static final double kClimbAngleSmall = -0.235; // -0.245
  public static final double kClimbAngleBig = -0.213; // -0.215

  // Funnel load algae on
  public static final double kL1FunnelLoadX = 3.25;

  // Offsets
  public static final Angle kStuckCoralElevatorOffset = Rotations.of(3.746);
  public static final Angle[][][] kBlueCoralElevatorOffset = {
    {
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)}
    },
    {
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)}
    },
    {
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)}
    },
    {
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)}
    },
    {
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)}
    },
    {
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)}
    }
  };

  public static final Angle[][][] kRedCoralElevatorOffset = {
    {
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)}
    },
    {
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)}
    },
    {
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)}
    },
    {
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)}
    },
    {
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)}
    },
    {
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)},
      {Rotations.of(0), Rotations.of(0)}
    }
  };
}
