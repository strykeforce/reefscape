package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

public class RobotConstants {
  public static final String protoSerial = "032243F2";
  public static final boolean isComp = !RobotController.getSerialNumber().equals(protoSerial);

  public static final int kTalonConfigTimeout = 10; // ms

  public static final double kJoystickDeadband = 0.1;
  public static final double kTriggerDeadband = 0.5;
  public static final double kTestingDeadband = 0.5;

  public static double kZero;
  public static Angle kFunnelSetpoint;
  public static Angle kStowSetpoint;

  public RobotConstants() {
    if (isComp) {
      // Comp bot constants
      kZero = CompConstants.kZero;
      kFunnelSetpoint = CompConstants.kFunnelSetpoint;
      kStowSetpoint = CompConstants.kStowSetpoint;
    } else {
      // Proto constants
      kZero = ProtoConstants.kZero;
      kFunnelSetpoint = ProtoConstants.kFunnelSetpoint;
      kStowSetpoint = ProtoConstants.kStowSetpoint;
    }
  }

  public static class ProtoConstants {
    public static final double kZero = 36;
    public static final Angle kFunnelSetpoint = Rotations.of(2.03125);
    public static final Angle kStowSetpoint = kFunnelSetpoint;
  }

  public static class CompConstants {
    public static final double kZero = .37;
    public static final Angle kFunnelSetpoint = Rotations.of(0.3676757);
    public static final Angle kStowSetpoint = kFunnelSetpoint;
  }
}
