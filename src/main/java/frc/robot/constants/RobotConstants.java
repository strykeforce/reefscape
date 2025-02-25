package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotController;

public class RobotConstants {
  public static final String protoSerial = "032243F2";
  public static final boolean isComp = !RobotController.getSerialNumber().equals(protoSerial);

  public static final int kTalonConfigTimeout = 10; // ms

  public static final double kJoystickDeadband = 0.1;
  public static final double kTriggerDeadband = 0.5;
  public static final double kTestingDeadband = 0.5;

  public RobotConstants() {
    if (isComp) {
      // Fill with comp bot constants
    } else {
      // Fill with proto constants
    }
  }
}
