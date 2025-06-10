package frc.robot.constants;

import com.ctre.phoenix6.configs.CANrangeConfiguration;

public class LaserConstants {
  public static final int LaserCanId = 0;

  public static CANrangeConfiguration getLaserConfig() {
    CANrangeConfiguration config = new CANrangeConfiguration();
    return config;
  }
}
