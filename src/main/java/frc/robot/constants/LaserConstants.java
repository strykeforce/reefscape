package frc.robot.constants;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class LaserConstants {
  public static final int LaserCanId = 50;

  public static CANrangeConfiguration getLaserConfig() {
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ToFParams =
        config.ToFParams.withUpdateMode(UpdateModeValue.LongRangeUserFreq).withUpdateFrequency(50);
    config.ProximityParams = config.ProximityParams.withProximityThreshold(1.0);
    config.FovParams = config.FovParams.withFOVRangeX(10.0).withFOVRangeY(10.0);
    return config;
  }
}
