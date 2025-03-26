package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void setPosition(Angle position) {}

  public default void setVelocityOpenLoop(double dutyCycleOut) {}

  public default void setCurrentLimitConfig(CurrentLimitsConfigs config) {}

  public default void setVoltageOpenLoop(double voltsOut) {}

  public default void setSoftLimitConfig(SoftwareLimitSwitchConfigs config) {}

  public default void zero() {}
}
