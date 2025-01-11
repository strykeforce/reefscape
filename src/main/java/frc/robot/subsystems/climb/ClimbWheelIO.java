package frc.robot.subsystems.climb;

import org.strykeforce.telemetry.TelemetryService;

public interface ClimbWheelIO {

  public default void setPercent(double pct) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
