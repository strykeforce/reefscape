package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface AlgaeIO {
  @AutoLog
  public static class AlgaeIOInputs {
    public double velocity;
    public double statorCurrent;
    public boolean isBeamBroken;
    public boolean isCoralBeamBroken;
  }

  public default void updateInputs(AlgaeIOInputs inputs) {}

  public default void setSpeed(double speed) {}

  public default void setPct(double pct) {}

  public default void zero() {}

  public default void registerWith(TelemetryService telemetryService) {}
}
