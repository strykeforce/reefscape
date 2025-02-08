package frc.robot.subsystems.funnel;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface FunnelIO {

  @AutoLog
  public static class FunnelIOInputs {
    public AngularVelocity velocity;
    public boolean isRevBeamBroken = false;
  }

  public default void setPct(double percentOutput) {}

  public default void updateInputs(FunnelIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
