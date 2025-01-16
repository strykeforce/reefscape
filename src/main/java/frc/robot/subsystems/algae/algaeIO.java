package frc.robot.subsystems.algae;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix6.signals.ReverseLimitValue;

public interface algaeIO {
  @AutoLog
  public static class IOInputs {
    public AngularVelocity velocity;
    public ReverseLimitValue reverseLimitSwitch;
  }

  public default void updateInputs(IOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void zero() {}

  public default void registerWith(TelemetryService telemetryService) {}
}
