package frc.robot.subsystems.algae;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix6.signals.ReverseLimitValue;

public interface algaeIO {
  @AutoLog
  public static class AlgaeIOInputs {
    public AngularVelocity velocity;
    public static Angle position;
    public ReverseLimitValue reverseLimitSwitch;
  }

  public default void updateInputs(AlgaeIOInputs inputs) {}

  public default void setSpeed(double speed) {}

  public default void zero() {}

  public default void registerWith(TelemetryService telemetryService) {}
}