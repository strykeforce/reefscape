package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface CoralIO {

  @AutoLog
  public static class CoralIOInputs {
    public AngularVelocity velocity = RotationsPerSecond.of(0.0);
    public boolean isFwdLimitSwitchClosed = false;
    public boolean isRevLimitSwitchClosed = false;
  }

  public default void setVelocity(AngularVelocity velocity) {}

  public default void enableFwdLimitSwitch(boolean enabled) {}

  public default void enableRevLimitSwitch(boolean enabled) {}

  public default void updateInputs(CoralIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
