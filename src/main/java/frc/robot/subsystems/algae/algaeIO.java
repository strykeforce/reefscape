package frc.robot.subsystems.algae;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface algaeIO {
  @AutoLog
  public static class AlgaeIOInputs {
    public AngularVelocity velocity;
    public boolean isFwdLimitSwitchClosed;
    public boolean isRevLimitSwitchClosed;
  }

  public default void updateInputs(AlgaeIOInputs inputs) {}

  public default void setSpeed(AngularVelocity speed) {}

  public default void zero() {}

  public default void registerWith(TelemetryService telemetryService) {}
}
