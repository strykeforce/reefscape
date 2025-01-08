package frc.robot.subsystems.algae;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

import edu.wpi.first.units.measure.AngularVelocity;


public interface algaeIO{
  @AutoLog
  public static class IOInputs {
    public AngularVelocity velocity;
  }

  public default void updateInputs(IOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void zero() {}

  public default void registerWith(TelemetryService telemetryService) {}
}