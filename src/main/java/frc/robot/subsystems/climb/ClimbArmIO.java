package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ClimbArmIO {

  @AutoLog
  static class ClimbArmIOInputs {
    public Angle position = Rotations.of(0.0);
  }

  public default void setPosition(Angle position) {}

  public default void updateInputs(ClimbArmIOInputs inputs) {}

  public default void zero() {}

  public default void registerWith(TelemetryService telemetryService) {}
}
