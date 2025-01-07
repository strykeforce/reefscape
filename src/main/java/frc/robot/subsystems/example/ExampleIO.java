package frc.robot.subsystems.example;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;
// import org.strykeforce.telemetry.TelemetryService;

public interface ExampleIO {

  @AutoLog
  public static class ExampleIOInputs {
    public Angle position = Rotations.of(0.0);
    public AngularVelocity velocity = RotationsPerSecond.of(0.0);
  }

  public default void updateInputs(ExampleIOInputs inputs) {}

  public default void setPosition(Angle position) {}

  public default void zero() {}

  //   public default void registerWith(TelemetryService telemetryService) {}
}
