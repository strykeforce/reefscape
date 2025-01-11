package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ClimbWheelIO {
    
    @AutoLog static class ClimbWheelIOInputs {
        public AngularVelocity velocity = RotationsPerSecond.of(0.0);
    }

  public default void setVelocity(AngularVelocity velocity) {}
  
  public default void updateInputs(ClimbWheelIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
