package frc.robot.subsystems.climb;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public interface ClimbIO {

  @AutoLog
  static class ClimbIOInputs {
    public Angle position = Rotations.of(0.0);
    public double velocity = 0.0;
    public double ratchetServoPosition = 0.0;
    public double pinServoPosition = 0.0;
  }

  public default void setPosition(Angle position) {}
  
  public default void setRatchetServoPosition(double position) {} //needed? since it's double, not angle
  
  public default void setPinServoPosition(double position) {} //may make a method to stick these both into one method

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void zero() {} 

  public default void registerWith(TelemetryService telemetryService) {}

  public default void setSoftLimitsEnabled(boolean enable) {}

  public default void setCurrentLimit(CurrentLimitsConfigs config) {}
}