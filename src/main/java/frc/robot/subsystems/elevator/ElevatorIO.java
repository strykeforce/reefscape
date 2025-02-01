package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
    // something about height and IO layer?
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void setPosition(Angle position) {}

  public default void setVelocityOpenLoop(double dutyCycleOut) {}

  public default void setCurrentLimitConfig(CurrentLimitsConfigs config) {}

  public default void zero() {}
}











/*
 * To-do: resolve all the comments, implement zero, find out if follower actually means every LOC for one motor does the same
 * thing to the other?, positionrequest never used in IOFX
 */