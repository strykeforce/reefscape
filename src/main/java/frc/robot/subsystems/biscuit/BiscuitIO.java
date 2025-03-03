package frc.robot.subsystems.biscuit;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface BiscuitIO {

  @AutoLog
  public class BiscuitIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
    public boolean fwdLimitSwitchOpen = false;
    public boolean didZero;
  }

  public default void setPosition(Angle position) {}

  public default void updateInputs(BiscuitIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetry) {}

  public default boolean zero() {
    return false;
  }
}
